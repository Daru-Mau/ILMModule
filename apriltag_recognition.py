"""
Optimized AprilTag Recognition System for Robot Navigation
Focused on accurate tag detection without video streaming features
"""

import cv2
import time
import os
import numpy as np
import sys
import signal
import serial
from threading import Event

# Check if running on Raspberry Pi
IS_RASPBERRY_PI = os.path.exists('/sys/firmware/devicetree/base/model')

if IS_RASPBERRY_PI:
    from picamera2 import Picamera2
    from pupil_apriltags import Detector
    from uart_communication import UARTCommunicator, TagData, DIR_STOP, DIR_FORWARD, DIR_BACKWARD, DIR_LEFT, DIR_RIGHT, get_direction_name

# === Configuration ===
# Camera settings
CAMERA_WIDTH = 1640
CAMERA_HEIGHT = 1232
FOCAL_LENGTH_PX = 1000
TAG_SIZE_MM = 150
TAG_SIZE_CM = TAG_SIZE_MM / 10

# Camera intrinsic parameters
CAM_FX = 1000.0
CAM_FY = 1000.0
CAM_CX = CAMERA_WIDTH / 2
CAM_CY = CAMERA_HEIGHT / 2

# Arduino communication
ARDUINO_ENABLED = True
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# Detection settings
VERBOSE = False
DETECTION_INTERVAL = 0.1

# Navigation parameters
CENTER_TOLERANCE = 0.15
DISTANCE_THRESHOLD = 30

# === Global state ===
shutdown_event = Event()


def setup_camera():
    """Initialize the Raspberry Pi camera with optimal settings for AprilTag detection"""
    if not IS_RASPBERRY_PI:
        return None

    try:
        # Initialize camera
        picam = Picamera2()

        # Configure camera with settings optimized for tag detection
        config = picam.create_preview_configuration(
            main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "BGR888"},
            controls={
                "FrameRate": 15,
                "AwbEnable": True,
                "ExposureTime": 8000,
                "AnalogueGain": 4.0,
                "Sharpness": 15.0,
                "Contrast": 2.0,
                "Brightness": 0.0,
                "NoiseReductionMode": 0,
                "AwbMode": 1
            }
        )

        picam.set_controls({"NoiseReductionMode": 0})
        picam.configure(config)
        picam.start()
        time.sleep(0.5)

        try:
            picam.set_controls({
                "FrameDurationLimits": (33333, 66666)
            })
        except Exception:
            pass  # Ignore if not supported

        print(
            f"Camera initialized with resolution {CAMERA_WIDTH}x{CAMERA_HEIGHT}")

        return picam

    except Exception as e:
        print(f"Camera setup failed: {e}")
        try:
            import subprocess
            subprocess.run(['sudo', 'pkill', '-f', 'libcamera'],
                           stderr=subprocess.DEVNULL)
        except:
            pass
        return None


def setup_serial():
    """Initialize serial connection to Arduino using UART communication"""
    if not ARDUINO_ENABLED:
        return None

    try:
        print("Attempting to connect to Arduino...")

        # Use the UARTCommunicator for robust communication
        communicator = UARTCommunicator(SERIAL_PORT, BAUD_RATE)
        if not communicator.connected:
            print("Auto-detection of Arduino port...")
            if communicator.connect():
                print(f"Arduino connected successfully on {communicator.port}")
                return communicator
            else:
                print("Failed to connect to Arduino. Check connections and try again.")
                return None
        else:
            print(f"Arduino connected successfully on {communicator.port}")
            return communicator

    except Exception as e:
        print(f"Failed to connect to Arduino: {e}")
        return None


def estimate_tag_size(corners):
    """Calculate tag size in pixels based on corner positions with robust metrics"""
    # Calculate all side lengths
    side_lengths = []
    for i in range(4):
        side = np.linalg.norm(corners[i] - corners[(i+1) % 4])
        side_lengths.append(side)

    # Calculate diagonal lengths
    diagonal1 = np.linalg.norm(corners[0] - corners[2])
    diagonal2 = np.linalg.norm(corners[1] - corners[3])

    # Verify if the shape is roughly square
    sides_std = np.std(side_lengths)
    sides_mean = np.mean(side_lengths)
    diag_ratio = max(diagonal1, diagonal2) / min(diagonal1, diagonal2)

    # Handle distorted tags
    if sides_std / sides_mean > 0.2 or diag_ratio > 1.3:
        estimated_side = np.median(side_lengths)
        estimated_side_from_diag = (diagonal1 + diagonal2) / (2 * np.sqrt(2))
        return 0.7 * estimated_side + 0.3 * estimated_side_from_diag
    else:
        return sides_mean


def calculate_distance(tag_size_px):
    """Calculate distance to AprilTag using a more accurate camera model"""
    distance_mm = (CAM_FX * TAG_SIZE_MM) / tag_size_px
    distance_cm = distance_mm / 10.0

    # Apply calibration correction
    if distance_cm < 50:
        return distance_cm * 1.05
    elif distance_cm < 150:
        return distance_cm * 1.02
    else:
        return distance_cm * 0.98


def get_direction(detection, frame_width):
    """Determine movement direction based on tag position and size"""
    center_x = detection.center[0]
    tag_size_px = estimate_tag_size(detection.corners)
    distance_cm = calculate_distance(tag_size_px)

    # Calculate frame center and tolerance zone
    frame_center = frame_width / 2
    tolerance = frame_width * CENTER_TOLERANCE

    # Determine direction based on tag position and distance
    # Direction codes: 0=STOP, 1=FORWARD, 2=BACKWARD, 3=LEFT, 4=RIGHT
    if distance_cm < DISTANCE_THRESHOLD:
        return DIR_STOP, distance_cm  # Stop when close enough

    if center_x < frame_center - tolerance:
        return DIR_LEFT, distance_cm  # Turn left
    elif center_x > frame_center + tolerance:
        return DIR_RIGHT, distance_cm  # Turn right
    else:
        return DIR_FORWARD, distance_cm  # Move forward


def preprocess_image(frame):
    """Advanced image preprocessing for better tag detection"""
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply advanced adaptive histogram equalization
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(6, 6))
    gray = clahe.apply(gray)

    # Apply bilateral filter
    gray = cv2.bilateralFilter(gray, 5, 75, 75)

    # Enhance contrast
    gray = cv2.convertScaleAbs(gray, alpha=1.5, beta=15)

    # Apply edge enhancement
    kernel = np.array([[-1, -1, -1], [-1, 9.5, -1], [-1, -1, -1]])
    gray = cv2.filter2D(gray, -1, kernel)

    return gray


def signal_handler(sig, frame):
    """Handle shutdown signals"""
    print("Shutting down...")
    shutdown_event.set()
    sys.exit(0)


def main():
    # Parse command line arguments
    import argparse
    global SERIAL_PORT
    parser = argparse.ArgumentParser(
        description='AprilTag Detection and Robot Control')
    parser.add_argument('--max-speed', type=int, default=50,
                        help='Maximum motor speed (50-255)')
    parser.add_argument('--min-speed', type=int, default=40,
                        help='Minimum motor speed (30-max_speed)')
    parser.add_argument('--port', type=str, default=SERIAL_PORT,
                        help=f'Serial port (default: {SERIAL_PORT})')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Enable verbose output')
    args = parser.parse_args()

    SERIAL_PORT = args.port
    if args.verbose:
        global VERBOSE
        VERBOSE = True

    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if not IS_RASPBERRY_PI:
        print("This script requires a Raspberry Pi to run")
        return

    # Initialize hardware
    camera = setup_camera()
    arduino = setup_serial()

    # Set motor speed parameters if Arduino connection successful
    if arduino:
        print(
            f"Setting motor speeds - MAX: {args.max_speed}, MIN: {args.min_speed}")
        arduino.set_speed(args.max_speed, args.min_speed)

    if not camera:
        print("Failed to initialize camera. Exiting.")
        return

    print("Setting up AprilTag detector...")

    # Initialize AprilTag detector
    detector = Detector(
        families="tag36h11",
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.6,
        refine_edges=True,
        decode_sharpening=0.7,
        debug=False
    )

    # Command smoothing variables
    last_commands = []
    MAX_HISTORY = 3

    # Detection parameters
    detection_confidence_threshold = 0.5
    tag_history = {}

    # Timing variables
    last_command_time = 0
    command_interval = 0.1
    last_frame_time = 0
    frame_count = 0

    print("AprilTag detection system running...")

    try:
        while not shutdown_event.is_set():
            current_time = time.time()

            # Limit frame rate
            if current_time - last_frame_time < DETECTION_INTERVAL:
                time.sleep(0.001)
                continue

            last_frame_time = current_time
            frame_count += 1

            # Capture frame
            try:
                frame = camera.capture_array("main")
                if frame is None:
                    time.sleep(0.1)
                    continue
            except Exception as e:
                print(f"Camera error: {e}")
                time.sleep(0.1)
                continue

            # Process image and detect AprilTags
            processed = preprocess_image(frame)
            detections = detector.detect(processed)

            # If no detections with primary method, try alternative processing
            if not detections:
                # Try higher contrast for low light
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                alt_processed = cv2.convertScaleAbs(gray, alpha=2.0, beta=30)
                detections = detector.detect(alt_processed)

                # If still no detections, try adaptive thresholding
                if not detections:
                    adaptive_thresh = cv2.adaptiveThreshold(
                        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv2.THRESH_BINARY, 11, 2
                    )
                    detections = detector.detect(adaptive_thresh)

            # Filter by decision margin (detection confidence)
            if detections:
                detections = [
                    d for d in detections if d.decision_margin > detection_confidence_threshold]

            # Process detected tags
            if detections:
                # Find the tag closest to the center
                main_tag = min(detections, key=lambda d: abs(
                    d.center[0] - frame.shape[1]/2))

                # Update tag history
                if main_tag.tag_id not in tag_history:
                    tag_history[main_tag.tag_id] = []

                # Store detection with timestamp
                tag_history[main_tag.tag_id].append({
                    'time': current_time,
                    'center': main_tag.center,
                    'corners': main_tag.corners
                })

                # Keep only recent history
                tag_history[main_tag.tag_id] = [
                    d for d in tag_history[main_tag.tag_id]
                    if current_time - d['time'] < 1.0
                ]

                # Calculate direction and distance
                direction, distance_cm = get_direction(
                    main_tag, frame.shape[1])

                # Apply command smoothing
                last_commands.append(direction)
                if len(last_commands) > MAX_HISTORY:
                    last_commands.pop(0)

                # Use majority vote for stable commands
                if len(last_commands) == MAX_HISTORY:
                    from collections import Counter
                    direction = Counter(last_commands).most_common(1)[0][0]

                if VERBOSE:
                    print(
                        f"Tag {main_tag.tag_id} at {distance_cm:.1f}cm -> {direction}")

                # Send command to Arduino at specified intervals
                if arduino and current_time - last_command_time >= command_interval:
                    # Scale up the distance to be used as motor speed - needs to be at least 150-200 for motors to move
                    # Scale distance by 3x with minimum of 200
                    motor_speed = max(200, int(distance_cm * 3))

                    tag_data = TagData(
                        tag_id=main_tag.tag_id,
                        distance=motor_speed,  # Use scaled value for motor speed
                        direction=direction)

                    if VERBOSE:
                        print(
                            f"Sending command: Tag {main_tag.tag_id}, Speed {motor_speed}, Direction {direction}")

                    arduino.send_tag_data(tag_data)
                    last_command_time = current_time
                elif arduino is None and VERBOSE and current_time - last_command_time >= command_interval:
                    command_text = get_direction_name(direction)

                    print(
                        f"COMMAND: {command_text} | Tag {main_tag.tag_id} | Distance: {distance_cm:.1f}cm")
                    last_command_time = current_time

            else:
                # No tags detected
                last_commands = []  # Clear command history

                # Send stop command when no tags are visible
                if arduino and current_time - last_command_time >= command_interval:
                    arduino.send_stop()
                    last_command_time = current_time
                # Display stop command when no tags are detected and Arduino is not connected
                elif arduino is None and VERBOSE and current_time - last_command_time >= command_interval:
                    if frame_count % 10 == 0:  # Limit frequency of these messages
                        print(
                            f"COMMAND: {get_direction_name(DIR_STOP)} | Direction code: {DIR_STOP} | No tags detected")
                    last_command_time = current_time

            # Brief pause to reduce CPU load
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        # Clean shutdown
        shutdown_event.set()

        if camera:
            try:
                camera.stop()
                print("Camera stopped")
            except:
                pass

        if arduino:
            arduino.send_stop()
            arduino.disconnect()
            print("Serial connection closed")

        print("System shutdown complete")


if __name__ == "__main__":
    main()
