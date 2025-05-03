"""
AprilTag Recognition System for Robot Navigation
This script uses a Raspberry Pi camera to detect AprilTags and guide robot movement.
It's optimized for Raspberry Pi 3 B+ with performance considerations for real-time processing.
"""

import cv2
import time
from picamera2 import Picamera2
from pupil_apriltags import Detector
import serial
import numpy as np
from threading import Thread
import queue

# Camera settings for optimal tag detection while maintaining performance
CAMERA_WIDTH = 224
CAMERA_HEIGHT = 224
# Camera's focal length (in pixels) calibrated for our 224x224 resolution
FOCAL_LENGTH_PX = 365

# Camera calibration settings
# Set CALIBRATION_MODE to True when you need to recalibrate the camera
CALIBRATION_MODE = False
TAG_REAL_SIZE_CM = 15  # Physical size of the AprilTag (15cm x 15cm)
# Place the tag exactly 100cm from camera when calibrating
CALIBRATION_DISTANCE_CM = 100

# Robot control settings
SEND_TO_ARDUINO = False  # Set to True when connected to the Arduino robot
# Robot stops when tag appears larger than this (closer)
DISTANCE_THRESHOLD = 45
CENTER_TOLERANCE_RATIO = 0.15  # How far from center the tag can be before turning
SERIAL_PORT = '/dev/ttyACM0'  # Arduino USB port (typically ACM0 on Linux)
BAUD_RATE = 9600
VERBOSE = True  # Set to True for detailed console output

# Display settings
SHOW_FEED = True  # Shows camera feed with AR overlay. Disable on headless setups

# Physical setup parameters
TAG_PHYSICAL_SIZE = 0.15  # AprilTag size in meters
last_direction = None

# Performance optimization settings
# These settings are tuned for Raspberry Pi 3 B+ to balance speed and reliability
FRAME_QUEUE_SIZE = 2  # Number of frames to buffer
frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)
USE_THREADING = True  # Enables parallel processing of frames
SKIP_FRAMES = 2  # Process every second frame to reduce CPU load


def estimate_tag_size(corners):
    """
    Calculate the apparent size of the detected tag in pixels
    Used for distance estimation and movement decisions
    """
    return ((corners[2][0] - corners[0][0])**2 +
            (corners[2][1] - corners[0][1])**2)**0.5


def get_direction(detection, frame_width, distance_threshold):
    """
    Determine robot movement direction based on tag position
    Returns: 
        - 'S' (Stop) when too close
        - 'L' (Left) when tag is left of center
        - 'R' (Right) when tag is right of center
        - 'F' (Forward) when tag is centered
    """
    center_x = detection.center[0]
    tag_size = estimate_tag_size(detection.corners)
    frame_center = frame_width // 2
    center_tolerance = frame_width * CENTER_TOLERANCE_RATIO

    if tag_size >= distance_threshold:
        return 'S', tag_size
    if center_x < frame_center - center_tolerance:
        return 'L', tag_size
    elif center_x > frame_center + center_tolerance:
        return 'R', tag_size
    return 'F', tag_size


def setup_serial():
    """
    Establish connection to Arduino via serial port
    Returns the serial object if successful, None otherwise
    """
    try:
        arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print("Arduino connected")
        return arduino
    except Exception as e:
        print(f"Serial error: {e}")
        return None


class StreamCapture:
    """
    Handles threaded camera capture to prevent frame processing from blocking captures
    Maintains a small queue of recent frames to ensure smooth processing
    """

    def __init__(self, picam):
        self.picam = picam
        self.thread = Thread(target=self._capture_frames, daemon=True)
        self.running = False

    def start(self):
        """
        Start the frame capture thread
        """
        self.running = True
        self.thread.start()

    def _capture_frames(self):
        """
        Continuously capture frames and add them to the queue
        """
        while self.running:
            if not frame_queue.full():
                frame = self.picam.capture_array()
                # Rotate to match orientation of the camera support
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                frame_queue.put(frame)
            else:
                time.sleep(0.001)  # Prevent CPU overload

    def stop(self):
        """
        Stop the frame capture thread
        """
        self.running = False
        if self.thread.is_alive():
            self.thread.join()


def setup_camera():
    """
    Initializes the Pi Camera with optimized settings for AprilTag detection:
    - Increased contrast and sharpness for better tag edges
    - Fixed exposure for consistent detection
    - Disabled auto white balance for stability
    """
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"},
        controls={
            "FrameDurationLimits": (33333, 33333),  # ~30fps
            "AnalogueGain": 8.0,  # Increased for better low-light performance
            "ExposureTime": 20000,  # Reduced for faster frame rate
            "AwbEnable": 0,
            "Contrast": 1.5,
            "Sharpness": 2.0  # Increased for better tag detection
        }
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)  # Reduced warm-up time
    return picam2


def preprocess_image(frame):
    """
    Prepare camera frame for AprilTag detection:
    1. Convert to grayscale for faster processing
    2. Equalize histogram to improve contrast in varying lighting
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    gray = cv2.equalizeHist(gray)  # Improve contrast
    return gray


# Initialize hardware connections
arduino = setup_serial() if SEND_TO_ARDUINO else None
picam2 = setup_camera()

# Configure AprilTag detector with optimized parameters for our setup
detector = Detector(
    families="tag36h11",  # Standard tag family, good balance of reliability and uniqueness
    nthreads=2,  # Use 2 threads on Pi 3 B+
    quad_decimate=1.0,  # Full resolution for accuracy
    quad_sigma=0.6,  # Gentle smoothing for noise reduction
    refine_edges=1,  # Improve edge detection for better accuracy
    decode_sharpening=0.5,  # Moderate sharpening to help detect tags
    debug=False
)

print("AprilTag tracking started")

# Main processing loop
try:
    frame_count = 0
    detection_interval = 0.1

    if USE_THREADING:
        stream = StreamCapture(picam2)
        stream.start()

    while True:
        if USE_THREADING:
            try:
                frame = frame_queue.get(timeout=1.0)
            except queue.Empty:
                continue
        else:
            frame = picam2.capture_array()
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

        frame_count += 1
        if frame_count % SKIP_FRAMES != 0:  # Skip frames for better performance
            continue

        processed = preprocess_image(frame)
        detections = detector.detect(processed)

        if detections:
            main_tag = min(detections, key=lambda d: abs(
                d.center[0] - frame.shape[1]/2))
            tag_size_px = estimate_tag_size(main_tag.corners)
            direction, _ = get_direction(
                main_tag, frame.shape[1], DISTANCE_THRESHOLD)

            if CALIBRATION_MODE:
                focal_length = (
                    tag_size_px * CALIBRATION_DISTANCE_CM) / TAG_REAL_SIZE_CM
                print(
                    f"[CALIBRATION] Estimated focal length: {focal_length:.1f} px")
                distance_cm = CALIBRATION_DISTANCE_CM
            else:
                distance_cm = (FOCAL_LENGTH_PX *
                               TAG_REAL_SIZE_CM) / tag_size_px

            detection_interval = max(0.05, 0.2 - tag_size_px / 1000)

            if direction != last_direction:
                if SEND_TO_ARDUINO and arduino:
                    arduino.write(direction.encode())
                    arduino.flush()
                last_direction = direction

            if VERBOSE:
                print(
                    f"Tag {main_tag.tag_id}: {direction} | Size: {tag_size_px:.1f}px | Est. Distance: {distance_cm:.1f} cm")

            # === Draw tag outline and ID ===
            pts = main_tag.corners.astype(int)
            for i in range(4):
                cv2.line(frame, tuple(pts[i]), tuple(
                    pts[(i+1) % 4]), (0, 255, 0), 2)
            cv2.putText(frame, f"ID:{main_tag.tag_id} D:{int(distance_cm)}cm", tuple(pts[0]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        else:
            detection_interval = min(0.5, detection_interval + 0.05)
            if SEND_TO_ARDUINO and arduino:
                arduino.write(b'S')
            if VERBOSE:
                print("Searching...")

        # === Show the camera view ===
        if SHOW_FEED:
            cv2.imshow("AprilTag View", frame)
            if cv2.waitKey(1) == 27:  # ESC key
                break

        time.sleep(detection_interval)
except KeyboardInterrupt:
    print("\nShutting down...")

finally:
    if USE_THREADING:
        stream.stop()
    picam2.stop()
    if arduino:
        arduino.close()
    if 'detector' in locals():
        del detector
    if SHOW_FEED:
        cv2.destroyAllWindows()
