"""
AprilTag Recognition System for Robot Navigation
This script uses a Raspberry Pi camera to detect AprilTags and guide robot movement.
It's optimized for Raspberry Pi 3 B+ with performance considerations for real-time processing.
Can run in cloud environment with mock camera for demonstration.
"""

import cv2
import time
import os
import numpy as np
from threading import Thread
import queue
from flask import Flask, Response, render_template
import threading

# Check if running on Raspberry Pi or in cloud
IS_RASPBERRY_PI = os.path.exists('/sys/firmware/devicetree/base/model')

if IS_RASPBERRY_PI:
    from picamera2 import Picamera2
    from pupil_apriltags import Detector
    from apriltag_communication import ArduinoCommunicator

# === Camera Configuration ===
CAMERA_WIDTH = 1280  # Increased resolution for better far detection
CAMERA_HEIGHT = 720
FOCAL_LENGTH_PX = 780  # Adjusted focal length for better distance calculation
MIN_DETECTION_DISTANCE = 5  # Reduced minimum detection distance in cm
MAX_DETECTION_DISTANCE = 300  # Increased maximum detection distance in cm

# === AprilTag Configuration ===
TAG_REAL_SIZE_CM = 15  # Physical size of the AprilTag
TAG_PHYSICAL_SIZE = 0.15  # AprilTag size in meters
CALIBRATION_DISTANCE_CM = 100  # Calibration distance
CALIBRATION_MODE = False  # Set to True when calibrating

# === Robot Control Parameters ===
DISTANCE_THRESHOLD = 5  # Reduced to allow closer approach
CENTER_TOLERANCE_RATIO = 0.2  # Increased tolerance for easier centering

# === Communication Settings ===
SEND_TO_ARDUINO = False  # Enable/disable Arduino communication
SERIAL_PORT = '/dev/ttyACM0'  # Arduino serial port
BAUD_RATE = 9600  # Serial communication speed

# === Performance Optimization ===
FRAME_QUEUE_SIZE = 2  # Frame buffer size
USE_THREADING = True  # Enable parallel processing
SKIP_FRAMES = 2  # Process every nth frame
VERBOSE = True  # Enable detailed logging

# === Global State Variables ===
global_frame = None
frame_lock = threading.Lock()
arduino_comm = None
frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)

# Initialize Flask app
app = Flask(__name__)

# Flask routes


@app.route('/')
def index():
    return """
    <html>
        <head>
            <title>Camera Feed</title>
            <style>
                body { text-align: center; background: #333; color: white; }
                img { max-width: 100%; height: auto; }
            </style>
        </head>
        <body>
            <h1></h1>
            <img src="/video_feed">
        </body>
    </html>
    """


def generate_frames():
    while True:
        with frame_lock:
            if global_frame is not None:
                # Encode the frame as JPEG
                ret, buffer = cv2.imencode('.jpg', global_frame)
                if not ret:
                    continue
                # Convert to bytes and yield for streaming
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.1)  # Reduce CPU usage


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def estimate_tag_size(corners):
    """
    Calculate the apparent size of the detected tag in pixels
    Used for distance estimation and movement decisions
    """
    return ((corners[2][0] - corners[0][0])**2 +
            (corners[2][1] - corners[0][1])**2)**0.5


def get_direction(detection, frame_width, distance_threshold):
    """
    Determine robot movement direction based on tag position and distance
    Returns: 
        - 'S' (Stop) when at optimal distance and centered
        - 'L' (Left) when tag is left of center
        - 'R' (Right) when tag is right of center
        - 'F' (Forward) when tag is centered but too far
        - 'B' (Back) when tag is too close
    """
    center_x = detection.center[0]
    tag_size = estimate_tag_size(detection.corners)
    frame_center = frame_width // 2
    center_tolerance = frame_width * CENTER_TOLERANCE_RATIO

    # Calculate actual distance
    distance_cm = (FOCAL_LENGTH_PX * TAG_REAL_SIZE_CM) / tag_size

    # Check if we're too close (distance is less than threshold)
    if distance_cm < DISTANCE_THRESHOLD:
        return 'B', tag_size

    # Check horizontal positioning
    if center_x < frame_center - center_tolerance:
        return 'L', tag_size
    elif center_x > frame_center + center_tolerance:
        return 'R', tag_size

    # If we're centered, check if we're at the right distance
    if abs(distance_cm - DISTANCE_THRESHOLD) <= DISTANCE_THRESHOLD * 0.1:  # Within 10% of target distance
        return 'S', tag_size

    # If we're centered but too far, move forward
    return 'F', tag_size


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


class MockCamera:
    """Mock camera class for cloud deployment"""

    def __init__(self):
        self.frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
        # Add a test pattern or message
        cv2.putText(self.frame, "Cloud Demo Mode - No Camera", (50, CAMERA_HEIGHT//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    def capture_array(self):
        return self.frame.copy()

    def start(self):
        pass

    def stop(self):
        pass


def setup_camera():
    """
    Initializes the Pi Camera or returns a mock camera for cloud deployment
    """
    if IS_RASPBERRY_PI:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"},
            controls={
                # ~60fps for faster updates
                "FrameDurationLimits": (16666, 16666),
                "AnalogueGain": 4.0,                    # Balanced for both close and far detection
                "ExposureTime": 15000,                  # Faster exposure for clearer images
                "AwbEnable": 0,                         # Disabled for consistent colors
                "Contrast": 1.8,                        # Increased contrast for better edge detection
                "Sharpness": 2.5,                       # Increased sharpness for better tag detection
                "Brightness": 0.5,                      # Balanced brightness
                "NoiseReductionMode": 2                 # Enhanced noise reduction
            }
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(0.5)  # Reduced warm-up time
        return picam2
    else:
        return MockCamera()


def preprocess_image(frame):
    """
    Enhanced image preprocessing pipeline for better AprilTag detection:
    1. Convert to grayscale
    2. Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
    3. Apply mild denoising
    4. Enhance edges
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # Apply CLAHE for better contrast in varying lighting conditions
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)

    # Denoise while preserving edges
    gray = cv2.fastNlMeansDenoising(
        gray, None, h=10, templateWindowSize=7, searchWindowSize=21)

    # Enhance edges
    kernel = np.array([[-1, -1, -1],
                      [-1, 9, -1],
                      [-1, -1, -1]])
    gray = cv2.filter2D(gray, -1, kernel)

    return gray


def update_global_frame(frame):
    global global_frame
    with frame_lock:
        global_frame = frame.copy()


# Main processing function
def main_processing():
    global global_frame, arduino_comm

    if SEND_TO_ARDUINO and IS_RASPBERRY_PI:
        arduino_comm = ArduinoCommunicator(SERIAL_PORT, BAUD_RATE)
        arduino_comm.connect()

    picam2 = setup_camera()

    if IS_RASPBERRY_PI:
        detector = Detector(
            families="tag36h11",
            nthreads=4,            # Increased thread count for faster processing
            quad_decimate=1.0,     # No decimation for maximum accuracy
            quad_sigma=0.4,        # Reduced blur for better edge detection
            refine_edges=True,
            decode_sharpening=0.8,  # Increased sharpening for better detection
            debug=False
        )

    print("AprilTag tracking started")

    frame_count = 0
    detection_interval = 0.1

    if USE_THREADING:
        stream = StreamCapture(picam2)
        stream.start()

    try:
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
            if frame_count % SKIP_FRAMES != 0:
                continue

            if IS_RASPBERRY_PI:
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

                    # Send commands to Arduino through the communicator
                    if SEND_TO_ARDUINO and arduino_comm:
                        arduino_comm.process_tag_data(
                            main_tag.tag_id, distance_cm, direction)

                    if VERBOSE:
                        print(
                            f"Tag {main_tag.tag_id}: {direction} | Size: {tag_size_px:.1f}px | Est. Distance: {distance_cm:.1f} cm")

                    # Draw tag outline and ID
                    pts = main_tag.corners.astype(int)
                    for i in range(4):
                        cv2.line(frame, tuple(pts[i]), tuple(
                            pts[(i+1) % 4]), (0, 255, 0), 2)
                    cv2.putText(frame, f"ID:{main_tag.tag_id} D:{int(distance_cm)}cm", tuple(pts[0]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # Update the global frame for streaming
            update_global_frame(frame)
            time.sleep(detection_interval)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if USE_THREADING:
            stream.stop()
        picam2.stop()
        if arduino_comm:
            arduino_comm.close()
        if IS_RASPBERRY_PI and 'detector' in locals():
            del detector


if __name__ == '__main__':
    # Start the processing in a separate thread
    processing_thread = threading.Thread(target=main_processing)
    processing_thread.daemon = True
    processing_thread.start()

    # Run the Flask app
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
