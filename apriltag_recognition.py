"""
AprilTag Recognition System for Robot Navigation
Optimized for Raspberry Pi 4 with web-based visualization
"""

import cv2
import time
import os
import numpy as np
from threading import Thread, Lock
import queue
import serial
from flask import Flask, Response, render_template_string

# Check if running on Raspberry Pi
IS_RASPBERRY_PI = os.path.exists('/sys/firmware/devicetree/base/model')

if IS_RASPBERRY_PI:
    from picamera2 import Picamera2
    from pupil_apriltags import Detector

# === Camera Configuration ===
CAMERA_WIDTH = 224
CAMERA_HEIGHT = 224
FOCAL_LENGTH_PX = 365  # Initial calibration for 224x224
TAG_REAL_SIZE_CM = 15
DISTANCE_THRESHOLD = 45
CENTER_TOLERANCE_RATIO = 0.15

# === Arduino Communication ===
SEND_TO_ARDUINO = False
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

# === Performance Settings ===
FRAME_QUEUE_SIZE = 2
SKIP_FRAMES = 2
USE_THREADING = True
SHOW_FEED = True
VERBOSE = True

# === Global Variables ===
frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)
frame_lock = Lock()
last_direction = None
global_frame = None

# Initialize Flask app
app = Flask(__name__)

# HTML template for the web interface
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>AprilTag Detection</title>
    <style>
        body {
            margin: 0;
            background: #000;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
        }
        img {
            max-width: 100%;
            height: auto;
        }
    </style>
</head>
<body>
    <img src="{{ url_for('video_feed') }}">
</body>
</html>
"""

@app.route('/')
def index():
    """Serve the main page"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

def generate_frames():
    """Generate frames for the web stream"""
    while True:
        with frame_lock:
            if global_frame is not None:
                try:
                    ret, buffer = cv2.imencode('.jpg', global_frame)
                    if ret:
                        frame_data = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')
                except Exception as e:
                    print(f"Error generating frame: {e}")
        time.sleep(0.016)  # ~60fps max

def setup_serial():
    """Initialize serial connection to Arduino"""
    if not SEND_TO_ARDUINO:
        return None
    try:
        arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for Arduino reset
        print("Arduino connected successfully")
        return arduino
    except Exception as e:
        print(f"Failed to connect to Arduino: {e}")
        return None

def estimate_tag_size(corners):
    """Calculate tag size based on corner positions"""
    side_lengths = [np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]
    return np.mean(side_lengths)

def get_direction(detection, frame_width, distance_threshold):
    """Determine movement direction based on tag position and distance"""
    center_x = detection.center[0]
    tag_size = estimate_tag_size(detection.corners)
    frame_center = frame_width // 2
    center_tolerance = frame_width * CENTER_TOLERANCE_RATIO

    # Check distance first
    if tag_size >= distance_threshold:
        return 'S', tag_size

    # Check horizontal position
    if center_x < frame_center - center_tolerance:
        return 'L', tag_size
    elif center_x > frame_center + center_tolerance:
        return 'R', tag_size
    return 'F', tag_size

class StreamCapture:
    def __init__(self, picam):
        self.picam = picam
        self.thread = Thread(target=self._capture_frames, daemon=True)
        self.running = False

    def start(self):
        self.running = True
        self.thread.start()

    def _capture_frames(self):
        while self.running:
            if not frame_queue.full():
                frame = self.picam.capture_array()
                frame_queue.put(frame)
            time.sleep(0.001)

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()

def setup_camera():
    """Initialize Pi Camera with optimized settings"""
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"},
        controls={
            "FrameDurationLimits": (33333, 33333),  # ~30fps
            "AnalogueGain": 8.0,
            "ExposureTime": 20000,
            "AwbEnable": 0,
            "Contrast": 1.5,
            "Sharpness": 2.0
        }
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.5)
    return picam2

def preprocess_image(frame):
    """Optimize image for tag detection"""
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    gray = cv2.equalizeHist(gray)
    return gray

def update_frame_display(frame):
    """Update the display frame with thread safety"""
    global global_frame
    with frame_lock:
        global_frame = frame.copy()

def main():
    global last_direction, global_frame

    if not IS_RASPBERRY_PI:
        print("This script requires a Raspberry Pi to run")
        return

    # Initialize hardware
    arduino = setup_serial()
    picam2 = setup_camera()
    detector = Detector(
        families="tag36h11",
        nthreads=4,  # Optimized for Pi 4
        quad_decimate=1.0,
        quad_sigma=0.6,
        refine_edges=1,
        decode_sharpening=0.5,
        debug=False
    )

    # Start frame capture thread
    stream = StreamCapture(picam2)
    stream.start()
    
    # Start Flask server in a separate thread if SHOW_FEED is enabled
    if SHOW_FEED:
        flask_thread = Thread(target=lambda: app.run(host='0.0.0.0', port=5000, threaded=True), daemon=True)
        flask_thread.start()
    
    frame_count = 0
    detection_interval = 0.1

    try:
        while True:
            try:
                frame = frame_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            frame_count += 1
            if frame_count % SKIP_FRAMES != 0:
                continue

            # Process frame
            processed = preprocess_image(frame)
            detections = detector.detect(processed)

            if detections:
                # Get closest tag to center
                main_tag = min(detections, key=lambda d: abs(d.center[0] - frame.shape[1]/2))
                tag_size_px = estimate_tag_size(main_tag.corners)
                direction, _ = get_direction(main_tag, frame.shape[1], DISTANCE_THRESHOLD)

                # Calculate distance
                distance_cm = (FOCAL_LENGTH_PX * TAG_REAL_SIZE_CM) / tag_size_px
                detection_interval = max(0.05, 0.2 - tag_size_px / 1000)

                # Send command if direction changed
                if direction != last_direction:
                    if arduino:
                        arduino.write(direction.encode())
                        arduino.flush()
                    last_direction = direction

                if VERBOSE:
                    print(f"Tag {main_tag.tag_id}: {direction} | Distance: {distance_cm:.1f}cm")

                # Draw visual feedback
                if SHOW_FEED:
                    pts = main_tag.corners.astype(int)
                    cv2.polylines(frame, [pts.reshape((-1,1,2))], True, (0,255,0), 2)
                    cv2.putText(frame, f"ID:{main_tag.tag_id} D:{int(distance_cm)}cm",
                              tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

            else:
                detection_interval = min(0.5, detection_interval + 0.05)
                if arduino:
                    arduino.write(b'S')
                if VERBOSE:
                    print("Searching...")

            # Update web display if enabled
            if SHOW_FEED:
                update_frame_display(frame)

            time.sleep(detection_interval)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        stream.stop()
        picam2.stop()
        if arduino:
            arduino.close()

if __name__ == '__main__':
    main()