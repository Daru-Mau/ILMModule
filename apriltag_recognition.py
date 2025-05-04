"""
AprilTag Recognition System for Robot Navigation - Optimized Version
"""

import cv2
import time
import os
import numpy as np
from threading import Thread
import queue
from flask import Flask, Response
import threading

# Check if running on Raspberry Pi
IS_RASPBERRY_PI = os.path.exists('/sys/firmware/devicetree/base/model')

if IS_RASPBERRY_PI:
    from picamera2 import Picamera2
    from pupil_apriltags import Detector
    from apriltag_communication import ArduinoCommunicator

# === Optimized Configuration ===
CAMERA_WIDTH = 640  # Reduced resolution for better performance
CAMERA_HEIGHT = 480
FOCAL_LENGTH_PX = 390  # Adjusted for new resolution
DISTANCE_THRESHOLD = 5
CENTER_TOLERANCE_RATIO = 0.2
TAG_REAL_SIZE_CM = 15

# === Performance Settings ===
FRAME_QUEUE_SIZE = 1  # Reduced queue size
USE_THREADING = True
SKIP_FRAMES = 1  # Process every frame
SEND_TO_ARDUINO = False

# Global variables
global_frame = None
frame_lock = threading.Lock()
arduino_comm = None
frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)

# Initialize Flask app
app = Flask(__name__)


@app.route('/')
def index():
    return """
    <html><head><title>Camera Feed</title></head>
    <body style='margin:0;background:#000;'>
        <img src='/video_feed' style='width:100%;max-width:960px;display:block;margin:0 auto;'>
    </body></html>
    """


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def generate_frames():
    while True:
        with frame_lock:
            if global_frame is not None:
                ret, buffer = cv2.imencode('.jpg', global_frame, [
                                           cv2.IMWRITE_JPEG_QUALITY, 70])
                if not ret:
                    continue
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.016)  # ~60fps max


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
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                frame_queue.put(frame)
            time.sleep(0.016)  # Limit capture rate

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
            "AnalogueGain": 2.0,
            "ExposureTime": 20000,
            "AwbEnable": 0,
            "Brightness": 0.5,
        }
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.1)
    return picam2


def get_direction(center_x, frame_width):
    """Simplified direction detection"""
    frame_center = frame_width // 2
    center_tolerance = frame_width * CENTER_TOLERANCE_RATIO

    if center_x < frame_center - center_tolerance:
        return 'L'
    elif center_x > frame_center + center_tolerance:
        return 'R'
    return 'F'


def main_processing():
    global global_frame, arduino_comm

    if SEND_TO_ARDUINO and IS_RASPBERRY_PI:
        arduino_comm = ArduinoCommunicator('/dev/ttyACM0', 9600)
        arduino_comm.connect()

    picam2 = setup_camera()

    if IS_RASPBERRY_PI:
        detector = Detector(
            families="tag36h11",
            nthreads=4,
            quad_decimate=2.0,  # Increase decimation for better performance
            quad_sigma=0.0,     # Disable blurring
            refine_edges=True,
            decode_sharpening=0.25,
            debug=False
        )

    stream = StreamCapture(picam2)
    stream.start()

    try:
        while True:
            try:
                frame = frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if IS_RASPBERRY_PI:
                # Convert to grayscale for faster processing
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                detections = detector.detect(gray)

                if detections:
                    main_tag = detections[0]  # Use first detected tag
                    direction = get_direction(
                        main_tag.center[0], frame.shape[1])

                    # Simple visualization
                    pts = main_tag.corners.astype(int)
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

                    if SEND_TO_ARDUINO and arduino_comm:
                        arduino_comm.process_tag_data(
                            main_tag.tag_id, 0, direction)

            update_global_frame(frame)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        stream.stop()
        picam2.stop()
        if arduino_comm:
            arduino_comm.close()


def update_global_frame(frame):
    global global_frame
    with frame_lock:
        global_frame = frame.copy()


if __name__ == '__main__':
    processing_thread = threading.Thread(target=main_processing)
    processing_thread.daemon = True
    processing_thread.start()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
