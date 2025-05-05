"""
AprilTag Recognition System
This script handles the detection of AprilTags using a camera and processes the data
for robot navigation.
"""

import cv2
import numpy as np
from pupil_apriltags import Detector
from apriltag_communication import ArduinoCommunicator
import math
import time
from flask import Flask, Response, render_template
import threading

app = Flask(__name__)

class AprilTagTracker:
    def __init__(self, camera_id=0, tag_size=0.08):
        """
        Initialize the AprilTag tracking system
        Args:
            camera_id (int): Camera device ID (default will be overridden by port detection)
            tag_size (float): Size of the AprilTag in meters
        """
        # Try different camera ports commonly used on Raspberry Pi
        camera_ports = [0, 2, -1]  # Common Raspberry Pi camera ports
        self.cap = None
        
        for port in camera_ports:
            print(f"Trying camera port: {port}")
            cap = cv2.VideoCapture(port)
            if cap.isOpened():
                self.cap = cap
                print(f"Successfully connected to camera on port: {port}")
                break
            cap.release()
        
        if self.cap is None:
            raise RuntimeError("Could not open any camera port. Please check camera connection.")

        self.detector = Detector(families='tag36h11')
        self.tag_size = tag_size
        self.camera_params = None
        self.communicator = ArduinoCommunicator()

        # Try to load camera calibration
        try:
            calib_data = np.load('camera_calibration.npz')
            self.camera_matrix = calib_data['camera_matrix']
            self.dist_coeffs = calib_data['dist_coeffs']
            self.camera_params = [self.camera_matrix[0, 0], self.camera_matrix[1, 1],
                                  self.camera_matrix[0, 2], self.camera_matrix[1, 2]]
        except:
            print("Warning: No camera calibration found. Using default parameters.")
            self.camera_matrix = np.array([[1000, 0, 320],
                                           [0, 1000, 240],
                                           [0, 0, 1]])
            self.dist_coeffs = np.zeros((4, 1))
            self.camera_params = [1000, 1000, 320, 240]

        self.frame_lock = threading.Lock()
        self.current_frame = None

    def process_frame(self):
        """Process a single frame from the camera"""
        ret, frame = self.cap.read()
        if not ret:
            return None

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        tags = self.detector.detect(gray, estimate_tag_pose=True,
                                    camera_params=self.camera_params,
                                    tag_size=self.tag_size)

        if len(tags) > 0:
            # Process the first detected tag
            tag = tags[0]

            # Extract position and orientation
            x = tag.pose_t[0] * 1000  # Convert to mm
            y = tag.pose_t[2] * 1000  # Z is forward in camera frame
            yaw = math.atan2(tag.pose_R[1, 0], tag.pose_R[0, 0])

            # Draw detection on frame
            cv2.polylines(frame, [np.int32(tag.corners)], True, (0, 255, 0), 2)
            cv2.putText(frame, f"ID: {tag.tag_id}", (int(tag.center[0]), int(tag.center[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Dist: {y:.0f}mm", (int(tag.center[0]), int(tag.center[1]) + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        with self.frame_lock:
            self.current_frame = frame
        return frame

    def get_current_frame(self):
        with self.frame_lock:
            return None if self.current_frame is None else self.current_frame.copy()

    def run(self):
        """Main processing loop"""
        while True:
            frame = self.process_frame()
            if frame is None:
                break
            time.sleep(0.01)  # Small delay to prevent CPU overload

    def cleanup(self):
        """Clean up resources"""
        self.cap.release()
        cv2.destroyAllWindows()

def generate_frames(tracker):
    while True:
        frame = tracker.get_current_frame()
        if frame is not None:
            ret, buffer = cv2.imencode('.jpg', frame)
            if ret:
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.01)

@app.route('/')
def index():
    return """
    <html>
    <head>
        <title>AprilTag Detection Stream</title>
    </head>
    <body>
        <h1>AprilTag Detection Stream</h1>
        <img src="/video_feed" width="640" height="480" />
    </body>
    </html>
    """

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(tracker),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    tracker = AprilTagTracker()
    # Start the tracking in a separate thread
    tracking_thread = threading.Thread(target=tracker.run)
    tracking_thread.daemon = True
    tracking_thread.start()
    
    # Run the Flask server
    app.run(host='0.0.0.0', port=5000)
