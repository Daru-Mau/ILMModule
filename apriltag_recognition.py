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


class AprilTagTracker:
    def __init__(self, camera_id=0, tag_size=0.08):
        """
        Initialize the AprilTag tracking system
        Args:
            camera_id (int): Camera device ID
            tag_size (float): Size of the AprilTag in meters
        """
        self.cap = cv2.VideoCapture(camera_id)
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

            # Send data to Arduino
            self.communicator.send_tag_data(tag.tag_id, x, y, yaw)

            # Draw detection on frame
            cv2.polylines(frame, [np.int32(tag.corners)], True, (0, 255, 0), 2)
            cv2.putText(frame, f"ID: {tag.tag_id}", (int(tag.center[0]), int(tag.center[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            # No tag detected
            self.communicator.clear_tag_data()

        return frame

    def run(self):
        """Main processing loop"""
        try:
            while True:
                frame = self.process_frame()
                if frame is None:
                    break

                # Show frame
                cv2.imshow('AprilTag Detection', frame)

                # Check for Arduino responses
                response = self.communicator.read_response()
                if response:
                    print(f"Arduino response: {response}")

                # Exit on 'q' press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        self.cap.release()
        cv2.destroyAllWindows()
        self.communicator.close()


if __name__ == "__main__":
    tracker = AprilTagTracker()
    tracker.run()
