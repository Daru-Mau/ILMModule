#!/usr/bin/env python3
"""
AprilTag UART Controller

This module integrates AprilTag recognition with UART communication to control the robot.
It acts as a bridge between the AprilTag detection system and the Arduino's integrated movement controller.
"""

import cv2
import time
import os
import numpy as np
import sys
import signal
from threading import Event
import argparse
from typing import Optional, Tuple, List
import logging

# Import the UART communication module
from uart_communication import UARTCommunicator, TagData, DIR_FORWARD, DIR_BACKWARD, DIR_LEFT, DIR_RIGHT, DIR_STOP

# Check if running on Raspberry Pi
IS_RASPBERRY_PI = os.path.exists('/sys/firmware/devicetree/base/model')

if IS_RASPBERRY_PI:
    from picamera2 import Picamera2
    from pupil_apriltags import Detector

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

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
DEFAULT_SERIAL_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200

# Detection settings
VERBOSE = False
DETECTION_INTERVAL = 0.1

# Navigation parameters
CENTER_TOLERANCE = 0.15
DISTANCE_THRESHOLD = 30

# === Global state ===
shutdown_event = Event()


class AprilTagUARTController:
    """
    Controller that integrates AprilTag detection with UART communication
    to control the robot movement.
    """

    def __init__(self,
                 port: Optional[str] = None,
                 baud_rate: int = DEFAULT_BAUD_RATE,
                 max_speed: int = 150,
                 min_speed: int = 100,
                 verbose: bool = False):
        """Initialize the controller with communication parameters"""
        self.port = port
        self.baud_rate = baud_rate
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.verbose = verbose
        self.communicator = None
        self.camera = None
        self.detector = None
        self.running = False

        if verbose:
            logger.setLevel(logging.DEBUG)

    def setup(self) -> bool:
        """
        Initialize hardware and communication

        Returns:
            bool: True if setup successful, False otherwise
        """
        # Initialize UART communication
        logger.info("Setting up UART communication...")
        self.communicator = UARTCommunicator(
            port=self.port,
            baud_rate=self.baud_rate,
            debug=self.verbose
        )

        if not self.communicator.connected:
            logger.info("Auto-detection of Arduino port...")
            if not self.communicator.connect():
                logger.error(
                    "Failed to connect to Arduino. Check connections and try again.")
                return False

        # Set motor speed parameters
        logger.info(
            f"Setting motor speeds - MAX: {self.max_speed}, MIN: {self.min_speed}")
        self.communicator.set_speed(self.max_speed, self.min_speed)

        # Initialize camera if on Raspberry Pi
        if IS_RASPBERRY_PI:
            logger.info("Setting up camera...")
            self.camera = self.setup_camera()
            if not self.camera:
                logger.error("Failed to initialize camera.")
                return False

            # Initialize AprilTag detector
            logger.info("Setting up AprilTag detector...")
            self.detector = Detector(
                families="tag36h11",
                nthreads=4,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=True,
                decode_sharpening=0.5,
                debug=False
            )
        else:
            logger.warning(
                "Not running on Raspberry Pi - camera features unavailable")

        logger.info("Setup complete!")
        return True

    def setup_camera(self):
        """Initialize the Raspberry Pi camera with optimal settings for AprilTag detection"""
        if not IS_RASPBERRY_PI:
            return None

        try:
            # Initialize camera
            picam = Picamera2()

            # Configure camera with settings optimized for tag detection
            config = picam.create_preview_configuration(
                main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT),
                      "format": "BGR888"},
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

            logger.info(
                f"Camera initialized with resolution {CAMERA_WIDTH}x{CAMERA_HEIGHT}")

            return picam

        except Exception as e:
            logger.error(f"Camera setup failed: {e}")
            try:
                import subprocess
                subprocess.run(['sudo', 'pkill', '-f', 'libcamera'],
                               stderr=subprocess.DEVNULL)
            except:
                pass
            return None

    def preprocess_image(self, frame):
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

    def estimate_tag_size(self, corners):
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
            estimated_side_from_diag = (
                diagonal1 + diagonal2) / (2 * np.sqrt(2))
            return 0.7 * estimated_side + 0.3 * estimated_side_from_diag
        else:
            return sides_mean

    def calculate_distance(self, tag_size_px):
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

    def get_direction(self, detection, frame_width):
        """Determine movement direction based on tag position and size"""
        center_x = detection.center[0]
        tag_size_px = self.estimate_tag_size(detection.corners)
        distance_cm = self.calculate_distance(tag_size_px)

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

    def process_frame(self, frame):
        """Process a single frame to detect AprilTags and control robot"""
        if not self.detector:
            logger.error("AprilTag detector not initialized")
            return

        # Preprocess the image
        gray = self.preprocess_image(frame)

        # Detect AprilTags
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(CAM_FX, CAM_FY, CAM_CX, CAM_CY),
            tag_size=TAG_SIZE_CM
        )

        # If no detections, tell the robot to stop
        if not detections:
            if self.verbose:
                logger.info("No tags detected")
            self.communicator.send_stop()
            return

        # Find the best detection (largest tag, which is closest)
        best_detection = max(
            detections, key=lambda d: self.estimate_tag_size(d.corners))

        # Determine direction and distance
        direction, distance = self.get_direction(
            best_detection, frame.shape[1])

        # Create TagData object
        tag_data = TagData(
            tag_id=best_detection.tag_id,
            distance=distance,
            direction=direction
        )

        if self.verbose:
            logger.debug(f"Detected {str(tag_data)}")

        # Send the tag data to control the robot
        self.communicator.send_tag_data(tag_data)

    def run(self):
        """Run the main detection and control loop"""
        if not IS_RASPBERRY_PI:
            logger.error(
                "This module requires a Raspberry Pi to run in camera mode")
            return

        if not self.camera or not self.detector or not self.communicator:
            logger.error(
                "Controller not properly initialized. Run setup() first.")
            return

        self.running = True
        logger.info("Starting AprilTag detection and robot control loop...")

        try:
            last_time = time.time()

            while self.running and not shutdown_event.is_set():
                # Capture frame
                frame = self.camera.capture_array()

                # Process at fixed interval for stability
                current_time = time.time()
                if current_time - last_time >= DETECTION_INTERVAL:
                    self.process_frame(frame)
                    last_time = current_time

                # Check for keyboard interrupt
                if cv2.waitKey(1) == ord('q'):
                    break

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        except Exception as e:
            logger.error(f"Error in detection loop: {e}")
        finally:
            self.stop()

    def stop(self):
        """Stop the controller and clean up resources"""
        self.running = False

        logger.info("Shutting down...")

        # Stop the robot
        if self.communicator:
            logger.info("Sending stop command to robot")
            self.communicator.send_stop()

        # Release camera resources
        if IS_RASPBERRY_PI and self.camera:
            logger.info("Releasing camera")
            self.camera.close()

        logger.info("Shutdown complete")


def signal_handler(sig, frame):
    """Handle shutdown signals"""
    logger.info("Shutdown signal received")
    shutdown_event.set()


def main():
    """Main function to run the controller from command line"""
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='AprilTag Detection with UART Robot Control')
    parser.add_argument('--port', type=str, default=None,
                        help=f'Serial port (default: auto-detect)')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD_RATE,
                        help=f'Baud rate (default: {DEFAULT_BAUD_RATE})')
    parser.add_argument('--max-speed', type=int, default=150,
                        help='Maximum motor speed (50-255)')
    parser.add_argument('--min-speed', type=int, default=100,
                        help='Minimum motor speed (30-max_speed)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Enable verbose output')
    args = parser.parse_args()

    # Create and start the controller
    controller = AprilTagUARTController(
        port=args.port,
        baud_rate=args.baud,
        max_speed=args.max_speed,
        min_speed=args.min_speed,
        verbose=args.verbose
    )

    if controller.setup():
        controller.run()
    else:
        logger.error("Failed to set up controller. Exiting.")
        sys.exit(1)


if __name__ == "__main__":
    main()
