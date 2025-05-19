#!/usr/bin/env python3
"""
Test AprilTag UART Controller

A simple test script for verifying UART communication with the Arduino
without requiring a camera or actual AprilTag detection.
"""

import time
import sys
import signal
import argparse
import logging
from threading import Event
from typing import Optional

from uart_communication import UARTCommunicator, TagData, DIR_STOP, DIR_FORWARD, DIR_BACKWARD, DIR_LEFT, DIR_RIGHT
from uart_communication import get_direction_name

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# Default settings
DEFAULT_BAUD_RATE = 115200

# Global state
shutdown_event = Event()


class UARTTestController:
    """
    Test controller for UART communication with the Arduino
    Allows manual commands and simulated tag detection
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

        if verbose:
            logger.setLevel(logging.DEBUG)

    def setup(self) -> bool:
        """
        Initialize communication

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

        logger.info(
            f"Successfully connected to Arduino on {self.communicator.port}")
        return True

    def run_interactive(self):
        """Run interactive test mode"""
        if not self.communicator:
            logger.error(
                "Controller not properly initialized. Run setup() first.")
            return

        logger.info("\n=== Interactive UART Test Mode ===")
        logger.info("Available commands:")
        logger.info("  tag <id> <distance> <direction>  - Send AprilTag data")
        logger.info(
            "  move <direction> [speed]         - Move robot (0=stop,1=forward,2=backward,3=left,4=right)")
        logger.info(
            "  motors <left> <right> <back>     - Direct motor control (-255 to 255)")
        logger.info("  sensor                           - Request sensor data")
        logger.info("  stop                             - Stop the robot")
        logger.info("  ping                             - Test connection")
        logger.info("  test                             - Run diagnostics")
        logger.info("  speed <max> <min>                - Set motor speeds")
        logger.info("  quit                             - Exit the program")
        logger.info("=========================================")

        try:
            while not shutdown_event.is_set():
                cmd = input("\nEnter command > ").strip()

                if not cmd:
                    continue

                tokens = cmd.split()
                cmd_type = tokens[0].lower()

                if cmd_type == "quit" or cmd_type == "exit":
                    break

                elif cmd_type == "tag":
                    if len(tokens) >= 4:
                        try:
                            tag_id = int(tokens[1])
                            distance = float(tokens[2])
                            direction = int(tokens[3])

                            tag_data = TagData(tag_id, distance, direction)
                            logger.info(f"Sending {str(tag_data)}")

                            result = self.communicator.send_tag_data(tag_data)
                            logger.info(
                                f"Result: {'Success' if result else 'Failed'}")
                        except ValueError:
                            logger.error(
                                "Invalid parameters. Format: tag <id> <distance> <direction>")
                    else:
                        logger.error(
                            "Not enough parameters. Format: tag <id> <distance> <direction>")

                elif cmd_type == "move":
                    if len(tokens) >= 2:
                        try:
                            direction = int(tokens[1])
                            speed = int(tokens[2]) if len(
                                tokens) >= 3 else None

                            dir_name = get_direction_name(direction)
                            speed_info = f" at speed {speed}" if speed is not None else ""
                            logger.info(f"Moving {dir_name}{speed_info}")

                            result = self.communicator.send_movement(
                                direction, speed)
                            logger.info(
                                f"Result: {'Success' if result else 'Failed'}")
                        except ValueError:
                            logger.error(
                                "Invalid parameters. Format: move <direction> [speed]")
                    else:
                        logger.error(
                            "Not enough parameters. Format: move <direction> [speed]")

                elif cmd_type == "motors":
                    if len(tokens) >= 4:
                        try:
                            left = int(tokens[1])
                            right = int(tokens[2])
                            back = int(tokens[3])

                            logger.info(
                                f"Setting motors - Left: {left}, Right: {right}, Back: {back}")

                            result = self.communicator.set_motor_speeds(
                                left, right, back)
                            logger.info(
                                f"Result: {'Success' if result else 'Failed'}")
                        except ValueError:
                            logger.error(
                                "Invalid parameters. Format: motors <left> <right> <back>")
                    else:
                        logger.error(
                            "Not enough parameters. Format: motors <left> <right> <back>")

                elif cmd_type == "sensor":
                    logger.info("Requesting sensor data...")
                    sensor_data = self.communicator.request_sensor_data()

                    if sensor_data:
                        logger.info(f"Sensor data: {str(sensor_data)}")
                    else:
                        logger.error("Failed to get sensor data")

                elif cmd_type == "stop":
                    logger.info("Stopping robot...")
                    result = self.communicator.send_stop()
                    logger.info(f"Result: {'Success' if result else 'Failed'}")

                elif cmd_type == "ping":
                    logger.info("Pinging Arduino...")
                    result = self.communicator.ping()
                    logger.info(f"Result: {'Success' if result else 'Failed'}")

                elif cmd_type == "test":
                    logger.info("Running diagnostics...")
                    result = self.communicator.run_diagnostics()
                    logger.info(f"Result: {'Success' if result else 'Failed'}")

                elif cmd_type == "speed":
                    if len(tokens) >= 3:
                        try:
                            max_speed = int(tokens[1])
                            min_speed = int(tokens[2])

                            logger.info(
                                f"Setting motor speeds - MAX: {max_speed}, MIN: {min_speed}")

                            result = self.communicator.set_speed(
                                max_speed, min_speed)
                            logger.info(
                                f"Result: {'Success' if result else 'Failed'}")
                        except ValueError:
                            logger.error(
                                "Invalid parameters. Format: speed <max> <min>")
                    else:
                        logger.error(
                            "Not enough parameters. Format: speed <max> <min>")

                else:
                    logger.error(f"Unknown command: {cmd_type}")

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self.stop()

    def run_demo_sequence(self):
        """Run a demonstration sequence of movements"""
        if not self.communicator:
            logger.error(
                "Controller not properly initialized. Run setup() first.")
            return

        logger.info("\n=== Running Demo Sequence ===")

        try:
            # Test connection
            logger.info("Testing connection...")
            if not self.communicator.ping():
                logger.error("Connection test failed, aborting demo")
                return

            # Get sensor data
            logger.info("Requesting sensor data...")
            sensor_data = self.communicator.request_sensor_data()
            if sensor_data:
                logger.info(f"Sensor data: {str(sensor_data)}")
            else:
                logger.warning("Failed to get sensor data, continuing anyway")

            # Simulate tag detection sequence
            logger.info("\nSimulating AprilTag detection sequence...")

            # Simulate tag at distance 100cm, centered
            logger.info("Simulating tag detection - Forward")
            tag_data = TagData(tag_id=1, distance=100.0, direction=DIR_FORWARD)
            self.communicator.send_tag_data(tag_data)
            time.sleep(2)

            # Simulate tag at distance 100cm, to the right
            logger.info("Simulating tag detection - Turn Right")
            tag_data = TagData(tag_id=1, distance=100.0, direction=DIR_RIGHT)
            self.communicator.send_tag_data(tag_data)
            time.sleep(2)

            # Simulate tag at distance 100cm, to the left
            logger.info("Simulating tag detection - Turn Left")
            tag_data = TagData(tag_id=1, distance=100.0, direction=DIR_LEFT)
            self.communicator.send_tag_data(tag_data)
            time.sleep(2)

            # Simulate tag at distance 20cm, centered (should stop)
            logger.info("Simulating tag detection - Stop (Close Tag)")
            tag_data = TagData(tag_id=1, distance=20.0, direction=DIR_STOP)
            self.communicator.send_tag_data(tag_data)
            time.sleep(2)

            # Stop the robot
            logger.info("Demo complete, stopping robot")
            self.communicator.send_stop()

        except Exception as e:
            logger.error(f"Error in demo sequence: {e}")
        finally:
            self.stop()

    def stop(self):
        """Stop the controller and clean up resources"""
        logger.info("Shutting down...")

        # Stop the robot
        if self.communicator:
            logger.info("Sending stop command to robot")
            self.communicator.send_stop()

        logger.info("Shutdown complete")


def signal_handler(sig, frame):
    """Handle shutdown signals"""
    logger.info("Shutdown signal received")
    shutdown_event.set()


def main():
    """Main function to run the test controller from command line"""
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='UART Communication Test Controller')
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port (default: auto-detect)')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD_RATE,
                        help=f'Baud rate (default: {DEFAULT_BAUD_RATE})')
    parser.add_argument('--max-speed', type=int, default=150,
                        help='Maximum motor speed (50-255)')
    parser.add_argument('--min-speed', type=int, default=100,
                        help='Minimum motor speed (30-max_speed)')
    parser.add_argument('--demo', action='store_true',
                        help='Run demo sequence instead of interactive mode')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Enable verbose output')
    args = parser.parse_args()

    # Create and start the controller
    controller = UARTTestController(
        port=args.port,
        baud_rate=args.baud,
        max_speed=args.max_speed,
        min_speed=args.min_speed,
        verbose=args.verbose
    )

    if controller.setup():
        if args.demo:
            controller.run_demo_sequence()
        else:
            controller.run_interactive()
    else:
        logger.error("Failed to set up controller. Exiting.")
        sys.exit(1)


if __name__ == "__main__":
    main()
