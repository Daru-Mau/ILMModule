#!/usr/bin/env python3
"""
AprilTag Camera Simulation for Robot Control

This script provides an intuitive interface for controlling the robot as if
you were the AprilTag detection system. You specify where you see the tag in the
camera frame, and the robot responds accordingly, mimicking the real AprilTag tracking
system without needing an actual camera.

Features:
- Intuitive visual position input (left, center, right, etc.)
- Distance estimation simulation (close, medium, far)
- Visual feedback of the robot's "view" 
- Support for both 2-wheel and 3-wheel configurations
- Proper speed adjustments based on tag position
- Direct UART communication with proper command framing

Usage:
- Camera simulation mode: python integrated_movement_controller.py
- With options: python integrated_movement_controller.py --port /dev/ttyACM0 --max-speed 150 --min-speed 60 --three-wheels
"""

import time
import argparse
import sys
import signal
import logging
from typing import Optional, Tuple, List

# Import the UART communication module
from uart_communication import (
    UARTCommunicator, TagData, DIR_FORWARD, DIR_BACKWARD, 
    DIR_LEFT, DIR_RIGHT, DIR_STOP, DIR_ROTATE_LEFT, DIR_ROTATE_RIGHT,
    get_direction_name
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger(__name__)

# Default settings
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200
DEFAULT_MAX_SPEED = 100
DEFAULT_MIN_SPEED = 40
DEFAULT_TEST_DURATION = 2.0  # seconds

class IntegratedMovementController:
    """
    Controller for the integrated_movement.ino Arduino sketch
    
    This class provides a high-level interface to control the robot
    using the UART protocol with proper message framing.
    """
    
    def __init__(
        self,
        port: str = None,
        baud_rate: int = DEFAULT_BAUD_RATE,
        max_speed: int = DEFAULT_MAX_SPEED,
        min_speed: int = DEFAULT_MIN_SPEED,
        use_three_wheels: bool = False,
        verbose: bool = False
    ):
        """Initialize the controller with connection parameters"""
        self.port = port
        self.baud_rate = baud_rate
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.use_three_wheels = use_three_wheels
        self.verbose = verbose
        
        # Initialize UART communicator
        self.communicator = UARTCommunicator(
            port=port, 
            baud_rate=baud_rate,
            debug=verbose
        )
        
        # Set log level based on verbose flag
        if verbose:
            logger.setLevel(logging.DEBUG)
        
        # Track current state
        self.current_direction = DIR_STOP
        self.current_speed = 0
        
        # Flag to track shutdown
        self.running = True
    
    def connect(self) -> bool:
        """Establish connection to the Arduino"""
        if not self.communicator.connected:
            logger.info("Connecting to Arduino...")
            if not self.communicator.connect():
                logger.error("Failed to connect to Arduino. Check connections.")
                return False
        
        # Configure speed parameters
        logger.info(f"Setting speed parameters: max={self.max_speed}, min={self.min_speed}")
        self.communicator.set_speed(self.max_speed, self.min_speed)
        
        # Configure wheel mode
        wheel_mode = "THREE_WHEEL" if self.use_three_wheels else "TWO_WHEEL"
        logger.info(f"Setting wheel configuration: {wheel_mode}")
        self.communicator.set_wheel_mode(self.use_three_wheels)
        
        # Verify connection with ping
        if self.communicator.ping():
            logger.info("Connected to Arduino and communication verified!")
        else:
            logger.warning("Connected to Arduino but ping failed. Continuing anyway.")
        
        return True
    
    def disconnect(self) -> None:
        """Disconnect from the Arduino"""
        if self.communicator:
            logger.info("Stopping all motors before disconnecting")
            self.stop()
            logger.info("Disconnecting from Arduino")
            self.communicator.disconnect()
    
    def stop(self) -> bool:
        """Stop all motors"""
        result = self.communicator.send_stop()
        self.current_direction = DIR_STOP
        self.current_speed = 0
        return result
    
    def move(self, direction: int, speed: int, duration: Optional[float] = None) -> bool:
        """
        Move the robot in the specified direction
        
        Parameters:
        - direction: Direction code (0-6)
        - speed: Speed value (0-255)
        - duration: How long to move in seconds (optional)
        
        Returns:
        - True if command was successful, False otherwise
        """
        # Update current state
        self.current_direction = direction
        self.current_speed = speed
        
        # Log the movement
        logger.info(f"Moving {get_direction_name(direction)} at speed {speed}")
        
        # Send the movement command
        result = self.communicator.send_movement(direction, speed)
        
        # If duration specified, move for that time then stop
        if duration and result:
            time.sleep(duration)
            return self.stop()
        
        return result
    
    def read_sensor_distances(self) -> Tuple[float, float, float, float, float, float]:
        """
        Read distances from all ultrasonic sensors
        
        Returns tuple of distances (front-left, front, front-right, back-left, back, back-right)
        """
        # TODO: Implement sensor reading via UART
        # This requires adding the SENS command handling to UARTCommunicator
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # Placeholder
    
    def run_test_sequence(self) -> None:
        """Run a basic test sequence to verify movement in all directions"""
        if not self.connect():
            return
        
        try:
            logger.info("Starting test sequence...")
            
            # Forward and backward
            logger.info("Testing forward movement")
            self.move(DIR_FORWARD, self.max_speed, 2.0)
            time.sleep(1.0)
            
            logger.info("Testing backward movement")
            self.move(DIR_BACKWARD, self.max_speed, 2.0)
            time.sleep(1.0)
            
            # Left and right
            logger.info("Testing left movement")
            self.move(DIR_LEFT, self.max_speed, 2.0)
            time.sleep(1.0)
            
            logger.info("Testing right movement")
            self.move(DIR_RIGHT, self.max_speed, 2.0)
            time.sleep(1.0)
            
            # Rotation
            logger.info("Testing rotation left")
            self.move(DIR_ROTATE_LEFT, self.max_speed, 2.0)
            time.sleep(1.0)
            
            logger.info("Testing rotation right")
            self.move(DIR_ROTATE_RIGHT, self.max_speed, 2.0)
            time.sleep(1.0)
            
            logger.info("Test sequence complete!")
        
        finally:
            # Ensure we stop and disconnect properly
            self.stop()
            self.disconnect()
    
    def run_interactive_mode(self) -> None:
        """Run in camera simulation mode - you specify where the 'tag' is in the camera's view"""
        if not self.connect():
            return
        
        try:
            print("\n" + "="*50)
            print("     CAMERA SIMULATION - APRILTAG TRACKING     ")
            print("="*50)
            print("\nImagine you're looking through the camera lens:")
            print("┌───────────────────────────┐")
            print("│                           │")
            print("│     LEFT    CENTER RIGHT  │")
            print("│      ◄        ●      ►    │")
            print("│                           │")
            print("└───────────────────────────┘")
            print("\nCOMMANDS:")
            print("  [l] - AprilTag is on the LEFT side of frame")
            print("  [c] - AprilTag is in the CENTER of frame")
            print("  [r] - AprilTag is on the RIGHT side of frame")
            print("  [f] - AprilTag appears FAR away")
            print("  [m] - AprilTag is at MEDIUM distance")
            print("  [n] - AprilTag is NEAR (close)")
            print("  [s] - STOP all motors")
            print("  [+] - Increase maximum speed")
            print("  [-] - Decrease maximum speed")
            print("  [q] - Quit the simulation")
            
            # Initial state
            tag_position = 'c'  # center
            tag_distance = 'm'  # medium distance
            
            # Main simulation loop
            while self.running:
                # Display current state with visual representation
                self._display_camera_view(tag_position, tag_distance)
                
                # Get user input
                cmd = input("\nCommand [l/c/r/f/m/n/s/+/-/q]: ").lower()
                
                if cmd == 'q':
                    break
                elif cmd == 's':
                    self.stop()
                    continue
                elif cmd == '+':
                    self.max_speed = min(255, self.max_speed + 10)
                    print(f"Maximum speed increased to {self.max_speed}")
                    continue
                elif cmd == '-':
                    self.max_speed = max(self.min_speed + 10, self.max_speed - 10)
                    print(f"Maximum speed decreased to {self.max_speed}")
                    continue
                
                # Update state based on input
                if cmd in ['l', 'c', 'r']:
                    tag_position = cmd
                elif cmd in ['f', 'm', 'n']:
                    tag_distance = cmd
                else:
                    print("Invalid input. Use l/c/r for position, f/m/n for distance.")
                    continue
                
                # Calculate simulated tag metrics
                direction, speed = self._get_simulated_direction_and_speed(tag_position, tag_distance)
                
                # Send the movement command
                tag_data = TagData(
                    tag_id=1,  # Simulated tag ID
                    direction=direction,
                    speed=speed
                )
                
                print(f"Sending command: {get_direction_name(direction)} at speed {speed}")
                self.communicator.send_tag_data(tag_data)
        
        except KeyboardInterrupt:
            logger.info("Simulation interrupted by user")
        except Exception as e:
            logger.error(f"Error in camera simulation mode: {e}")
        finally:
            # Ensure we stop and disconnect properly
            self.stop()
            self.disconnect()
            
    def _display_camera_view(self, position, distance):
        """Display a visual representation of the camera view with the tag position"""
        # Distance representation
        if distance == 'f':
            size = "□"  # Small square for far
            dist_text = "FAR"
        elif distance == 'm':
            size = "■"  # Medium square for medium
            dist_text = "MEDIUM"
        else:  # 'n' for near
            size = "█"  # Large square for near
            dist_text = "NEAR"
            
        # Create the view with the tag at the correct position
        left = size if position == 'l' else " "
        center = size if position == 'c' else " "
        right = size if position == 'r' else " "
        
        print("\n" + "─"*40)
        print(f"CAMERA VIEW - Tag Position: {position.upper()}, Distance: {dist_text}")
        print("┌──────────────────────────────┐")
        print("│                              │")
        print(f"│    {left}       {center}       {right}    │")
        print("│                              │")
        print("└──────────────────────────────┘")
        print(f"Current Max Speed: {self.max_speed}  Min Speed: {self.min_speed}")
        print("─"*40)
    
    def _get_simulated_direction_and_speed(self, position, distance):
        """Calculate direction and speed based on simulated tag position and distance"""
        direction = DIR_STOP
        speed = self.min_speed
        
        # Convert distance to centimeters (for simulation)
        if distance == 'f':
            distance_cm = 200.0  # Far - 2 meters
        elif distance == 'm':
            distance_cm = 100.0  # Medium - 1 meter
        else:  # 'n' for near
            distance_cm = 40.0   # Near - 40 cm
        
        # Calculate dynamic speed based on distance
        base_speed = self.calculate_dynamic_speed(distance_cm)
        
        # Determine direction based on tag position
        if position == 'l':
            # Tag is on the left, robot should turn left to center it
            direction = DIR_LEFT
            # Calculate proportional turn speed
            offset_ratio = 0.75  # Simulate how far off-center (0.0-1.0)
            turn_factor = 0.25 + (0.25 * offset_ratio)  # 25-50% of speed
            
            # Further reduce turn speed when close to the tag
            if distance == 'n':  # When close
                turn_factor *= 0.4  # 40% of normal turning speed when close
            elif distance == 'm':  # Medium distance
                turn_factor *= 0.7  # 70% of normal turning speed
                
            speed = max(self.min_speed, int(base_speed * turn_factor))
        
        elif position == 'r':
            # Tag is on the right, robot should turn right to center it
            direction = DIR_RIGHT
            # Calculate proportional turn speed
            offset_ratio = 0.75  # Simulate how far off-center (0.0-1.0)
            turn_factor = 0.25 + (0.25 * offset_ratio)  # 25-50% of speed
            
            # Further reduce turn speed when close to the tag
            if distance == 'n':  # When close
                turn_factor *= 0.4  # 40% of normal turning speed when close
            elif distance == 'm':  # Medium distance
                turn_factor *= 0.7  # 70% of normal turning speed
                
            speed = max(self.min_speed, int(base_speed * turn_factor))
        
        elif position == 'c':
            # Tag is centered, move toward it (remember DIR_BACKWARD is FORWARD for our robot)
            direction = DIR_BACKWARD
            
            # Adjust speed based on distance
            if distance == 'f':
                # Tag is far, move at 90% of max speed
                speed = int(base_speed * 0.9)
            elif distance == 'm':
                # Tag is at medium distance, move at 70% of max speed
                speed = int(base_speed * 0.7)
            else:  # 'n' for near
                # Tag is close, move at 40% of max speed
                speed = int(base_speed * 0.4)
        
        return direction, speed
            
    def calculate_dynamic_speed(self, distance_cm: float) -> int:
        """Calculate an appropriate speed based on distance"""
        # Calculate a speed value that decreases as we get closer to the target
        if distance_cm < 30:  # Very close
            return self.min_speed
        elif distance_cm > 150:  # Far away
            return self.max_speed
        else:
            # Linear interpolation between min and max speed
            speed_range = self.max_speed - self.min_speed
            distance_factor = (distance_cm - 30) / (150 - 30)
            return int(self.min_speed + (speed_range * distance_factor))


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    logger.info("Shutdown signal received, exiting...")
    sys.exit(0)


def main():
    """Main function to parse arguments and run the controller"""
    # Register signal handler for clean exit on Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Integrated Movement Controller for ILM Module")
    
    parser.add_argument("--port", type=str, default=None,
                        help=f"Serial port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD_RATE,
                        help=f"Baud rate (default: {DEFAULT_BAUD_RATE})")
    parser.add_argument("--max-speed", type=int, default=DEFAULT_MAX_SPEED,
                        help=f"Maximum speed (default: {DEFAULT_MAX_SPEED})")
    parser.add_argument("--min-speed", type=int, default=DEFAULT_MIN_SPEED,
                        help=f"Minimum speed (default: {DEFAULT_MIN_SPEED})")
    parser.add_argument("--test", action="store_true",
                        help="Run the automated test sequence")
    parser.add_argument("--three-wheels", action="store_true",
                        help="Use 3-wheel configuration (default: 2-wheel)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable verbose output")
    
    args = parser.parse_args()
    
    # Create the controller
    controller = IntegratedMovementController(
        port=args.port,
        baud_rate=args.baud,
        max_speed=args.max_speed,
        min_speed=args.min_speed,
        use_three_wheels=args.three_wheels,
        verbose=args.verbose
    )
    
    # Run in requested mode
    if args.test:
        controller.run_test_sequence()
    else:
        controller.run_interactive_mode()


if __name__ == "__main__":
    main()
