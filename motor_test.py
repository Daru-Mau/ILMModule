#!/usr/bin/env python3
"""
Motor Test Script for the ILM Module
This script allows testing of individual motors and movement patterns
for the 3-wheeled holonomic robot.

Compatible with integrated_movement.ino
"""

import time
import argparse
import sys
from typing import Optional

# Import the Arduino communication class from existing module
try:
    from apriltag_communication import ArduinoCommunicator, TagData
except ImportError:
    print("Error: apriltag_communication.py module not found.")
    print("Make sure this script is in the same directory as apriltag_communication.py")
    sys.exit(1)

# Default settings
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200
DEFAULT_TEST_DURATION = 2.0  # seconds - increased from 1.0
DEFAULT_SPEED = 150.0  # mm or arbitrary units - increased from 50.0 for better movement


class MotorTester:
    """Class to handle motor testing for the 3-wheeled robot"""
    
    def __init__(self, port: str = DEFAULT_PORT, baud_rate: int = DEFAULT_BAUD_RATE):
        """Initialize the tester with connection parameters"""
        self.arduino = ArduinoCommunicator(port, baud_rate)
        self.connected = False
    
    def connect(self) -> bool:
        """Establish connection to the Arduino"""
        self.connected = self.arduino.connect()
        if not self.connected:
            print("Failed to connect to Arduino.")
            return False
            
        # Optional: Send ping to verify connection
        ping_success = self.arduino.ping()
        if not ping_success:
            print("Note: Arduino connected but not responding to ping. Continuing anyway.")
            # Don't set connected to False, just continue
            
        print("Successfully connected to Arduino.")
        return True
    
    def disconnect(self) -> None:
        """Disconnect from the Arduino"""
        if self.connected:
            self.arduino.disconnect()
            print("Disconnected from Arduino.")
    
    def test_diagnostics(self) -> bool:
        """Run the built-in diagnostics command"""
        if not self.connected:
            print("Not connected to Arduino.")
            return False
        
        print("\n=== Running Arduino Diagnostics ===")
        result = self.arduino.send_test_command()
        if not result:
            print("Diagnostics command failed.")
            return False
        
        # Get additional response data
        response = self.arduino._get_response(1.0)
        print(f"Diagnostics Response: {response}")
        print("=== Diagnostics Complete ===")
        return True
    
    def move(self, direction: str, distance: float, duration: float) -> bool:
        """
        Send movement command in a specific direction
        
        Parameters:
        - direction: 'F'(forward), 'B'(backward), 'L'(left), 'R'(right), 'CW'(clockwise), 'CCW'(counterclockwise)
        - distance: arbitrary units used by the Arduino for speed control
        - duration: how long to move in seconds
        """
        if not self.connected:
            print("Not connected to Arduino.")
            return False
        
        # Convert CW/CCW to actual directions the Arduino understands
        tag_direction = direction
        if direction == 'CW':
            tag_direction = 'R'  # Use right with specific tag_id for rotation
        elif direction == 'CCW':
            tag_direction = 'L'  # Use left with specific tag_id for rotation
        
        # Use tag_id=0 for regular movement, tag_id=99 for rotation
        tag_id = 99 if direction in ['CW', 'CCW'] else 0
        
        # Create tag data
        tag = TagData(tag_id=tag_id, distance=distance, direction=tag_direction)
        
        print(f"Moving {direction} at speed {distance} for {duration} seconds...")
        success = self.arduino.send_tag_data(tag)
        
        if not success:
            print("Failed to send movement command.")
            return False
        
        # Wait for the specified duration
        time.sleep(duration)
        
        # Stop movement
        print("Stopping...")
        success = self.arduino.send_stop()
        if not success:
            print("Failed to send stop command.")
            return False
        
        return True
    
    def test_individual_motor(self, motor: str, duration: float = DEFAULT_TEST_DURATION, speed: float = DEFAULT_SPEED) -> bool:
        """
        Test an individual motor
        
        Parameters:
        - motor: 'left', 'right', or 'back'
        - duration: how long to run the test in seconds
        - speed: speed to run the motor at
        """
        if not self.connected:
            print("Not connected to Arduino.")
            return False
        
        print(f"\n=== Testing {motor.upper()} motor ===")
        
        # Using TAG commands with specific parameters to isolate motor movements
        # Based on the holonomic drive equations in moveRobot function:
        #   leftSpeed = -0.5 * vx - 0.866 * vy + omega
        #   rightSpeed = -0.5 * vx + 0.866 * vy + omega
        #   backSpeed = vx + omega
        
        success = False
        
        if motor.lower() == 'left':
            # To isolate left motor: vx=0, vy=-1, omega=0 (backward)
            print("Testing LEFT motor FORWARD...")
            tag = TagData(tag_id=1, distance=speed, direction='B')
            success = self.arduino.send_tag_data(tag)
        elif motor.lower() == 'right':
            # To isolate right motor: vx=0, vy=1, omega=0 (forward)
            print("Testing RIGHT motor FORWARD...")
            tag = TagData(tag_id=2, distance=speed, direction='F')
            success = self.arduino.send_tag_data(tag)
        elif motor.lower() == 'back':
            # To isolate back motor: vx=1, vy=0, omega=0 (right)
            print("Testing BACK motor FORWARD...")
            tag = TagData(tag_id=3, distance=speed, direction='R')
            success = self.arduino.send_tag_data(tag)
        else:
            print(f"Unknown motor: {motor}")
            return False
        
        if not success:
            print("Failed to send motor test command.")
            return False
        
        # Wait for the specified duration
        time.sleep(duration)
        
        # Stop the motor
        print(f"Stopping {motor} motor...")
        success = self.arduino.send_stop()
        if not success:
            print("Failed to send stop command.")
            return False
        
        print(f"=== {motor.upper()} motor test completed ===")
        return True
    
    def run_all_tests(self, duration: float = DEFAULT_TEST_DURATION, speed: float = DEFAULT_SPEED) -> bool:
        """Run a sequence of tests on all motors and movement patterns"""
        if not self.connected:
            print("Not connected to Arduino.")
            return False
        
        tests = [
            # Individual motors
            ("left", "LEFT motor", self.test_individual_motor, ["left", duration, speed]),
            ("right", "RIGHT motor", self.test_individual_motor, ["right", duration, speed]),
            ("back", "BACK motor", self.test_individual_motor, ["back", duration, speed]),
            
            # Basic movements
            ("forward", "FORWARD movement", self.move, ["F", speed, duration]),
            ("backward", "BACKWARD movement", self.move, ["B", speed, duration]),
            ("left", "LEFT movement", self.move, ["L", speed, duration]),
            ("right", "RIGHT movement", self.move, ["R", speed, duration]),
            
            # Rotations
            ("clockwise", "CLOCKWISE rotation", self.move, ["CW", speed, duration]),
            ("counterclockwise", "COUNTERCLOCKWISE rotation", self.move, ["CCW", speed, duration]),
        ]
        
        print("\n=== Starting Comprehensive Motor Tests ===")
        
        # First run diagnostics
        self.test_diagnostics()
        time.sleep(1)
        
        # Run each test with a pause between them
        for test_id, test_name, test_func, test_args in tests:
            print(f"\nStarting test: {test_name}")
            try:
                test_func(*test_args)
            except Exception as e:
                print(f"Error during {test_name}: {e}")
            
            # Pause between tests
            time.sleep(0.5)
        
        print("\n=== Motor Tests Completed ===")
        return True


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Test motors on the ILM robot')
    parser.add_argument('--port', '-p', default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', '-b', type=int, default=DEFAULT_BAUD_RATE,
                        help=f'Baud rate (default: {DEFAULT_BAUD_RATE})')
    parser.add_argument('--duration', '-d', type=float, default=DEFAULT_TEST_DURATION,
                        help=f'Duration for each test in seconds (default: {DEFAULT_TEST_DURATION})')
    parser.add_argument('--speed', '-s', type=float, default=DEFAULT_SPEED,
                        help=f'Motor speed (default: {DEFAULT_SPEED})')
    parser.add_argument('--test', '-t', choices=['all', 'left', 'right', 'back', 
                                                'forward', 'backward', 'rotate', 'diagnostics'],
                        default='all', help='Which test to run (default: all)')
    return parser.parse_args()


def main():
    """Main function"""
    args = parse_args()
    
    tester = MotorTester(args.port, args.baud)
    if not tester.connect():
        return 1
    
    try:
        if args.test == 'all':
            tester.run_all_tests(args.duration, args.speed)
        elif args.test == 'left':
            tester.test_individual_motor('left', args.duration, args.speed)
        elif args.test == 'right':
            tester.test_individual_motor('right', args.duration, args.speed)
        elif args.test == 'back':
            tester.test_individual_motor('back', args.duration, args.speed)
        elif args.test == 'forward':
            tester.move('F', args.speed, args.duration)
        elif args.test == 'backward':
            tester.move('B', args.speed, args.duration)
        elif args.test == 'rotate':
            print("\n=== Testing rotation ===")
            tester.move('CW', args.speed, args.duration)
            time.sleep(0.5)
            tester.move('CCW', args.speed, args.duration)
        elif args.test == 'diagnostics':
            tester.test_diagnostics()
    finally:
        # Always stop and disconnect
        tester.arduino.send_stop()
        tester.disconnect()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())