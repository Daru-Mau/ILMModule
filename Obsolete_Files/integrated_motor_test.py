#!/usr/bin/env python3
"""
Integrated Motor Test Script for the ILM Module
This script is specifically designed to work with integrated_movement.ino

Features:
- Automated motor tests
- Direct serial communication mode
- Customized for the integrated_movement.ino sketch

Usage:
- Automated tests: python integrated_motor_test.py --test [test_name]
- Interactive mode: python integrated_motor_test.py --interactive
- AprilTag simulation: python integrated_motor_test.py --apriltag-sim
"""

import time
import argparse
import sys
import serial
from typing import Optional

# Import the Arduino communication class from the centralized module
try:
    from raspy_communication import ArduinoCommunicator, TagData
except ImportError:
    print("Error: raspy_communication.py module not found.")
    print("Make sure this script is in the same directory as raspy_communication.py")
    sys.exit(1)

# Default settings
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200
DEFAULT_TEST_DURATION = 2.0  # seconds
DEFAULT_SPEED = 150.0  # Higher default speed for integrated_movement


class IntegratedMotorTester:
    """Class to handle motor testing for the integrated movement controller"""
    
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
            
        # Set higher speeds for integrated_movement - it needs more power
        self.arduino.set_speed(200, 150)
        
        # Send ping to verify connection
        ping_success = self.arduino.ping()
        if not ping_success:
            print("Note: Arduino connected but not responding to ping. Continuing anyway.")
            
        print("Successfully connected to Arduino (integrated_movement).")
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
    
    def move_integrated(self, direction: str, speed: float, duration: float) -> bool:
        """
        Send direct movement command using integrated_movement.ino's format
        
        Parameters:
        - direction: 'F'(forward), 'B'(backward), 'L'(left), 'R'(right), 'CW'(clockwise), 'CCW'(counterclockwise)
        - speed: speed value (0-255)
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
        tag_id = 99 if direction in ['CW', 'CCW'] else 1
        
        # Create tag data - ensure speed is high enough
        actual_speed = max(speed, 150)  # Force minimum speed of 150
        tag = TagData(tag_id=tag_id, distance=actual_speed, direction=tag_direction)
        
        print(f"Moving {direction} at speed {actual_speed} for {duration} seconds...")
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
        
        # Force minimum speed for reliable movement
        actual_speed = max(speed, 150)
        
        # Set up specific motor commands
        if motor.lower() == 'left':
            print("Testing LEFT motor...")
            if not self._send_direct_motor_command('left', actual_speed, duration):
                return False
        elif motor.lower() == 'right':
            print("Testing RIGHT motor...")
            if not self._send_direct_motor_command('right', actual_speed, duration):
                return False
        elif motor.lower() == 'back':
            print("Testing BACK motor...")
            if not self._send_direct_motor_command('back', actual_speed, duration):
                return False
        else:
            print(f"Unknown motor: {motor}")
            return False
            
        print(f"=== {motor.upper()} motor test completed ===")
        return True
    
    def _send_direct_motor_command(self, motor: str, speed: float, duration: float) -> bool:
        """Helper method to send direct motor commands via custom commands"""
        command = ""
        tag_id = 10  # Special tag ID to indicate direct motor control
        
        # Speed must be integer for Arduino
        speed_int = int(speed)
        
        if motor == 'left':
            command = f"TAG:{tag_id},{speed_int},L"
        elif motor == 'right':
            command = f"TAG:{tag_id},{speed_int},R"
        elif motor == 'back':
            command = f"TAG:{tag_id},{speed_int},B"  # Using B for Back motor
        else:
            return False
        
        print(f"Sending direct motor command: {command}")
        success = self.arduino._send_command(command)
        
        if not success:
            print("Failed to send motor command.")
            return False
            
        time.sleep(duration)
        
        # Stop the motor
        print(f"Stopping {motor} motor...")
        success = self.arduino.send_stop()
        return success
    
    def run_obstacle_test(self, speed: float = DEFAULT_SPEED, duration: float = 5.0) -> bool:
        """Test the obstacle detection functionality"""
        if not self.connected:
            print("Not connected to Arduino.")
            return False
            
        print("\n=== Testing Obstacle Detection ===")
        print("Robot will move forward for 5 seconds or until obstacle detected")
        print("Place your hand in front of sensors to test obstacle detection")
        
        # First send forward movement
        tag = TagData(tag_id=0, distance=speed, direction='F')
        success = self.arduino.send_tag_data(tag)
        
        if not success:
            print("Failed to send movement command.")
            return False
            
        # Monitor for the specified duration and print sensor readings
        start_time = time.time()
        detected_obstacle = False
        
        while time.time() - start_time < duration and not detected_obstacle:
            # Send test command to get sensor readings
            self.arduino._send_command("TEST")
            response = self.arduino._get_response(0.2)
            
            if "EMERGENCY STOP" in response:
                print("✓ OBSTACLE DETECTED - Emergency stop activated")
                detected_obstacle = True
                break
                
            time.sleep(0.5)
            
        # Stop the motors
        self.arduino.send_stop()
        
        if not detected_obstacle:
            print("No obstacles detected during test.")
            
        return True
        
    def run_all_tests(self, duration: float = DEFAULT_TEST_DURATION, speed: float = DEFAULT_SPEED) -> bool:
        """Run a comprehensive sequence of tests for the integrated movement system"""
        if not self.connected:
            print("Not connected to Arduino.")
            return False
        
        # Ensure speed is high enough
        actual_speed = max(speed, 150)
        
        tests = [
            # Core movement tests
            ("forward", "FORWARD movement", self.move_integrated, ["F", actual_speed, duration]),
            ("backward", "BACKWARD movement", self.move_integrated, ["B", actual_speed, duration]),
            ("left", "LEFT movement", self.move_integrated, ["L", actual_speed, duration]),
            ("right", "RIGHT movement", self.move_integrated, ["R", actual_speed, duration]),
            
            # Rotations
            ("clockwise", "CLOCKWISE rotation", self.move_integrated, ["CW", actual_speed, duration]),
            ("counterclockwise", "COUNTERCLOCKWISE rotation", self.move_integrated, ["CCW", actual_speed, duration]),
            
            # Individual motors (optional - may not work perfectly with integrated movement)
            ("left_motor", "LEFT motor", self.test_individual_motor, ["left", duration, actual_speed]),
            ("right_motor", "RIGHT motor", self.test_individual_motor, ["right", duration, actual_speed]),
            ("back_motor", "BACK motor", self.test_individual_motor, ["back", duration, actual_speed]),
        ]
        
        print("\n=== Starting Comprehensive Tests for Integrated Movement ===")
        
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
        
        # Finally test obstacle detection
        self.run_obstacle_test(actual_speed)
        
        print("\n=== Integrated Movement Tests Completed ===")
        return True
    
    def interactive_mode(self) -> None:
        """Run an interactive serial control session"""
        if hasattr(self, 'arduino') and hasattr(self.arduino, 'serial') and self.arduino.serial:
            # We'll use the existing serial connection
            ser = self.arduino.serial
            print("Using existing serial connection...")
        else:
            try:
                ser = serial.Serial(DEFAULT_PORT, DEFAULT_BAUD_RATE, timeout=1)
                time.sleep(2)  # Give Arduino time to reset
                ser.reset_input_buffer()
            except serial.SerialException as e:
                print(f"Error opening serial port: {e}")
                return
            
        print("\n==== Interactive Serial Control for Integrated Movement ====")
        print("Available commands:")
        print("  1. Move FORWARD (TAG:1,200,F)")
        print("  2. Move BACKWARD (TAG:1,200,B)")
        print("  3. Move LEFT (TAG:1,200,L)")
        print("  4. Move RIGHT (TAG:1,200,R)")
        print("  5. Stop movement (STOP)")
        print("  6. Run diagnostics (TEST)")
        print("  7. Send custom TAG command")
        print("  8. Set motor speeds (SPEED:max,min)")
        print("  9. Run obstacle detection test")
        print("  0. Quit")
        
        while True:
            try:
                choice = input("\nEnter command number (0-9): ")
                
                if choice == '1':
                    print("Moving FORWARD...")
                    # Use TAG ID 1 instead of 0 which might not be recognized
                    ser.write(b'TAG:1,200,F\r\n')
                
                elif choice == '2':
                    print("Moving BACKWARD...")
                    ser.write(b'TAG:1,200,B\r\n')
                
                elif choice == '3':
                    print("Moving LEFT...")
                    ser.write(b'TAG:1,200,L\r\n')
                
                elif choice == '4':
                    print("Moving RIGHT...")
                    ser.write(b'TAG:1,200,R\r\n')
                
                elif choice == '5':
                    print("Stopping movement...")
                    ser.write(b'STOP\r\n')
                
                elif choice == '6':
                    print("Running diagnostics...")
                    ser.write(b'TEST\r\n')
                
                elif choice == '7':
                    # Default to tag ID 1 which is known to work with integrated_movement.ino
                    tag_id = input("Enter tag ID (default: 1): ") or "1"
                    distance = input("Enter distance/speed (default: 200): ") or "200"
                    direction = input("Enter direction (F,B,L,R,S): ").upper() or "F"
                    
                    if direction not in ['F', 'B', 'L', 'R', 'S']:
                        print("Invalid direction! Using F (forward).")
                        direction = 'F'
                        
                    cmd = f"TAG:{tag_id},{distance},{direction}\r\n"
                    print(f"Sending TAG command: {cmd.strip()}")
                    ser.write(cmd.encode())
                
                elif choice == '8':
                    max_speed = input("Enter maximum speed (50-255, default: 200): ") or "200"
                    min_speed = input("Enter minimum speed (30-max, default: 150): ") or "150"
                    cmd = f"SPEED:{max_speed},{min_speed}\r\n"
                    print(f"Setting motor speed parameters: {cmd.strip()}")
                    ser.write(cmd.encode())
                
                elif choice == '9':
                    print("Running obstacle detection test...")
                    print("Move your hand in front of sensors to trigger detection")
                    # Use tag ID 1 for consistent behavior
                    ser.write(b'TAG:1,200,F\r\n')
                    print("Moving forward for 5 seconds...")
                    time.sleep(5)
                    print("Stopping test...")
                    ser.write(b'STOP\r\n')
                
                elif choice == '0':
                    print("Quitting...")
                    ser.write(b'STOP\r\n')  # Stop motors before exiting
                    break
                
                else:
                    print("Invalid choice!")
                    continue
                
                # Read and print the response
                print("\nArduino Response:")
                start_time = time.time()
                while (time.time() - start_time) < 1.0:
                    if ser.in_waiting:
                        try:
                            line = ser.readline().decode('utf-8').strip()
                            print(f"> {line}")
                        except UnicodeDecodeError:
                            print("> [Decode Error]")
                    else:
                        time.sleep(0.05)
                        
            except KeyboardInterrupt:
                print("\nSending STOP command before exiting...")
                ser.write(b'STOP\r\n')
                break
            
            except Exception as e:
                print(f"Error: {e}")
        
        # Don't close the serial connection if it's from the Arduino object
        if not (hasattr(self, 'arduino') and hasattr(self.arduino, 'serial') and self.arduino.serial == ser):
            ser.close()
            print("Serial connection closed.")
    
    def apriltag_simulation_mode(self) -> None:
        """
        Simulate the exact message format that apriltag_recognition.py uses
        
        This function emulates how apriltag_recognition.py communicates with the Arduino
        to ensure the format is correct. It specifically emulates the key components:
        1. Tag detection and message sending
        2. The direction determination based on tag position
        3. Distance calculation and passing it in the message
        """
        if hasattr(self, 'arduino') and hasattr(self.arduino, 'serial') and self.arduino.serial:
            # We'll use the existing serial connection
            ser = self.arduino.serial
            print("Using existing serial connection...")
        else:
            try:
                ser = serial.Serial(DEFAULT_PORT, DEFAULT_BAUD_RATE, timeout=1)
                time.sleep(2)  # Give Arduino time to reset
                ser.reset_input_buffer()
            except serial.SerialException as e:
                print(f"Error opening serial port: {e}")
                return
        
        print("\n==== AprilTag Recognition Simulation Mode ====")
        print("This mode simulates the exact message format used by apriltag_recognition.py")
        print("We'll test if the integrated_movement.ino firmware properly responds to these messages.")
        
        # Setting options that match those in apriltag_recognition.py
        print("\nAvailable simulated tag detections:")
        print("  1. Tag in center (Forward movement)")
        print("  2. Tag on left (Left turn)")
        print("  3. Tag on right (Right turn)")
        print("  4. Tag close by (Stop)")
        print("  5. No tag detected (Stop)")
        print("  6. Custom tag data")
        print("  7. Run automated simulation sequence")
        print("  0. Quit")
        
        while True:
            try:
                choice = input("\nEnter choice (0-7): ")
                
                if choice == '1':
                    # Tag in center - Forward
                    tag_id = 1  # Using ID 1 as default
                    distance = 60.0  # 60cm away
                    direction = 'F'
                    
                    cmd = f"TAG:{tag_id},{distance},{direction}\r\n"
                    print(f"Simulating tag in center: {cmd.strip()}")
                    ser.write(cmd.encode())
                
                elif choice == '2':
                    # Tag on left - Turn left
                    tag_id = 1
                    distance = 70.0
                    direction = 'L'
                    
                    cmd = f"TAG:{tag_id},{distance},{direction}\r\n"
                    print(f"Simulating tag on left: {cmd.strip()}")
                    ser.write(cmd.encode())
                
                elif choice == '3':
                    # Tag on right - Turn right
                    tag_id = 1
                    distance = 70.0
                    direction = 'R'
                    
                    cmd = f"TAG:{tag_id},{distance},{direction}\r\n"
                    print(f"Simulating tag on right: {cmd.strip()}")
                    ser.write(cmd.encode())
                
                elif choice == '4':
                    # Tag close by - Stop
                    tag_id = 1
                    distance = 20.0  # Close to threshold (for apriltag_recognition.py threshold is 30cm)
                    direction = 'S'
                    
                    cmd = f"TAG:{tag_id},{distance},{direction}\r\n"
                    print(f"Simulating tag nearby: {cmd.strip()}")
                    ser.write(cmd.encode())
                
                elif choice == '5':
                    # No tag detected - Stop
                    print("Simulating no tag detected - sending STOP")
                    ser.write(b'STOP\r\n')
                
                elif choice == '6':
                    # Custom tag data
                    tag_id = input("Enter tag ID (default: 1): ") or "1"
                    distance = input("Enter distance in cm (default: 50.0): ") or "50.0"
                    direction = input("Enter direction (F,B,L,R,S): ").upper() or "F"
                    
                    if direction not in ['F', 'B', 'L', 'R', 'S']:
                        print("Invalid direction! Using F (forward).")
                        direction = 'F'
                    
                    try:
                        distance_float = float(distance)
                        cmd = f"TAG:{tag_id},{distance_float},{direction}\r\n"
                        print(f"Sending custom tag data: {cmd.strip()}")
                        ser.write(cmd.encode())
                    except ValueError:
                        print("Invalid distance value! Using 50.0")
                        cmd = f"TAG:{tag_id},50.0,{direction}\r\n"
                        print(f"Sending tag data: {cmd.strip()}")
                        ser.write(cmd.encode())
                
                elif choice == '7':
                    # Run automated sequence
                    print("\n=== Starting automated simulation sequence ===")
                    print("This will cycle through various tag positions and distances")
                    print("Make sure the robot has space to move!")
                    
                    # Format: (description, tag_id, distance, direction, duration)
                    sequence = [
                        ("Tag in center - Forward", 1, 60.0, 'F', 2.0),
                        ("Tag on left - Turn left", 1, 70.0, 'L', 2.0),
                        ("Tag in center - Forward", 1, 60.0, 'F', 2.0),
                        ("Tag on right - Turn right", 1, 70.0, 'R', 2.0),
                        ("Tag in center - Forward", 1, 60.0, 'F', 2.0),
                        ("Tag close by - Stop", 1, 20.0, 'S', 1.0),
                        ("No tag detected - Stop", None, None, None, 1.0)
                    ]
                    
                    for desc, tag_id, distance, direction, duration in sequence:
                        print(f"\nSimulating: {desc}")
                        
                        if tag_id is not None:
                            cmd = f"TAG:{tag_id},{distance},{direction}\r\n"
                            print(f"Sending: {cmd.strip()}")
                            ser.write(cmd.encode())
                        else:
                            print(f"Sending: STOP")
                            ser.write(b'STOP\r\n')
                        
                        # Read and print the response
                        start_time = time.time()
                        while (time.time() - start_time) < 0.5:  # Brief response window
                            if ser.in_waiting:
                                try:
                                    line = ser.readline().decode('utf-8').strip()
                                    print(f"> {line}")
                                except UnicodeDecodeError:
                                    print("> [Decode Error]")
                            else:
                                time.sleep(0.05)
                        
                        # Wait for the specified duration
                        print(f"Waiting {duration} seconds...")
                        time.sleep(duration)
                    
                    # Final stop
                    print("\nSimulation sequence complete - stopping motors")
                    ser.write(b'STOP\r\n')
                
                elif choice == '0':
                    print("Quitting AprilTag simulation mode...")
                    ser.write(b'STOP\r\n')  # Stop motors before exiting
                    break
                
                else:
                    print("Invalid choice!")
                    continue
                
                # Read and print the response
                print("\nArduino Response:")
                start_time = time.time()
                while (time.time() - start_time) < 1.0:
                    if ser.in_waiting:
                        try:
                            line = ser.readline().decode('utf-8').strip()
                            print(f"> {line}")
                        except UnicodeDecodeError:
                            print("> [Decode Error]")
                    else:
                        time.sleep(0.05)
                        
            except KeyboardInterrupt:
                print("\nSending STOP command before exiting...")
                ser.write(b'STOP\r\n')
                break
            
            except Exception as e:
                print(f"Error: {e}")
        
        # Don't close the serial connection if it's from the Arduino object
        if not (hasattr(self, 'arduino') and hasattr(self.arduino, 'serial') and self.arduino.serial == ser):
            ser.close()
            print("Serial connection closed.")


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Test integrated movement controller')
    parser.add_argument('--port', '-p', default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', '-b', type=int, default=DEFAULT_BAUD_RATE,
                        help=f'Baud rate (default: {DEFAULT_BAUD_RATE})')
    parser.add_argument('--duration', '-d', type=float, default=DEFAULT_TEST_DURATION,
                        help=f'Duration for each test in seconds (default: {DEFAULT_TEST_DURATION})')
    parser.add_argument('--speed', '-s', type=float, default=DEFAULT_SPEED,
                        help=f'Motor speed (default: {DEFAULT_SPEED})')
    parser.add_argument('--test', '-t', choices=['all', 'forward', 'backward', 'left', 'right', 
                                               'rotate', 'obstacle', 'diagnostics'],
                        help='Which test to run (default: all)')
    parser.add_argument('--interactive', '-i', action='store_true',
                        help='Run in interactive mode for direct serial control')
    parser.add_argument('--apriltag-sim', '-a', action='store_true',
                        help='Simulate AprilTag detection messages from apriltag_recognition.py')
    return parser.parse_args()


def main():
    """Main function"""
    args = parse_args()
    
    tester = IntegratedMotorTester(args.port, args.baud)
    
    if args.interactive:
        # For interactive mode, we'll initialize the connection inside the method
        tester.interactive_mode()
        return 0
    
    if args.apriltag_sim:
        # For AprilTag simulation mode, initialize connection inside the method
        tester.apriltag_simulation_mode()
        return 0
    
    # For standard operation, we need to connect first
    if not tester.connect():
        return 1
    
    try:
        if args.test == 'all' or args.test is None:
            tester.run_all_tests(args.duration, args.speed)
        elif args.test == 'forward':
            tester.move_integrated('F', args.speed, args.duration)
        elif args.test == 'backward':
            tester.move_integrated('B', args.speed, args.duration)
        elif args.test == 'left':
            tester.move_integrated('L', args.speed, args.duration)
        elif args.test == 'right':
            tester.move_integrated('R', args.speed, args.duration)
        elif args.test == 'rotate':
            print("\n=== Testing rotation ===")
            tester.move_integrated('CW', args.speed, args.duration)
            time.sleep(0.5)
            tester.move_integrated('CCW', args.speed, args.duration)
        elif args.test == 'obstacle':
            tester.run_obstacle_test(args.speed)
        elif args.test == 'diagnostics':
            tester.test_diagnostics()
    finally:
        # Only stop and disconnect if we're in standard mode
        if not args.interactive and not args.apriltag_sim and hasattr(tester, 'arduino'):
            tester.arduino.send_stop()
            tester.disconnect()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())