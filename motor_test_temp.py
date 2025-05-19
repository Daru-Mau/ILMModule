#!/usr/bin/env python3
"""
Enhanced Motor Test Script for the ILM Module
This script allows testing of individual motors and movement patterns
for the 3-wheeled holonomic robot with dynamic velocity control.

Features:
- Automated motor tests
- Dynamic velocity control
- Support for both 2-wheel mode (back wheel disabled) and 3-wheel mode (all wheels active)
- Direct serial communication mode
- Support for both integrated_movement.ino and basic_moveset.ino

Usage:
- Automated tests: python motor_test.py --test [test_name]
- Interactive mode: python motor_test.py --interactive
- Set wheel mode: python motor_test.py --mode 2wheel (or 3wheel)
- Set speed: python motor_test.py --speed 150
"""

import time
import argparse
import sys
import serial
from typing import Optional, Tuple

# Import the Arduino communication class from existing module
try:
    from communication_test import ArduinoCommunicator, TagData
except ImportError:
    print("Error: communication_test.py module not found.")
    print("Make sure this script is in the same directory as communication_test.py")
    sys.exit(1)

# Default settings
DEFAULT_PORT = '/dev/ttyACM0'  # Updated to the correct port where Arduino is detected
DEFAULT_BAUD_RATE = 115200
DEFAULT_TEST_DURATION = 2.0  # seconds
DEFAULT_SPEED = 150.0  # mm or arbitrary units (0-255)
DEFAULT_MODE = "3WHEEL"  # Use all 3 wheels by default (use "2WHEEL" to disable back wheel)


class MotorTester:
    """Class to handle motor testing for the 3-wheeled robot with enhanced control"""
    
    def __init__(self, port: str = DEFAULT_PORT, baud_rate: int = DEFAULT_BAUD_RATE, 
                 speed: float = DEFAULT_SPEED, mode: str = DEFAULT_MODE):
        """Initialize the tester with connection parameters"""
        self.arduino = ArduinoCommunicator(port, baud_rate)
        self.connected = False
        self.speed = speed
        self.mode = mode  # "2WHEEL" or "3WHEEL"
    
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
        
        # Set the initial mode and speed
        time.sleep(0.5)  # Give Arduino time to process
        self._set_mode(self.mode)
        time.sleep(0.5)  # Give Arduino time to process
        self._set_speed(self.speed)
        
        print("Successfully connected to Arduino.")
        return True
    
    def disconnect(self) -> None:
        """Disconnect from the Arduino"""
        if self.connected:
            self.arduino.disconnect()
            print("Disconnected from Arduino.")
    
    def _set_mode(self, mode: str) -> bool:
        """Set the wheel mode (2WHEEL or 3WHEEL)"""
        if not self.connected:
            print("Not connected to Arduino.")
            return False
        
        # Ensure mode is capitalized
        mode = mode.upper()
        if mode not in ["2WHEEL", "3WHEEL"]:
            print(f"Invalid mode: {mode}. Must be 2WHEEL or 3WHEEL.")
            return False
        
        self.mode = mode
        command = f"MODE:{mode}\r\n"
        
        try:
            self.arduino.serial.write(command.encode())
            print(f"Set mode to: {mode}")
            time.sleep(0.1)  # Give Arduino time to process
            
            # Read response
            response = self.arduino._get_response(0.5)
            print(f"Mode Response: {response}")
            return True
        except Exception as e:
            print(f"Error setting mode: {e}")
            return False
    
    def _set_speed(self, speed: float) -> bool:
        """Set the motor speed (0-255)"""
        if not self.connected:
            print("Not connected to Arduino.")
            return False
        
        # Ensure speed is within valid range
        speed = min(255, max(0, int(speed)))
        self.speed = speed
        
        command = f"SPEED:{speed}\r\n"
        
        try:
            self.arduino.serial.write(command.encode())
            print(f"Set speed to: {speed}")
            time.sleep(0.1)  # Give Arduino time to process
            
            # Read response
            response = self.arduino._get_response(0.5)
            print(f"Speed Response: {response}")
            return True
        except Exception as e:
            print(f"Error setting speed: {e}")
            return False
    
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
    
    def move(self, direction: str, distance: float, duration: float, mode: Optional[str] = None) -> bool:
        """
        Send movement command in a specific direction
        
        Parameters:
        - direction: 'F'(forward), 'B'(backward), 'L'(left), 'R'(right), 'CW'(clockwise), 'CCW'(counterclockwise)
                    'SL'(slide left), 'SR'(slide right), 'TL'(turn left), 'TR'(turn right)
        - distance: arbitrary units used by the Arduino for speed control (0-255)
        - duration: how long to move in seconds
        - mode: Override the wheel mode for this command only (2WHEEL or 3WHEEL)
        """
        if not self.connected:
            print("Not connected to Arduino.")
            return False
        
        # Convert special directions to the formats Arduino understands
        tag_direction = direction
        tag_id = 0
        
        # For rotation commands, use tag_id=99
        if direction == 'CW':
            tag_direction = 'R'
            tag_id = 99
        elif direction == 'CCW':
            tag_direction = 'L'
            tag_id = 99
        elif direction == 'SL':  # Slide left
            # For sliding, we'll use a special character
            tag_direction = 'Q'
        elif direction == 'SR':  # Slide right
            tag_direction = 'E'
        elif direction == 'TL':  # Turn left
            tag_direction = 'L'
        elif direction == 'TR':  # Turn right
            tag_direction = 'R'
        
        # For regular SLIDE_LEFT, SLIDE_RIGHT, TURN_LEFT, TURN_RIGHT, send as text commands
        if direction in ['SL', 'SR', 'TL', 'TR']:
            cmd_map = {
                'SL': 'SLIDE_LEFT',
                'SR': 'SLIDE_RIGHT',
                'TL': 'TURN_LEFT',
                'TR': 'TURN_RIGHT'
            }
            
            command = f"{cmd_map[direction]}\r\n"
            print(f"Moving {direction} at speed {self.speed} for {duration} seconds...")
            
            try:
                self.arduino.serial.write(command.encode())
                # Wait for the specified duration
                time.sleep(duration)
                
                # Stop movement
                print("Stopping...")
                self.arduino.send_stop()
                return True
            except Exception as e:
                print(f"Error sending movement command: {e}")
                return False
        
        # Create tag data with mode if provided
        tag = TagData(tag_id=tag_id, distance=distance, direction=tag_direction)
        
        # Build the TAG command manually to include mode
        cmd = f"TAG:{tag.tag_id},{tag.distance},{tag.direction}"
        if mode:
            cmd += f",{mode}"
        cmd += "\r\n"
        
        print(f"Moving {direction} at speed {distance} for {duration} seconds" + 
              (f" with {mode} mode" if mode else ""))
        
        try:
            self.arduino.serial.write(cmd.encode())
            # Wait for the specified duration
            time.sleep(duration)
            
            # Stop movement
            print("Stopping...")
            self.arduino.send_stop()
            return True
        except Exception as e:
            print(f"Error sending movement command: {e}")
            return False
    
    def test_individual_motor(self, motor: str, duration: float = DEFAULT_TEST_DURATION, 
                              speed: Optional[float] = None, mode: Optional[str] = None) -> bool:
        """
        Test an individual motor
        
        Parameters:
        - motor: 'left', 'right', or 'back'
        - duration: how long to run the test in seconds
        - speed: speed to run the motor at (defaults to self.speed if None)
        - mode: wheel mode to use (defaults to self.mode if None)
        """
        if not self.connected:
            print("Not connected to Arduino.")
            return False
        
        speed = speed if speed is not None else self.speed
        mode_to_use = mode if mode else self.mode
        
        print(f"\n=== Testing {motor.upper()} motor (using {mode_to_use} mode) ===")
        
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
            cmd = f"TAG:{tag.tag_id},{tag.distance},{tag.direction},{mode_to_use}\r\n"
            self.arduino.serial.write(cmd.encode())
            success = True
        elif motor.lower() == 'right':
            # To isolate right motor: vx=0, vy=1, omega=0 (forward)
            print("Testing RIGHT motor FORWARD...")
            tag = TagData(tag_id=2, distance=speed, direction='F')
            cmd = f"TAG:{tag.tag_id},{tag.distance},{tag.direction},{mode_to_use}\r\n"
            self.arduino.serial.write(cmd.encode())
            success = True
        elif motor.lower() == 'back':
            # To isolate back motor: vx=1, vy=0, omega=0 (right)
            print("Testing BACK motor FORWARD...")
            tag = TagData(tag_id=3, distance=speed, direction='R')
            cmd = f"TAG:{tag.tag_id},{tag.distance},{tag.direction},{mode_to_use}\r\n"
            self.arduino.serial.write(cmd.encode())
            success = True
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
    
    def run_all_tests(self, duration: float = DEFAULT_TEST_DURATION, 
                      speed: Optional[float] = None, mode: Optional[str] = None) -> bool:
        """
        Run a sequence of tests on all motors and movement patterns
        
        Parameters:
        - duration: duration for each test in seconds
        - speed: speed to use for tests (defaults to self.speed if None)
        - mode: wheel mode to use (defaults to self.mode if None)
        """
        if not self.connected:
            print("Not connected to Arduino.")
            return False
          test_speed = speed if speed is not None else self.speed
        test_mode = mode if mode else self.mode
        
        tests = [
            # Individual motors
            ("left", "LEFT motor", self.test_individual_motor, ["left", duration, test_speed, test_mode]),
            ("right", "RIGHT motor", self.test_individual_motor, ["right", duration, test_speed, test_mode]),
            ("back", "BACK motor", self.test_individual_motor, ["back", duration, test_speed, test_mode]),
            
            # Basic movements
            ("forward", "FORWARD movement", self.move, ["F", test_speed, duration, test_mode]),
            ("backward", "BACKWARD movement", self.move, ["B", test_speed, duration, test_mode]),
            
            # Rotations
            ("clockwise", "CLOCKWISE rotation", self.move, ["CW", test_speed, duration, test_mode]),
            ("counterclockwise", "COUNTERCLOCKWISE rotation", self.move, ["CCW", test_speed, duration, test_mode]),
            
            # New movements
            ("slide_left", "SLIDE LEFT movement", self.move, ["SL", test_speed, duration, test_mode]),
            ("slide_right", "SLIDE RIGHT movement", self.move, ["SR", test_speed, duration, test_mode]),
            ("turn_left", "TURN LEFT movement", self.move, ["TL", test_speed, duration, test_mode]),
            ("turn_right", "TURN RIGHT movement", self.move, ["TR", test_speed, duration, test_mode]),
        ]
        
        print(f"\n=== Starting Comprehensive Motor Tests ===")
        print(f"Mode: {test_mode} {'(all wheels active)' if test_mode == '3WHEEL' else '(back wheel disabled)'}")
        print(f"Speed: {test_speed}")
        
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
            
        print("\n==== Interactive Serial Control Mode ====")
        print("Available commands:")
        print("  1. Send 'F' (forward)")
        print("  2. Send 'B' (backward)")
        print("  3. Send 'L' (rotate left)")
        print("  4. Send 'R' (rotate right)")
        print("  5. Send 'S' (stop)")
        print("  6. Send 'TEST' command")
        print("  7. Send TAG command")
        print("  8. Change speed")        print("  9. Change wheel mode (2-wheel/3-wheel)")
        print("     - 3-wheel: All omnidirectional wheels active")
        print("     - 2-wheel: Back omnidirectional wheel disabled")
        print(" 10. Special movements (slide, turn)")
        print(" 11. Custom command")
        print(" 12. Quit")
        
        while True:
            try:
                choice = input("\nEnter command number (1-12): ")
                
                if choice == '1':
                    print("Sending FORWARD ('F') command...")
                    ser.write(b'F\r\n')
                
                elif choice == '2':
                    print("Sending BACKWARD ('B') command...")
                    ser.write(b'B\r\n')
                
                elif choice == '3':
                    print("Sending LEFT/ROTATE LEFT ('L') command...")
                    ser.write(b'L\r\n')
                
                elif choice == '4':
                    print("Sending RIGHT/ROTATE RIGHT ('R') command...")
                    ser.write(b'R\r\n')
                
                elif choice == '5':
                    print("Sending STOP ('S') command...")
                    ser.write(b'S\r\n')
                
                elif choice == '6':
                    print("Sending TEST command...")
                    ser.write(b'TEST\r\n')
                
                elif choice == '7':
                    tag_id = input("Enter tag ID (default: 0): ") or "0"
                    distance = input("Enter distance/speed (default: 150): ") or "150"
                    direction = input("Enter direction (F,B,L,R,S,Q,E): ").upper() or "F"
                      if direction not in ['F', 'B', 'L', 'R', 'S', 'Q', 'E']:
                        print("Invalid direction! Using F (forward).")
                        direction = 'F'
                    
                    print("Wheel Mode Options:")
                    print("  2WHEEL - Only left and right wheels active (back wheel disabled)")
                    print("  3WHEEL - All three omnidirectional wheels active")
                    mode = input("Enter wheel mode (2WHEEL/3WHEEL, default: current): ").upper() or ""
                    
                    if mode and mode not in ['2WHEEL', '3WHEEL']:
                        print("Invalid mode! Using current mode.")
                        mode = ""
                        
                    cmd = f"TAG:{tag_id},{distance},{direction}"
                    if mode:
                        cmd += f",{mode}"
                    cmd += "\r\n"
                    
                    print(f"Sending TAG command: {cmd.strip()}")
                    ser.write(cmd.encode())
                
                elif choice == '8':
                    new_speed = input("Enter new speed (0-255): ")
                    try:
                        speed = int(new_speed)
                        if 0 <= speed <= 255:
                            cmd = f"SPEED:{speed}\r\n"
                            print(f"Setting speed to: {speed}")
                            ser.write(cmd.encode())
                        else:
                            print("Speed must be between 0-255")
                    except ValueError:
                        print("Invalid speed value")
                  elif choice == '9':
                    print("Wheel Mode Options:")
                    print("  2WHEEL - Only left and right wheels active (back wheel disabled)")
                    print("  3WHEEL - All three omnidirectional wheels active")
                    new_mode = input("Enter wheel mode (2WHEEL/3WHEEL): ").upper()
                    if new_mode in ['2WHEEL', '3WHEEL']:
                        cmd = f"MODE:{new_mode}\r\n"
                        print(f"Setting mode to: {new_mode}")
                        ser.write(cmd.encode())
                    else:
                        print("Invalid mode. Must be 2WHEEL or 3WHEEL")
                
                elif choice == '10':
                    print("Special movements:")
                    print("  a. Slide Left")
                    print("  b. Slide Right")
                    print("  c. Turn Left")
                    print("  d. Turn Right")
                    special_choice = input("Select movement (a-d): ").lower()
                    
                    if special_choice == 'a':
                        print("Sending SLIDE_LEFT command...")
                        ser.write(b'SLIDE_LEFT\r\n')
                    elif special_choice == 'b':
                        print("Sending SLIDE_RIGHT command...")
                        ser.write(b'SLIDE_RIGHT\r\n')
                    elif special_choice == 'c':
                        print("Sending TURN_LEFT command...")
                        ser.write(b'TURN_LEFT\r\n')
                    elif special_choice == 'd':
                        print("Sending TURN_RIGHT command...")
                        ser.write(b'TURN_RIGHT\r\n')
                    else:
                        print("Invalid choice")
                
                elif choice == '11':
                    custom_cmd = input("Enter custom command: ")
                    if custom_cmd:
                        if not custom_cmd.endswith('\r\n'):
                            custom_cmd += '\r\n'
                        print(f"Sending: {custom_cmd.strip()}")
                        ser.write(custom_cmd.encode())
                    else:
                        print("Empty command, not sending.")
                
                elif choice == '12':
                    print("Quitting...")
                    ser.write(b'S\r\n')  # Stop motors before exiting
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
                ser.write(b'S\r\n')
                break
            
            except Exception as e:
                print(f"Error: {e}")
        
        # Don't close the serial connection if it's from the Arduino object
        if not (hasattr(self, 'arduino') and hasattr(self.arduino, 'serial') and self.arduino.serial == ser):
            ser.close()
            print("Serial connection closed.")


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Test motors on the ILM robot with enhanced controls')
    parser.add_argument('--port', '-p', default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', '-b', type=int, default=DEFAULT_BAUD_RATE,
                        help=f'Baud rate (default: {DEFAULT_BAUD_RATE})')
    parser.add_argument('--duration', '-d', type=float, default=DEFAULT_TEST_DURATION,
                        help=f'Duration for each test in seconds (default: {DEFAULT_TEST_DURATION})')
    parser.add_argument('--speed', '-s', type=float, default=DEFAULT_SPEED,
                        help=f'Motor speed (0-255, default: {DEFAULT_SPEED})')    parser.add_argument('--mode', '-m', choices=['2wheel', '3wheel'], default=DEFAULT_MODE.lower(),
                        help=f'Operating mode: 3wheel uses all wheels, 2wheel disables the back wheel (default: {DEFAULT_MODE.lower()})')
    parser.add_argument('--test', '-t', choices=['all', 'left', 'right', 'back', 
                                               'forward', 'backward', 'rotate', 'slide', 'turn', 'diagnostics'],
                        help='Which automated test to run (default: all)')
    parser.add_argument('--interactive', '-i', action='store_true',
                        help='Run in interactive mode for direct serial control')
    return parser.parse_args()


def main():
    """Main function"""
    args = parse_args()
    
    # Convert mode to the format Arduino expects
    mode = args.mode.upper() + "WHEEL" if args.mode.endswith("wheel") else args.mode.upper()
    
    tester = MotorTester(args.port, args.baud, args.speed, mode)
    
    if args.interactive:
        # For interactive mode, we'll initialize the connection inside the method
        if not tester.connect():
            return 1
        tester.interactive_mode()
        return 0
    
    # For standard operation, we need to connect first
    if not tester.connect():
        return 1
    
    try:
        if args.test == 'all' or args.test is None:
            tester.run_all_tests(args.duration, args.speed, mode)
        elif args.test == 'left':
            tester.test_individual_motor('left', args.duration, args.speed, mode)
        elif args.test == 'right':
            tester.test_individual_motor('right', args.duration, args.speed, mode)
        elif args.test == 'back':
            tester.test_individual_motor('back', args.duration, args.speed, mode)
        elif args.test == 'forward':
            tester.move('F', args.speed, args.duration, mode)
        elif args.test == 'backward':
            tester.move('B', args.speed, args.duration, mode)
        elif args.test == 'rotate':
            print("\n=== Testing rotation ===")
            tester.move('CW', args.speed, args.duration, mode)
            time.sleep(0.5)
            tester.move('CCW', args.speed, args.duration, mode)
        elif args.test == 'slide':
            print("\n=== Testing sliding ===")
            tester.move('SL', args.speed, args.duration, mode)
            time.sleep(0.5)
            tester.move('SR', args.speed, args.duration, mode)
        elif args.test == 'turn':
            print("\n=== Testing turning ===")
            tester.move('TL', args.speed, args.duration, mode)
            time.sleep(0.5)
            tester.move('TR', args.speed, args.duration, mode)
        elif args.test == 'diagnostics':
            tester.test_diagnostics()
    finally:
        # Only stop and disconnect if we're in standard mode
        if hasattr(tester, 'arduino'):
            tester.arduino.send_stop()
            tester.disconnect()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())