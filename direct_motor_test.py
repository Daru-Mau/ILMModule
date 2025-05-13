#!/usr/bin/env python3
"""
Direct Motor Test for ILM Robot - Modified to match Arduino IDE Serial Monitor
This simple script tests motors by sending single-character commands
to the basic_moveset.ino Arduino sketch, exactly as the Arduino IDE Serial Monitor would.
"""

import serial
import time
import argparse
import sys

# Default settings
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200
DEFAULT_DURATION = 1.0  # seconds

class DirectMotorTester:
    """Simple class to test motors using direct commands"""
    
    def __init__(self, port=DEFAULT_PORT, baud_rate=DEFAULT_BAUD_RATE):
        """Initialize with serial port settings"""
        self.port = port
        self.baud_rate = baud_rate
        self.serial = None
        self.connected = False
        
    def connect(self):
        """Connect to the Arduino"""
        try:
            print(f"Connecting to Arduino on {self.port} at {self.baud_rate} baud...")
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=1)
            # Give more time for Arduino to reset after connection
            time.sleep(3)  # Increased from 2 to 3 seconds
            self.connected = True
            
            # Flush any existing data
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            # Read and display the initial output from Arduino
            self.read_response(3.0)  # Wait up to 3 seconds for initial messages
            
            print("Connected to Arduino.")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
            
    def disconnect(self):
        """Close the serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected from Arduino.")
        self.connected = False
    
    def read_response(self, timeout=1.0):
        """Read and print response from Arduino with timeout"""
        start_time = time.time()
        response_received = False
        
        # Continue reading until timeout or no more data
        while (time.time() - start_time) < timeout:
            if self.serial.in_waiting > 0:
                response_received = True
                line = self.serial.readline().decode('ascii', errors='replace').strip()
                if line:
                    print(f"Arduino: {line}")
            else:
                if response_received:
                    break  # Exit if we've received data but buffer is now empty
                time.sleep(0.01)  # Small delay to prevent CPU hogging
        
        return response_received
        
    def send_command(self, command):
        """Send a single character command to Arduino"""
        if not self.connected:
            print("Not connected to Arduino.")
            return False
            
        try:
            print(f"Sending command: '{command}'")
            
            # Add carriage return and newline - match Arduino IDE Serial Monitor
            self.serial.write(f"{command}\r\n".encode())
            self.serial.flush()
            
            # Read and print response from Arduino
            return self.read_response(2.0)
                    
        except Exception as e:
            print(f"Error sending command: {e}")
            self.connected = False
            return False
            
    def test_motor(self, direction, duration):
        """Test motor movement in a specific direction"""
        if not self.connected:
            print("Not connected to Arduino.")
            return False
            
        direction_cmd = {
            'forward': 'F',
            'backward': 'B',
            'left': 'L',
            'right': 'R',
            'stop': 'S'
        }.get(direction.lower())
        
        if not direction_cmd:
            print(f"Unknown direction: {direction}")
            return False
            
        print(f"Testing {direction} movement for {duration} seconds...")
        success = self.send_command(direction_cmd)
        
        if not success:
            print("Failed to send command or receive response.")
            return False
            
        # Wait for the specified duration (plus a little extra)
        # The Arduino will stop the motors after its own delay
        # But we'll wait extra here to make sure the command completes
        print(f"Waiting {duration} seconds...")
        time.sleep(duration + 0.5)
            
        return True
        
    def run_full_test(self, duration):
        """Run a full test sequence"""
        if not self.connected:
            print("Not connected to Arduino.")
            return False
            
        tests = [
            ("forward", "FORWARD"),
            ("backward", "BACKWARD"),
            ("left", "LEFT rotation"),
            ("right", "RIGHT rotation"),
            ("stop", "STOP")
        ]
        
        print("\n=== Starting Motor Test Sequence ===")
        
        for cmd, name in tests:
            print(f"\nTesting {name}...")
            self.test_motor(cmd, duration)
            time.sleep(1.5)  # Longer pause between tests - increased from 0.5
            
        print("\n=== Motor Test Sequence Complete ===")
        return True


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Test motors using basic_moveset.ino')
    parser.add_argument('--port', '-p', default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', '-b', type=int, default=DEFAULT_BAUD_RATE,
                        help=f'Baud rate (default: {DEFAULT_BAUD_RATE})')
    parser.add_argument('--duration', '-d', type=float, default=DEFAULT_DURATION,
                        help=f'Duration for each movement test in seconds (default: {DEFAULT_DURATION})')
    parser.add_argument('--test', '-t', 
                        choices=['all', 'forward', 'backward', 'left', 'right', 'stop'],
                        default='all', 
                        help='Which test to run (default: all)')
    return parser.parse_args()


def main():
    """Main function"""
    args = parse_args()
    
    tester = DirectMotorTester(args.port, args.baud)
    if not tester.connect():
        return 1
    
    try:
        if args.test == 'all':
            tester.run_full_test(args.duration)
        else:
            tester.test_motor(args.test, args.duration)
    finally:
        # Make sure motors are stopped before disconnecting
        print("Sending final STOP command...")
        tester.send_command('S')
        tester.disconnect()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())