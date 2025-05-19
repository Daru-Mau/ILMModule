#!/usr/bin/env python3
"""
Arduino Sensor Data Format Tester
This script focuses solely on the format of the ultrasonic sensor data packets
coming from the Arduino, helping debug parsing issues.
"""

import time
import serial
import argparse
import re

# Default settings
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200

def test_sensor_format(port, baud_rate, duration=30):
    """
    Test the format of sensor data coming from Arduino
    
    Args:
        port (str): Serial port to connect to
        baud_rate (int): Baud rate for serial communication
        duration (int): Duration in seconds to monitor
    """
    try:
        # Connect to Arduino
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to Arduino on {port} at {baud_rate} baud")
        ser.reset_input_buffer()
        
        # Wait for Arduino to reset if necessary
        time.sleep(2)
        
        # Send a command to request sensor data
        print("Sending TEST command to Arduino...")
        ser.write(b'TEST\r\n')
        
        # Monitor for a short time to detect format of the data
        print("\nMonitoring for sensor data format...")
        print("Press Ctrl+C to stop early\n")
        
        start_time = time.time()
        
        # Define the expected regex pattern for sensor data
        sensor_pattern = re.compile(r'SENS:FL:([\d\.]+),F:([\d\.]+),FR:([\d\.]+),BL:([\d\.]+),B:([\d\.]+),BR:([\d\.]+)')
        
        # Statistics about message format
        message_count = 0
        valid_format_count = 0
        invalid_values_count = 0
        
        while (time.time() - start_time) < duration:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"[{time.time() - start_time:.1f}s] Raw data: '{line}'")
                        
                        # Look for sensor data format
                        if line.startswith("SENS:"):
                            message_count += 1
                            
                            # Verbose analysis of the message format
                            print(f"  Message #{message_count}: Length={len(line)} chars")
                            
                            # Check if message follows expected format
                            match = sensor_pattern.match(line)
                            if match:
                                valid_format_count += 1
                                print("  ✓ Format matches expected pattern")
                                
                                # Extract values and check if they're valid numbers
                                values = match.groups()
                                invalid_values = 0
                                
                                # Print each value and check if it's a valid number
                                for i, (sensor, value) in enumerate(zip(['FL', 'F', 'FR', 'BL', 'B', 'BR'], values)):
                                    try:
                                        float_val = float(value)
                                        if float_val >= 400.0:  # Check for timeout/invalid value
                                            print(f"  - {sensor}: {value} (WARNING: This appears to be a timeout/invalid reading)")
                                            invalid_values += 1
                                        else:
                                            print(f"  - {sensor}: {value} (valid number)")
                                    except ValueError:
                                        print(f"  - {sensor}: {value} (ERROR: Not a valid number)")
                                        invalid_values += 1
                                
                                if invalid_values > 0:
                                    invalid_values_count += 1
                                    print(f"  ⚠ Message contains {invalid_values} invalid value(s)")
                            else:
                                print("  ✗ Format does NOT match expected pattern 'SENS:FL:n.n,F:n.n,FR:n.n,BL:n.n,B:n.n,BR:n.n'")
                                print("  Actual format:", line)
                                
                                # More detailed format analysis for debugging
                                parts = line.split(',')
                                print(f"  Split into {len(parts)} parts: {parts}")
                except UnicodeDecodeError:
                    print("Warning: Decode error (non-UTF8 data received)")
            
            time.sleep(0.01)  # Small delay to prevent CPU hogging
        
        # Print summary
        print("\n=== FORMAT TEST SUMMARY ===")
        print(f"Total sensor messages received: {message_count}")
        print(f"Messages with correct format: {valid_format_count} ({valid_format_count/message_count*100:.1f}% if messages > 0)")
        print(f"Messages with invalid values: {invalid_values_count} ({invalid_values_count/message_count*100:.1f}% if messages > 0)")
        
        if message_count > 0:
            if valid_format_count == message_count:
                print("✓ SUCCESS: All messages have the correct format")
            else:
                print("✗ FAIL: Some messages have incorrect format")
            
            if invalid_values_count > 0:
                print("⚠ WARNING: Some messages contain invalid values (possible sensor errors)")
        else:
            print("✗ FAIL: No sensor messages received at all")
    
    except KeyboardInterrupt:
        print("\nTest stopped by user")
    except serial.SerialException as e:
        print(f"\nSerial port error: {e}")
    finally:
        # Clean up
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed")

def main():
    """Main function to parse arguments and run the test"""
    parser = argparse.ArgumentParser(description="Test Arduino sensor data format")
    parser.add_argument('--port', type=str, default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD_RATE,
                        help=f'Baud rate (default: {DEFAULT_BAUD_RATE})')
    parser.add_argument('--duration', type=int, default=30,
                        help='Duration in seconds to test (default: 30)')
    args = parser.parse_args()
    
    # Run the test
    test_sensor_format(args.port, args.baud, args.duration)

if __name__ == "__main__":
    main()
