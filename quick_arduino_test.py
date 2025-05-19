#!/usr/bin/env python3
"""
Simple Test Script for Arduino Communication
This script sends basic commands to verify Arduino communication
"""

import serial
import time
import sys

# Default parameters
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

def test_arduino_communication():
    """Test basic Arduino communication"""
    try:
        print(f"Connecting to Arduino on {SERIAL_PORT} at {BAUD_RATE} baud...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("Connected! Waiting for Arduino to initialize...")
        time.sleep(2)  # Give Arduino time to reset
        
        # Clear input buffer
        ser.reset_input_buffer()
        
        print("\nSending PING command...")
        ser.write(b'PING\r\n')
        time.sleep(0.5)
        response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        print(f"Response: {response}")
        
        print("\nSending TEST command...")
        ser.write(b'TEST\r\n')
        time.sleep(1)
        response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        print(f"Response: {response}")
        
        print("\nWaiting for sensor data (10 seconds)...")
        start_time = time.time()
        while time.time() - start_time < 10:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"Received: {line}")
            time.sleep(0.01)
        
        print("\nSending STOP command...")
        ser.write(b'STOP\r\n')
        time.sleep(0.5)
        response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        print(f"Response: {response}")
        
        print("\nTest complete!")
        return True
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        return False
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    # Check if a different port was specified
    if len(sys.argv) > 1:
        SERIAL_PORT = sys.argv[1]
    
    success = test_arduino_communication()
    sys.exit(0 if success else 1)
