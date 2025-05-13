#!/usr/bin/env python3
"""
Simple serial test script to directly send commands to the Arduino
"""

import serial
import time
import sys


def main():
    # Setup
    port = '/dev/ttyACM0'  # Default Arduino port on Linux
    baud_rate = 115200     # Make sure this matches Arduino's baud rate
    
    print(f"Connecting to Arduino on {port} at {baud_rate} baud...")
    
    # Try to open the serial port
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        # Give Arduino time to reset
        time.sleep(2)
        
        # Flush any existing data
        ser.reset_input_buffer()
        print("Connected successfully!")
        print("Available commands:")
        print("  1. Send 'F' (forward)")
        print("  2. Send 'B' (backward)")
        print("  3. Send 'L' (left)")
        print("  4. Send 'R' (right)")
        print("  5. Send 'S' (stop)")
        print("  6. Send 'TEST' command")
        print("  7. Send TAG command")
        print("  8. Quit")
        
        while True:
            choice = input("\nEnter command number (1-8): ")
            
            if choice == '1':
                print("Sending FORWARD ('F') command...")
                ser.write(b'F\r\n')
                time.sleep(0.1)
                
            elif choice == '2':
                print("Sending BACKWARD ('B') command...")
                ser.write(b'B\r\n')
                time.sleep(0.1)
                
            elif choice == '3':
                print("Sending LEFT ('L') command...")
                ser.write(b'L\r\n')
                time.sleep(0.1)
                
            elif choice == '4':
                print("Sending RIGHT ('R') command...")
                ser.write(b'R\r\n')
                time.sleep(0.1)
                
            elif choice == '5':
                print("Sending STOP ('S') command...")
                ser.write(b'S\r\n')
                time.sleep(0.1)
                
            elif choice == '6':
                print("Sending TEST command...")
                ser.write(b'TEST\r\n')
                time.sleep(0.1)
                
            elif choice == '7':
                print("Sending TAG command (tag_id=0, distance=150, direction=F)...")
                ser.write(b'TAG:0,150.0,F\r\n')
                time.sleep(0.1)
                
            elif choice == '8':
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
        
        # Clean up
        ser.close()
        print("Serial connection closed.")
        
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())