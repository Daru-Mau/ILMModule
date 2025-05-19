#!/usr/bin/env python3
"""
Integrated Test for AprilTag Recognition with Ultrasonic Sensor Data
This script tests the integration between the Arduino sending sensor data
and the apriltag_recognition.py script processing it.
"""

import os
import time
import signal
import argparse
import threading
import serial
import queue

# Default settings
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200

# Global variables for thread management
stop_event = threading.Event()
sensor_data_queue = queue.Queue()

def signal_handler(sig, frame):
    """Handle interrupt signals (Ctrl+C)"""
    print("\nStopping test...")
    stop_event.set()

def sensor_reader_thread(port, baud_rate):
    """Thread to continuously read sensor data from Arduino"""
    try:
        # Connect to Arduino
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Sensor reader connected to Arduino on {port}")
        ser.reset_input_buffer()
        
        # Send a command to request sensor data
        ser.write(b'SENSOR\r\n')
        
        last_data_time = 0
        
        while not stop_event.is_set():
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # Only process sensor data lines
                        if line.startswith("SENS:"):
                            # Get the current time for timestamping
                            now = time.time()
                            
                            # Put the data in the queue for processing by the main thread
                            sensor_data_queue.put((now, line))
                            
                            # Track interval between readings
                            if last_data_time > 0:
                                interval = now - last_data_time
                                if interval > 0.7:  # More than 700ms between readings
                                    print(f"WARNING: Slow sensor data interval: {interval:.1f}s")
                            
                            last_data_time = now
                            
                            # Request a new sensor reading every 2 seconds
                            if not sensor_data_queue.empty() and sensor_data_queue.qsize() < 2:
                                ser.write(b'SENSOR\r\n')
                                
                except UnicodeDecodeError:
                    pass  # Ignore decode errors
            
            time.sleep(0.01)  # Small delay to prevent CPU hogging
            
    except serial.SerialException as e:
        print(f"Serial error in sensor reader: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.write(b'STOP\r\n')
            time.sleep(0.1)
            ser.close()
            print("Sensor reader connection closed")

def main():
    """Main function to run the integrated test"""
    parser = argparse.ArgumentParser(description="Test AprilTag with Sensor Data Integration")
    parser.add_argument('--port', type=str, default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD_RATE, 
                        help=f'Baud rate (default: {DEFAULT_BAUD_RATE})')
    parser.add_argument('--duration', type=int, default=0,
                        help='Duration in seconds (default: 0 for indefinite)')
    args = parser.parse_args()
    
    # Register signal handler for clean shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    # Start sensor reader thread
    reader_thread = threading.Thread(target=sensor_reader_thread, 
                                     args=(args.port, args.baud))
    reader_thread.daemon = True
    reader_thread.start()
    
    print("Started sensor reader thread")
    print("Waiting for sensor data...")
    print("Press Ctrl+C to stop\n")
    
    start_time = time.time()
    
    # Process and display sensor data from the queue
    try:
        # Run for the specified duration or indefinitely
        while args.duration == 0 or (time.time() - start_time < args.duration):
            try:
                # Try to get data from queue with timeout
                timestamp, data = sensor_data_queue.get(timeout=1)
                
                # Process and display the data
                print(f"[{timestamp - start_time:.1f}s] {data}")
                
                # Parse the sensor data
                try:
                    parts = data[5:].split(',')  # Remove "SENS:" prefix and split by comma
                    values = {}
                    for part in parts:
                        if ':' in part:
                            sensor, value = part.split(':')
                            values[sensor] = float(value)
                    
                    # Display parsed values
                    if values:
                        print("  Parsed values:")
                        print(f"    Front sensors: FL={values.get('FL', 'N/A')}cm, "
                              f"F={values.get('F', 'N/A')}cm, "
                              f"FR={values.get('FR', 'N/A')}cm")
                        print(f"    Back sensors:  BL={values.get('BL', 'N/A')}cm, "
                              f"B={values.get('B', 'N/A')}cm, "
                              f"BR={values.get('BR', 'N/A')}cm")
                    
                    # Simulate how this would integrate with apriltag_recognition.py
                    print("  Obstacle detection simulation:")
                    
                    # Calculate minimum distances in each direction
                    front_min = min(values.get('FL', 999), values.get('F', 999), values.get('FR', 999))
                    back_min = min(values.get('BL', 999), values.get('B', 999), values.get('BR', 999))
                    left_min = min(values.get('FL', 999), values.get('BL', 999))
                    right_min = min(values.get('FR', 999), values.get('BR', 999))
                    
                    # Check for obstacles and show possible movement limitations
                    critical_distance = 10.0  # cm
                    slow_distance = 30.0      # cm
                    
                    print(f"    Forward movement: {'BLOCKED' if front_min < critical_distance else 'SLOW' if front_min < slow_distance else 'OK'}")
                    print(f"    Backward movement: {'BLOCKED' if back_min < critical_distance else 'SLOW' if back_min < slow_distance else 'OK'}")
                    print(f"    Left movement: {'BLOCKED' if left_min < critical_distance else 'SLOW' if left_min < slow_distance else 'OK'}")
                    print(f"    Right movement: {'BLOCKED' if right_min < critical_distance else 'SLOW' if right_min < slow_distance else 'OK'}")
                    
                except Exception as e:
                    print(f"  Error parsing sensor data: {e}")
                
                # Mark task as done
                sensor_data_queue.task_done()
                
            except queue.Empty:
                # No data in queue, just continue
                pass
            
            # Check if we should stop
            if stop_event.is_set():
                break
        
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        stop_event.set()
        reader_thread.join(timeout=1.0)
        print("Test completed")

if __name__ == "__main__":
    main()
