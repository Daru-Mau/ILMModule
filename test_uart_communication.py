#!/usr/bin/env python3
"""
UART Communication Test

This script tests the UART communication module with the Arduino Mega.
"""

from uart_communication import UARTCommunicator, TagData, SensorData, get_direction_name
import time
import sys
import argparse
import glob

def test_uart_connection(port=None, debug=False, simulate=False):
    """Run test sequence to verify UART communication"""
    print("=== UART Communication Test ===")
    
    # Create communicator
    print(f"Connecting to Arduino{' on '+port if port else ''}...")
    
    if simulate:
        print("Running in simulation mode (no Arduino connected)")
        
        # Create a mock UARTCommunicator that mimics successful responses
        class MockUARTCommunicator:
            def __init__(self, port=None, debug=False):
                self.connected = True
                self.port = port or "/dev/ttyACM0"
                self.debug = debug
                
            def ping(self):
                return True
                
            def request_sensor_data(self):
                return SensorData(
                    front=50.0,
                    front_left=45.0,
                    front_right=55.0,
                    back=60.0,
                    back_left=65.0,
                    back_right=70.0
                )
                
            def send_movement(self, direction, speed=None):
                return True
                
            def send_tag_data(self, tag_data):
                return True
                
            def send_stop(self):
                return True
                
            def disconnect(self):
                pass
                
            def get_connection_stats(self):
                return {
                    "connected": True,
                    "port": self.port,
                    "baud_rate": 115200,
                    "commands_sent": 10,
                    "last_command_time": time.time(),
                    "command_history": {
                        "PING": {"sent": 2, "failed": 0, "success_rate": "100.0%"},
                        "MOVE": {"sent": 5, "failed": 0, "success_rate": "100.0%"},
                        "TAG": {"sent": 2, "failed": 0, "success_rate": "100.0%"},
                        "STOP": {"sent": 1, "failed": 0, "success_rate": "100.0%"}
                    }
                }
        
        comm = MockUARTCommunicator(port=port, debug=debug)
    else:
        # Real connection
        comm = UARTCommunicator(port=port, debug=debug)
        
        # If not connected automatically, try more ports
        if not comm.connected:
            print("Initial connection failed, attempting to find Arduino...")
            
            # List available ports for debugging
            ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
            if ports:
                print(f"Available ports: {', '.join(ports)}")
                for test_port in ports:
                    print(f"Trying to connect to {test_port}...")
                    comm.port = test_port
                    if comm.connect():
                        print(f"Successfully connected to {test_port}")
                        break
                    else:
                        print(f"Could not connect to {test_port}")
            else:
                print("No serial ports found")
            
            if not comm.connected:
                print("Error: Failed to connect to Arduino")
                return False
    
    print(f"Successfully connected to Arduino on {comm.port}")
    
    # Test ping
    print("\nTesting PING command...")
    if comm.ping():
        print("✓ Ping successful")
    else:
        print("✗ Ping failed")
    
    # Test sensor data
    print("\nRequesting sensor data...")
    sensor_data = comm.request_sensor_data()
    if sensor_data:
        print(f"✓ Received sensor data: {sensor_data}")
    else:
        print("✗ Failed to get sensor data")
    
    # Test movement commands
    print("\nTesting movement commands (2s each)...")
    for direction in range(5):
        dir_name = get_direction_name(direction)
        print(f"  Sending {dir_name} command...")
        response = comm.send_movement(direction=direction, speed=150)
        if response:
            print(f"  ✓ {dir_name} command successful")
        else:
            print(f"  ✗ {dir_name} command failed")
        time.sleep(2)
    
    # Test tag command
    print("\nTesting TAG command...")
    tag = TagData(tag_id=1, distance=150, direction=1)  # Forward
    if comm.send_tag_data(tag):
        print(f"✓ Tag command successful: {tag}")
    else:
        print(f"✗ Tag command failed")
    time.sleep(2)
    
    # Test stop command
    print("\nTesting STOP command...")
    if comm.send_stop():
        print("✓ Stop command successful")
    else:
        print("✗ Stop command failed")
    
    # Display connection stats
    print("\nConnection Statistics:")
    stats = comm.get_connection_stats()
    for key, value in stats.items():
        print(f"  {key}: {value}")
    
    # Clean up
    comm.disconnect()
    print("\nDisconnected from Arduino")
    return True

def interactive_test(port=None):
    """Interactive test mode for manual testing"""
    print("=== UART Interactive Test Mode ===")
    
    # Create communicator
    print(f"Connecting to Arduino{' on '+port if port else ''}...")
    comm = UARTCommunicator(port=port, debug=True)
    
    if not comm.connected:
        print("Error: Failed to connect to Arduino")
        return False
    
    print(f"Successfully connected to Arduino on {comm.port}")
    print("\nAvailable commands:")
    print("  ping - Send ping command")
    print("  stop - Send stop command")
    print("  move <direction> [speed] - Send movement command (0-4)")
    print("  tag <id> <distance> <direction> - Send tag command")
    print("  sensor - Request sensor data")
    print("  stats - Show connection statistics")
    print("  exit/quit - Exit program")
    
    while True:
        try:
            cmd = input("\nCommand> ").strip().lower()
            
            if cmd in ['exit', 'quit', 'q']:
                break
                
            if cmd == 'ping':
                if comm.ping():
                    print("✓ Ping successful")
                else:
                    print("✗ Ping failed")
                    
            elif cmd == 'stop':
                if comm.send_stop():
                    print("✓ Stop successful")
                else:
                    print("✗ Stop failed")
                    
            elif cmd.startswith('move'):
                parts = cmd.split()
                if len(parts) < 2:
                    print("Usage: move <direction> [speed]")
                    print("  direction: 0=STOP, 1=FORWARD, 2=BACKWARD, 3=LEFT, 4=RIGHT")
                    continue
                    
                try:
                    direction = int(parts[1])
                    speed = int(parts[2]) if len(parts) > 2 else 150
                    
                    if direction < 0 or direction > 4:
                        print("Invalid direction. Use 0-4.")
                        continue
                        
                    if comm.send_movement(direction=direction, speed=speed):
                        print(f"✓ Moving {get_direction_name(direction)} at speed {speed}")
                    else:
                        print("✗ Movement command failed")
                except ValueError:
                    print("Invalid number format")
                    
            elif cmd.startswith('tag'):
                parts = cmd.split()
                if len(parts) < 4:
                    print("Usage: tag <id> <distance> <direction>")
                    print("  direction: 0=STOP, 1=FORWARD, 2=BACKWARD, 3=LEFT, 4=RIGHT")
                    continue
                    
                try:
                    tag_id = int(parts[1])
                    distance = float(parts[2])
                    direction = int(parts[3])
                    
                    if direction < 0 or direction > 4:
                        print("Invalid direction. Use 0-4.")
                        continue
                        
                    tag = TagData(tag_id=tag_id, distance=distance, direction=direction)
                    if comm.send_tag_data(tag):
                        print(f"✓ Tag command sent: {tag}")
                    else:
                        print("✗ Tag command failed")
                except ValueError:
                    print("Invalid number format")
                    
            elif cmd == 'sensor':
                sensor_data = comm.request_sensor_data()
                if sensor_data:
                    print(f"Sensor data: {sensor_data}")
                else:
                    print("Failed to get sensor data")
                    
            elif cmd == 'stats':
                stats = comm.get_connection_stats()
                for key, value in stats.items():
                    print(f"  {key}: {value}")
                    
            else:
                print("Unknown command")
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    # Clean up
    comm.disconnect()
    print("Disconnected from Arduino")
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Test UART communication with Arduino')
    parser.add_argument('-p', '--port', help='Serial port to use')
    parser.add_argument('-i', '--interactive', action='store_true', help='Run in interactive mode')
    parser.add_argument('-d', '--debug', action='store_true', help='Enable debug output')
    args = parser.parse_args()
    
    if args.interactive:
        interactive_test(port=args.port)
    else:
        test_uart_connection(port=args.port, debug=args.debug)
