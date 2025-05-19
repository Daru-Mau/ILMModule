#!/usr/bin/env python3
"""
UART Communication Interface for Raspberry Pi 4 and Arduino Mega
This module provides reliable serial communication between the Raspberry Pi
and Arduino using the UART protocol with error handling and retry mechanisms.
"""

import serial
import time
import logging
import glob
from dataclasses import dataclass
from typing import Optional, Dict, Any, List, Tuple

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# UART/Serial communication settings
DEFAULT_UART_PORT = '/dev/ttyACM0'  # Common port for Arduino Mega on RPi
DEFAULT_BAUD_RATE = 115200
DEFAULT_TIMEOUT = 1.0
RETRY_COUNT = 3
RETRY_DELAY = 0.1
HANDSHAKE_TIMEOUT = 3.0  # Timeout for initial connection handshake

# Common Arduino serial ports to try (in order of preference)
POSSIBLE_PORTS = [
    '/dev/ttyACM0',  # Most common for Arduino Mega
    '/dev/ttyACM1',
    '/dev/ttyUSB0',  # For Arduino with FTDI or CH340 USB-to-serial adapters
    '/dev/ttyUSB1',
    '/dev/serial0',  # Hardware UART on Raspberry Pi GPIO pins
    '/dev/ttyS0'     # Alternative name for hardware UART
]

# Command protocol constants
CMD_TAG = "TAG"        # Send tag data
CMD_POSITION = "POS"   # Send position data
CMD_STOP = "STOP"      # Stop robot
CMD_TEST = "TEST"      # Test connection
CMD_PING = "PING"      # Quick ping
CMD_CLEAR = "CLEAR"    # Clear current movement
CMD_SPEED = "SPEED"    # Set motor speeds
CMD_MOTORS = "MOTORS"  # Direct motor control
CMD_SENSOR = "SENSOR"  # Request sensor data

# Message framing characters
MSG_START = "<"        # Start marker for command framing
MSG_END = ">"          # End marker for command framing
ACK = "ACK"            # Acknowledgment response
NACK = "NACK"          # Negative acknowledgment

# Direction code constants
DIR_STOP = 0           # Stop movement
DIR_FORWARD = 1        # Move forward
DIR_BACKWARD = 2       # Move backward
DIR_LEFT = 3           # Move left
DIR_RIGHT = 4          # Move right

def get_direction_name(direction_code: int) -> str:
    """
    Convert numeric direction code to readable name
    
    Args:
        direction_code: Integer direction code (0-4)
        
    Returns:
        String representation of direction
    """
    direction_map = {
        DIR_STOP: "STOP",
        DIR_FORWARD: "FORWARD",
        DIR_BACKWARD: "BACKWARD",
        DIR_LEFT: "LEFT",
        DIR_RIGHT: "RIGHT"
    }
    return direction_map.get(direction_code, f"UNKNOWN({direction_code})")

def constrain(value: int, min_val: int, max_val: int) -> int:
    """
    Constrain a value to be within the specified range
    Similar to Arduino's constrain() function
    
    Args:
        value: The value to constrain
        min_val: Minimum allowed value
        max_val: Maximum allowed value
        
    Returns:
        Value constrained to the range [min_val, max_val]
    """
    return max(min_val, min(max_val, value))


@dataclass
class TagData:
    """Container for AprilTag detection data"""
    tag_id: int
    distance: float
    direction: int  # 0=STOP, 1=FORWARD, 2=BACKWARD, 3=LEFT, 4=RIGHT
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
    
    def __str__(self) -> str:
        """String representation for debugging"""
        direction_name = get_direction_name(self.direction)
        return f"Tag({self.tag_id}): {self.distance:.1f}cm, Dir={direction_name}"


@dataclass
class SensorData:
    """Container for ultrasonic sensor readings"""
    front: float
    front_left: float
    front_right: float
    back: float
    back_left: float
    back_right: float
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
    
    def __str__(self) -> str:
        """String representation for debugging"""
        return (f"Sensors: F={self.front:.1f}cm, FL={self.front_left:.1f}cm, "
                f"FR={self.front_right:.1f}cm, B={self.back:.1f}cm, "
                f"BL={self.back_left:.1f}cm, BR={self.back_right:.1f}cm")


def find_arduino_port() -> str:
    """
    Automatically detect available Arduino serial ports
    Returns the first available port or DEFAULT_UART_PORT if none found
    """
    # First try using glob to find ports
    arduino_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    
    if arduino_ports:
        logger.info(f"Found potential Arduino ports: {arduino_ports}")
        for port in arduino_ports:
            try:
                ser = serial.Serial(port, DEFAULT_BAUD_RATE, timeout=0.5)
                ser.close()
                logger.info(f"Found active port: {port}")
                return port
            except (serial.SerialException, OSError):
                pass
    
    # If glob failed, try the predefined list
    logger.info("No Arduino ports found with glob, trying predefined list")
    for port in POSSIBLE_PORTS:
        try:
            ser = serial.Serial(port, DEFAULT_BAUD_RATE, timeout=0.5)
            ser.close()
            logger.info(f"Found active port: {port}")
            return port
        except (serial.SerialException, OSError):
            pass
            
    logger.error("No active Arduino ports found")
    return DEFAULT_UART_PORT  # Return default as fallback


class UARTCommunicator:
    """
    Handles UART/Serial communication with Arduino with improved reliability
    Features:
    - Automatic port detection
    - Robust connection handling
    - Command retry logic
    - Response parsing
    - Statistics tracking
    """
    
    def __init__(self, port: str = None, baud_rate: int = DEFAULT_BAUD_RATE, debug: bool = False):
        """Initialize the UART communicator"""
        self.port = port if port else DEFAULT_UART_PORT
        self.baud_rate = baud_rate
        self.serial = None
        self.connected = False
        self.last_tag = None
        self.last_sensor_data = None
        self.last_position = None
        self.last_command_time = 0
        self.command_count = 0
        self.command_history = {}  # Track success rate of different commands
        self.debug = debug
        
        if self.debug:
            logger.setLevel(logging.DEBUG)
            
        # Auto-connect if port is specified
        if port:
            self.connect()
    
    def connect(self) -> bool:
        """
        Establish connection to Arduino with proper initialization
        Returns True if connection successful, False otherwise
        """
        # Auto-detect port if using default
        if self.port == DEFAULT_UART_PORT:
            detected_port = find_arduino_port()
            if detected_port != DEFAULT_UART_PORT:
                logger.info(f"Auto-detected Arduino port: {detected_port}")
                self.port = detected_port
        
        # Try to connect to the selected port
        try:
            # Close any existing connection first
            if self.serial and self.serial.is_open:
                self.serial.close()
                time.sleep(0.5)
                
            logger.info(f"Connecting to Arduino on {self.port} at {self.baud_rate} baud...")
            
            # Open new connection
            self.serial = serial.Serial(
                self.port, 
                self.baud_rate, 
                timeout=DEFAULT_TIMEOUT,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            # Wait for Arduino to reset and initialize
            time.sleep(2)
            
            # Clear any initialization data
            if self.serial.in_waiting:
                self.serial.read(self.serial.in_waiting)
            
            # Try handshake sequences
            if self._handshake():
                self.connected = True
                logger.info(f"Successfully connected to Arduino on {self.port}")
                return True
            
            # If we got here, handshake failed but port is open
            if self.serial and self.serial.is_open:
                logger.warning(f"Connected to {self.port} but handshake failed")
                logger.warning("Will try to continue anyway")
                self.connected = True
                return True
                
            # If we got here, connection completely failed
            logger.error(f"Failed to connect to Arduino on {self.port}")
            if self.serial and self.serial.is_open:
                self.serial.close()
            self.connected = False
            return False
            
        except serial.SerialException as e:
            logger.error(f"Serial error on {self.port}: {e}")
            self.connected = False
            return False
            
        except Exception as e:
            logger.error(f"Unexpected error during connection: {e}")
            self.connected = False
            return False
    
    def _handshake(self) -> bool:
        """
        Perform handshake with Arduino to verify communication
        Tries multiple handshake methods
        """
        # Try PING command
        if self._send_raw_command(f"{MSG_START}{CMD_PING}{MSG_END}"):
            response = self._get_response(timeout=1.0)
            if ACK in response:
                logger.info("Handshake successful with PING command")
                return True
        
        # Try TEST command
        if self._send_raw_command(f"{MSG_START}{CMD_TEST}{MSG_END}"):
            response = self._get_response(timeout=1.5)  # Longer timeout for TEST
            if len(response) > 0:
                logger.info("Handshake successful with TEST command")
                return True
        
        # Try with a simple newline to trigger Arduino's serial event
        if self._send_raw_command("\r\n"):
            time.sleep(0.5)  # Give Arduino time to respond
            if self.serial.in_waiting > 0:
                response = self._get_response(timeout=0.5)
                if len(response) > 0:
                    logger.info("Received response from Arduino after newline")
                    return True
        
        return False
    
    def disconnect(self) -> None:
        """Close the serial connection cleanly"""
        try:
            if self.connected and self.serial and self.serial.is_open:
                # Send stop command before disconnecting
                self._send_raw_command(f"{MSG_START}{CMD_STOP}{MSG_END}")
                time.sleep(0.1)
                self.serial.close()
                logger.info("Disconnected from Arduino")
        except Exception as e:
            logger.error(f"Error during disconnect: {e}")
        finally:
            self.connected = False
            self.serial = None
    
    def _send_raw_command(self, command: str) -> bool:
        """
        Send a raw command to Arduino (internal use)
        Returns True if command was sent, False otherwise
        """
        if not self.connected or not self.serial or not self.serial.is_open:
            return False
        
        # Ensure proper line endings (\r\n)
        if not command.endswith('\r\n'):
            command = command.rstrip('\n\r') + '\r\n'
            
        try:
            self.serial.write(command.encode())
            self.serial.flush()
            self.last_command_time = time.time()
            self.command_count += 1
            return True
        except Exception as e:
            logger.error(f"Send error: {e}")
            return False
    
    def _send_command(self, command: str) -> bool:
        """
        Send a command with retry logic
        Returns True if command was sent successfully, False otherwise
        """
        if not self.connected:
            return False
        
        # Track command type for diagnostics
        cmd_type = command.split(':')[0] if ':' in command else command.split('\r')[0]
        if cmd_type not in self.command_history:
            self.command_history[cmd_type] = {'sent': 0, 'failed': 0}
        
        self.command_history[cmd_type]['sent'] += 1
        
        # Try sending with retries
        for attempt in range(RETRY_COUNT):
            if self._send_raw_command(command):
                return True
            else:
                self.command_history[cmd_type]['failed'] += 1
                time.sleep(RETRY_DELAY)
                
        # All retries failed
        logger.error(f"Failed to send command after {RETRY_COUNT} attempts: {command.strip()}")
        self.connected = False
        return False
    
    def _get_response(self, timeout=0.5) -> str:
        """
        Get response from Arduino with efficient timeout handling
        Returns the response as a string, or empty string if no response
        """
        if not self.connected or not self.serial:
            return ""
        
        start_time = time.time()
        response = ""
        
        while (time.time() - start_time) < timeout:
            if self.serial.in_waiting > 0:
                try:
                    line = self.serial.readline().decode('ascii', errors='ignore').strip()
                    if line:
                        response += line + " "
                        # If we got a complete response, no need to wait further
                        if ACK in line or "ready" in line.lower():
                            break
                except Exception:
                    pass  # Ignore decode errors
            else:
                # Only sleep if no data available
                time.sleep(0.01)
        
        return response.strip()
    
    def _parse_sensor_data(self, data_str: str) -> Optional[SensorData]:
        """
        Parse sensor data from Arduino response string
        Returns SensorData object if successful, None otherwise
        """
        try:
            # Expected format: "SENSORS:front,front_left,front_right,back,back_left,back_right"
            if "SENSORS:" not in data_str:
                return None
                
            values_str = data_str.split("SENSORS:")[1].strip()
            values = [float(v) for v in values_str.split(",")]
            
            if len(values) >= 6:
                return SensorData(
                    front=values[0],
                    front_left=values[1],
                    front_right=values[2],
                    back=values[3],
                    back_left=values[4],
                    back_right=values[5]
                )
            return None
        except Exception as e:
            logger.error(f"Error parsing sensor data: {e}")
            return None
    
    def send_test_command(self) -> bool:
        """
        Send diagnostic test command
        Returns True if test was successful, False otherwise
        """
        command = f"{MSG_START}{CMD_TEST}{MSG_END}"
        success = self._send_command(command)
        if not success:
            return False
            
        response = self._get_response(1.0)  # Longer timeout for test command
        return len(response) > 0
    
    def ping(self) -> bool:
        """
        Ping Arduino to check if it's responsive
        Returns True if Arduino responded, False otherwise
        """
        if not self.connected:
            return False
            
        # Clear buffer before ping
        if self.serial and self.serial.in_waiting:
            self.serial.read(self.serial.in_waiting)
            
        command = f"{MSG_START}{CMD_PING}{MSG_END}"
        success = self._send_command(command)
        if not success:
            return False
            
        response = self._get_response(0.5)
        return ACK in response
    
    def send_tag_data(self, tag_data: TagData) -> bool:
        """
        Send tag detection data
        Returns True if command was sent successfully, False otherwise
        """
        if not self.connected:
            return False
        
        # Format: <TAG:id,distance,direction>
        command = f"{MSG_START}{CMD_TAG}:{tag_data.tag_id},{tag_data.distance},{tag_data.direction}{MSG_END}"
        success = self._send_command(command)
        
        if success:
            self.last_tag = tag_data
            
        return success
    
    def send_position(self, x: float, y: float, theta: float) -> bool:
        """
        Send position data to Arduino
        Returns True if command was sent successfully, False otherwise
        """
        if not self.connected:
            return False
        
        # Format: <POS:x,y,theta>
        command = f"{MSG_START}{CMD_POSITION}:{x:.1f},{y:.1f},{theta:.3f}{MSG_END}"
        success = self._send_command(command)
        
        if success:
            self.last_position = (x, y, theta)
            
        return success
    
    def send_stop(self) -> bool:
        """
        Send stop command to Arduino
        Returns True if command was sent successfully, False otherwise
        """
        command = f"{MSG_START}{CMD_STOP}{MSG_END}"
        return self._send_command(command)
    
    def send_movement(self, direction: int, speed: Optional[int] = None) -> bool:
        """
        Send movement command using direction codes
        
        Args:
            direction: Direction code (0-4)
                0=STOP, 1=FORWARD, 2=BACKWARD, 3=LEFT, 4=RIGHT
            speed: Optional motor speed (0-255)
            
        Returns:
            True if command acknowledged, False otherwise
        """
        # Validate direction
        if direction < DIR_STOP or direction > DIR_RIGHT:
            logger.error(f"Invalid direction code: {direction}")
            return False
        
        # Format command
        if speed is not None:
            command = f"{MSG_START}MOVE:{direction},{speed}{MSG_END}"
        else:
            command = f"{MSG_START}MOVE:{direction}{MSG_END}"
            
        # Send command
        success = self._send_command(command)
        
        # Get response with timeout
        if success:
            response = self._get_response(0.5)
            return response is not None and "ACK" in response
        
        return False
    
    def send_clear(self) -> bool:
        """
        Send clear tag data command
        Returns True if command was sent successfully, False otherwise
        """
        command = f"{MSG_START}{CMD_CLEAR}{MSG_END}"
        return self._send_command(command)
    
    def set_speed(self, max_speed: int, min_speed: int) -> bool:
        """
        Set motor speed parameters within safe ranges
        Returns True if command was sent successfully, False otherwise
        """
        # Validate speed parameters
        max_speed = max(50, min(255, max_speed))
        min_speed = max(30, min(max_speed, min_speed))
        
        # Format: <SPEED:max_speed,min_speed>
        command = f"{MSG_START}{CMD_SPEED}:{max_speed},{min_speed}{MSG_END}"
        success = self._send_command(command)
        
        if success:
            logger.info(f"Speed parameters set: MAX={max_speed}, MIN={min_speed}")
        
        return success
    
    def send_motor_command(self, left: int, right: int, back: int) -> bool:
        """
        Send direct motor control command
        Returns True if command was sent successfully, False otherwise
        """
        # Validate motor speeds
        left = constrain(left, -255, 255)
        right = constrain(right, -255, 255)
        back = constrain(back, -255, 255)
        
        command = f"{MSG_START}{CMD_MOTORS}:{left},{right},{back}{MSG_END}"
        return self._send_command(command)
        
    def request_sensor_data(self) -> Optional[SensorData]:
        """
        Request sensor data from Arduino
        
        Returns:
            SensorData object if successful, None otherwise
        """
        # Send sensor data request
        command = f"{MSG_START}{CMD_SENSOR}{MSG_END}"
        success = self._send_command(command)
        
        if not success:
            return None
            
        # Wait for response
        response = self._get_response(0.5)
        if not response:
            return None
            
        # Try to parse sensor data from response
        sensor_data = self._parse_sensor_data(response)
        return sensor_data
    
    def get_connection_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the connection
        
        Returns:
            Dictionary with connection statistics
        """
        stats = {
            "connected": self.connected,
            "port": self.port,
            "baud_rate": self.baud_rate,
            "commands_sent": self.command_count,
            "last_command_time": self.last_command_time,
        }
        
        # Add command history stats
        if self.command_history:
            stats["command_history"] = {}
            for cmd, data in self.command_history.items():
                success_rate = 0
                if data["sent"] > 0:
                    success_rate = ((data["sent"] - data["failed"]) / data["sent"]) * 100
                stats["command_history"][cmd] = {
                    "sent": data["sent"],
                    "failed": data["failed"],
                    "success_rate": f"{success_rate:.1f}%"
                }
        
        return stats


# Helper function to test connections on multiple ports
def try_connect_all_ports() -> Optional[UARTCommunicator]:
    """
    Try to connect to Arduino on all possible ports
    Returns UARTCommunicator object if successful, None otherwise
    """
    for port in POSSIBLE_PORTS:
        logger.info(f"Trying to connect on {port}...")
        comm = UARTCommunicator(port=port)
        if comm.connect():
            logger.info(f"Successfully connected on {port}")
            return comm
    
    logger.error("Failed to connect on any port")
    return None


# Example usage
def main():
    """Test the UARTCommunicator with common commands"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Test UART communication with Arduino')
    parser.add_argument('--port', default=None, help='Serial port (default: auto-detect)')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD_RATE, help='Baud rate')
    parser.add_argument('--interactive', '-i', action='store_true', help='Run in interactive mode')
    parser.add_argument('--try-all', '-a', action='store_true', help='Try all possible ports')
    args = parser.parse_args()
    
    # Create communicator
    if args.try_all:
        communicator = try_connect_all_ports()
        if not communicator:
            logger.error("Failed to connect on any port")
            return
    else:
        communicator = UARTCommunicator(port=args.port, baud_rate=args.baud)
        if not communicator.connect():
            logger.error("Failed to connect to Arduino")
            return
    
    try:
        if args.interactive:
            # Interactive test mode
            print("\nUART Communication Tester")
            print("--------------------------")
            print("1. Send TEST command")
            print("2. Send TAG command (simulated detection)")
            print("3. Send STOP command")
            print("4. Set motor speeds")
            print("5. Get sensor data")
            print("6. Send direct motor command")
            print("7. Show communication stats")
            print("q. Quit")
            
            while True:
                choice = input("\nEnter choice (1-7, q): ").strip().lower()
                
                if choice == 'q':
                    break
                elif choice == '1':
                    success = communicator.send_test_command()
                    print(f"Test command: {'Success' if success else 'Failed'}")
                elif choice == '2':
                    tag_id = int(input("Tag ID (default=1): ") or "1")
                    distance = float(input("Distance (default=50): ") or "50")
                    print("Direction options:")
                    print("0 = STOP, 1 = FORWARD, 2 = BACKWARD, 3 = LEFT, 4 = RIGHT")
                    direction = int(input("Direction (0-4) (default=1): ") or "1")
                    if direction < 0 or direction > 4:
                        print("Invalid direction! Using 1 (FORWARD)")
                        direction = 1
                        
                    tag = TagData(tag_id=tag_id, distance=distance, direction=direction)
                    success = communicator.send_tag_data(tag)
                    print(f"Tag command: {'Success' if success else 'Failed'}")
                elif choice == '3':
                    success = communicator.send_stop()
                    print(f"Stop command: {'Success' if success else 'Failed'}")
                elif choice == '4':
                    max_speed = int(input("Max speed (50-255) (default=200): ") or "200")
                    min_speed = int(input("Min speed (30-max) (default=100): ") or "100")
                    success = communicator.set_speed(max_speed, min_speed)
                    print(f"Set speed: {'Success' if success else 'Failed'}")
                elif choice == '5':
                    sensor_data = communicator.get_sensor_data()
                    if sensor_data:
                        print(f"Sensor data: {sensor_data}")
                    else:
                        print("Failed to get sensor data")
                elif choice == '6':
                    left = int(input("Left motor (-255 to 255, default=0): ") or "0")
                    right = int(input("Right motor (-255 to 255, default=0): ") or "0")
                    back = int(input("Back motor (-255 to 255, default=0): ") or "0")
                    success = communicator.send_motor_command(left, right, back)
                    print(f"Motor command: {'Success' if success else 'Failed'}")
                elif choice == '7':
                    stats = communicator.get_stats()
                    print("\nCommunication Stats:")
                    print(f"Connected: {stats['connected']}")
                    print(f"Port: {stats['port']}")
                    print(f"Baud rate: {stats['baud_rate']}")
                    print(f"Commands sent: {stats['commands_sent']}")
                    print(f"Command history: {stats['command_history']}")
                else:
                    print("Invalid choice!")
        else:
            # Automated test sequence
            print("Running automated test sequence...")
            
            # Test basic commands
            print("\n1. Testing TEST command")
            test_success = communicator.send_test_command()
            print(f"TEST command: {'Success' if test_success else 'Failed'}")
            time.sleep(0.5)
            
            # Send a TAG command
            print("\n2. Testing TAG command")
            tag = TagData(tag_id=1, distance=200.0, direction=1)  # 1 = FORWARD
            tag_success = communicator.send_tag_data(tag)
            print(f"TAG command: {'Success' if tag_success else 'Failed'}")
            time.sleep(1.0)
            
            # Send STOP command
            print("\n3. Testing STOP command")
            stop_success = communicator.send_stop()
            print(f"STOP command: {'Success' if stop_success else 'Failed'}")
            time.sleep(0.5)
            
            # Set motor speeds
            print("\n4. Testing SPEED command")
            speed_success = communicator.set_speed(200, 100)
            print(f"SPEED command: {'Success' if speed_success else 'Failed'}")
            
            # Get sensor data
            print("\n5. Testing SENSOR command")
            sensor_data = communicator.get_sensor_data()
            if sensor_data:
                print(f"Sensor data: {sensor_data}")
            else:
                print("Failed to get sensor data")
            
            print("\nTest sequence complete!")
            
    finally:
        # Ensure we always disconnect properly
        communicator.send_stop()
        communicator.disconnect()


if __name__ == "__main__":
    main()
