"""
AprilTag Communication System
This script handles the communication between the AprilTag recognition system and the Arduino robot.
It receives tag detection data and sends appropriate movement commands to the robot.
"""

import serial
import time
import math
from typing import Optional, Tuple

# === Serial Communication Settings ===
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200  # Matches Arduino baud rate
SERIAL_TIMEOUT = 1
ARDUINO_RESET_DELAY = 2  # Seconds to wait for Arduino reset


class ArduinoCommunicator:
    def __init__(self, port: str = DEFAULT_PORT, baud_rate: int = DEFAULT_BAUD_RATE):
        """Initialize the Arduino communication interface"""
        self.port = port
        self.baud_rate = baud_rate
        self.arduino: Optional[serial.Serial] = None
        self.connected = False
        self.last_position = (0.0, 0.0, 0.0)  # x, y, yaw
        self.try_connect()

    def try_connect(self) -> bool:
        """Attempt to establish connection with Arduino"""
        try:
            self.arduino = serial.Serial(
                self.port, self.baud_rate, timeout=SERIAL_TIMEOUT)
            time.sleep(ARDUINO_RESET_DELAY)  # Wait for Arduino reset
            self.connected = True
            print(f"Connected to Arduino on {self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect to Arduino: {e}")
            self.connected = False
            return False

    def send_tag_data(self, tag_id: int, x: float, y: float, yaw: float) -> bool:
        """
        Send AprilTag detection data to Arduino
        Args:
            tag_id (int): ID of the detected AprilTag
            x (float): X coordinate relative to tag in mm
            y (float): Y coordinate relative to tag in mm
            yaw (float): Yaw angle relative to tag in radians
        """
        if not self.connected:
            return False

        try:
            # Format: TAG:id,x,y,yaw
            command = f"TAG:{tag_id},{x},{y},{yaw}\n"
            self.arduino.write(command.encode())
            self.arduino.flush()
            return True
        except Exception as e:
            print(f"Failed to send tag data: {e}")
            self.connected = False
            return False

    def update_position(self, x: float, y: float, yaw: float) -> bool:
        """
        Send current position data to Arduino
        Args:
            x (float): Current X position in mm
            y (float): Current Y position in mm
            yaw (float): Current yaw angle in radians
        """
        if not self.connected:
            return False

        try:
            # Format: POS:x,y,yaw
            command = f"POS:{x},{y},{yaw}\n"
            self.arduino.write(command.encode())
            self.arduino.flush()
            self.last_position = (x, y, yaw)
            return True
        except Exception as e:
            print(f"Failed to send position data: {e}")
            self.connected = False
            return False

    def clear_tag_data(self) -> bool:
        """Send command to clear tag data when tag is lost"""
        if not self.connected:
            return False

        try:
            self.arduino.write(b"CLEAR\n")
            self.arduino.flush()
            return True
        except Exception as e:
            print(f"Failed to send clear command: {e}")
            self.connected = False
            return False

    def send_command(self, command: str) -> bool:
        """
        Send a direct command to Arduino
        Args:
            command (str): Command to send ('M' for mode switch, 'T' for sensor test, 
                          'S' for stop, 'R' for resume)
        """
        if not self.connected:
            return False

        try:
            self.arduino.write(f"{command}\n".encode())
            self.arduino.flush()
            return True
        except Exception as e:
            print(f"Failed to send command: {e}")
            self.connected = False
            return False

    def read_response(self) -> Optional[str]:
        """Read and process any responses from Arduino"""
        if not self.connected:
            return None

        try:
            if self.arduino.in_waiting:
                response = self.arduino.readline().decode().strip()
                print(f"Arduino: {response}")
                return response
            return None
        except Exception as e:
            print(f"Failed to read from Arduino: {e}")
            self.connected = False
            return None

    def close(self):
        """Close the serial connection"""
        if self.arduino:
            self.arduino.close()
            self.connected = False

    def get_last_position(self) -> Tuple[float, float, float]:
        """Get the last known position"""
        return self.last_position
