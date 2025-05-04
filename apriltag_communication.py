"""
AprilTag Communication System
This script handles the communication between the AprilTag recognition system and the Arduino robot.
It receives tag detection data and sends appropriate movement commands to the robot.
"""

import serial
import time

# === Serial Communication Settings ===
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 9600
SERIAL_TIMEOUT = 1
ARDUINO_RESET_DELAY = 2  # Seconds to wait for Arduino reset

# === Movement Commands ===
MOVEMENT_DIRECTIONS = {
    'F': 'Forward',
    'B': 'Backward',
    'L': 'Left',
    'R': 'Right',
    'S': 'Stop'
}


class ArduinoCommunicator:
    def __init__(self, port=DEFAULT_PORT, baud_rate=DEFAULT_BAUD_RATE):
        """
        Initialize the Arduino communication handler
        Args:
            port (str): Serial port for Arduino communication
            baud_rate (int): Baud rate for serial communication
        """
        self.port = port
        self.baud_rate = baud_rate
        self.arduino = None
        self.last_direction = None

    def connect(self):
        """
        Establish connection to Arduino via serial port
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.arduino = serial.Serial(
                self.port, self.baud_rate, timeout=SERIAL_TIMEOUT)
            time.sleep(ARDUINO_RESET_DELAY)  # Wait for Arduino to reset
            print("Arduino connected successfully")
            return True
        except Exception as e:
            print(f"Failed to connect to Arduino: {e}")
            return False

    def send_command(self, direction):
        """
        Send movement command to Arduino if it differs from last command
        Args:
            direction (str): Movement direction ('F', 'B', 'L', 'R', 'S')
        Returns:
            bool: True if command was sent, False otherwise
        """
        if not self.arduino or direction == self.last_direction:
            return False

        try:
            self.arduino.write(direction.encode())
            self.arduino.flush()
            self.last_direction = direction
            return True
        except Exception as e:
            print(f"Failed to send command: {e}")
            return False

    def process_tag_data(self, tag_id, distance_cm, direction):
        """
        Process tag detection data and send appropriate commands
        Args:
            tag_id (int): ID of the detected AprilTag
            distance_cm (float): Estimated distance to tag in centimeters
            direction (str): Calculated movement direction
        """
        if self.arduino:
            self.send_command(direction)

    def close(self):
        """
        Close the serial connection to Arduino
        """
        if self.arduino:
            self.arduino.close()
            self.arduino = None
            print("Arduino connection closed")
