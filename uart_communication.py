#!/usr/bin/env python3
"""
UART Communication Interface for Raspberry Pi 4 and Arduino Mega
This module provides reliable serial communication between the Raspberry Pi
and Arduino using the UART protocol with error handling and retry mechanisms.
"""

import serial
import time
import logging
from dataclasses import dataclass
from typing import Optional, Dict, Any

# Configure logging
logging.basicConfig(level=logging.INFO,
                   format='%(asctime)s [%(levelname)s] %(message)s',
                   datefmt='%H:%M:%S')
logger = logging.getLogger(__name__)

# UART Communication constants
BAUD_RATE = 115200
DEFAULT_UART_PORT = '/dev/ttyUSB0'
MSG_START = '<'
MSG_END = '>'
CMD_MOVE = 'MOV'
CMD_SPEED = 'SPEED'
CMD_STOP = 'STOP'
CMD_PING = 'PING'
CMD_SENS = 'SENS'

# Direction codes
DIR_STOP = 0
DIR_FORWARD = 1
DIR_BACKWARD = 2
DIR_LEFT = 3
DIR_RIGHT = 4

def get_direction_name(direction_code: int) -> str:
    """Convert direction code to readable string"""
    return {
        DIR_STOP: "STOP",
        DIR_FORWARD: "FORWARD",
        DIR_BACKWARD: "BACKWARD",
        DIR_LEFT: "LEFT",
        DIR_RIGHT: "RIGHT"
    }.get(direction_code, "UNKNOWN")

@dataclass
class TagData:
    """Container for movement data from tag tracking"""
    direction: int
    speed: int
    timestamp: float = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()

    def __str__(self) -> str:
        """String representation for debugging"""
        return f"Movement: Dir={get_direction_name(self.direction)}, Speed={self.speed}"

class UARTCommunicator:
    def __init__(self, port: str = None, baud_rate: int = BAUD_RATE, debug: bool = False):
        """Initialize UART communication"""
        self.port = port if port else DEFAULT_UART_PORT
        self.baud_rate = baud_rate
        self.serial = None
        self.connected = False
        self.debug = debug
        self.min_speed = 30  # Default minimum speed
        self.max_speed = 255 # Default maximum speed

    def connect(self) -> bool:
        """Establish serial connection with Arduino"""
        try:
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.connected = True
            logger.info(f"Connected to {self.port} at {self.baud_rate} baud")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Close the serial connection"""
        if self.serial:
            self.serial.close()
        self.connected = False

    def _send_command(self, command: str) -> bool:
        """Send a command to Arduino and wait for acknowledgment"""
        if not self.connected:
            return False

        try:
            self.serial.write(command.encode())
            self.serial.flush()
            
            if self.debug:
                logger.debug(f"Sent: {command}")
            
            # Wait for acknowledgment
            response = self.serial.readline().decode().strip()
            if response.startswith('<ACK:'):
                return True
            else:
                logger.warning(f"Unexpected response: {response}")
                return False

        except Exception as e:
            logger.error(f"Communication error: {e}")
            return False

    def send_movement(self, direction: int, speed: int) -> bool:
        """Send movement command with direction and speed"""
        if not self.connected:
            return False

        # If direction is STOP, speed should be 0
        if direction == DIR_STOP:
            speed = 0

        # Constrain speed to valid range
        if speed > 0:
            speed = max(self.min_speed, min(self.max_speed, speed))

        command = f"{MSG_START}MOV:{direction},{speed}{MSG_END}"
        return self._send_command(command)

    def send_tag_data(self, tag_data: TagData) -> bool:
        """Send movement data from tag tracking"""
        return self.send_movement(tag_data.direction, tag_data.speed)

    def set_speed(self, max_speed: int, min_speed: int) -> bool:
        """Set motor speed parameters"""
        # Validate speed parameters
        max_speed = max(50, min(255, max_speed))
        min_speed = max(30, min(max_speed, min_speed))
        
        # Store the values
        self.max_speed = max_speed
        self.min_speed = min_speed
        
        # Send to Arduino
        command = f"{MSG_START}SPEED:{max_speed},{min_speed}{MSG_END}"
        return self._send_command(command)

    def send_stop(self) -> bool:
        """Send stop command"""
        command = f"{MSG_START}STOP{MSG_END}"
        return self._send_command(command)

    def ping(self) -> bool:
        """Send ping command to check connection"""
        command = f"{MSG_START}PING{MSG_END}"
        return self._send_command(command)
