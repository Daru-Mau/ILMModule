"""
AprilTag Communication System - Optimized Version
This script handles the communication between the AprilTag recognition system and the Arduino robot.
It receives tag detection data and sends appropriate movement commands to the robot.
"""

import serial
import time
import math
from typing import Optional, Tuple, Dict
from threading import Lock, Event
import logging
import queue
from dataclasses import dataclass
from contextlib import contextmanager

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# === Serial Communication Settings ===
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200  # Increased for better performance
SERIAL_TIMEOUT = 0.1  # Reduced for faster timeout handling
ARDUINO_RESET_DELAY = 1.0  # Reduced startup delay
COMMAND_QUEUE_SIZE = 50
RETRY_LIMIT = 3
RETRY_DELAY = 0.1

@dataclass
class TagData:
    tag_id: int
    x: float
    y: float
    yaw: float
    timestamp: float

class ArduinoCommunicator:
    def __init__(self, port: str = DEFAULT_PORT, baud_rate: int = DEFAULT_BAUD_RATE):
        """Initialize the Arduino communication interface with improved error handling"""
        self.port = port
        self.baud_rate = baud_rate
        self.arduino: Optional[serial.Serial] = None
        self.connected = False
        self.last_position = (0.0, 0.0, 0.0)
        self.command_queue = queue.Queue(maxsize=COMMAND_QUEUE_SIZE)
        self.lock = Lock()
        self.shutdown_event = Event()
        self.last_tag_data: Optional[TagData] = None
        self.retry_count: Dict[str, int] = {}
        self.try_connect()

    @contextmanager
    def serial_lock(self):
        """Thread-safe context manager for serial operations"""
        try:
            self.lock.acquire()
            yield
        finally:
            self.lock.release()

    def try_connect(self) -> bool:
        """Attempt to establish connection with Arduino with retry mechanism"""
        with self.serial_lock():
            try:
                if self.arduino:
                    self.arduino.close()
                
                self.arduino = serial.Serial(
                    self.port,
                    self.baud_rate,
                    timeout=SERIAL_TIMEOUT,
                    write_timeout=SERIAL_TIMEOUT
                )
                time.sleep(ARDUINO_RESET_DELAY)
                self.connected = True
                self.retry_count.clear()
                logger.info(f"Connected to Arduino on {self.port}")
                return True
            except Exception as e:
                logger.error(f"Failed to connect to Arduino: {e}")
                self.connected = False
                return False

    def _send_with_retry(self, command: str, command_type: str) -> bool:
        """Send command with retry mechanism"""
        if not self.connected:
            return False

        self.retry_count.setdefault(command_type, 0)
        
        with self.serial_lock():
            try:
                self.arduino.write(command.encode())
                self.arduino.flush()
                self.retry_count[command_type] = 0
                return True
            except Exception as e:
                logger.warning(f"Failed to send {command_type}: {e}")
                self.retry_count[command_type] += 1
                
                if self.retry_count[command_type] >= RETRY_LIMIT:
                    logger.error(f"Max retries reached for {command_type}")
                    self.connected = False
                    return False
                
                time.sleep(RETRY_DELAY)
                return self._send_with_retry(command, command_type)

    def process_tag_data(self, tag_id: int, distance: float, direction: str) -> bool:
        """Process and send tag data with optimized format"""
        if not self.connected:
            return False

        command = f"TAG:{tag_id},{distance:.1f},{direction}\n"
        return self._send_with_retry(command, "tag_data")

    def update_position(self, x: float, y: float, yaw: float) -> bool:
        """Send current position data with optimized precision"""
        if not self.connected:
            return False

        command = f"POS:{x:.1f},{y:.1f},{yaw:.2f}\n"
        if self._send_with_retry(command, "position"):
            self.last_position = (x, y, yaw)
            return True
        return False

    def clear_tag_data(self) -> bool:
        """Send command to clear tag data"""
        return self._send_with_retry("CLEAR\n", "clear")

    def send_command(self, command: str) -> bool:
        """Send a direct command with validation"""
        if not command in ['M', 'T', 'S', 'R']:
            logger.warning(f"Invalid command: {command}")
            return False
        return self._send_with_retry(f"{command}\n", "command")

    def read_response(self) -> Optional[str]:
        """Read response with timeout and error handling"""
        if not self.connected:
            return None

        with self.serial_lock():
            try:
                if self.arduino.in_waiting:
                    response = self.arduino.readline().decode().strip()
                    if response:
                        logger.debug(f"Arduino: {response}")
                        return response
                return None
            except Exception as e:
                logger.error(f"Failed to read from Arduino: {e}")
                self.connected = False
                return None

    def close(self):
        """Safely close the serial connection"""
        with self.serial_lock():
            if self.arduino:
                try:
                    self.arduino.close()
                except Exception as e:
                    logger.error(f"Error closing connection: {e}")
                finally:
                    self.connected = False
                    self.shutdown_event.set()

    def get_last_position(self) -> Tuple[float, float, float]:
        """Get the last known position (thread-safe)"""
        with self.serial_lock():
            return self.last_position
