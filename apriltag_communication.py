"""
Simple AprilTag Communication Module
Handles serial communication with Arduino for robot movement control
"""

import serial
import time
import logging
from dataclasses import dataclass
from typing import Optional, Tuple

# Configure logging
logging.basicConfig(level=logging.INFO, 
                   format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Serial communication settings
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200
RETRY_COUNT = 3
RETRY_DELAY = 0.1

@dataclass
class TagData:
    """Simple container for AprilTag detection data"""
    tag_id: int
    distance: float
    direction: str
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()


class ArduinoCommunicator:
    """Handles serial communication with Arduino"""
    
    def __init__(self, port: str = DEFAULT_PORT, baud_rate: int = DEFAULT_BAUD_RATE):
        """Initialize the communicator with given port and baud rate"""
        self.port = port
        self.baud_rate = baud_rate
        self.serial = None
        self.connected = False
        self.last_tag = None
        self.last_position = None
    
    def connect(self) -> bool:
        """Establish connection to Arduino"""
        try:
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(1)  # Allow Arduino to reset
            self.connected = True
            logger.info(f"Connected to Arduino on {self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            self.connected = False
            return False
    
    def disconnect(self) -> None:
        """Close the serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            logger.info("Disconnected from Arduino")
        self.connected = False
    
    def _send_command(self, command: str) -> bool:
        """Send a command to Arduino with retry logic"""
        if not self.connected or not self.serial:
            logger.warning("Cannot send command: not connected")
            return False
        
        for attempt in range(RETRY_COUNT):
            try:
                self.serial.write(command.encode())
                self.serial.flush()
                return True
            except Exception as e:
                logger.warning(f"Send failed (attempt {attempt+1}/{RETRY_COUNT}): {e}")
                time.sleep(RETRY_DELAY)
        
        logger.error("Failed to send command after retries")
        self.connected = False
        return False
    
    def send_tag_data(self, tag_data: TagData) -> bool:
        """Send tag detection data to Arduino"""
        if not self.connected:
            return False
        
        # Format: TAG:id,distance,direction
        command = f"TAG:{tag_data.tag_id},{tag_data.distance:.1f},{tag_data.direction}\n"
        success = self._send_command(command)
        
        if success:
            self.last_tag = tag_data
            
        return success
    
    def send_position(self, x: float, y: float, theta: float) -> bool:
        """Send position data to Arduino"""
        if not self.connected:
            return False
        
        # Format: POS:x,y,theta
        command = f"POS:{x:.1f},{y:.1f},{theta:.3f}\n"
        success = self._send_command(command)
        
        if success:
            self.last_position = (x, y, theta)
            
        return success
    
    def send_stop(self) -> bool:
        """Send stop command to Arduino"""
        return self._send_command("STOP\n")
    
    def send_clear(self) -> bool:
        """Send clear tag data command to Arduino"""
        return self._send_command("CLEAR\n")


def main():
    """Demo usage of the ArduinoCommunicator class"""
    communicator = ArduinoCommunicator()
    if communicator.connect():
        # Send test commands
        tag = TagData(tag_id=1, distance=50.0, direction='F')
        communicator.send_tag_data(tag)
        time.sleep(1)
        communicator.send_position(100, 200, 1.5)
        time.sleep(1)
        communicator.send_stop()
        communicator.disconnect()
    else:
        logger.error("Demo failed: Could not connect to Arduino")


if __name__ == "__main__":
    main()
