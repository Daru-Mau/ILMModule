"""
ILM Module Communication Interface
Centralized communication system for robot control via serial interface
"""

import serial
import time
import logging
from dataclasses import dataclass
from typing import Optional, Dict, Any

# Configure logging with cleaner format
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# Serial communication settings
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD_RATE = 115200
DEFAULT_TIMEOUT = 1.0
RETRY_COUNT = 3
RETRY_DELAY = 0.1

# Protocol constants - centralizing all command formats
CMD_TAG = "TAG"
CMD_POSITION = "POS"
CMD_STOP = "STOP"
CMD_TEST = "TEST"
CMD_PING = "PING"
CMD_CLEAR = "CLEAR"
CMD_SPEED = "SPEED"

@dataclass
class TagData:
    """Container for AprilTag detection data"""
    tag_id: int
    distance: float
    direction: str
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
    
    def __str__(self) -> str:
        """String representation for debugging"""
        return f"Tag({self.tag_id}): {self.distance:.1f}cm, Dir={self.direction}"


class ArduinoCommunicator:
    """Handles serial communication with Arduino with improved reliability"""
    
    def __init__(self, port: str = DEFAULT_PORT, baud_rate: int = DEFAULT_BAUD_RATE):
        """Initialize the communicator"""
        self.port = port
        self.baud_rate = baud_rate
        self.serial = None
        self.connected = False
        self.last_tag = None
        self.last_position = None
        self.last_command_time = 0
        self.command_count = 0
        self.command_history = {}  # Track success rate of different commands
    
    def connect(self) -> bool:
        """Establish connection to Arduino with proper initialization"""
        try:
            # Close any existing connection first
            if self.serial and self.serial.is_open:
                self.serial.close()
                time.sleep(0.5)
                
            # Open new connection
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=DEFAULT_TIMEOUT)
            # Wait for Arduino to reset and initialize
            time.sleep(2)
            
            # Clear any initialization data
            if self.serial.in_waiting:
                self.serial.read(self.serial.in_waiting)
                
            # Simple handshake to verify connection
            if self._send_raw_command("PING"):
                response = self._get_response(0.5)
                if response:
                    logger.info(f"Arduino connected on {self.port} at {self.baud_rate} baud")
                    self.connected = True
                    return True
                    
            # Try another approach - some Arduinos don't respond to PING
            if self._send_raw_command("TEST"):
                response = self._get_response(0.5)
                if response:
                    logger.info(f"Arduino connected (using TEST) on {self.port}")
                    self.connected = True
                    return True
            
            # Connection failed
            logger.error("Arduino handshake failed")
            if self.serial and self.serial.is_open:
                self.serial.close()
            self.connected = False
            return False
            
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            self.connected = False
            return False
    
    def disconnect(self) -> None:
        """Close the serial connection cleanly"""
        try:
            if self.connected and self.serial and self.serial.is_open:
                # Send stop command before disconnecting
                self._send_raw_command(CMD_STOP)
                time.sleep(0.1)
                self.serial.close()
                logger.info("Disconnected from Arduino")
        except Exception as e:
            logger.error(f"Error during disconnect: {e}")
        finally:
            self.connected = False
            self.serial = None
    
    def _send_raw_command(self, command: str) -> bool:
        """Send a raw command to Arduino (internal use)"""
        if not self.connected or not self.serial or not self.serial.is_open:
            return False
        
        # Ensure proper line endings (\r\n)
        if not command.endswith('\r\n'):
            command = command.rstrip('\n') + '\r\n'
            
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
        """Send a command with retry logic"""
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
        self.connected = False
        return False
    
    def _get_response(self, timeout=0.5) -> str:
        """Get response from Arduino with efficient timeout handling"""
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
                        if line.endswith('OK') or 'ready' in line.lower():
                            break
                except Exception:
                    pass  # Ignore decode errors
            else:
                # Only sleep if no data available
                time.sleep(0.01)
        
        return response.strip()
    
    def send_test_command(self) -> bool:
        """Send diagnostic test command"""
        success = self._send_command(f"{CMD_TEST}\n")
        if not success:
            return False
            
        response = self._get_response(1.0)  # Longer timeout for test command
        return len(response) > 0
    
    def ping(self) -> bool:
        """Ping Arduino to check if it's responsive"""
        if not self.connected:
            return False
            
        # Clear buffer before ping
        if self.serial and self.serial.in_waiting:
            self.serial.read(self.serial.in_waiting)
            
        success = self._send_command(f"{CMD_PING}\n")
        if not success:
            return False
            
        response = self._get_response(0.5)
        return len(response) > 0
    
    def send_tag_data(self, tag_data: TagData) -> bool:
        """Send tag detection data with optimized format"""
        if not self.connected:
            return False
        
        # Format: TAG:id,distance,direction
        command = f"{CMD_TAG}:{tag_data.tag_id},{tag_data.distance},{tag_data.direction}\n"
        success = self._send_command(command)
        
        if success:
            self.last_tag = tag_data
            
        return success
    
    def send_position(self, x: float, y: float, theta: float) -> bool:
        """Send position data to Arduino"""
        if not self.connected:
            return False
        
        # Format: POS:x,y,theta
        command = f"{CMD_POSITION}:{x:.1f},{y:.1f},{theta:.3f}\n"
        success = self._send_command(command)
        
        if success:
            self.last_position = (x, y, theta)
            
        return success
    
    def send_stop(self) -> bool:
        """Send stop command to Arduino"""
        return self._send_command(f"{CMD_STOP}\n")
    
    def send_clear(self) -> bool:
        """Send clear tag data command"""
        return self._send_command(f"{CMD_CLEAR}\n")
    
    def set_speed(self, max_speed: int, min_speed: int) -> bool:
        """Set motor speed parameters within safe ranges"""
        # Validate speed parameters
        max_speed = max(50, min(255, max_speed))
        min_speed = max(30, min(max_speed, min_speed))
        
        # Send speed command
        command = f"{CMD_SPEED}:{max_speed},{min_speed}\n"
        success = self._send_command(command)
        
        if success:
            logger.info(f"Speed parameters set: MAX={max_speed}, MIN={min_speed}")
        
        return success
    
    def get_stats(self) -> Dict[str, Any]:
        """Return communication statistics"""
        stats = {
            "connected": self.connected,
            "commands_sent": self.command_count,
            "last_command_time": self.last_command_time,
            "command_history": self.command_history,
        }
        return stats


# Simple demonstration function
def main():
    """Test the ArduinoCommunicator with common commands"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Test Arduino communication')
    parser.add_argument('--port', default=DEFAULT_PORT, help='Serial port')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD_RATE, help='Baud rate')
    parser.add_argument('--interactive', '-i', action='store_true', help='Run in interactive mode')
    args = parser.parse_args()
    
    # Create communicator
    comm = ArduinoCommunicator(args.port, args.baud)
    
    if not comm.connect():
        logger.error("Connection failed")
        return
    
    try:
        if args.interactive:
            # Interactive test mode
            print("\nArduino Communication Tester")
            print("--------------------------")
            print("1. Send TEST command")
            print("2. Send TAG command (simulated detection)")
            print("3. Send STOP command")
            print("4. Set motor speeds")
            print("5. Show communication stats")
            print("q. Quit")
            
            while True:
                choice = input("\nEnter choice (1-5, q): ").strip().lower()
                
                if choice == 'q':
                    break
                elif choice == '1':
                    success = comm.send_test_command()
                    print(f"Test command: {'Success' if success else 'Failed'}")
                elif choice == '2':
                    tag_id = int(input("Tag ID (default=1): ") or "1")
                    distance = float(input("Distance (default=50): ") or "50")
                    direction = input("Direction (F,B,L,R,S) (default=F): ").upper() or "F"
                    if direction not in 'FBLRS':
                        print("Invalid direction! Using F")
                        direction = 'F'
                        
                    tag = TagData(tag_id=tag_id, distance=distance, direction=direction)
                    success = comm.send_tag_data(tag)
                    print(f"Tag command: {'Success' if success else 'Failed'}")
                elif choice == '3':
                    success = comm.send_stop()
                    print(f"Stop command: {'Success' if success else 'Failed'}")
                elif choice == '4':
                    max_speed = int(input("Max speed (50-255) (default=200): ") or "200")
                    min_speed = int(input("Min speed (30-max) (default=100): ") or "100")
                    success = comm.set_speed(max_speed, min_speed)
                    print(f"Set speed: {'Success' if success else 'Failed'}")
                elif choice == '5':
                    stats = comm.get_stats()
                    print("\nCommunication Stats:")
                    print(f"Connected: {stats['connected']}")
                    print(f"Commands sent: {stats['commands_sent']}")
                    print(f"Command history: {stats['command_history']}")
                else:
                    print("Invalid choice!")
        else:
            # Automated test sequence
            print("Running automated test sequence...")
            
            # Test basic commands
            print("\n1. Testing TEST command")
            test_success = comm.send_test_command()
            print(f"TEST command: {'Success' if test_success else 'Failed'}")
            time.sleep(0.5)
            
            # Send a TAG command
            print("\n2. Testing TAG command")
            tag = TagData(tag_id=1, distance=50.0, direction='F')
            tag_success = comm.send_tag_data(tag)
            print(f"TAG command: {'Success' if tag_success else 'Failed'}")
            time.sleep(1.0)
            
            # Send STOP command
            print("\n3. Testing STOP command")
            stop_success = comm.send_stop()
            print(f"STOP command: {'Success' if stop_success else 'Failed'}")
            time.sleep(0.5)
            
            # Set motor speeds
            print("\n4. Testing SPEED command")
            speed_success = comm.set_speed(200, 100)
            print(f"SPEED command: {'Success' if speed_success else 'Failed'}")
            
            print("\nTest sequence complete!")
            
    finally:
        # Ensure we always disconnect properly
        comm.send_stop()
        comm.disconnect()


if __name__ == "__main__":
    main()
