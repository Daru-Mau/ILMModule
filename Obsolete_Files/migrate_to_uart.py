#!/usr/bin/env python3
"""
AprilTag UART Migration Helper

This script helps users migrate from the old raspy_communication approach
to the new UART-based approach for the Integrated Movement system.

It provides:
1. A compatibility layer for old code
2. Examples of how to use the new UART communication system
3. Tools to verify the system is working correctly
"""

import os
import sys
import argparse
import time
import logging
from typing import Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)


def print_header():
    """Print header information"""
    print("\n========================================================")
    print("    APRILTAG UART MIGRATION HELPER")
    print("    Transition from raspy_communication to uart_communication")
    print("========================================================\n")


def check_environment():
    """Check if we're on a Raspberry Pi and if needed packages are installed"""
    print("Checking environment...")

    # Check if running on Raspberry Pi
    is_raspberry_pi = os.path.exists('/sys/firmware/devicetree/base/model')
    print(f"  Running on Raspberry Pi: {'Yes' if is_raspberry_pi else 'No'}")

    # Check for required packages
    required_modules = [
        ('uart_communication', 'UART communication module'),
        ('serial', 'PySerial for hardware communication'),
        ('picamera2', 'PiCamera2 for camera access (Raspberry Pi only)'),
        ('pupil_apriltags', 'AprilTag detection library')
    ]

    missing_modules = []

    for module_name, description in required_modules:
        try:
            if module_name == 'picamera2' and not is_raspberry_pi:
                # Skip PiCamera2 check if not on Raspberry Pi
                print(f"  Skipping {module_name} check (not on Raspberry Pi)")
                continue

            # Try to import the module
            __import__(module_name)
            print(f"  ✓ {module_name}: Found")
        except ImportError:
            print(f"  ✗ {module_name}: Missing - {description}")
            missing_modules.append((module_name, description))

    # Print installation instructions if modules are missing
    if missing_modules:
        print("\nSome required packages are missing. Please install them:")
        for module_name, description in missing_modules:
            if module_name == 'uart_communication':
                print(
                    f"  - {module_name}: This module is already in your workspace")
            elif module_name == 'picamera2':
                print(
                    f"  - {module_name}: Run 'sudo apt install -y python3-picamera2'")
            elif module_name == 'pupil_apriltags':
                print(f"  - {module_name}: Run 'pip install pupil-apriltags'")
            else:
                print(f"  - {module_name}: Run 'pip install {module_name}'")
    else:
        print("\nAll required packages are installed!")


def test_uart_connection(port: Optional[str] = None, baud_rate: int = 115200):
    """Test connection to Arduino via UART"""
    print("\nTesting UART connection...")

    try:
        # Import necessary modules
        from uart_communication import UARTCommunicator

        # Create communicator
        print(f"Connecting to Arduino{' on '+port if port else ''}...")
        communicator = UARTCommunicator(port=port, baud_rate=baud_rate)

        if not communicator.connected:
            print("Attempting to auto-detect port...")
            if not communicator.connect():
                print("❌ Failed to connect to Arduino. Check connections and try again.")
                return False

        # Get the actual port being used
        actual_port = communicator.port
        print(f"✓ Connected to Arduino on {actual_port}")

        # Test ping
        print("Testing PING command...")
        if communicator.ping():
            print("✓ Ping successful!")
        else:
            print("❌ Ping failed")
            return False

        # Request sensor data
        print("Requesting sensor data...")
        sensor_data = communicator.request_sensor_data()
        if sensor_data:
            print(f"✓ Received sensor data: {str(sensor_data)}")
        else:
            print("❌ Failed to get sensor data")

        # Send stop command
        print("Sending STOP command...")
        if communicator.send_stop():
            print("✓ Stop command sent successfully")
        else:
            print("❌ Stop command failed")

        print("\n✅ UART connection test completed successfully!")
        return True

    except ImportError as e:
        print(f"❌ Import error: {e}")
        print("Make sure all required packages are installed.")
        return False
    except Exception as e:
        print(f"❌ Error during connection test: {e}")
        return False


def show_usage_examples():
    """Show examples of how to use the new UART communication system"""
    print("\nUsage Examples:")
    print("==============")

    print("\n1. Basic Connection:")
    print("```python")
    print("from uart_communication import UARTCommunicator")
    print("")
    print("# Auto-detect Arduino port")
    print("comm = UARTCommunicator()")
    print("")
    print("# Or specify port manually")
    print("# comm = UARTCommunicator(port='/dev/ttyACM0')")
    print("")
    print("if comm.connected:")
    print("    print(f\"Connected to Arduino on {comm.port}\")")
    print("else:")
    print("    print(\"Failed to connect to Arduino\")")
    print("```")

    print("\n2. Sending AprilTag Data:")
    print("```python")
    print("from uart_communication import UARTCommunicator, TagData, DIR_FORWARD")
    print("")
    print("comm = UARTCommunicator()")
    print("")
    print("# Create tag data (id, distance, direction)")
    print("tag_data = TagData(tag_id=1, distance=100.0, direction=DIR_FORWARD)")
    print("")
    print("# Send to Arduino")
    print("if comm.send_tag_data(tag_data):")
    print("    print(\"Tag data sent successfully\")")
    print("else:")
    print("    print(\"Failed to send tag data\")")
    print("```")

    print("\n3. Movement Commands:")
    print("```python")
    print("from uart_communication import UARTCommunicator, DIR_FORWARD, DIR_LEFT, DIR_RIGHT, DIR_STOP")
    print("")
    print("comm = UARTCommunicator()")
    print("")
    print("# Move forward at speed 150")
    print("comm.send_movement(DIR_FORWARD, 150)")
    print("")
    print("# Turn left at speed 120")
    print("comm.send_movement(DIR_LEFT, 120)")
    print("")
    print("# Stop")
    print("comm.send_movement(DIR_STOP)")
    print("# or")
    print("comm.send_stop()")
    print("```")


def show_migration_guide():
    """Show a guide on how to migrate from raspy_communication to uart_communication"""
    print("\nMigration Guide:")
    print("===============")

    print("\n1. Replace Imports:")
    print("   Old: from raspy_communication import ArduinoCommunicator, TagData")
    print("   New: from uart_communication import UARTCommunicator, TagData")

    print("\n2. Update Connection Code:")
    print("   Old: comm = ArduinoCommunicator(port, baud_rate)")
    print("        comm.connect()")
    print("   New: comm = UARTCommunicator(port, baud_rate)")
    print("        # Auto-connects by default, or call comm.connect() manually")

    print("\n3. Update Direction Constants:")
    print("   Old: Use string or character codes like 'F', 'B', 'L', 'R'")
    print("   New: Use numeric direction codes:")
    print("        - DIR_STOP = 0      (Stop movement)")
    print("        - DIR_FORWARD = 1   (Move forward)")
    print("        - DIR_BACKWARD = 2  (Move backward)")
    print("        - DIR_LEFT = 3      (Turn left)")
    print("        - DIR_RIGHT = 4     (Turn right)")

    print("\n4. Update TagData Usage:")
    print("   Old: tag_data = TagData(tag_id, distance, 'F')")
    print("   New: tag_data = TagData(tag_id, distance, DIR_FORWARD)")

    print("\n5. Update Methods:")
    print("   - send_tag_data() - Same usage")
    print("   - send_stop() - Same usage")
    print("   - set_speed() - Same usage")
    print("   - Old: direct_motor_control() → New: set_motor_speeds()")

    print("\nFor more details, see the examples provided in this script and refer to the")
    print("UART_COMMUNICATION_GUIDE.md file in your workspace.")


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description="AprilTag UART Migration Helper")
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port for Arduino connection (auto-detect if not specified)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Baud rate for serial communication (default: 115200)')
    parser.add_argument('--test', action='store_true',
                        help='Test UART connection to Arduino')
    parser.add_argument('--examples', action='store_true',
                        help='Show usage examples')
    parser.add_argument('--migration', action='store_true',
                        help='Show migration guide')

    args = parser.parse_args()

    print_header()

    # Default behavior: show all information
    if not (args.test or args.examples or args.migration):
        check_environment()
        test_uart_connection(args.port, args.baud)
        show_migration_guide()
        show_usage_examples()
    else:
        # Selective behavior based on arguments
        if args.test:
            check_environment()
            test_uart_connection(args.port, args.baud)

        if args.migration:
            show_migration_guide()

        if args.examples:
            show_usage_examples()

    print("\nFor more information, refer to the UART_COMMUNICATION_GUIDE.md file in your workspace.")


if __name__ == "__main__":
    main()
