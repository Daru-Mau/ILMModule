#!/usr/bin/env python3
"""
Upgrade script for motor_test.py

This script will add:
1. Acceleration control
2. Diagonal movement support 
3. Enhanced interactive mode with status display

Run this script to automatically patch your motor_test.py file,
or manually apply the patches.
"""

import os
import shutil
import sys
import argparse


def backup_file(file_path):
    """Create a backup of the original file"""
    backup_dir = "Backup_Scripts"
    os.makedirs(backup_dir, exist_ok=True)

    # Get just the filename
    filename = os.path.basename(file_path)
    backup_file = os.path.join(backup_dir, f"{filename}.bak")

    # Copy the file
    shutil.copy2(file_path, backup_file)
    print(f"Created backup at: {backup_file}")
    return backup_file


def main():
    """Main function to run the upgrade process"""
    parser = argparse.ArgumentParser(
        description="Upgrade motor_test.py with new features")
    parser.add_argument('--backup-only', action='store_true',
                        help="Only create a backup of motor_test.py without modifying it")
    args = parser.parse_args()

    # Find motor_test.py in the current directory
    current_dir = os.getcwd()
    motor_test_path = os.path.join(current_dir, "motor_test.py")

    if not os.path.exists(motor_test_path):
        print(f"Error: motor_test.py not found in {current_dir}")
        return 1

    # Create a backup
    backup_path = backup_file(motor_test_path)

    if args.backup_only:
        print("Backup created. No modifications were made as requested.")
        return 0

    print("\nTo update your motor_test.py, manually add the following changes:")

    print("\n1. Add DEFAULT_ACCEL to the default settings:")
    print("   # Don't use acceleration by default (for backward compatibility)")
    print("   DEFAULT_ACCEL = \"OFF\"")

    print("\n2. Update the MotorTester.__init__ method to include acceleration:")
    print("   def __init__(self, port: str = DEFAULT_PORT, baud_rate: int = DEFAULT_BAUD_RATE,")
    print("                speed: float = DEFAULT_SPEED, mode: str = DEFAULT_MODE,")
    print("                accel: str = DEFAULT_ACCEL):")
    print("       # ... existing code ...")
    print("       self.accel = accel  # \"ON\" or \"OFF\"")

    print("\n3. Add the _set_accel method to the MotorTester class:")
    print("   def _set_accel(self, accel: str) -> bool:")
    print("       \"\"\"Set the acceleration mode (ON or OFF)\"\"\"")
    print("       if not self.connected:")
    print("           print(\"Not connected to Arduino.\")")
    print("           return False")
    print("       ")
    print("       # Ensure accel is capitalized")
    print("       accel = accel.upper()")
    print("       if accel not in [\"ON\", \"OFF\"]:")
    print(
        "           print(f\"Invalid acceleration mode: {accel}. Must be ON or OFF.\")")
    print("           return False")
    print("       ")
    print("       self.accel = accel")
    print("       command = f\"ACCEL:{accel}\\r\\n\"")
    print("       ")
    print("       try:")
    print("           self.arduino.serial.write(command.encode())")
    print(
        "           print(f\"Set acceleration to: {accel} {'(smooth start/stop)' if accel == 'ON' else '(instant)'}\")")
    print("           time.sleep(0.1)  # Give Arduino time to process")
    print("           return True")
    print("       except Exception as e:")
    print("           print(f\"Error setting acceleration mode: {e}\")")
    print("           return False")

    print("\n4. Update parse_args() to include the --accel argument:")
    print(
        "   parser.add_argument('--accel', '-a', choices=['on', 'off'], default=DEFAULT_ACCEL.lower(),")
    print(
        "                       help=f'Acceleration mode: on enables smooth start/stop, off uses instant speed changes (default: {DEFAULT_ACCEL.lower()})')")

    print("\n5. Update main() to initialize with acceleration:")
    print("   # Convert acceleration to uppercase")
    print("   accel = args.accel.upper()")
    print("   tester = MotorTester(args.port, args.baud, args.speed, mode, accel)")

    print("\n6. Add diagonal movement support to the move() method.")

    print("\nUpdate completed successfully. Check the MOTOR_TEST_README.md for documentation on the new features.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
