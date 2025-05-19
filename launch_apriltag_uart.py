#!/usr/bin/env python3
"""
AprilTag UART Control Launcher

This script launches the AprilTag recognition system with UART communication.
It provides a convenient way to start the system with common settings.
"""

import os
import sys
import argparse
import subprocess
import time
import platform


def main():
    parser = argparse.ArgumentParser(
        description='Launch AprilTag UART Controller')
    parser.add_argument('--max-speed', type=int, default=150,
                        help='Maximum motor speed (100-255)')
    parser.add_argument('--min-speed', type=int, default=100,
                        help='Minimum motor speed (80-max_speed)')
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port (auto-detected if not specified)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Enable verbose output')
    parser.add_argument('--test', '-t', action='store_true',
                        help='Run in test mode without camera')
    args = parser.parse_args()

    # Determine the script to run
    if args.test:
        script = "test_apriltag_uart.py"
        print(f"Starting AprilTag UART Test mode...")
    else:
        script = "apriltag_recognition.py"
        print(f"Starting AprilTag Recognition with UART communication...")

    # Build the command
    cmd = [sys.executable, script]

    if args.max_speed:
        cmd.extend(["--max-speed", str(args.max_speed)])

    if args.min_speed:
        cmd.extend(["--min-speed", str(args.min_speed)])

    if args.port:
        cmd.extend(["--port", args.port])

    if args.verbose:
        cmd.append("--verbose")

    # Print info
    print(f"Using script: {script}")
    print(f"Max speed: {args.max_speed}")
    print(f"Min speed: {args.min_speed}")
    if args.port:
        print(f"Serial port: {args.port}")
    else:
        print("Serial port: Auto-detect")
    print(f"Verbose mode: {'ON' if args.verbose else 'OFF'}")

    try:
        # Run the command
        process = subprocess.Popen(cmd)
        process.wait()
    except KeyboardInterrupt:
        print("Launcher interrupted by user.")
        process.terminate()
    except Exception as e:
        print(f"Error launching script: {e}")
    finally:
        print("Launcher exiting.")


if __name__ == "__main__":
    main()
