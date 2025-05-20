#!/usr/bin/env python3
"""
Test script for wheel configuration in the integrated movement system.
This script demonstrates how to use the wheel configuration switching feature.
"""

import argparse
import time
import logging
from uart_communication import UARTCommunicator, DIR_FORWARD, DIR_BACKWARD, DIR_LEFT, DIR_RIGHT, DIR_ROTATE_LEFT, DIR_ROTATE_RIGHT

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    datefmt='%H:%M:%S')
logger = logging.getLogger(__name__)


def test_wheel_modes():
    """Test the two-wheel and three-wheel modes"""
    # Initialize UART
    comm = UARTCommunicator(debug=True)

    if not comm.connect():
        logger.error("Failed to connect to Arduino.")
        return False

    # Set initial speeds
    logger.info("Setting motor speeds...")
    comm.set_speed(max_speed=70, min_speed=50)

    try:
        # Test two-wheel mode
        logger.info("SETTING TWO-WHEEL MODE")
        comm.set_wheel_mode(use_three_wheels=False)
        time.sleep(1)

        # Test basic movements in two-wheel mode
        logger.info("Testing FORWARD in two-wheel mode...")
        comm.send_movement(DIR_FORWARD, 60)
        time.sleep(2)
        comm.send_stop()
        time.sleep(1)

        logger.info("Testing ROTATION in two-wheel mode...")
        comm.send_movement(DIR_ROTATE_LEFT, 60)
        time.sleep(2)
        comm.send_stop()
        time.sleep(1)

        # Test three-wheel mode
        logger.info("SETTING THREE-WHEEL MODE")
        comm.set_wheel_mode(use_three_wheels=True)
        time.sleep(1)

        # Test basic movements in three-wheel mode
        logger.info("Testing LEFT lateral movement in three-wheel mode...")
        comm.send_movement(DIR_LEFT, 60)
        time.sleep(2)
        comm.send_stop()
        time.sleep(1)

        logger.info("Testing RIGHT lateral movement in three-wheel mode...")
        comm.send_movement(DIR_RIGHT, 60)
        time.sleep(2)
        comm.send_stop()
        time.sleep(1)

        logger.info("Testing ROTATION in three-wheel mode...")
        comm.send_movement(DIR_ROTATE_RIGHT, 60)
        time.sleep(2)
        comm.send_stop()

        return True
    except KeyboardInterrupt:
        logger.info("Test interrupted.")
        comm.send_stop()
        return False
    finally:
        # Always clean up
        comm.send_stop()
        comm.disconnect()


def main():
    """Main function to run the test from command line"""
    parser = argparse.ArgumentParser(
        description='Test wheel configuration for the robot')

    parser.add_argument('--test-all', action='store_true',
                        help='Run all wheel configuration tests')

    args = parser.parse_args()

    if args.test_all:
        logger.info("Running wheel configuration tests...")
        if test_wheel_modes():
            logger.info("Wheel configuration tests completed successfully!")
        else:
            logger.error("Wheel configuration tests failed.")
    else:
        # Just connect and test the mode switching
        comm = UARTCommunicator(debug=True)
        if not comm.connect():
            logger.error("Failed to connect to Arduino.")
            return

        logger.info("Setting TWO-WHEEL mode...")
        comm.set_wheel_mode(False)
        time.sleep(2)

        logger.info("Setting THREE-WHEEL mode...")
        comm.set_wheel_mode(True)
        time.sleep(2)

        logger.info("Setting back to TWO-WHEEL mode...")
        comm.set_wheel_mode(False)

        comm.disconnect()
        logger.info("Basic wheel mode switching test complete.")


if __name__ == "__main__":
    main()
