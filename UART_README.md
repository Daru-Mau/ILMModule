# UART-based AprilTag Robot Control

## Overview

This project has been updated to use the UART communication protocol as the mediator between the AprilTag recognition system and the Arduino's integrated movement controller. This improves reliability, error handling, and provides a more standardized communication interface.

## Key Components

1. **uart_communication.py** - The core communication module that provides reliable serial communication with the Arduino
2. **apriltag_uart_controller.py** - An integrated controller that uses UART for communication with the Arduino
3. **test_apriltag_uart.py** - A test script for verifying UART communication without needing a camera
4. **migrate_to_uart.py** - A helper script to assist with migrating from the old system to the new UART-based approach

## Changes from Previous Version

- Replaced `raspy_communication.py` with the more robust `uart_communication.py`
- Standardized on numeric direction codes (0=STOP, 1=FORWARD, 2=BACKWARD, 3=LEFT, 4=RIGHT)
- Added message framing with start/end markers for better error detection
- Improved error handling and retry mechanisms
- Enhanced port auto-detection

## Direction Codes

The system now uses the following numeric direction codes:

- `0` = STOP
- `1` = FORWARD
- `2` = BACKWARD
- `3` = LEFT
- `4` = RIGHT

## Getting Started

### Testing UART Communication

You can test the UART communication with the Arduino without using the camera:

```bash
python test_apriltag_uart.py
```

This starts an interactive mode where you can send commands manually to verify communication.

To run a simple demo sequence:

```bash
python test_apriltag_uart.py --demo
```

### Running the Full System

To run the complete AprilTag detection and robot control system:

```bash
python apriltag_uart_controller.py
```

### Migration Help

If you need help migrating from the old system to the new UART-based approach, use:

```bash
python migrate_to_uart.py
```

This will show a guide on how to update your code to use the new system.

## Advanced Usage

For more detailed information on the UART communication system, refer to the `UART_COMMUNICATION_GUIDE.md` file.

## Troubleshooting

If you encounter connection issues:

1. Make sure the Arduino is properly connected and has the `integrated_movement.ino` sketch uploaded
2. Check that the serial port is correct (use auto-detection or specify with `--port`)
3. Verify the baud rate matches (default is 115200)
4. Run `python migrate_to_uart.py --test` to verify communication
