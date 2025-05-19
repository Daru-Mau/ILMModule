# UART Communication Guide for Robot Control

This guide explains how to use the UART communication module for communication between the Raspberry Pi 4 and Arduino Mega for robot control.

## Overview

The UART Communication Module provides a robust serial communication interface between the Raspberry Pi and Arduino with the following features:

- Message framing with start/end markers
- Reliable command acknowledgment
- Auto-detection of Arduino ports
- Support for numeric direction codes (0-4)
- Enhanced error handling
- Direct motor control capability
- Sensor data retrieval

## Hardware Setup

1. Connect the Raspberry Pi to the Arduino Mega using a USB cable
2. Make sure the Arduino is powered on
3. The system will automatically detect the port

## Available Commands

The following commands are supported:

| Command | Description | Format |
|---------|-------------|--------|
| `PING` | Check connection | `<PING>` |
| `STOP` | Stop all motors | `<STOP>` |
| `MOVE` | Move in specified direction | `<MOVE:direction,speed>` |
| `TAG` | Send AprilTag data | `<TAG:id,distance,direction>` |
| `SENS` | Request sensor data | `<SENS>` |
| `MCTL` | Direct motor control | `<MCTL:left_speed,right_speed,back_speed>` |
| `SPEED` | Set speed parameters | `<SPEED:max_speed,min_speed>` |
| `TEST` | Run diagnostics | `<TEST>` |

### Direction Codes

The following numeric direction codes are used:

- `0` = STOP
- `1` = FORWARD
- `2` = BACKWARD
- `3` = LEFT
- `4` = RIGHT

## Python Usage Examples

### Basic Connection

```python
from uart_communication import UARTCommunicator

# Connect to Arduino (auto-detection)
comm = UARTCommunicator()

# Or specify port manually
# comm = UARTCommunicator(port='/dev/ttyACM0')

# Check if connected
if comm.connected:
    print(f"Connected to Arduino on {comm.port}")
else:
    print("Failed to connect")
```

### Movement Commands

```python
# Move forward at speed 150
comm.send_movement(direction=1, speed=150)

# Move backward
comm.send_movement(direction=2, speed=150)

# Turn left
comm.send_movement(direction=3, speed=150)

# Turn right
comm.send_movement(direction=4, speed=150)

# Stop
comm.send_movement(direction=0)
```

### Direct Motor Control

```python
# Control each motor directly
# Parameters: left_speed, right_speed, back_speed (-255 to 255)
# Positive values = forward, Negative values = backward

# Spin in place (left backward, right forward, back motor turn)
comm.set_motor_speeds(-150, 150, 150)

# Custom movement
comm.set_motor_speeds(100, 150, 0)
```

### Tag Data Commands

```python
from uart_communication import TagData

# Create a tag data object
tag = TagData(tag_id=1, distance=150, direction=1)  # Forward

# Send it to Arduino
comm.send_tag_data(tag)
```

### Sensor Data

```python
# Request sensor data
sensor_data = comm.request_sensor_data()

if sensor_data:
    print(f"Front: {sensor_data.front}cm")
    print(f"Front Left: {sensor_data.front_left}cm")
    print(f"Front Right: {sensor_data.front_right}cm")
    print(f"Back: {sensor_data.back}cm")
    print(f"Back Left: {sensor_data.back_left}cm")
    print(f"Back Right: {sensor_data.back_right}cm")
    
    # Check minimum distance in front
    min_front = sensor_data.min_front
    print(f"Minimum front distance: {min_front}cm")
```

## Testing the Communication

Use the included test script to verify the communication:

```bash
# Run automatic test
python test_uart_communication.py

# Specify a port manually
python test_uart_communication.py -p /dev/ttyACM0

# Run in interactive mode
python test_uart_communication.py -i

# Show debug information
python test_uart_communication.py -d
```

## Troubleshooting

If you're having connection issues:

1. Check if the Arduino is properly connected and powered
2. Make sure the correct sketch is uploaded to the Arduino
3. Try unplugging and reconnecting the Arduino
4. Check the baud rate (default: 115200)
5. Check for errors in the Arduino serial monitor

## Arduino Setup

The Arduino needs to have the matching firmware with UART message framing support. Make sure you've loaded the `integrated_movement.ino` sketch onto your Arduino Mega.

## Advanced Usage

For more advanced usage and customization options, see the full documentation in the module's docstrings.
