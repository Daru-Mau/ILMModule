# Wheel Configuration Guide

This document explains how to use the wheel configuration feature of the integrated movement system.

## Overview

The robot supports two wheel configurations:

1. **Two-Wheel Mode**: Uses only the left and right motors for differential drive. Simpler control but limited to forward/backward movement and rotation.

2. **Three-Wheel Mode**: Uses all three motors (left, right, and back) for omnidirectional movement. Enables lateral (strafing) movement.

## How to Set the Wheel Configuration

### Option 1: Using the UART Communicator Directly

```python
from uart_communication import UARTCommunicator

# Initialize communication
comm = UARTCommunicator()
comm.connect()

# Set to two-wheel mode (differential drive)
comm.set_wheel_mode(False)

# Set to three-wheel mode (omnidirectional)
comm.set_wheel_mode(True)
```

### Option 2: Using the AprilTag UART Controller

When initializing the controller:

```python
from apriltag_uart_controller import AprilTagUARTController

# Initialize with three-wheel configuration
controller = AprilTagUARTController(
    # ... other parameters ...
    use_three_wheels=True
)
```

Or when using the command-line interface:

```bash
python apriltag_uart_controller.py --three-wheels
```

## Movement Behavior in Different Modes

### Two-Wheel Mode

- **FORWARD/BACKWARD**: Uses left and right motors in opposite directions
- **LEFT/RIGHT TURN**: Uses left and right motors in the same direction
- **ROTATION**: Uses left and right motors in the same direction
- **LATERAL**: Not directly supported, but simulated with both motors in same direction

### Three-Wheel Mode

- **FORWARD/BACKWARD**: Uses left and right motors (back motor disabled)
- **LEFT/RIGHT LATERAL**: Uses specific motor combinations to enable true strafing
- **ROTATION**: Uses all three motors for more precise rotation

## Testing

You can test the wheel configuration with:

```bash
python test_wheel_configuration.py --test-all
```

This will run a sequence of movements in both wheel configurations to demonstrate the differences.

## Recommendations

- Use **Two-Wheel Mode** when:

  - Battery power is a concern
  - You only need forward/backward movement and rotation
  - Simplicity is preferred

- Use **Three-Wheel Mode** when:
  - Precise positioning is required
  - You need lateral movement capabilities
  - Full omnidirectional control is needed

The default configuration is **Two-Wheel Mode** to maintain compatibility with older code.
