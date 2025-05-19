# Enhanced Motor Test System

This enhanced motor test system allows for dynamic control of the 3-wheeled omnidirectional robot with two operational modes:

1. **3-wheel mode** - Uses all three omnidirectional wheels for full holonomic movement
2. **2-wheel mode** - Uses only the left and right omnidirectional wheels for movement, with the back wheel disabled (but still present)

## New Features

- **Dynamic velocity control** - Change speed on-the-fly during operation
- **Mode switching** - Toggle between full 3-wheel mode and 2-wheel mode (with back wheel disabled)
- **Smooth acceleration/deceleration** - Optional ramping of motor speed for smoother starts and stops
- **Enhanced movement patterns**:
  - Forward/backward movement
  - Rotation (left/right)
  - Turning (arc left/right)
  - Sliding (lateral movement left/right, fully available in 3-wheel mode only)
  - Diagonal movement (forward-left, forward-right, backward-left, backward-right)

## Usage

### Command-line Arguments

```bash
python motor_test.py [options]
```

Options:

- `--port`, `-p`: Serial port (default: /dev/ttyACM0)
- `--baud`, `-b`: Baud rate (default: 115200)
- `--duration`, `-d`: Duration for each test in seconds (default: 2.0)
- `--speed`, `-s`: Motor speed (0-255, default: 150)
- `--mode`, `-m`: Operating mode, either '2wheel' or '3wheel' (default: 3wheel)
- `--accel`, `-a`: Acceleration mode, either 'on' or 'off' (default: off)
- `--test`, `-t`: Specific test to run (options: all, left, right, back, forward, backward, rotate, slide, turn, diagnostics)
- `--interactive`, `-i`: Run in interactive mode for direct serial control

### Examples

Run all tests with default settings:

```bash
python motor_test.py
```

Test forward movement at speed 200 in 2-wheel mode:

```bash
python motor_test.py --test forward --speed 200 --mode 2wheel
```

Interactive mode with custom port and smooth acceleration:

```bash
python motor_test.py --interactive --port COM3 --accel on
```

Test diagonal movements with smooth acceleration:

```bash
python motor_test.py --test all --accel on --speed 150
```

## Movement Commands

The following movement commands are available:

| Command     | Description            | 3-wheel Mode         | 2-wheel Mode            |
| ----------- | ---------------------- | -------------------- | ----------------------- |
| F           | Forward                | All wheels           | Left/Right wheels only  |
| B           | Backward               | All wheels           | Left/Right wheels only  |
| L           | Rotate Left            | All wheels           | Left/Right wheels only  |
| R           | Rotate Right           | All wheels           | Left/Right wheels only  |
| SLIDE_LEFT  | Lateral movement left  | Full sideways motion | Falls back to rotation  |
| SLIDE_RIGHT | Lateral movement right | Full sideways motion | Falls back to rotation  |
| TURN_LEFT   | Arc turn left          | Smooth arc           | Left/Right differential |
| TURN_RIGHT  | Arc turn right         | Smooth arc           | Left/Right differential |

## Interactive Mode

In interactive mode, you can:

1. Send basic movement commands (F, B, L, R, S)
2. Run diagnostics tests
3. Send TAG commands with custom parameters
4. Change speed dynamically
5. Toggle between 2-wheel and 3-wheel modes
6. Execute special movement patterns
7. Send custom commands to the Arduino

## Using the Enhanced Interactive Mode

The interactive mode now includes additional features:

1. **Status Display**: Shows current speed, wheel mode, and acceleration settings
2. **Diagonal Movement**: Access diagonal movement patterns (forward-left, forward-right, backward-left, backward-right)
3. **Acceleration Toggle**: Switch between smooth acceleration/deceleration and instant speed changes

### Interactive Mode Menu

```
==== Interactive Serial Control Mode ====
Available commands:
  1. Send 'F' (forward)
  2. Send 'B' (backward)
  3. Send 'L' (rotate left)
  4. Send 'R' (rotate right)
  5. Send 'S' (stop)
  6. Send 'TEST' command
  7. Send TAG command
  8. Change speed
  9. Change wheel mode (2-wheel/3-wheel)
 10. Special movements (slide, turn)
 11. Diagonal movements
 12. Toggle acceleration mode (ON/OFF)
 13. Show current status
 14. Custom command
 15. Quit
```

## Diagonal Movement Details

Diagonal movements combine forward/backward motion with left/right motion for more efficient point-to-point navigation:

- **Diagonal Forward-Left**: Combines forward and left movement
- **Diagonal Forward-Right**: Combines forward and right movement
- **Diagonal Backward-Left**: Combines backward and left movement
- **Diagonal Backward-Right**: Combines backward and right movement

In 3-wheel mode, these movements use all wheels optimally. In 2-wheel mode, diagonal movements are still possible but with reduced efficiency since the back wheel is disabled.

## Acceleration Control

Smooth acceleration provides the following benefits:

- Reduced mechanical stress on the robot
- Smoother start and stop behaviors
- Better control in precision movements
- Decreased wheel slippage

Enable acceleration with the `--accel on` parameter or toggle in interactive mode with option 12.

## Arduino Setup

Flash the `basic_moveset.ino` file to your Arduino Mega. The code is already configured for:

- The 3-wheel omnidirectional setup (all three wheels are omnidirectional in both modes)
- UART communication at 115200 baud
- All movement patterns and commands
- Dynamic speed control
- Mode switching (3-wheel active mode or 2-wheel mode with disabled back wheel)
