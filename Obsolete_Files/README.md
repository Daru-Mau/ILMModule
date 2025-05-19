# Obsolete and Legacy Files

This directory contains files and directory structures that were part of the original project but are no longer needed for the current UART-based implementation of the AprilTag localization system.

## Contents

### Python Files

- `raspy_communication.py` - Replaced by uart_communication.py
- `communication_test.py` - Redundant with test_uart_communication.py
- `migrate_to_uart.py` - No longer needed after completing the migration
- `debug_movement_test.py` - Redundant with test_uart_communication.py
- `test_sensor_format.py` - Redundant test file
- `monitor_sensor_data.py` - Replaced by monitor_sensor_data_fixed.py
- `integrated_sensor_test.py` - Redundant with test_uart_communication.py
- `integrated_motor_test.py` - Redundant with test_uart_communication.py
- `quick_arduino_test.py` - Redundant Arduino test script
- `motor_test.py` - Redundant with UART motor control testing
- `motors_ccw.py` - Redundant motor test script

### Documentation

- `UART_README.md` - Redundant with APRILTAG_UART_README.md

### Scripts

- Various shell scripts (.sh files) - Unnecessary on Windows

### Empty Directories

The `EmptyDirs` subdirectory contains:

- `src/` and all subdirectories - Replaced by direct imports and flat file structure
- `docs/` - Moved to anthology
- `tests/` - Replaced by direct test files
- `arduino/` - Replaced by ILMMCodes directory

## Retention Policy

These files are kept for historical reference but are not needed for the current implementation of the project.
