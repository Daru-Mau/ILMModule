# Localization Module (LMModule)

A robust localization and navigation system for an omnidirectional robot featuring AprilTag-based positioning, sensor fusion, and advanced movement control with UART communication.

## Features

- **Omnidirectional Movement Control**

  - Holonomic drive system with three-wheel configuration
  - Smooth acceleration and deceleration
  - Precise position and orientation control
  - Maximum safe speed: 0.5 m/s

- **Advanced Localization**

  - Sensor fusion from multiple sources
  - High-resolution encoder readings (1120 ticks/revolution)
  - IMU integration for improved heading estimation
  - AprilTag-based absolute positioning
  - Position accuracy: ±5mm in controlled conditions
  - Heading accuracy: ±2 degrees with IMU fusion

- **Multiple Operation Modes**

  - Manual control
  - Autonomous navigation
  - AprilTag following
  - Automated docking/charging
  - Emergency stop functionality

- **Obstacle Detection**
  - 6x HC-SR04 ultrasonic sensors
  - Real-time obstacle avoidance
  - Social navigation features
  - Multi-level safety checks

## System Architecture

### Hardware Components

- 3x JGB37-520 Encoder DC motors (178 RPM, 1.8 Nm torque)
- 3x BTS7960 43A H-bridge motor drivers
- 6x HC-SR04 ultrasonic sensors
- MPU6050 IMU
- Raspberry Pi (for vision processing)
- Elegoo Mega (for motor control and sensor fusion)

### Software Components

1. **AprilTag Recognition (Python)**

   - Camera-based tag detection using OpenCV
   - Real-time pose estimation
   - 30 FPS detection rate

2. **Communication Layer (Python)**

   - High-speed serial protocol (115200 baud)
   - Error handling and retry mechanisms
   - Standardized message formats

3. **Localization System (Arduino)**

   - Encoder-based odometry
   - IMU data integration
   - Position and orientation tracking
   - 20Hz control loop frequency

4. **Movement Control (Arduino)**
   - Fine-grained motor control
   - Dynamic speed adjustment
   - Obstacle avoidance logic
   - Emergency stop handling

## Communication Protocol

### Message Formats

- Tag Detection: `TAG:id,x,y,yaw`
- Position Updates: `POS:x,y,theta`
- Clear Tag Data: `CLEAR`
- Control Commands: Single characters ('M', 'T', 'S', 'R')

## Getting Started

1. **Hardware Setup**

   - Connect motors to the BTS7960 drivers
   - Wire ultrasonic sensors to specified pins
   - Connect MPU6050 via I2C
   - Establish serial connection between Raspberry Pi and Arduino

2. **Software Installation**

   - Install required Python packages:
     ```
     pip install -r requirements.txt
     ```
   - Upload appropriate Arduino code to the Mega
   - Calibrate sensors using built-in calibration routines

3. **Testing**
   - Run individual component tests using `Testing_Motors/Testing_Motors.ino` and `Testing_Sensors/Testing_Sensors.ino`
   - Verify sensor readings and motor operation
   - Check AprilTag detection accuracy with `test_apriltag_uart.py`
   - Test UART communication with `test_uart_communication.py`
   - Validate emergency stop functionality

## Project Structure

- **Core Components**:

  - `apriltag_recognition.py`: Vision processing and tag detection (updated for UART)
  - `apriltag_uart_controller.py`: Alternative implementation with UART communication
  - `uart_communication.py`: Robust UART communication library
  - `test_apriltag_uart.py`: Testing tool for UART communication
  - `test_uart_communication.py`: UART communication test utility
  - `camera_test.py`: Utility for testing camera functionality
  - `monitor_sensor_data_fixed.py`: Tool for monitoring sensor data

- **Launcher Scripts**:

  - `launch_apriltag_uart.py`: Easy-to-use launcher for AprilTag recognition system
  - `launch_apriltag_controller.py`: Launcher for alternative controller implementation

- **Documentation**:

  - `APRILTAG_UART_README.md`: User guide for the UART AprilTag system
  - `UART_COMMUNICATION_GUIDE.md`: Detailed guide for UART communication
  - `UART_INTEGRATION_SUMMARY.md`: Summary of the UART integration process

- **Arduino Code**:
  - `ILMMCodes/`: All Arduino implementations:
    - `integrated_movement/`: Main Arduino implementation for the robot
    - `apriltag_movement/`: Tag-following implementation
    - `localization/`: Sensor fusion and positioning
    - `basic_moveset/`: Basic movement functions
    - `fine_moveset/`: Fine-tuned movement control
- **Testing Tools**:

  - `Testing_Motors/`: Test code for motor functionality
  - `Testing_Sensors/`: Test code for sensor functionality

- **Project Documentation**:
  - `Robotics & Design_ Localization module anthology/`: Complete project documentation including:
    - Technical drawings
    - Hardware schematics
    - Project photos
    - LaTeX documentation

## Safety Features

- Multi-level obstacle detection
- Emergency stop functionality
- Motor current monitoring
- Sensor validation checks
- Fail-safe operation modes

## Performance Metrics

- Position accuracy: ±5mm in controlled conditions
- Heading accuracy: ±2 degrees
- Maximum safe speed: 0.5 m/s
- Control loop frequency: 20Hz
- Tag detection rate: 30 FPS
- Emergency stop response: <100ms

## Future Improvements

- Web-based control interface
- Real-time telemetry visualization
- Remote diagnostics capabilities
- User-friendly calibration interfaces
- Configuration backup/restore features

## Contributors

- Project documentation and technical drawings available in the `Robotics & Design_ Localization module anthology/` directory
- Hardware schematics provided in `Robotics & Design_ Localization module anthology/pfds/IL_schema.pdf`
- Obsolete and legacy files safely archived in `Obsolete_Files/`
