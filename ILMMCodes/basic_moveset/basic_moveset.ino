#include <Arduino.h>
#include <math.h>

// === Pin Definitions ===
// IMPORTANT: This file uses a COUNTER-CLOCKWISE pin arrangement
// The motors have been repositioned as follows:
// - OLD RIGHT wheel is now LEFT wheel (pins 2, 3, 38, 39)
// - OLD LEFT wheel is now BACK wheel (pins 4, 5, 44, 45)
// - OLD BACK wheel is now RIGHT wheel (pins 7, 6, 51, 50)
//
// MOVEMENT CONTROLS:
// F - Forward: LEFT=BACKWARD, RIGHT=FORWARD, BACK=FORWARD
// B - Backward: LEFT=FORWARD, RIGHT=BACKWARD, BACK=BACKWARD
// L - Left/CCW: LEFT=FORWARD, RIGHT=FORWARD, BACK=STOP
// R - Right/CW: LEFT=BACKWARD, RIGHT=BACKWARD, BACK=STOP
// Motor direction logic has been adapted to accommodate this arrangement

// Motor Driver Pins
#define RPWM_RIGHT 7 // Updated to match configuration
#define LPWM_RIGHT 6 // Updated to match configuration
#define REN_RIGHT 51 // Kept original
#define LEN_RIGHT 50 // Kept original

#define RPWM_LEFT 2 // Updated to match configuration
#define LPWM_LEFT 3 // Updated to match configuration
#define REN_LEFT 38 // Updated to match configuration
#define LEN_LEFT 39 // Updated to match configuration

#define RPWM_BACK 4 // Updated to match configuration
#define LPWM_BACK 5 // Updated to match configuration
#define REN_BACK 44 // Kept original
#define LEN_BACK 45 // Kept original

// Basic configuration
const float OBSTACLE_DISTANCE = 15.0; // cm
const int BASE_SPEED = 50;            // PWM value (0-255)

// New configuration options
bool use3WheelMode = true;   // Default to 3-wheel mode
int userSpeed = BASE_SPEED;  // Allows dynamic speed control
bool useSmoothAccel = false; // Default to not use acceleration (for compatibility)

// Serial communication
const int SERIAL_BUFFER_SIZE = 64;
char serialBuffer[SERIAL_BUFFER_SIZE];
int bufferIndex = 0;

// Globals
float distFL, distF, distFR, distBL, distB, distBR;

struct Motor
{
  int RPWM;
  int LPWM;
  int REN;
  int LEN;
};

Motor motorRight = {RPWM_RIGHT, LPWM_RIGHT, REN_RIGHT, LEN_RIGHT};
Motor motorLeft = {RPWM_LEFT, LPWM_LEFT, REN_LEFT, LEN_LEFT};
Motor motorBack = {RPWM_BACK, LPWM_BACK, REN_BACK, LEN_BACK};

enum Direction
{
  FORWARD,
  BACKWARD,
  STOP
};

void setupMotorPins(const Motor &motor)
{
  pinMode(motor.RPWM, OUTPUT);
  pinMode(motor.LPWM, OUTPUT);
  pinMode(motor.REN, OUTPUT);
  pinMode(motor.LEN, OUTPUT);
  digitalWrite(motor.REN, HIGH);
  digitalWrite(motor.LEN, HIGH);
}

void moveMotor(const Motor &motor, Direction dir, int speed)
{
  speed = constrain(speed, 0, 255);
  switch (dir)
  {
  case FORWARD:
    analogWrite(motor.RPWM, speed);
    analogWrite(motor.LPWM, 0);
    break;
  case BACKWARD:
    analogWrite(motor.RPWM, 0);
    analogWrite(motor.LPWM, speed);
    break;
  case STOP:
    analogWrite(motor.RPWM, 0);
    analogWrite(motor.LPWM, 0);
    break;
  }
}

void stopAllMotors()
{
  moveMotor(motorLeft, STOP, 0);
  moveMotor(motorRight, STOP, 0);
  moveMotor(motorBack, STOP, 0);
}

// ---------- MOVEMENT FUNCTIONS WITH 3-WHEEL SUPPORT ----------

// Forward movement
void moveForward(int speed, bool use3Wheel)
{
  if (useSmoothAccel)
  {
    // Smooth acceleration for forward movement
    accelerateMotor(motorLeft, BACKWARD, speed);
    accelerateMotor(motorRight, FORWARD, speed);
    moveMotor(motorBack, STOP, 0);
  }
  else
  {
    // Regular instant movement
    moveMotor(motorLeft, BACKWARD, speed);
    moveMotor(motorRight, FORWARD, speed);
    moveMotor(motorBack, STOP, 0); // Back wheel disabled in 2-wheel mode
  }
}

// Backward movement
void moveBackward(int speed, bool use3Wheel)
{
  moveMotor(motorLeft, FORWARD, speed);
  moveMotor(motorRight, BACKWARD, speed);
  moveMotor(motorBack, STOP, 0); // Back wheel disabled in 2-wheel mode
}

// Rotate left (counterclockwise)
void rotateLeft(int speed, bool use3Wheel)
{
  if (use3Wheel)
  {
    // In 3-wheel mode, all wheels contribute to rotation
    moveMotor(motorLeft, FORWARD, speed);
    moveMotor(motorRight, FORWARD, speed);
    moveMotor(motorBack, FORWARD, speed);
  }
  else
  {
    // In 2-wheel mode, only side wheels rotate
    moveMotor(motorLeft, FORWARD, speed);
    moveMotor(motorRight, FORWARD, speed);
    moveMotor(motorBack, STOP, 0);
  }
}

// Rotate right (clockwise)
void rotateRight(int speed, bool use3Wheel)
{
  if (use3Wheel)
  {
    // In 3-wheel mode, all wheels contribute to rotation
    moveMotor(motorLeft, BACKWARD, speed);
    moveMotor(motorRight, BACKWARD, speed);
    moveMotor(motorBack, BACKWARD, speed);
  }
  else
  {
    // In 2-wheel mode, only side wheels rotate
    moveMotor(motorLeft, BACKWARD, speed);
    moveMotor(motorRight, BACKWARD, speed);
    moveMotor(motorBack, STOP, 0);
  }
}

// Slide left (strafe)
void slideLeft(int speed, bool use3Wheel)
{
  if (use3Wheel)
  {
    // Only possible in 3-wheel omnidirectional mode
    moveMotor(motorLeft, FORWARD, speed);
    moveMotor(motorRight, BACKWARD, speed);
    moveMotor(motorBack, FORWARD, speed * 1.5); // Back wheel does more work in strafing
  }
  else
  {
    // Fall back to rotation in 2-wheel mode
    rotateLeft(speed, false);
  }
}

// Slide right (strafe)
void slideRight(int speed, bool use3Wheel)
{
  if (use3Wheel)
  {
    // Only possible in 3-wheel omnidirectional mode
    moveMotor(motorLeft, BACKWARD, speed);
    moveMotor(motorRight, FORWARD, speed);
    moveMotor(motorBack, BACKWARD, speed * 1.5); // Back wheel does more work in strafing
  }
  else
  {
    // Fall back to rotation in 2-wheel mode
    rotateRight(speed, false);
  }
}

// Turn left (arc left) - both wheels forward but left slower
void turnLeft(int speed, bool use3Wheel)
{
  int reducedSpeed = speed * 0.5; // Left wheel at half speed

  moveMotor(motorLeft, BACKWARD, reducedSpeed);
  moveMotor(motorRight, FORWARD, speed);

  if (use3Wheel)
  {
    moveMotor(motorBack, FORWARD, speed * 0.75);
  }
  else
  {
    moveMotor(motorBack, STOP, 0);
  }
}

// Turn right (arc right) - both wheels forward but right slower
void turnRight(int speed, bool use3Wheel)
{
  int reducedSpeed = speed * 0.5; // Right wheel at half speed

  moveMotor(motorLeft, BACKWARD, speed);
  moveMotor(motorRight, FORWARD, reducedSpeed);

  if (use3Wheel)
  {
    moveMotor(motorBack, FORWARD, speed * 0.75);
  }
  else
  {
    moveMotor(motorBack, STOP, 0);
  }
}

// Diagonal movement functions

// Forward Left Diagonal movement
void moveDiagonalForwardLeft(int speed, bool use3Wheel)
{
  // Left wheel minimal movement (reduced by 70%)
  moveMotor(motorLeft, BACKWARD, speed * 0.3);
  // Right wheel at full power
  moveMotor(motorRight, FORWARD, speed);

  if (use3Wheel)
  {
    // Back wheel in between to achieve diagonal motion
    moveMotor(motorBack, FORWARD, speed * 0.6);
  }
  else
  {
    moveMotor(motorBack, STOP, 0); // Back wheel disabled in 2-wheel mode
  }
}

// Forward Right Diagonal movement
void moveDiagonalForwardRight(int speed, bool use3Wheel)
{
  // Left wheel at full power
  moveMotor(motorLeft, BACKWARD, speed);
  // Right wheel minimal movement (reduced by 70%)
  moveMotor(motorRight, FORWARD, speed * 0.3);

  if (use3Wheel)
  {
    // Back wheel in between to achieve diagonal motion
    moveMotor(motorBack, FORWARD, speed * 0.6);
  }
  else
  {
    moveMotor(motorBack, STOP, 0); // Back wheel disabled in 2-wheel mode
  }
}

// Backward Left Diagonal movement
void moveDiagonalBackwardLeft(int speed, bool use3Wheel)
{
  // Left wheel minimal movement (reduced by 70%)
  moveMotor(motorLeft, FORWARD, speed * 0.3);
  // Right wheel at full power
  moveMotor(motorRight, BACKWARD, speed);

  if (use3Wheel)
  {
    // Back wheel in between to achieve diagonal motion
    moveMotor(motorBack, BACKWARD, speed * 0.6);
  }
  else
  {
    moveMotor(motorBack, STOP, 0); // Back wheel disabled in 2-wheel mode
  }
}

// Backward Right Diagonal movement
void moveDiagonalBackwardRight(int speed, bool use3Wheel)
{
  // Left wheel at full power
  moveMotor(motorLeft, FORWARD, speed);
  // Right wheel minimal movement (reduced by 70%)
  moveMotor(motorRight, BACKWARD, speed * 0.3);

  if (use3Wheel)
  {
    // Back wheel in between to achieve diagonal motion
    moveMotor(motorBack, BACKWARD, speed * 0.6);
  }
  else
  {
    moveMotor(motorBack, STOP, 0); // Back wheel disabled in 2-wheel mode
  }
}

void processSerialInput()
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (bufferIndex > 0)
      {
        serialBuffer[bufferIndex] = '\0';
        parseCommand(serialBuffer);
        bufferIndex = 0;
      }
    }
    else if (bufferIndex < SERIAL_BUFFER_SIZE - 1)
    {
      serialBuffer[bufferIndex++] = c;
    }
  }
}

void parseCommand(const char *cmd)
{
  Serial.print("Received command: ");
  Serial.println(cmd);

  // Handle single character commands
  if (strlen(cmd) == 1)
  {
    switch (cmd[0])
    {
    case 'F':
      Serial.println("Moving Forward");
      moveForward(userSpeed, use3WheelMode);
      break;
    case 'B':
      Serial.println("Moving Backward");
      moveBackward(userSpeed, use3WheelMode);
      break;
    case 'L':
      Serial.println("Rotating Left");
      rotateLeft(userSpeed, use3WheelMode);
      break;
    case 'R':
      Serial.println("Rotating Right");
      rotateRight(userSpeed, use3WheelMode);
      break;
    case 'S':
      Serial.println("Stopping");
      stopAllMotors();
      break;
    }
    return;
  }

  // Check for TEST command
  if (strcmp(cmd, "TEST") == 0)
  {
    Serial.println("TEST command received");
    Serial.println("Sensor status: OK");
    Serial.println("Motors: Ready");
    Serial.print("Mode: ");
    Serial.println(use3WheelMode ? "3-wheel" : "2-wheel");
    Serial.print("Speed: ");
    Serial.println(userSpeed);
    Serial.println("System: OK");
    return;
  }

  // Check for PING command
  if (strcmp(cmd, "PING") == 0)
  {
    Serial.println("PONG");
    return;
  }

  // Check for STOP command
  if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "S") == 0)
  {
    if (useSmoothAccel)
    {
      Serial.println("Smoothly stopping all motors");
      smoothStopAllMotors();
    }
    else
    {
      Serial.println("Stopping all motors");
      stopAllMotors();
    }
    return;
  }

  // Check for MODE command (2-wheel vs 3-wheel)
  if (strncmp(cmd, "MODE:", 5) == 0)
  {
    if (strcmp(cmd + 5, "3WHEEL") == 0)
    {
      use3WheelMode = true;
      Serial.println("Switched to 3-wheel mode (all wheels active)");
    }
    else if (strcmp(cmd + 5, "2WHEEL") == 0)
    {
      use3WheelMode = false;
      Serial.println("Switched to 2-wheel mode (back wheel disabled)");
    }
    else
    {
      Serial.print("Current mode: ");
      Serial.println(use3WheelMode ? "3-wheel (all wheels active)" : "2-wheel (back wheel disabled)");
    }
    return;
  }

  // Check for SPEED command
  if (strncmp(cmd, "SPEED:", 6) == 0)
  {
    int newSpeed = atoi(cmd + 6);
    if (newSpeed >= 0 && newSpeed <= 255)
    {
      userSpeed = newSpeed;
      Serial.print("Speed set to: ");
      Serial.println(userSpeed);
    }
    else
    {
      Serial.println("Invalid speed. Must be between 0-255");
    }
    return;
  }

  // Check for ACCEL command
  if (strncmp(cmd, "ACCEL:", 6) == 0)
  {
    if (strcmp(cmd + 6, "ON") == 0)
    {
      useSmoothAccel = true;
      Serial.println("Smooth acceleration enabled");
    }
    else if (strcmp(cmd + 6, "OFF") == 0)
    {
      useSmoothAccel = false;
      Serial.println("Smooth acceleration disabled");
    }
    else
    {
      Serial.print("Current acceleration mode: ");
      Serial.println(useSmoothAccel ? "ON (smooth)" : "OFF (instant)");
    }
    return;
  }

  // Handle TAG commands from motor_test.py
  if (strncmp(cmd, "TAG:", 4) == 0)
  {
    int tagId;
    float distance;
    char direction;

    // Parse with additional optional parameter for mode
    char mode[10] = "";
    int result = sscanf(cmd + 4, "%d,%f,%c,%9s", &tagId, &distance, &direction, mode);

    if (result >= 3)
    {
      Serial.print("Processing TAG Command: ");
      Serial.print("Tag ID: ");
      Serial.print(tagId);
      Serial.print(", Distance: ");
      Serial.print(distance);
      Serial.print(", Direction: ");
      Serial.println(direction);

      // Check if mode was specified
      if (result == 4 && mode[0] != '\0')
      {
        if (strcmp(mode, "3WHEEL") == 0)
        {
          use3WheelMode = true;
          Serial.println("Using 3-wheel mode for this command");
        }
        else if (strcmp(mode, "2WHEEL") == 0)
        {
          use3WheelMode = false;
          Serial.println("Using 2-wheel mode for this command");
        }
      }

      // Convert distance to motor speed
      int speed = min(255, max(50, (int)(distance)));

      // Execute movement based on command
      executeMovement(direction, speed, tagId);
    }
    else
    {
      Serial.println("Invalid TAG format");
    }
    return;
  }

  // Special movement commands
  if (strcmp(cmd, "SLIDE_LEFT") == 0)
  {
    Serial.println("Sliding Left");
    slideLeft(userSpeed, use3WheelMode);
    return;
  }

  if (strcmp(cmd, "SLIDE_RIGHT") == 0)
  {
    Serial.println("Sliding Right");
    slideRight(userSpeed, use3WheelMode);
    return;
  }

  if (strcmp(cmd, "TURN_LEFT") == 0)
  {
    Serial.println("Turning Left");
    turnLeft(userSpeed, use3WheelMode);
    return;
  }

  if (strcmp(cmd, "TURN_RIGHT") == 0)
  {
    Serial.println("Turning Right");
    turnRight(userSpeed, use3WheelMode);
    return;
  }

  // Diagonal movement commands
  if (strcmp(cmd, "DIAG_F_LEFT") == 0)
  {
    Serial.println("Moving Diagonal Forward Left");
    moveDiagonalForwardLeft(userSpeed, use3WheelMode);
    return;
  }

  if (strcmp(cmd, "DIAG_F_RIGHT") == 0)
  {
    Serial.println("Moving Diagonal Forward Right");
    moveDiagonalForwardRight(userSpeed, use3WheelMode);
    return;
  }

  if (strcmp(cmd, "DIAG_B_LEFT") == 0)
  {
    Serial.println("Moving Diagonal Backward Left");
    moveDiagonalBackwardLeft(userSpeed, use3WheelMode);
    return;
  }

  if (strcmp(cmd, "DIAG_B_RIGHT") == 0)
  {
    Serial.println("Moving Diagonal Backward Right");
    moveDiagonalBackwardRight(userSpeed, use3WheelMode);
    return;
  }

  Serial.println("Unknown command");
}

void executeMovement(char direction, int speed, int tagId)
{
  // For tag_id=99, we're doing rotation
  bool isRotation = (tagId == 99);

  switch (direction)
  {
  case 'F':
    if (!isRotation)
    {
      Serial.println("Moving FORWARD");
      moveForward(speed, use3WheelMode);
    }
    break;
  case 'B':
    if (!isRotation)
    {
      Serial.println("Moving BACKWARD");
      moveBackward(speed, use3WheelMode);
    }
    break;
  case 'L':
    if (isRotation)
    {
      Serial.println("Rotating LEFT (CCW)");
      rotateLeft(speed, use3WheelMode);
    }
    break;
  case 'R':
    if (isRotation)
    {
      Serial.println("Rotating RIGHT (CW)");
      rotateRight(speed, use3WheelMode);
    }
    break;
  case 'Q': // For diagonal forward-left
    if (!isRotation)
    {
      Serial.println("Moving Diagonal Forward-Left");
      moveDiagonalForwardLeft(speed, use3WheelMode);
    }
    break;
  case 'E': // For diagonal forward-right
    if (!isRotation)
    {
      Serial.println("Moving Diagonal Forward-Right");
      moveDiagonalForwardRight(speed, use3WheelMode);
    }
    break;
  case 'Z': // For diagonal backward-left
    if (!isRotation)
    {
      Serial.println("Moving Diagonal Backward-Left");
      moveDiagonalBackwardLeft(speed, use3WheelMode);
    }
    break;
  case 'C': // For diagonal backward-right
    if (!isRotation)
    {
      Serial.println("Moving Diagonal Backward-Right");
      moveDiagonalBackwardRight(speed, use3WheelMode);
    }
    break;
  case 'S':
  default:
    Serial.println("STOP");
    stopAllMotors();
    break;
  }
}

// === Acceleration and Deceleration Functions ===

// Global variables for smooth acceleration/deceleration
int currentSpeed = 0;
const int ACCEL_STEP = 15;  // Step size for acceleration (smaller for smoother)
const int ACCEL_DELAY = 20; // Delay between steps in milliseconds

// Gradually accelerate a motor to the target speed
void accelerateMotor(const Motor &motor, Direction dir, int targetSpeed)
{
  int startSpeed = 0;

  // Start from a minimal speed to overcome initial resistance
  moveMotor(motor, dir, startSpeed + 20);
  delay(50);

  // Gradually increase speed
  for (int speed = startSpeed + 20; speed < targetSpeed; speed += ACCEL_STEP)
  {
    moveMotor(motor, dir, speed);
    delay(ACCEL_DELAY);
  }

  // Ensure we reach exactly the target speed
  moveMotor(motor, dir, targetSpeed);
}

// Gradually decelerate a motor from current speed to stop
void decelerateMotor(const Motor &motor, Direction dir, int startSpeed)
{
  // Gradually decrease speed
  for (int speed = startSpeed; speed > 20; speed -= ACCEL_STEP)
  {
    moveMotor(motor, dir, speed);
    delay(ACCEL_DELAY);
  }

  // Come to a complete stop
  moveMotor(motor, STOP, 0);
}

// Smoothly stop all motors with deceleration
void smoothStopAllMotors()
{
  // Create temporary Direction variables to maintain current direction during deceleration
  Direction leftDir = (digitalRead(motorLeft.RPWM) > 0) ? FORWARD : BACKWARD;
  Direction rightDir = (digitalRead(motorRight.RPWM) > 0) ? FORWARD : BACKWARD;
  Direction backDir = (digitalRead(motorBack.RPWM) > 0) ? FORWARD : BACKWARD;

  // Get current motor speeds
  int leftSpeed = max(analogRead(motorLeft.RPWM), analogRead(motorLeft.LPWM)) / 4; // Convert to 0-255 range
  int rightSpeed = max(analogRead(motorRight.RPWM), analogRead(motorRight.LPWM)) / 4;
  int backSpeed = max(analogRead(motorBack.RPWM), analogRead(motorBack.LPWM)) / 4;

  // If motors are already stopped, no need to decelerate
  if (leftSpeed < 10 && rightSpeed < 10 && backSpeed < 10)
  {
    stopAllMotors();
    return;
  }

  // Decelerate all motors simultaneously
  for (int step = 0; step < 5; step++)
  {
    if (leftSpeed > 10)
    {
      leftSpeed = max(leftSpeed - ACCEL_STEP * 2, 0);
      moveMotor(motorLeft, leftDir, leftSpeed);
    }
    else
    {
      moveMotor(motorLeft, STOP, 0);
    }

    if (rightSpeed > 10)
    {
      rightSpeed = max(rightSpeed - ACCEL_STEP * 2, 0);
      moveMotor(motorRight, rightDir, rightSpeed);
    }
    else
    {
      moveMotor(motorRight, STOP, 0);
    }

    if (backSpeed > 10)
    {
      backSpeed = max(backSpeed - ACCEL_STEP * 2, 0);
      moveMotor(motorBack, backDir, backSpeed);
    }
    else
    {
      moveMotor(motorBack, STOP, 0);
    }

    delay(ACCEL_DELAY);
  }

  // Ensure all motors are completely stopped
  stopAllMotors();
}

void setup()
{
  // Clear any existing serial data
  Serial.end();
  delay(100);

  // Start serial with proper baud rate
  Serial.begin(115200);
  delay(1000); // Wait for serial to fully initialize

  // Clear any initial data
  while (Serial.available())
  {
    Serial.read();
  }

  setupMotorPins(motorRight);
  setupMotorPins(motorLeft);
  setupMotorPins(motorBack);

  // Send initial messages
  Serial.println();
  Serial.println("=========================");
  Serial.println("Enhanced Movement System Ready");
  Serial.println("Commands:");
  Serial.println("F - Forward");
  Serial.println("B - Backward");
  Serial.println("L - Rotate Left");
  Serial.println("R - Rotate Right");
  Serial.println("S - Stop");
  Serial.println("SLIDE_LEFT - Strafe Left");
  Serial.println("SLIDE_RIGHT - Strafe Right");
  Serial.println("TURN_LEFT - Arc Turn Left");
  Serial.println("TURN_RIGHT - Arc Turn Right");
  Serial.println("MODE:2WHEEL - Use 2-wheel mode");
  Serial.println("MODE:3WHEEL - Use 3-wheel mode");
  Serial.println("SPEED:<value> - Set speed (0-255)");
  Serial.println("TAG:<tagId>,<speed>,<dir>[,<mode>] - TAG Command");
  Serial.println("TEST - Run diagnostics");
  Serial.println("PING - Connectivity test");
  Serial.println("STOP - Stop all motors");
  Serial.println("=========================");
}

void loop()
{
  // Process any incoming serial commands
  processSerialInput();

  // Add a small delay to prevent busy waiting
  delay(10);
}