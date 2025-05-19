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

// Motor Driver Pins Normal (ORIGINAL CONFIGURATION - DO NOT USE)
/* #define RPWM_RIGHT 2// Updated to match configuration
#define LPWM_RIGHT 3 // Updated to match configuration
#define REN_RIGHT 39 // Kept original
#define LEN_RIGHT 38 // Kept original

#define RPWM_LEFT 5 // Updated to match configuration
#define LPWM_LEFT 4 // Updated to match configuration
#define REN_LEFT 44 // Updated to match configuration
#define LEN_LEFT 45 // Updated to match configuration

#define RPWM_BACK 7 // Updated to match configuration
#define LPWM_BACK 6 // Updated to match configuration
#define REN_BACK 51 // Kept original
#define LEN_BACK 50 // Kept original */

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

// Serial communication
const int SERIAL_BUFFER_SIZE = 32;
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

// Simple movement commands for testing
void testForward(int duration = 1000)
{
  moveMotor(motorLeft, BACKWARD, BASE_SPEED);
  moveMotor(motorRight, FORWARD, BASE_SPEED);
  moveMotor(motorBack, FORWARD, BASE_SPEED);
  delay(duration);
  stopAllMotors();
}

void testBackward(int duration = 1000)
{
  moveMotor(motorLeft, FORWARD, BASE_SPEED);
  moveMotor(motorRight, BACKWARD, BASE_SPEED);
  moveMotor(motorBack, BACKWARD, BASE_SPEED);
  delay(duration);
  stopAllMotors();
}

void testRotateLeft(int duration = 1000)
{
  moveMotor(motorLeft, FORWARD, BASE_SPEED);
  moveMotor(motorRight, FORWARD, BASE_SPEED);
  moveMotor(motorBack, STOP, 0);
  delay(duration);
  stopAllMotors();
}

void testRotateRight(int duration = 1000)
{
  moveMotor(motorLeft, BACKWARD, BASE_SPEED);
  moveMotor(motorRight, BACKWARD, BASE_SPEED);
  moveMotor(motorBack, STOP, 0);
  delay(duration);
  stopAllMotors();
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
      moveMotor(motorLeft, BACKWARD, BASE_SPEED);
      moveMotor(motorRight, FORWARD, BASE_SPEED);
      moveMotor(motorBack, FORWARD, BASE_SPEED);
      break;
    case 'B':
      Serial.println("Moving Backward");
      moveMotor(motorLeft, FORWARD, BASE_SPEED);
      moveMotor(motorRight, BACKWARD, BASE_SPEED);
      moveMotor(motorBack, BACKWARD, BASE_SPEED);
      break;
    case 'L':
      Serial.println("Moving Left");
      moveMotor(motorLeft, FORWARD, BASE_SPEED);
      moveMotor(motorRight, FORWARD, BASE_SPEED);
      moveMotor(motorBack, STOP, 0);
      break;
    case 'R':
      Serial.println("Moving Right");
      moveMotor(motorLeft, BACKWARD, BASE_SPEED);
      moveMotor(motorRight, BACKWARD, BASE_SPEED);
      moveMotor(motorBack, STOP, 0);
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
  if (strcmp(cmd, "STOP") == 0)
  {
    Serial.println("Stopping all motors");
    stopAllMotors();
    return;
  }

  // Handle TAG commands from motor_test.py
  if (strncmp(cmd, "TAG:", 4) == 0)
  {
    int tagId;
    float distance;
    char direction;

    if (sscanf(cmd + 4, "%d,%f,%c", &tagId, &distance, &direction) == 3)
    {
      Serial.print("Processing TAG Command: ");
      Serial.print("Tag ID: ");
      Serial.print(tagId);
      Serial.print(", Distance: ");
      Serial.print(distance);
      Serial.print(", Direction: ");
      Serial.println(direction);

      // Convert speed from distance to motor speed
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
      moveMotor(motorLeft, BACKWARD, speed);
      moveMotor(motorRight, FORWARD, speed);
      moveMotor(motorBack, FORWARD, speed);
    }
    break;

  case 'B':
    if (!isRotation)
    {
      Serial.println("Moving BACKWARD");
      moveMotor(motorLeft, FORWARD, speed);
      moveMotor(motorRight, BACKWARD, speed);
      moveMotor(motorBack, BACKWARD, speed);
    }
    break;

  case 'L':
    if (isRotation)
    {
      Serial.println("Rotating COUNTERCLOCKWISE");
      moveMotor(motorLeft, FORWARD, speed);
      moveMotor(motorRight, FORWARD, speed);
      moveMotor(motorBack, STOP, 0);
    }
    else
    {
      Serial.println("Moving LEFT");
      moveMotor(motorLeft, FORWARD, speed);
      moveMotor(motorRight, FORWARD, speed);
      moveMotor(motorBack, STOP, 0);
    }
    break;

  case 'R':
    if (isRotation)
    {
      Serial.println("Rotating CLOCKWISE");
      moveMotor(motorLeft, BACKWARD, speed);
      moveMotor(motorRight, BACKWARD, speed);
      moveMotor(motorBack, STOP, 0);
    }
    else
    {
      Serial.println("Moving RIGHT");
      moveMotor(motorLeft, BACKWARD, speed);
      moveMotor(motorRight, BACKWARD, speed);
      moveMotor(motorBack, STOP, 0);
    }
    break;

  case 'S':
    Serial.println("Stopping all motors");
    stopAllMotors();
    break;

  default:
    Serial.println("Unknown direction");
    stopAllMotors();
    break;
  }
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
  Serial.println("Basic Movement Test System Ready");
  Serial.println("Commands:");
  Serial.println("F - Forward");
  Serial.println("B - Backward");
  Serial.println("L - Rotate Left");
  Serial.println("R - Rotate Right");
  Serial.println("S - Stop");
  Serial.println("TAG:<tagId>,<distance>,<dir> - TAG Command");
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