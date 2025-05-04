#include <Arduino.h>
#include <math.h>

// === Pin Definitions ===

// Motor Driver Pins
#define RPWM_RIGHT 3 // Updated to match configuration
#define LPWM_RIGHT 2 // Updated to match configuration
#define REN_RIGHT 39 // Kept original
#define LEN_RIGHT 38 // Kept original

#define RPWM_LEFT 4 // Updated to match configuration
#define LPWM_LEFT 5 // Updated to match configuration
#define REN_LEFT 44 // Updated to match configuration
#define LEN_LEFT 45 // Updated to match configuration

#define RPWM_BACK 7 // Updated to match configuration
#define LPWM_BACK 6 // Updated to match configuration
#define REN_BACK 51 // Kept original
#define LEN_BACK 50 // Kept original

// Basic configuration
const float OBSTACLE_DISTANCE = 15.0; // cm
const int BASE_SPEED = 150;           // PWM value (0-255)

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
  moveMotor(motorLeft, FORWARD, BASE_SPEED);
  moveMotor(motorRight, FORWARD, BASE_SPEED);
  moveMotor(motorBack, STOP, 0);
  delay(duration);
  stopAllMotors();
}

void testBackward(int duration = 1000)
{
  moveMotor(motorLeft, BACKWARD, BASE_SPEED);
  moveMotor(motorRight, BACKWARD, BASE_SPEED);
  moveMotor(motorBack, STOP, 0);
  delay(duration);
  stopAllMotors();
}

void testRotateLeft(int duration = 1000)
{
  moveMotor(motorLeft, BACKWARD, BASE_SPEED);
  moveMotor(motorRight, FORWARD, BASE_SPEED);
  moveMotor(motorBack, FORWARD, BASE_SPEED);
  delay(duration);
  stopAllMotors();
}

void testRotateRight(int duration = 1000)
{
  moveMotor(motorLeft, FORWARD, BASE_SPEED);
  moveMotor(motorRight, BACKWARD, BASE_SPEED);
  moveMotor(motorBack, BACKWARD, BASE_SPEED);
  delay(duration);
  stopAllMotors();
}

void setup()
{
  Serial.begin(115200);
  setupMotorPins(motorRight);
  setupMotorPins(motorLeft);
  setupMotorPins(motorBack);
  Serial.println("Basic Movement Test System Ready");
  Serial.println("Commands:");
  Serial.println("F - Forward");
  Serial.println("B - Backward");
  Serial.println("L - Rotate Left");
  Serial.println("R - Rotate Right");
  Serial.println("S - Stop");
}

void loop()
{
  if (Serial.available())
  {
    char cmd = Serial.read();
    switch (cmd)
    {
    case 'F':
      Serial.println("Testing Forward Movement");
      testForward();
      break;
    case 'B':
      Serial.println("Testing Backward Movement");
      testBackward();
      break;
    case 'L':
      Serial.println("Testing Left Rotation");
      testRotateLeft();
      break;
    case 'R':
      Serial.println("Testing Right Rotation");
      testRotateRight();
      break;
    case 'S':
      Serial.println("Stopping");
      stopAllMotors();
      break;
    }
  }
}
