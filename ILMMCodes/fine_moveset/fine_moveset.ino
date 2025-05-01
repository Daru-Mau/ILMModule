#include <Arduino.h>
#include <math.h>

// === Pin Definitions === (same as basic_moveset)
#define RPWM_RIGHT 3
#define LPWM_RIGHT 2
#define REN_RIGHT 39
#define LEN_RIGHT 38
#define RPWM_LEFT 4
#define LPWM_LEFT 5
#define REN_LEFT 44
#define LEN_LEFT 45
#define RPWM_BACK 7
#define LPWM_BACK 6
#define REN_BACK 51
#define LEN_BACK 50

// Encoder Pins
#define ENC_RIGHT_C1 40
#define ENC_RIGHT_C2 41
#define ENC_LEFT_C1 46
#define ENC_LEFT_C2 47
#define ENC_BACK_C1 52
#define ENC_BACK_C2 53

// Ultrasonic Sensor Pins
#define TRIG_F 22
#define ECHO_F 23
#define TRIG_FL 24
#define ECHO_FL 25
#define TRIG_FR 26
#define ECHO_FR 27
#define TRIG_BL 28
#define ECHO_BL 29
#define TRIG_BR 30
#define ECHO_BR 31
#define TRIG_B 32
#define ECHO_B 33

// === Enhanced Configuration ===
const float CRITICAL_DISTANCE = 10.0;  // Emergency stop distance (cm)
const float SLOW_DOWN_DISTANCE = 30.0; // Start slowing down distance (cm)
const float SAFE_DISTANCE = 50.0;      // Safe operating distance (cm)
const int MAX_SPEED = 255;             // Maximum motor speed
const int MIN_SPEED = 50;              // Minimum effective motor speed
const float ACCELERATION_RATE = 0.15;  // Speed change per cycle (0-1)
const int REACTION_DELAY = 50;         // Milliseconds between updates

// === Globals ===
float distFL, distF, distFR, distBL, distB, distBR;
float currentSpeed = 0.0;
unsigned long lastUpdateTime = 0;
bool emergencyStop = false;

// Motor Driver Pin Definitions
struct Motor
{
  int RPWM;
  int LPWM;
  int REN;
  int LEN;
  float currentSpeed;
};

Motor motorRight = {RPWM_RIGHT, LPWM_RIGHT, REN_RIGHT, LEN_RIGHT, 0};
Motor motorLeft = {RPWM_LEFT, LPWM_LEFT, REN_LEFT, LEN_LEFT, 0};
Motor motorBack = {RPWM_BACK, LPWM_BACK, REN_BACK, LEN_BACK, 0};

enum Direction
{
  FORWARD,
  BACKWARD,
  STOP
};

// === Enhanced Motor Control ===
void setupMotorPins(Motor &motor)
{
  pinMode(motor.RPWM, OUTPUT);
  pinMode(motor.LPWM, OUTPUT);
  pinMode(motor.REN, OUTPUT);
  pinMode(motor.LEN, OUTPUT);
  digitalWrite(motor.REN, HIGH);
  digitalWrite(motor.LEN, HIGH);
}

float calculateDynamicSpeed(float distance, float targetSpeed)
{
  if (distance <= CRITICAL_DISTANCE)
  {
    return 0; // Emergency stop
  }
  else if (distance <= SLOW_DOWN_DISTANCE)
  {
    // Linear interpolation between MIN_SPEED and targetSpeed
    float factor = (distance - CRITICAL_DISTANCE) / (SLOW_DOWN_DISTANCE - CRITICAL_DISTANCE);
    return MIN_SPEED + (targetSpeed - MIN_SPEED) * factor;
  }
  return targetSpeed;
}

void moveMotor(Motor &motor, Direction dir, float targetSpeed)
{
  targetSpeed = constrain(targetSpeed, 0, MAX_SPEED);

  // Smooth acceleration/deceleration
  if (targetSpeed > motor.currentSpeed)
  {
    motor.currentSpeed = min(targetSpeed, motor.currentSpeed + MAX_SPEED * ACCELERATION_RATE);
  }
  else if (targetSpeed < motor.currentSpeed)
  {
    motor.currentSpeed = max(targetSpeed, motor.currentSpeed - MAX_SPEED * ACCELERATION_RATE);
  }

  int speed = (int)motor.currentSpeed;

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
    motor.currentSpeed = 0;
    break;
  }
}

void moveRobot(float vy, float vx, float omega, float baseSpeed = 150)
{
  // Get minimum distance in movement direction
  float forwardDist = min(min(distFL, distF), distFR);
  float backwardDist = min(min(distBL, distB), distBR);
  float leftDist = min(distFL, distBL);
  float rightDist = min(distFR, distBR);

  // Calculate magnitude of movement for speed scaling
  float magnitude = max(max(abs(vy), abs(vx)), abs(omega));
  // Scale baseSpeed by magnitude, but keep it within MIN_SPEED and MAX_SPEED
  float scaledBaseSpeed = constrain(baseSpeed * magnitude, MIN_SPEED, MAX_SPEED);

  // Calculate dynamic speeds based on obstacles
  float targetSpeed = scaledBaseSpeed;

  if (emergencyStop)
  {
    stopAllMotors();
    return;
  }

  // Adjust speeds based on direction and nearby obstacles
  if (abs(vy) > 0.1)
  {
    targetSpeed = calculateDynamicSpeed(vy > 0 ? forwardDist : backwardDist, scaledBaseSpeed);
    if (vy > 0)
    {
      moveMotor(motorLeft, FORWARD, targetSpeed);
      moveMotor(motorRight, FORWARD, targetSpeed);
      moveMotor(motorBack, STOP, 0);
    }
    else
    {
      moveMotor(motorLeft, BACKWARD, targetSpeed);
      moveMotor(motorRight, BACKWARD, targetSpeed);
      moveMotor(motorBack, STOP, 0);
    }
  }
  else if (abs(vx) > 0.1)
  {
    targetSpeed = calculateDynamicSpeed(vx > 0 ? rightDist : leftDist, scaledBaseSpeed);
    if (vx > 0)
    {
      moveMotor(motorLeft, FORWARD, targetSpeed);
      moveMotor(motorRight, STOP, 0);
      moveMotor(motorBack, FORWARD, targetSpeed);
    }
    else
    {
      moveMotor(motorLeft, STOP, 0);
      moveMotor(motorRight, FORWARD, targetSpeed);
      moveMotor(motorBack, BACKWARD, targetSpeed);
    }
  }
  else if (abs(omega) > 0.1)
  {
    // For rotation, check all distances
    float minDist = min(min(forwardDist, backwardDist), min(leftDist, rightDist));
    targetSpeed = calculateDynamicSpeed(minDist, scaledBaseSpeed * 0.7); // Rotate slower

    if (omega > 0)
    {
      moveMotor(motorLeft, FORWARD, targetSpeed);
      moveMotor(motorRight, BACKWARD, targetSpeed);
      moveMotor(motorBack, BACKWARD, targetSpeed);
    }
    else
    {
      moveMotor(motorLeft, BACKWARD, targetSpeed);
      moveMotor(motorRight, FORWARD, targetSpeed);
      moveMotor(motorBack, FORWARD, targetSpeed);
    }
  }
  else
  {
    stopAllMotors();
  }
}

void stopAllMotors()
{
  moveMotor(motorLeft, STOP, 0);
  moveMotor(motorRight, STOP, 0);
  moveMotor(motorBack, STOP, 0);
}

float readDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  return (duration <= 0) ? 999.0 : duration * 0.034 / 2.0;
}

void updateDistances()
{
  distFL = readDistance(TRIG_FL, ECHO_FL);
  distF = readDistance(TRIG_F, ECHO_F);
  distFR = readDistance(TRIG_FR, ECHO_FR);
  distBL = readDistance(TRIG_BL, ECHO_BL);
  distB = readDistance(TRIG_B, ECHO_B);
  distBR = readDistance(TRIG_BR, ECHO_BR);

  // Check for emergency stop condition
  emergencyStop = (distF < CRITICAL_DISTANCE || distFL < CRITICAL_DISTANCE ||
                   distFR < CRITICAL_DISTANCE || distB < CRITICAL_DISTANCE ||
                   distBL < CRITICAL_DISTANCE || distBR < CRITICAL_DISTANCE);
}

void setup()
{
  Serial.begin(9600);

  setupMotorPins(motorRight);
  setupMotorPins(motorLeft);
  setupMotorPins(motorBack);

  // Setup encoder pins
  pinMode(ENC_RIGHT_C1, INPUT);
  pinMode(ENC_RIGHT_C2, INPUT);
  pinMode(ENC_LEFT_C1, INPUT);
  pinMode(ENC_LEFT_C2, INPUT);
  pinMode(ENC_BACK_C1, INPUT);
  pinMode(ENC_BACK_C2, INPUT);

  // Setup ultrasonic sensor pins
  int trigPins[] = {TRIG_FL, TRIG_F, TRIG_FR, TRIG_BL, TRIG_B, TRIG_BR};
  int echoPins[] = {ECHO_FL, ECHO_F, ECHO_FR, ECHO_BL, ECHO_B, ECHO_BR};
  for (int i = 0; i < 6; i++)
  {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  Serial.println("Enhanced Movement System Ready");
}

// === Movement Controller Functions ===
void moveForward(float speed = 1.0)
{
  moveRobot(speed, 0, 0);
}

void moveBackward(float speed = 1.0)
{
  moveRobot(-speed, 0, 0);
}

void moveLeft(float speed = 1.0)
{
  moveRobot(0, -speed, 0);
}

void moveRight(float speed = 1.0)
{
  moveRobot(0, speed, 0);
}

void rotateLeft(float speed = 0.7)
{
  moveRobot(0, 0, -speed);
}

void rotateRight(float speed = 0.7)
{
  moveRobot(0, 0, speed);
}

void moveForwardLeft(float speed = 0.7)
{
  moveRobot(speed, -speed, 0);
}

void moveForwardRight(float speed = 0.7)
{
  moveRobot(speed, speed, 0);
}

void moveBackwardLeft(float speed = 0.7)
{
  moveRobot(-speed, -speed, 0);
}

void moveBackwardRight(float speed = 0.7)
{
  moveRobot(-speed, speed, 0);
}

void stop()
{
  stopAllMotors();
}

void loop()
{
  unsigned long currentTime = millis();

  // Update sensor readings at fixed intervals
  if (currentTime - lastUpdateTime >= REACTION_DELAY)
  {
    updateDistances();
    lastUpdateTime = currentTime;

    // Debug output
    Serial.print("Distances (cm) - FL: ");
    Serial.print(distFL);
    Serial.print(" F: ");
    Serial.print(distF);
    Serial.print(" FR: ");
    Serial.print(distFR);
    Serial.print(" BL: ");
    Serial.print(distBL);
    Serial.print(" B: ");
    Serial.print(distB);
    Serial.print(" BR: ");
    Serial.println(distBR);

    if (emergencyStop)
    {
      Serial.println("EMERGENCY STOP ACTIVATED");
      stop();
    }
    else
    {
      float minFrontDist = min(min(distFL, distF), distFR);

      if (minFrontDist < SLOW_DOWN_DISTANCE)
      {
        // If approaching obstacle, try to find clear path
        if (distFL > distFR && distFL > SAFE_DISTANCE)
        {
          moveForwardLeft(0.5); // Turn left while moving forward
        }
        else if (distFR > distFL && distFR > SAFE_DISTANCE)
        {
          moveForwardRight(0.5); // Turn right while moving forward
        }
        else
        {
          moveBackward(0.5); // Back up if no clear path
        }
      }
      else
      {
        moveForward(1.0); // Move forward if path is clear
      }
    }
  }
}
