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

// Encoder Pins
#define ENC_RIGHT_C1 40 // Was LEFT
#define ENC_RIGHT_C2 41 // Was LEFT
#define ENC_LEFT_C1 46  // Was RIGHT
#define ENC_LEFT_C2 47  // Was RIGHT
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

// === Globals ===
float distFL, distF, distFR, distBL, distB, distBR;
const float OBSTACLE_DISTANCE = 15.0; // cm, can adjust
bool lastObstacle = false;

// Motor Driver Pin Definitions
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

// Motor direction enum
enum Direction
{
  FORWARD,
  BACKWARD,
  STOP
};

// === Motor Control ===
void setupMotorPins(const Motor &motor)
{
  pinMode(motor.RPWM, OUTPUT);
  pinMode(motor.LPWM, OUTPUT);
  pinMode(motor.REN, OUTPUT);
  pinMode(motor.LEN, OUTPUT);

  // Enable H-Bridge
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

void moveRobot(float vy, float vx, float omega, int speed = 150)
{
  // Convert the movement vector into motor commands
  if (abs(vy) > 0.1)
  { // Vertical Axis movement
    if (vy > 0)
    { // Move Forward
      moveMotor(motorLeft, FORWARD, speed);
      moveMotor(motorRight, FORWARD, speed);
      moveMotor(motorBack, STOP, 0);
    }
    else
    { // Move Backwards
      moveMotor(motorLeft, BACKWARD, speed);
      moveMotor(motorRight, BACKWARD, speed);
      moveMotor(motorBack, STOP, 0);
    }
  }
  else if (abs(vx) > 0.1)
  { // Horizontal Axis movement
    if (vx > 0)
    { // Move right
      moveMotor(motorLeft, FORWARD, speed);
      moveMotor(motorRight, STOP, 0);
      moveMotor(motorBack, FORWARD, speed);
    }
    else
    { // Move left
      moveMotor(motorLeft, STOP, 0);
      moveMotor(motorRight, FORWARD, speed);
      moveMotor(motorBack, BACKWARD, speed);
    }
  }
  else if (abs(omega) > 0.1)
  { // Rotation
    if (omega > 0)
    { // Rotate right
      moveMotor(motorLeft, FORWARD, speed);
      moveMotor(motorRight, BACKWARD, speed);
      moveMotor(motorBack, BACKWARD, speed);
    }
    else
    { // Rotate left
      moveMotor(motorLeft, BACKWARD, speed);
      moveMotor(motorRight, FORWARD, speed);
      moveMotor(motorBack, FORWARD, speed);
    }
  }
  else
  { // Stop if no significant movement
    stopAllMotors();
  }
}

void stopAllMotors()
{
  moveMotor(motorLeft, STOP, 0);
  moveMotor(motorRight, STOP, 0);
  moveMotor(motorBack, STOP, 0);
}

// === Setup ===
void setup()
{
  Serial.begin(9600);

  setupMotorPins(motorRight);
  setupMotorPins(motorLeft);
  setupMotorPins(motorBack);

  pinMode(ENC_RIGHT_C1, INPUT);
  pinMode(ENC_RIGHT_C2, INPUT);
  pinMode(ENC_LEFT_C1, INPUT);
  pinMode(ENC_LEFT_C2, INPUT);
  pinMode(ENC_BACK_C1, INPUT);
  pinMode(ENC_BACK_C2, INPUT);

  int trigPins[] = {TRIG_FL, TRIG_F, TRIG_FR, TRIG_BL, TRIG_B, TRIG_BR};
  int echoPins[] = {ECHO_FL, ECHO_F, ECHO_FR, ECHO_BL, ECHO_B, ECHO_BR};
  for (int i = 0; i < 6; i++)
  {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  Serial.println("ROBOT READY");
}

// === Distance Sensing ===
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
}

// === Main Loop ===
void loop()
{
  updateDistances();

  Serial.print("FL: ");
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

  bool triggered = false;

  // Highest priority: Front Center
  if (distF < OBSTACLE_DISTANCE)
  {
    Serial.println("F triggered - move back");
    moveRobot(-1.0, 0, 0)
    triggered = true;
  }
  // High priority: Front Left
  else if (distFL < OBSTACLE_DISTANCE)
  {
    Serial.println("FL triggered - turn right");
    moveRobot(0, 0, 1.0)
    triggered = true;
  }
  // High priority: Front Right
  else if (distFR < OBSTACLE_DISTANCE)
  {
    Serial.println("FR triggered - turn left");
    moveRobot(0, 0, -1.0)
    triggered = true;
  }
  // Medium priority: Back
  else if (distB < OBSTACLE_DISTANCE)
  {
    Serial.println("B triggered - move forward");
    moveRobot(1.0, 0, 0)
    triggered = true;
  }
  // Lower priority: Back Left
  else if (distBL < OBSTACLE_DISTANCE)
  {
    Serial.println("BL triggered - slide right");
    moveRobot(0, 1.0, 0)
    triggered = true;
  }
  // Lower priority: Back Right
  else if (distBR < OBSTACLE_DISTANCE)
  {
    Serial.println("BR triggered - slide left");
    moveRobot(0, -1.0, 0)
    triggered = true;
  }

  if (!triggered)
  {
    stopAllMotors();
    Serial.println("No movement - all sensors clear");
  }

  delay(200);
}
