// === Pin Definitions ===

// Motor Driver Pins
#define RPWM_F 37
#define LPWM_F 36
#define REN_F 39
#define LEN_F 38

#define RPWM_L 43
#define LPWM_L 42
#define REN_L 45
#define LEN_L 44

#define RPWM_R 49
#define LPWM_R 48
#define REN_R 51
#define LEN_R 50

// Encoder Pins
#define ENC_F_C1 40
#define ENC_F_C2 41
#define ENC_L_C1 46
#define ENC_L_C2 47
#define ENC_R_C1 52
#define ENC_R_C2 53

// Ultrasonic Sensor Pins
#define TRIG_FL 22
#define ECHO_FL 23
#define TRIG_F 24
#define ECHO_F 25
#define TRIG_FR 26
#define ECHO_FR 27
#define TRIG_BL 28
#define ECHO_BL 29
#define TRIG_B 30
#define ECHO_B 31
#define TRIG_BR 32
#define ECHO_BR 33

// === Globals ===
float distFL, distF, distFR, distBL, distB, distBR;
const float OBSTACLE_DISTANCE = 15.0; // cm, can adjust
bool lastObstacle = false;

// === Setup ===
void setup()
{
  Serial.begin(9600);

  int motorPins[] = {
      RPWM_F, LPWM_F, REN_F, LEN_F,
      RPWM_L, LPWM_L, REN_L, LEN_L,
      RPWM_R, LPWM_R, REN_R, LEN_R};
  for (int i = 0; i < 12; i++)
    pinMode(motorPins[i], OUTPUT);

  digitalWrite(REN_F, HIGH);
  digitalWrite(LEN_F, HIGH);
  digitalWrite(REN_L, HIGH);
  digitalWrite(LEN_L, HIGH);
  digitalWrite(REN_R, HIGH);
  digitalWrite(LEN_R, HIGH);

  pinMode(ENC_F_C1, INPUT);
  pinMode(ENC_F_C2, INPUT);
  pinMode(ENC_L_C1, INPUT);
  pinMode(ENC_L_C2, INPUT);
  pinMode(ENC_R_C1, INPUT);
  pinMode(ENC_R_C2, INPUT);

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

// === Motor Control ===
void setMotor(int rpwm, int lpwm, float motorSpeed)
{
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0)
  {
    analogWrite(rpwm, motorSpeed);
    analogWrite(lpwm, 0);
  }
  else
  {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, -motorSpeed);
  }
}

void moveRobot(float vx, float vy, float omega, int speed = 180)
{
  float v_front = -vx;
  float v_left = 0.5 * vx - 0.866 * vy;
  float v_right = 0.5 * vx + 0.866 * vy;

  v_front += omega;
  v_left += omega;
  v_right += omega;

  float maxVal = max(max(abs(v_front), abs(v_left)), abs(v_right));
  if (maxVal > 1.0)
  {
    v_front /= maxVal;
    v_left /= maxVal;
    v_right /= maxVal;
  }

  setMotor(RPWM_F, LPWM_F, v_front * speed);
  setMotor(RPWM_L, LPWM_L, v_left * speed);
  setMotor(RPWM_R, LPWM_R, v_right * speed);
}

void stopAllMotors()
{
  analogWrite(RPWM_F, 0);
  analogWrite(LPWM_F, 0);
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_L, 0);
  analogWrite(RPWM_R, 0);
  analogWrite(LPWM_R, 0);
}

// === Main Logic ===
void loop()
{
  updateDistances();

  bool obstacleFront = (distFL < OBSTACLE_DISTANCE) || (distF < OBSTACLE_DISTANCE) || (distFR < OBSTACLE_DISTANCE);
  bool obstacleBack = (distBL < OBSTACLE_DISTANCE) || (distB < OBSTACLE_DISTANCE) || (distBR < OBSTACLE_DISTANCE);
  bool obstacleLeft = (distFL < OBSTACLE_DISTANCE) || (distBL < OBSTACLE_DISTANCE);
  bool obstacleRight = (distFR < OBSTACLE_DISTANCE) || (distBR < OBSTACLE_DISTANCE);

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

  if (obstacleFront)
  {
    Serial.println("Obstacle ahead - moving backward");
    moveRobot(-1.0, 0.0, 0.0);
    lastObstacle = true;
  }
  else if (obstacleBack)
  {
    Serial.println("Obstacle behind - moving forward");
    moveRobot(1.0, 0.0, 0.0);
    lastObstacle = true;
  }
  else if (obstacleLeft)
  {
    Serial.println("Obstacle left - sliding right");
    moveRobot(0.0, 1.0, 0.0);
    lastObstacle = true;
  }
  else if (obstacleRight)
  {
    Serial.println("Obstacle right - sliding left");
    moveRobot(0.0, -1.0, 0.0);
    lastObstacle = true;
  }
  else
  {
    // No obstacles
    if (lastObstacle)
    {
      stopAllMotors();
      delay(100); // brief pause after obstacle clearance
      lastObstacle = false;
    }
    Serial.println("Path clear - moving forward");
    moveRobot(1.0, 0.0, 0.0);
  }

  delay(100);
}
