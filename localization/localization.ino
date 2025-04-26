// ---- Motor Pins ----
// Front motor
#define RPWM_F 37
#define LPWM_F 36
#define REN_F 39
#define LEN_F 38

// Left motor
#define RPWM_L 43
#define LPWM_L 42
#define REN_L 45
#define LEN_L 44

// Right motor
#define RPWM_R 49
#define LPWM_R 48
#define REN_R 51
#define LEN_R 50

// ---- Encoder Pins ----
#define ENC_F_C1 40
#define ENC_F_C2 41
#define ENC_L_C1 46
#define ENC_L_C2 47
#define ENC_R_C1 52
#define ENC_R_C2 53

// ---- Ultrasonic Sensor Pins ----
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

// ---- Encoder & PID Variables ----
volatile long encoderCountF = 0;
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

float targetRPM_F = 0, targetRPM_L = 0, targetRPM_R = 0;
float lastErrorF = 0, lastErrorL = 0, lastErrorR = 0;
float integralF = 0, integralL = 0, integralR = 0;
unsigned long lastPIDTime = 0;
const int sampleTime = 100; // ms

// PID tuning
const float Kp = 1.5;
const float Ki = 0.2;
const float Kd = 0.1;

// ---- Encoder ISRs ----
void IRAM_ATTR encoderF() { encoderCountF++; }
void IRAM_ATTR encoderL() { encoderCountL++; }
void IRAM_ATTR encoderR() { encoderCountR++; }

// ---- Setup ----
void setup() {
  Serial.begin(9600);

  // Motor & encoder setup
  pinMode(RPWM_F, OUTPUT); pinMode(LPWM_F, OUTPUT);
  pinMode(REN_F, OUTPUT);  pinMode(LEN_F, OUTPUT);
  pinMode(RPWM_L, OUTPUT); pinMode(LPWM_L, OUTPUT);
  pinMode(REN_L, OUTPUT);  pinMode(LEN_L, OUTPUT);
  pinMode(RPWM_R, OUTPUT); pinMode(LPWM_R, OUTPUT);
  pinMode(REN_R, OUTPUT);  pinMode(LEN_R, OUTPUT);
  digitalWrite(REN_F, HIGH); digitalWrite(LEN_F, HIGH);
  digitalWrite(REN_L, HIGH); digitalWrite(LEN_L, HIGH);
  digitalWrite(REN_R, HIGH); digitalWrite(LEN_R, HIGH);

  pinMode(ENC_F_C1, INPUT); attachInterrupt(digitalPinToInterrupt(ENC_F_C1), encoderF, RISING);
  pinMode(ENC_L_C1, INPUT); attachInterrupt(digitalPinToInterrupt(ENC_L_C1), encoderL, RISING);
  pinMode(ENC_R_C1, INPUT); attachInterrupt(digitalPinToInterrupt(ENC_R_C1), encoderR, RISING);

  // Ultrasonic sensors
  int trigPins[] = {TRIG_FL, TRIG_F, TRIG_FR, TRIG_BL, TRIG_B, TRIG_BR};
  int echoPins[] = {ECHO_FL, ECHO_F, ECHO_FR, ECHO_BL, ECHO_B, ECHO_BR};

  for (int i = 0; i < 6; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}

// ---- Motor Helper ----
void setMotor(int rpwm, int lpwm, float speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    analogWrite(rpwm, speed);
    analogWrite(lpwm, 0);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, -speed);
  }
}

// ---- Ultrasonic Read ----
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout: 30ms
  return duration * 0.034 / 2; // cm
}

// ---- Obstacle Detection ----
bool obstacleDetected() {
  long front = readUltrasonic(TRIG_F, ECHO_F);
  long fl = readUltrasonic(TRIG_FL, ECHO_FL);
  long fr = readUltrasonic(TRIG_FR, ECHO_FR);

  return (front < 20 || fl < 20 || fr < 20);
}

// ---- Holonomic Movement Control ----
// vx = forward/backward, vy = lateral (strafe), w = rotation
void setTargetVelocity(float vx, float vy, float w) {
  // Inverse kinematics for 3-wheel holonomic
  float wheelF = vx;
  float wheelL = (-0.5 * vx + 0.866 * vy);  // sin(60°) = 0.866
  float wheelR = (-0.5 * vx - 0.866 * vy);

  // Add rotation (assuming symmetrical)
  wheelF += w;
  wheelL += w;
  wheelR += w;

  // Scale to RPM (adjust factor to your robot's max RPM)
  float scale = 100;
  targetRPM_F = wheelF * scale;
  targetRPM_L = wheelL * scale;
  targetRPM_R = wheelR * scale;
}

// ---- PID Update ----
void updatePID() {
  unsigned long now = millis();
  if (now - lastPIDTime >= sampleTime) {
    float deltaTime = (now - lastPIDTime) / 1000.0;

    float rpmF = (encoderCountF / 2.0) * (60.0 / deltaTime); encoderCountF = 0;
    float rpmL = (encoderCountL / 2.0) * (60.0 / deltaTime); encoderCountL = 0;
    float rpmR = (encoderCountR / 2.0) * (60.0 / deltaTime); encoderCountR = 0;

    float errorF = targetRPM_F - rpmF;
    float errorL = targetRPM_L - rpmL;
    float errorR = targetRPM_R - rpmR;

    integralF += errorF * deltaTime;
    integralL += errorL * deltaTime;
    integralR += errorR * deltaTime;

    float derivativeF = (errorF - lastErrorF) / deltaTime;
    float derivativeL = (errorL - lastErrorL) / deltaTime;
    float derivativeR = (errorR - lastErrorR) / deltaTime;

    float outputF = Kp * errorF + Ki * integralF + Kd * derivativeF;
    float outputL = Kp * errorL + Ki * integralL + Kd * derivativeL;
    float outputR = Kp * errorR + Ki * integralR + Kd * derivativeR;

    lastErrorF = errorF;
    lastErrorL = errorL;
    lastErrorR = errorR;

    setMotor(RPWM_F, LPWM_F, outputF);
    setMotor(RPWM_L, LPWM_L, outputL);
    setMotor(RPWM_R, LPWM_R, outputR);

    lastPIDTime = now;
  }
}

// ---- Additional Constants ----
const int EMERGENCY_DISTANCE = 15; // cm
const int SLOWDOWN_DISTANCE = 30;  // cm

// ---- Obstacle Priority Levels ----
enum ObstacleState {
  CLEAR,
  SLOWDOWN,
  EMERGENCY_STOP
};

// ---- Check All Zones ----
ObstacleState checkObstacles() {
  long distances[6];
  distances[0] = readUltrasonic(TRIG_FL, ECHO_FL);
  distances[1] = readUltrasonic(TRIG_F, ECHO_F);
  distances[2] = readUltrasonic(TRIG_FR, ECHO_FR);
  distances[3] = readUltrasonic(TRIG_BL, ECHO_BL);
  distances[4] = readUltrasonic(TRIG_B, ECHO_B);
  distances[5] = readUltrasonic(TRIG_BR, ECHO_BR);

  bool emergency = false;
  bool slowdown = false;

  for (int i = 0; i < 6; i++) {
    if (distances[i] > 0) { // valid reading
      if (distances[i] <= EMERGENCY_DISTANCE) {
        emergency = true;
        break; // no need to check further
      } else if (distances[i] <= SLOWDOWN_DISTANCE) {
        slowdown = true;
      }
    }
  }

  if (emergency) return EMERGENCY_STOP;
  if (slowdown) return SLOWDOWN;
  return CLEAR;
}

// ---- Main Loop ----
void loop() {
  // Base movement command
  float vx = 1.0;  // forward
  float vy = 0.0;  // no strafe
  float w  = 0.0;  // no rotation

  ObstacleState state = checkObstacles();

  switch (state) {
    case EMERGENCY_STOP:
      Serial.println("🚨 EMERGENCY STOP");
      setTargetVelocity(0, 0, 0);
      break;
    case SLOWDOWN:
      Serial.println("⚠️ Slowdown");
      setTargetVelocity(vx * 0.4, vy * 0.4, w * 0.4); // Reduce speed
      break;
    case CLEAR:
      setTargetVelocity(vx, vy, w);
      break;
  }

  updatePID();
}
