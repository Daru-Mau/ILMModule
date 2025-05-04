/*
 * Enhanced Robot Localization System
 *
 * This sketch provides improved localization capabilities for an Arduino-based robot
 * by fusing data from multiple sensors: encoders, IMU, and AprilTag detections.
 */

// Include required libraries
#include <Wire.h>
#include <MPU6050.h> // Ensure correct MPU6050 library

// Pin definitions for encoders
#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 3
#define RIGHT_ENCODER_A 18
#define RIGHT_ENCODER_B 19

// Robot physical parameters
#define WHEEL_DIAMETER 65.0       // in mm
#define WHEEL_BASE 150.0          // distance between wheels in mm
#define TICKS_PER_REVOLUTION 1120 // encoder ticks per wheel revolution (adjust for your encoders)

// IMU parameters
#define IMU_UPDATE_INTERVAL 20       // ms
#define POSITION_UPDATE_INTERVAL 100 // ms

// Communication
#define SERIAL_BAUD_RATE 115200

// Global variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
long prevLeftCount = 0;
long prevRightCount = 0;

// Position and orientation
float x_pos = 0.0;
float y_pos = 0.0;
float theta = 0.0; // radians

// IMU object
MPU6050 imu;

// Timing
unsigned long lastIMUUpdate = 0;
unsigned long lastPositionUpdate = 0;

// AprilTag data
bool tagDetected = false;
bool tagUpdated = false;
unsigned long lastTagUpdate = 0;
const unsigned long TAG_TIMEOUT = 1000; // ms before considering tag as lost

// Kalman filter parameters
float kalmanGain = 0.8;
float positionVariance = 0.1;
float measurementVariance = 0.2;

// Extended position tracking
struct Position
{
  float x;
  float y;
  float theta;
  bool valid;
} currentPos, targetPos;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("Enhanced Robot Localization System Initializing...");

  Wire.begin();
  setupEncoders();
  setupIMU();

  delay(1000); // Sensor stabilization
  calibrateSensors();

  // Initialize position structs
  currentPos = {0.0, 0.0, 0.0, true};
  targetPos = {0.0, 0.0, 0.0, false};

  Serial.println("Localization System Ready!");
}

void loop()
{
  unsigned long currentTime = millis();

  // Check for tag timeout
  if (tagDetected && (currentTime - lastTagUpdate > TAG_TIMEOUT))
  {
    tagDetected = false;
    Serial.println("CLEAR"); // Notify movement controller that tag is lost
  }

  if (currentTime - lastIMUUpdate >= IMU_UPDATE_INTERVAL)
  {
    updateIMUData();
    lastIMUUpdate = currentTime;
  }

  if (currentTime - lastPositionUpdate >= POSITION_UPDATE_INTERVAL)
  {
    updatePosition();
    lastPositionUpdate = currentTime;
    reportPosition();
  }

  while (Serial.available() > 0)
  {
    processSerialData();
  }
}

// --- Encoders Setup ---
void setupEncoders()
{
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), updateRightEncoder, CHANGE);

  Serial.println("Encoders initialized");
}

// --- IMU Setup ---
void setupIMU()
{
  Serial.println("Initializing IMU...");
  if (!imu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor!");
    while (1)
      ;
  }
  imu.calibrateGyro();
  imu.setThreshold(3);
  Serial.println("IMU initialized and calibrated");
}

// --- Calibration ---
void calibrateSensors()
{
  Serial.println("Calibrating sensors...");
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  x_pos = 0.0;
  y_pos = 0.0;
  theta = 0.0;
  Serial.println("Calibration complete");
}

// --- Encoder Interrupts ---
void updateLeftEncoder()
{
  bool A = digitalRead(LEFT_ENCODER_A);
  bool B = digitalRead(LEFT_ENCODER_B);
  if (A == B)
  {
    leftEncoderCount++;
  }
  else
  {
    leftEncoderCount--;
  }
}

void updateRightEncoder()
{
  bool A = digitalRead(RIGHT_ENCODER_A);
  bool B = digitalRead(RIGHT_ENCODER_B);
  if (A == B)
  {
    rightEncoderCount++;
  }
  else
  {
    rightEncoderCount--;
  }
}

// --- IMU Update ---
void updateIMUData()
{
  Vector norm = imu.readNormalizeGyro();
  float gyroZ = norm.ZAxis;
  float dt = IMU_UPDATE_INTERVAL / 1000.0; // convert ms to seconds
  theta += gyroZ * dt;                     // integrate gyroZ to estimate heading

  // Keep theta within -PI to +PI
  if (theta > PI)
    theta -= 2 * PI;
  if (theta < -PI)
    theta += 2 * PI;
}

// --- Position Update ---
void updatePosition()
{
  long leftCount = leftEncoderCount;
  long rightCount = rightEncoderCount;

  long deltaLeft = leftCount - prevLeftCount;
  long deltaRight = rightCount - prevRightCount;

  prevLeftCount = leftCount;
  prevRightCount = rightCount;

  float distanceLeft = (PI * WHEEL_DIAMETER) * (deltaLeft / (float)TICKS_PER_REVOLUTION);
  float distanceRight = (PI * WHEEL_DIAMETER) * (deltaRight / (float)TICKS_PER_REVOLUTION);
  float distanceCenter = (distanceLeft + distanceRight) / 2.0;

  // Update position based on heading (theta)
  x_pos += distanceCenter * cos(theta);
  y_pos += distanceCenter * sin(theta);

  // Update current position struct
  currentPos.x = x_pos;
  currentPos.y = y_pos;
  currentPos.theta = theta;
}

// --- Report Position ---
void reportPosition()
{
  // Send current position to movement controller
  Serial.print("POS:");
  Serial.print(currentPos.x);
  Serial.print(",");
  Serial.print(currentPos.y);
  Serial.print(",");
  Serial.println(currentPos.theta);

  // Debug output
  Serial.print("Position - X(mm): ");
  Serial.print(currentPos.x, 2);
  Serial.print(" Y(mm): ");
  Serial.print(currentPos.y, 2);
  Serial.print(" Theta(rad): ");
  Serial.println(currentPos.theta, 4);
}

// --- Serial Data (AprilTags) ---
void processSerialData()
{
  String data = Serial.readStringUntil('\n');
  if (data.startsWith("TAG:"))
  {
    parseTagData(data.substring(4));
  }
}

// --- Parse Tag Data ---
void parseTagData(String data)
{
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  int thirdComma = data.indexOf(',', secondComma + 1);

  if (firstComma > 0 && secondComma > firstComma && thirdComma > secondComma)
  {
    int id = data.substring(0, firstComma).toInt();
    float x = data.substring(firstComma + 1, secondComma).toFloat();
    float y = data.substring(secondComma + 1, thirdComma).toFloat();
    float yaw = data.substring(thirdComma + 1).toFloat();

    // Update target position
    targetPos.x = x;
    targetPos.y = y;
    targetPos.theta = yaw;
    targetPos.valid = true;

    // Update tag status
    tagDetected = true;
    tagUpdated = true;
    lastTagUpdate = millis();

    // Forward processed tag data to movement controller
    Serial.print("TAG:");
    Serial.print(id);
    Serial.print(",");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.println(yaw);
  }
}
