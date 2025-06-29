#include <Arduino.h>
#include <math.h>

// === Pin Definitions ===
#define RPWM_LEFT 7
#define LPWM_LEFT 6
#define REN_LEFT 38
#define LEN_LEFT 39

#define RPWM_RIGHT 10
#define LPWM_RIGHT 9
#define REN_RIGHT 51
#define LEN_RIGHT 50

#define RPWM_BACK 4 // Updated to match configuration
#define LPWM_BACK 5 // Updated to match configuration
#define REN_BACK 44 // Kept original
#define LEN_BACK 45 // Kept original

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
#define TRIG_FL 26
#define ECHO_FL 27
#define TRIG_FR 24
#define ECHO_FR 25
#define TRIG_BL 28
#define ECHO_BL 29
#define TRIG_BR 32
#define ECHO_BR 33
#define TRIG_B 30
#define ECHO_B 31

// === Enhanced Configuration ===
const float CRITICAL_DISTANCE = 10.0;  // Emergency stop distance (cm)
const float SLOW_DOWN_DISTANCE = 30.0; // Start slowing down distance (cm)
const float SAFE_DISTANCE = 50.0;      // Safe operating distance (cm)
const int MAX_SPEED = 255;             // Maximum motor speed
const int MIN_SPEED = 50;              // Minimum effective motor speed
const float ACCELERATION_RATE = 0.15;  // Speed change per cycle (0-1)
const int REACTION_DELAY = 50;         // Milliseconds between updates

// === Communication settings ===
const int SERIAL_BUFFER_SIZE = 64;
char serialBuffer[SERIAL_BUFFER_SIZE];
int bufferIndex = 0;
bool use3WheelMode = true; // Default to 3-wheel mode
int userSpeed = 150;       // Default user speed
bool debugMode = false;    // Debug output mode

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
  Serial.begin(115200); // Higher baud rate for better communication

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

  Serial.println("<READY:Enhanced Movement System with Communication Support>");
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

// === Navigation States and Constants ===
enum NavState
{
  IDLE,
  MOVING_TO_TARGET,
  DOCKING,
  UNDOCKING,
  ATTENDING_PERSON,
  RETURNING_TO_CHARGE,
  AUTONOMOUS_EXPLORE // New state for autonomous exploration
};

enum NavigationMode
{
  APRILTAG_MODE,
  AUTONOMOUS_MODE
};

// Social navigation parameters
const float PERSONAL_SPACE = 80.0;    // cm, keep this distance from people
const float APPROACH_SPEED = 0.6;     // 60% speed when approaching people
const float DOCK_SPEED = 0.3;         // 30% speed when docking
const float NORMAL_SPEED = 1.0;       // Full speed for normal navigation
const int PATH_UPDATE_INTERVAL = 100; // ms between path updates

// Autonomous exploration parameters
const float WALL_FOLLOW_DISTANCE = 60.0;          // cm, distance to maintain from walls
const float TURN_THRESHOLD = 45.0;                // cm, distance to trigger turns
const unsigned long EXPLORATION_TURN_TIME = 2000; // ms to perform exploration turns

// === COMMUNICATION HANDLER ===

// Process incoming serial data and build commands
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

// Parse and execute commands
void parseCommand(const char *cmd)
{
  if (debugMode)
  {
    Serial.print("<DEBUG:Received command: ");
    Serial.print(cmd);
    Serial.println(">");
  }

  // Handle single character commands
  if (strlen(cmd) == 1)
  {
    switch (cmd[0])
    {
    case 'W':
      Serial.println("<ACK:FORWARD>");
      moveForward(userSpeed / 255.0);
      return;
    case 'S':
      Serial.println("<ACK:BACKWARD>");
      moveBackward(userSpeed / 255.0);
      return;
    case 'A':
      Serial.println("<ACK:ARC_LEFT>");
      moveLeft(userSpeed / 255.0 * 0.7);
      return;
    case 'D':
      Serial.println("<ACK:ARC_RIGHT>");
      moveRight(userSpeed / 255.0 * 0.7);
      return;
    case 'Q':
      Serial.println("<ACK:ROTATE_LEFT>");
      rotateLeft(userSpeed / 255.0);
      return;
    case 'E':
      Serial.println("<ACK:ROTATE_RIGHT>");
      rotateRight(userSpeed / 255.0);
      return;
    case '5':
      Serial.println("<ACK:STOP>");
      stop();
      return;
    case '4':
      Serial.println("<ACK:SLIDE_LEFT>");
      moveLeft(userSpeed / 255.0);
      return;
    case '6':
      Serial.println("<ACK:SLIDE_RIGHT>");
      moveRight(userSpeed / 255.0);
      return;
    case '7':
      Serial.println("<ACK:DIAG_FORWARD_LEFT>");
      moveForwardLeft(userSpeed / 255.0);
      return;
    case '9':
      Serial.println("<ACK:DIAG_FORWARD_RIGHT>");
      moveForwardRight(userSpeed / 255.0);
      return;
    case '1':
      Serial.println("<ACK:DIAG_BACKWARD_LEFT>");
      moveBackwardLeft(userSpeed / 255.0);
      return;
    case '3':
      Serial.println("<ACK:DIAG_BACKWARD_RIGHT>");
      moveBackwardRight(userSpeed / 255.0);
      return;
    }
  }

  // Check for TEST command
  if (strcmp(cmd, "TEST") == 0)
  {
    Serial.println("<ACK:TEST:System Ready>");
    Serial.print("<INFO:Mode:");
    Serial.print(use3WheelMode ? "THREE_WHEEL" : "TWO_WHEEL");
    Serial.println(">");
    Serial.print("<INFO:Speed:");
    Serial.print(userSpeed);
    Serial.println(">");
    Serial.println("<INFO:Sensors:OK>");
    return;
  }

  // Check for PING command
  if (strcmp(cmd, "PING") == 0)
  {
    Serial.println("<ACK:PING>");
    return;
  }

  // Check for STOP command
  if (strcmp(cmd, "STOP") == 0)
  {
    stop();
    Serial.println("<ACK:STOP>");
    return;
  }

  // Check for SENS command (sensor data)
  if (strcmp(cmd, "SENS") == 0)
  {
    updateDistances();
    Serial.print("<ACK:SENS:");
    Serial.print(distFL);
    Serial.print(",");
    Serial.print(distF);
    Serial.print(",");
    Serial.print(distFR);
    Serial.print(",");
    Serial.print(distBL);
    Serial.print(",");
    Serial.print(distB);
    Serial.print(",");
    Serial.print(distBR);
    Serial.println(">");
    return;
  }

  // Check for DEBUG command
  if (strncmp(cmd, "DEBUG:", 6) == 0)
  {
    if (strcmp(cmd + 6, "ON") == 0)
    {
      debugMode = true;
      Serial.println("<ACK:DEBUG:ON>");
    }
    else if (strcmp(cmd + 6, "OFF") == 0)
    {
      debugMode = false;
      Serial.println("<ACK:DEBUG:OFF>");
    }
    return;
  }

  // Check for MODE command (2-wheel vs 3-wheel)
  if (strncmp(cmd, "MODE:", 5) == 0)
  {
    if (strcmp(cmd + 5, "3WHEEL") == 0 || strcmp(cmd + 5, "1") == 0)
    {
      use3WheelMode = true;
      Serial.println("<ACK:MODE:THREE_WHEEL>");
    }
    else if (strcmp(cmd + 5, "2WHEEL") == 0 || strcmp(cmd + 5, "0") == 0)
    {
      use3WheelMode = false;
      Serial.println("<ACK:MODE:TWO_WHEEL>");
    }
    else
    {
      Serial.print("<ACK:MODE:");
      Serial.print(use3WheelMode ? "THREE_WHEEL" : "TWO_WHEEL");
      Serial.println(">");
    }
    return;
  }

  // Check for SPEED command
  if (strncmp(cmd, "SPEED:", 6) == 0)
  {
    int newSpeed = atoi(cmd + 6);
    if (newSpeed > 0 && newSpeed <= 255)
    {
      userSpeed = newSpeed;
      Serial.print("<ACK:SPEED:");
      Serial.print(userSpeed);
      Serial.println(">");
    }
    else
    {
      Serial.println("<ERR:Invalid speed (1-255)>");
    }
    return;
  }

  // Handle TAG commands from apriltag_uart_controller.py
  if (strncmp(cmd, "TAG:", 4) == 0)
  {
    int tagId;
    float distance;
    int direction;
    int speed;

    // Parse TAG command format
    if (sscanf(cmd + 4, "%d,%f,%d,%d", &tagId, &distance, &direction, &speed) == 4)
    {
      if (debugMode)
      {
        Serial.print("<DEBUG:TAG ID=");
        Serial.print(tagId);
        Serial.print(", Distance=");
        Serial.print(distance);
        Serial.print(", Direction=");
        Serial.print(direction);
        Serial.print(", Speed=");
        Serial.print(speed);
        Serial.println(">");
      }

      // Process the tag data with our enhanced navigation
      executeTagCommand(tagId, distance, direction, speed);
      Serial.println("<ACK:TAG>");
      return;
    }
  }

  // Handle MOV command for integrated_movement.ino compatibility
  if (strncmp(cmd, "MOV:", 4) == 0)
  {
    int direction;
    int speed;

    if (sscanf(cmd + 4, "%d,%d", &direction, &speed) == 2)
    {
      executeMovementCommand(direction, speed);
      Serial.println("<ACK:MOV>");
      return;
    }
  }

  // Unknown command
  Serial.println("<ERR:Unknown command>");
}

// Execute tag-based movement command
void executeTagCommand(int tagId, float distance, int direction, int speed)
{
  // Convert speed to 0-1 range
  float normalizedSpeed = constrain(speed, 0, 255) / 255.0;

  // Handle special tag commands
  if (tagId == 99)
  {
    // Special search/rotation commands
    switch (direction)
    {
    case 5: // Rotate left
      rotateLeft(normalizedSpeed);
      break;
    case 6: // Rotate right
      rotateRight(normalizedSpeed);
      break;
    default:
      stop();
    }
    return;
  }

  // Regular movement commands
  switch (direction)
  {
  case 0: // Stop
    stop();
    break;
  case 1: // Forward
    moveForward(normalizedSpeed);
    break;
  case 2: // Backward
    moveBackward(normalizedSpeed);
    break;
  case 3: // Turn left
    moveLeft(normalizedSpeed);
    break;
  case 4: // Turn right
    moveRight(normalizedSpeed);
    break;
  case 5: // Rotate left
    rotateLeft(normalizedSpeed);
    break;
  case 6: // Rotate right
    rotateRight(normalizedSpeed);
    break;
  case 7: // Slide left
    moveLeft(normalizedSpeed);
    break;
  case 8: // Slide right
    moveRight(normalizedSpeed);
    break;
  case 9: // Diagonal forward-left
    moveForwardLeft(normalizedSpeed);
    break;
  case 10: // Diagonal forward-right
    moveForwardRight(normalizedSpeed);
    break;
  case 11: // Diagonal backward-left
    moveBackwardLeft(normalizedSpeed);
    break;
  case 12: // Diagonal backward-right
    moveBackwardRight(normalizedSpeed);
    break;
  default:
    stop();
  }
}

// Execute movement command from MOV command
void executeMovementCommand(int direction, int speed)
{
  float normalizedSpeed = constrain(speed, 0, 255) / 255.0;

  switch (direction)
  {
  case 0: // Stop
    stop();
    break;
  case 1: // Forward
    moveForward(normalizedSpeed);
    break;
  case 2: // Backward
    moveBackward(normalizedSpeed);
    break;
  case 3: // Turn left
    moveLeft(normalizedSpeed);
    break;
  case 4: // Turn right
    moveRight(normalizedSpeed);
    break;
  case 5: // Rotate left
    rotateLeft(normalizedSpeed);
    break;
  case 6: // Rotate right
    rotateRight(normalizedSpeed);
    break;
  case 7: // Slide left
    moveLeft(normalizedSpeed);
    break;
  case 8: // Slide right
    moveRight(normalizedSpeed);
    break;
  case 9: // Diagonal forward-left
    moveForwardLeft(normalizedSpeed);
    break;
  case 10: // Diagonal forward-right
    moveForwardRight(normalizedSpeed);
    break;
  case 11: // Diagonal backward-left
    moveBackwardLeft(normalizedSpeed);
    break;
  case 12: // Diagonal backward-right
    moveBackwardRight(normalizedSpeed);
    break;
  default:
    stop();
  }
}

// === Sensor Testing Functions ===
void testSensor(int trigPin, int echoPin, const char *sensorName)
{
  long distance = readDistance(trigPin, echoPin);
  Serial.print(sensorName);
  Serial.print(": ");
  if (distance < 999.0)
  {
    Serial.print(distance);
    Serial.print(" cm");
    if (distance < CRITICAL_DISTANCE)
    {
      Serial.print(" [CRITICAL]");
    }
    else if (distance < SLOW_DOWN_DISTANCE)
    {
      Serial.print(" [WARNING]");
    }
  }
  else
  {
    Serial.print("No reading");
  }
  Serial.println();
}

void runSensorDiagnostics()
{
  Serial.println("\n=== Running Sensor Diagnostics ===");
  testSensor(TRIG_FL, ECHO_FL, "Front-Left");
  testSensor(TRIG_F, ECHO_F, "Front");
  testSensor(TRIG_FR, ECHO_FR, "Front-Right");
  testSensor(TRIG_BL, ECHO_BL, "Back-Left");
  testSensor(TRIG_B, ECHO_B, "Back");
  testSensor(TRIG_BR, ECHO_BR, "Back-Right");
  Serial.println("================================");
}

class NavigationController
{
private:
  NavState currentState;
  NavigationMode navMode;
  float currentSpeed;
  float targetSpeed;
  unsigned long lastUpdate;
  unsigned long lastTurn;
  bool personDetected;
  bool wallOnRight;     // For wall following behavior
  int explorationPhase; // To track exploration behavior

public:
  NavigationController()
  {
    currentState = IDLE;
    navMode = APRILTAG_MODE;
    currentSpeed = 0;
    targetSpeed = 0;
    lastUpdate = 0;
    lastTurn = 0;
    personDetected = false;
    wallOnRight = true;
    explorationPhase = 0;
  }

  // Only process certain internal commands; most commands now handled by main loop
  void processSerialCommands()
  {
    if (Serial.available())
    {
      char cmd = Serial.read();
      switch (cmd)
      {
      case 'M': // Manual mode switch command
        toggleNavigationMode();
        break;
      case 'T': // Test sensors command
        runSensorDiagnostics();
        break;
      case 'S': // Stop command
        emergencyStop = true;
        transitionTo(IDLE);
        break;
      case 'R': // Resume command
        emergencyStop = false;
        break;
      }
    }
  }

  void updateNavigation()
  {
    unsigned long now = millis();
    if (now - lastUpdate < PATH_UPDATE_INTERVAL)
    {
      return;
    }
    lastUpdate = now;

    // Update speeds with smooth acceleration
    if (currentSpeed < targetSpeed)
    {
      currentSpeed = min(currentSpeed + ACCELERATION_RATE, targetSpeed);
    }
    else if (currentSpeed > targetSpeed)
    {
      currentSpeed = max(currentSpeed - ACCELERATION_RATE, targetSpeed);
    }

    // Execute current state behavior
    switch (currentState)
    {
    case MOVING_TO_TARGET:
      if (navMode == AUTONOMOUS_MODE)
      {
        performAutonomousExploration();
      }
      else
      {
        moveToTarget();
      }
      break;
    case DOCKING:
      performDocking();
      break;
    case UNDOCKING:
      performUndocking();
      break;
    case ATTENDING_PERSON:
      attendPerson();
      break;
    case RETURNING_TO_CHARGE:
      returnToCharge();
      break;
    case AUTONOMOUS_EXPLORE:
      performAutonomousExploration();
      break;
    case IDLE:
      stop();
      break;
    }
  }

  void performAutonomousExploration()
  {
    float minFrontDist = min(min(distFL, distF), distFR);
    float minRightDist = min(distFR, distBR);
    float minLeftDist = min(distFL, distBL);
    unsigned long now = millis();

    targetSpeed = NORMAL_SPEED;

    // Basic wall following and obstacle avoidance
    if (minFrontDist < TURN_THRESHOLD)
    {
      // Obstacle ahead - choose turn direction
      if (minLeftDist > minRightDist)
      {
        // More space on the left, turn left
        rotateLeft(currentSpeed);
        if (now - lastTurn > EXPLORATION_TURN_TIME)
        {
          lastTurn = now;
          explorationPhase = (explorationPhase + 1) % 4;
        }
      }
      else
      {
        // More space on the right, turn right
        rotateRight(currentSpeed);
        if (now - lastTurn > EXPLORATION_TURN_TIME)
        {
          lastTurn = now;
          explorationPhase = (explorationPhase + 1) % 4;
        }
      }
    }
    else
    {
      // No immediate obstacle - follow wall or explore
      if (wallOnRight)
      {
        if (minRightDist > WALL_FOLLOW_DISTANCE + 20)
        {
          // Lost wall - turn right to find it
          moveForwardRight(currentSpeed * 0.7);
        }
        else if (minRightDist < WALL_FOLLOW_DISTANCE - 20)
        {
          // Too close to wall - move away
          moveForwardLeft(currentSpeed * 0.7);
        }
        else
        {
          // Good distance - move forward
          moveForward(currentSpeed);
        }
      }
      else
      {
        // Mirror behavior for left wall following
        if (minLeftDist > WALL_FOLLOW_DISTANCE + 20)
        {
          moveForwardLeft(currentSpeed * 0.7);
        }
        else if (minLeftDist < WALL_FOLLOW_DISTANCE - 20)
        {
          moveForwardRight(currentSpeed * 0.7);
        }
        else
        {
          moveForward(currentSpeed);
        }
      }
    }

    // Check for potential person detection
    if (isPersonLikeObject())
    {
      transitionTo(ATTENDING_PERSON);
    }
  }

  bool isPersonLikeObject()
  {
    // Simple heuristic to detect person-like obstacles
    float centerDist = distF;
    float leftDist = distFL;
    float rightDist = distFR;

    // Look for obstacle of roughly person width (40-80cm) at appropriate distance
    return (centerDist < PERSONAL_SPACE && centerDist > SAFE_DISTANCE &&
            abs(leftDist - rightDist) < 30.0 &&  // Roughly symmetric
            abs(centerDist - leftDist) < 40.0 && // Consistent depth
            abs(centerDist - rightDist) < 40.0);
  }

  void toggleNavigationMode()
  {
    navMode = (navMode == APRILTAG_MODE) ? AUTONOMOUS_MODE : APRILTAG_MODE;
    Serial.print("<INFO:Switching to ");
    Serial.print(navMode == APRILTAG_MODE ? "AprilTag Mode" : "Autonomous Mode");
    Serial.println(">");

    // Reset relevant state variables
    currentSpeed = 0;
    targetSpeed = 0;
    explorationPhase = 0;

    // Choose appropriate initial state
    if (navMode == AUTONOMOUS_MODE)
    {
      transitionTo(AUTONOMOUS_EXPLORE);
    }
    else
    {
      transitionTo(MOVING_TO_TARGET);
    }
  }

  void moveToTarget()
  {
    float minFrontDist = min(min(distFL, distF), distFR);

    // Check for people or obstacles
    if (minFrontDist < PERSONAL_SPACE)
    {
      targetSpeed = APPROACH_SPEED;
      // If very close, transition to attending person
      if (minFrontDist < SAFE_DISTANCE * 1.5)
      {
        transitionTo(ATTENDING_PERSON);
        return;
      }
    }
    else
    {
      targetSpeed = NORMAL_SPEED;
    }

    // Basic obstacle avoidance while moving
    if (minFrontDist < SLOW_DOWN_DISTANCE)
    {
      if (distFL > distFR && distFL > SAFE_DISTANCE)
      {
        moveForwardLeft(currentSpeed);
      }
      else if (distFR > distFL && distFR > SAFE_DISTANCE)
      {
        moveForwardRight(currentSpeed);
      }
      else
      {
        moveBackward(currentSpeed * 0.5);
      }
    }
    else
    {
      moveForward(currentSpeed);
    }
  }

  void performDocking()
  {
    targetSpeed = DOCK_SPEED;
    // Implement precise docking movement using April tag feedback
    // This will be integrated with April tag system later
    float dockingDistance = distF; // Later use April tag distance

    if (dockingDistance > SAFE_DISTANCE * 2)
    {
      moveForward(currentSpeed);
    }
    else if (dockingDistance > SAFE_DISTANCE)
    {
      moveForward(currentSpeed * 0.5);
    }
    else
    {
      transitionTo(IDLE);
    }
  }

  void performUndocking()
  {
    targetSpeed = DOCK_SPEED;
    if (distB > SAFE_DISTANCE * 2)
    {
      moveBackward(currentSpeed);
    }
    else
    {
      transitionTo(IDLE);
    }
  }

  void attendPerson()
  {
    // Implement social interaction positioning
    // Stay at PERSONAL_SPACE distance and slightly angle the robot
    targetSpeed = APPROACH_SPEED;

    float minFrontDist = min(min(distFL, distF), distFR);
    if (abs(minFrontDist - PERSONAL_SPACE) < 10.0)
    {
      // We're at a good distance, just adjust orientation
      if (distFL > distFR)
      {
        rotateLeft(0.2); // Slight angle for more natural positioning
      }
      else
      {
        rotateRight(0.2);
      }
    }
    else if (minFrontDist < PERSONAL_SPACE)
    {
      moveBackward(currentSpeed);
    }
    else
    {
      moveForward(currentSpeed);
    }
  }

  void returnToCharge()
  {
    // Similar to moveToTarget but with docking preparation
    moveToTarget();
    // When close to charging station, transition to docking
    if (distF < SAFE_DISTANCE * 3)
    { // Will use April tag detection instead
      transitionTo(DOCKING);
    }
  }

  void transitionTo(NavState newState)
  {
    if (newState == currentState)
      return;

    // Handle state exit actions
    switch (currentState)
    {
    case ATTENDING_PERSON:
      targetSpeed = NORMAL_SPEED;
      break;
    case AUTONOMOUS_EXPLORE:
      lastTurn = millis(); // Reset turn timing
      break;
    }

    currentState = newState;

    // Handle state entry actions
    switch (currentState)
    {
    case DOCKING:
      targetSpeed = DOCK_SPEED;
      break;
    case ATTENDING_PERSON:
      targetSpeed = APPROACH_SPEED;
      break;
    case AUTONOMOUS_EXPLORE:
      targetSpeed = NORMAL_SPEED;
      explorationPhase = 0;
      break;
    }
  }

  NavState getCurrentState()
  {
    return currentState;
  }

  NavigationMode getNavigationMode()
  {
    return navMode;
  }
};

NavigationController navigator;

// Update the loop function to handle both autonomous navigation and command processing
void loop()
{
  // Process serial commands first
  processSerialInput();

  unsigned long currentTime = millis();

  // Update sensor readings at fixed intervals
  if (currentTime - lastUpdateTime >= REACTION_DELAY)
  {
    updateDistances();
    lastUpdateTime = currentTime;

    // Debug output if enabled
    if (debugMode)
    {
      Serial.print("<DEBUG:Distances - FL:");
      Serial.print(distFL);
      Serial.print(" F:");
      Serial.print(distF);
      Serial.print(" FR:");
      Serial.print(distFR);
      Serial.print(" BL:");
      Serial.print(distBL);
      Serial.print(" B:");
      Serial.print(distB);
      Serial.print(" BR:");
      Serial.print(distBR);
      Serial.println(">");
    }

    if (emergencyStop)
    {
      Serial.println("<WARN:EMERGENCY_STOP_ACTIVE>");
      stop();
    }
    else
    {
      // Only use autonomous navigation when no direct commands are being received
      if (currentTime - lastUpdateTime > 1000)
      {
        navigator.updateNavigation();
      }
    }
  }
}
