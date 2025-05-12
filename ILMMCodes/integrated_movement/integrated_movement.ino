/*
 * Integrated Movement Controller
 * 
 * This sketch combines:
 * 1. AprilTag movement tracking from apriltag_movement.ino
 * 2. Obstacle detection and avoidance from fine_moveset.ino
 * 
 * The robot can follow AprilTags while avoiding obstacles using ultrasonic sensors
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// === Performance Settings ===
#define SERIAL_BAUD_RATE 115200  // Increased baud rate
#define COMMAND_BUFFER_SIZE 64
#define CONTROL_LOOP_INTERVAL 50  // 20Hz control loop
#define POSITION_TOLERANCE 5.0f   // mm
#define ROTATION_TOLERANCE 0.05f  // radians
#define MAX_SPEED 255
#define MIN_SPEED 50

// Optimized movement parameters
const float PID_KP = 2.0f;
const float PID_KI = 0.1f;
const float PID_KD = 0.5f;
const float ACCEL_RATE = 0.15f;  // Speed change per cycle (0-1)

// === Pin Definitions === (Combined from both sketches)

// Motor Driver Pins
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
#define TRIG_B 24
#define ECHO_B 25
#define TRIG_BL 26
#define ECHO_BL 27
#define TRIG_BR 28
#define ECHO_BR 29
#define TRIG_FL 30
#define ECHO_FL 31
#define TRIG_F 32
#define ECHO_F 33
#define TRIG_FR 34
#define ECHO_FR 35

// === Enhanced Configuration ===
const float CRITICAL_DISTANCE = 10.0;  // Emergency stop distance (cm)
const float SLOW_DOWN_DISTANCE = 30.0; // Start slowing down distance (cm)
const float SAFE_DISTANCE = 50.0;      // Safe operating distance (cm)
const int REACTION_DELAY = 50;         // Milliseconds between updates

// Ring buffer for commands
char cmdBuffer[COMMAND_BUFFER_SIZE];
uint8_t bufferHead = 0;
uint8_t bufferTail = 0;

// Timing variables
unsigned long lastControlLoop = 0;
unsigned long lastTagUpdate = 0;
unsigned long lastSensorUpdate = 0;
const unsigned long TAG_TIMEOUT = 1000;  // 1 second timeout

// === Globals ===
float distFL, distF, distFR, distBL, distB, distBR;
float currentSpeed = 0.0;
bool emergencyStop = false;

// AprilTag tracking states
enum TrackingState
{
    IDLE,
    MOVING_TO_TAG,
    ALIGNING_WITH_TAG,
    MAINTAINING_POSITION
};

// Motor structure
struct Motor
{
    int RPWM;
    int LPWM;
    int REN;
    int LEN;
    float currentSpeed;
};

// Define motors
Motor motorRight = {RPWM_RIGHT, LPWM_RIGHT, REN_RIGHT, LEN_RIGHT, 0};
Motor motorLeft = {RPWM_LEFT, LPWM_LEFT, REN_LEFT, LEN_LEFT, 0};
Motor motorBack = {RPWM_BACK, LPWM_BACK, REN_BACK, LEN_BACK, 0};

enum Direction
{
    FORWARD,
    BACKWARD,
    STOP
};

// ===== APRIL TAG CONTROLLER CLASS =====
class AprilTagController {
private:
    TrackingState currentState;
    bool tagVisible;
    float targetX, targetY, targetYaw;
    float currentX, currentY, currentYaw;
    float integralError;
    float lastError;
    unsigned long lastPIDUpdate;
    
    // PID control for smooth movement
    float calculatePID(float error) {
        unsigned long now = millis();
        float dt = (now - lastPIDUpdate) / 1000.0f;
        lastPIDUpdate = now;
        
        integralError += error * dt;
        integralError = constrain(integralError, -MAX_SPEED, MAX_SPEED);
        
        float derivative = (error - lastError) / dt;
        lastError = error;
        
        return error * PID_KP + integralError * PID_KI + derivative * PID_KD;
    }
    
    float calculateRotationSpeed(float error) {
        return calculatePID(error);
    }

public:
    AprilTagController() {
        reset();
    }
    
    void reset() {
        currentState = IDLE;
        tagVisible = false;
        targetX = targetY = targetYaw = 0.0f;
        currentX = currentY = currentYaw = 0.0f;
        integralError = lastError = 0.0f;
        lastPIDUpdate = millis();
    }

    bool isActive() {
        return tagVisible && (millis() - lastTagUpdate <= TAG_TIMEOUT);
    }

    void updateTagData(float x, float y, float yaw) {
        tagVisible = true;
        targetX = x;
        targetY = y;
        targetYaw = yaw;
        lastTagUpdate = millis();
        
        if (currentState == IDLE) {
            currentState = MOVING_TO_TAG;
        }
    }

    // Process April tag movement WITH obstacle awareness
    void processMovement() {
        if (!tagVisible || millis() - lastTagUpdate > TAG_TIMEOUT) {
            safeRobotMove(0, 0, 0);  // Safe stop
            currentState = IDLE;
            return;
        }

        float distance = sqrt(sq(targetX - currentX) + sq(targetY - currentY));
        float rotationError = targetYaw - currentYaw;
        
        // Normalize rotation error to [-π, π]
        while (rotationError > PI) rotationError -= 2 * PI;
        while (rotationError < -PI) rotationError += 2 * PI;

        // Process movement based on current state
        switch (currentState) {
            case MOVING_TO_TAG: {
                if (distance <= POSITION_TOLERANCE) {
                    currentState = ALIGNING_WITH_TAG;
                    break;
                }
                
                // Calculate movement vector
                float angle = atan2(targetY - currentY, targetX - currentX);
                float vx = cos(angle) * constrain(distance * 0.5f, MIN_SPEED, MAX_SPEED);
                float vy = sin(angle) * constrain(distance * 0.5f, MIN_SPEED, MAX_SPEED);
                
                safeRobotMove(vy, vx, calculateRotationSpeed(rotationError));
                break;
            }
            
            case ALIGNING_WITH_TAG: {
                if (abs(rotationError) <= ROTATION_TOLERANCE) {
                    currentState = MAINTAINING_POSITION;
                    break;
                }
                safeRobotMove(0, 0, calculateRotationSpeed(rotationError));
                break;
            }
            
            case MAINTAINING_POSITION: {
                if (distance > POSITION_TOLERANCE || abs(rotationError) > ROTATION_TOLERANCE) {
                    currentState = MOVING_TO_TAG;
                    break;
                }
                // Small corrections to maintain position
                float vx = constrain(targetX - currentX, -MIN_SPEED, MIN_SPEED);
                float vy = constrain(targetY - currentY, -MIN_SPEED, MIN_SPEED);
                float vrot = constrain(calculateRotationSpeed(rotationError), -MIN_SPEED, MIN_SPEED);
                safeRobotMove(vy, vx, vrot);
                break;
            }
            
            default:
                safeRobotMove(0, 0, 0);
                break;
        }
    }
    
    // A wrapper that ensures we check obstacles before moving
    // This integrates the obstacle avoidance from fine_moveset
    void safeRobotMove(float vy, float vx, float omega) {
        // Check for emergency stop
        if (emergencyStop) {
            stopAllMotors();
            return;
        }
        
        // Adjust speed based on obstacles in our path
        // Get minimum distance in movement direction
        float forwardDist = min(min(distFL, distF), distFR);
        float backwardDist = min(min(distBL, distB), distBR);
        float leftDist = min(distFL, distBL);
        float rightDist = min(distFR, distBR);
        
        // Check if we're trying to move toward an obstacle
        if (vy > 0 && forwardDist < CRITICAL_DISTANCE) {
            // Don't move forward into obstacle
            vy = 0;
        } else if (vy < 0 && backwardDist < CRITICAL_DISTANCE) {
            // Don't move backward into obstacle
            vy = 0;
        }
        
        if (vx > 0 && rightDist < CRITICAL_DISTANCE) {
            // Don't move right into obstacle
            vx = 0;
        } else if (vx < 0 && leftDist < CRITICAL_DISTANCE) {
            // Don't move left into obstacle
            vx = 0;
        }
        
        // After safety checks, move robot
        moveRobot(vy, vx, omega);
    }
};

// === Motor Control Functions ===

// Setup motor pins
void setupMotorPins(Motor &motor) {
    pinMode(motor.RPWM, OUTPUT);
    pinMode(motor.LPWM, OUTPUT);
    pinMode(motor.REN, OUTPUT);
    pinMode(motor.LEN, OUTPUT);
    digitalWrite(motor.REN, HIGH);
    digitalWrite(motor.LEN, HIGH);
}

// Calculate dynamic speed based on obstacle distance
float calculateDynamicSpeed(float distance, float targetSpeed) {
    if (distance <= CRITICAL_DISTANCE) {
        return 0; // Emergency stop
    } else if (distance <= SLOW_DOWN_DISTANCE) {
        // Linear interpolation between MIN_SPEED and targetSpeed
        float factor = (distance - CRITICAL_DISTANCE) / (SLOW_DOWN_DISTANCE - CRITICAL_DISTANCE);
        return MIN_SPEED + (targetSpeed - MIN_SPEED) * factor;
    }
    return targetSpeed;
}

// Move a single motor
void moveMotor(Motor &motor, Direction dir, float targetSpeed) {
    targetSpeed = constrain(targetSpeed, 0, MAX_SPEED);

    // Smooth acceleration/deceleration
    if (targetSpeed > motor.currentSpeed) {
        motor.currentSpeed = min(targetSpeed, motor.currentSpeed + MAX_SPEED * ACCEL_RATE);
    } else if (targetSpeed < motor.currentSpeed) {
        motor.currentSpeed = max(targetSpeed, motor.currentSpeed - MAX_SPEED * ACCEL_RATE);
    }

    int speed = (int)motor.currentSpeed;

    switch (dir) {
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

// Main robot movement function 
void moveRobot(float vy, float vx, float omega) {
    // Scale inputs to motor speeds
    int speedScale = MAX_SPEED;
    
    // Calculate magnitudes and constrain to valid ranges
    vy = constrain(vy, -1.0, 1.0);
    vx = constrain(vx, -1.0, 1.0);
    omega = constrain(omega, -1.0, 1.0);
    
    // Calculate motor speeds based on holonomic drive equations
    // For a three-wheeled omnidirectional robot
    
    // For a typical triangular arrangement (120° spacing)
    float leftSpeed = -0.5 * vx - 0.866 * vy + omega;   // sin(120°) = 0.866
    float rightSpeed = -0.5 * vx + 0.866 * vy + omega;  // sin(240°) = -0.866
    float backSpeed = vx + omega;                       // sin(0°) = 0
    
    // Scale speeds to fit within [-1, 1]
    float maxSpeed = max(abs(leftSpeed), max(abs(rightSpeed), abs(backSpeed)));
    if (maxSpeed > 1.0) {
        leftSpeed /= maxSpeed;
        rightSpeed /= maxSpeed;
        backSpeed /= maxSpeed;
    }
    
    // Apply deadband to avoid small noisy movements
    const float DEADBAND = 0.1;
    leftSpeed = (abs(leftSpeed) < DEADBAND) ? 0 : leftSpeed;
    rightSpeed = (abs(rightSpeed) < DEADBAND) ? 0 : rightSpeed;
    backSpeed = (abs(backSpeed) < DEADBAND) ? 0 : backSpeed;
    
    // Calculate speeds based on obstacle proximity
    float forwardDist = min(min(distFL, distF), distFR);
    float backwardDist = min(min(distBL, distB), distBR);
    float leftDist = min(distFL, distBL);
    float rightDist = min(distFR, distBR);
    
    // Calculate dynamic speeds based on obstacles
    int leftPWM = abs(leftSpeed) * speedScale;
    int rightPWM = abs(rightSpeed) * speedScale;
    int backPWM = abs(backSpeed) * speedScale;
    
    // Adjust speeds based on obstacle proximity and motor contribution to movement
    // Left motor: positive = forward-left movement, negative = backward-right movement
    if (leftSpeed > 0) {
        // When left motor is positive, it contributes to forward-left movement
        leftPWM = calculateDynamicSpeed(min(forwardDist, leftDist), leftPWM);
    } else if (leftSpeed < 0) {
        // When left motor is negative, it contributes to backward-right movement
        leftPWM = calculateDynamicSpeed(min(backwardDist, rightDist), leftPWM);
    }
    
    // Right motor: positive = forward-right movement, negative = backward-left movement
    if (rightSpeed > 0) {
        // When right motor is positive, it contributes to forward-right movement
        rightPWM = calculateDynamicSpeed(min(forwardDist, rightDist), rightPWM);
    } else if (rightSpeed < 0) {
        // When right motor is negative, it contributes to backward-left movement
        rightPWM = calculateDynamicSpeed(min(backwardDist, leftDist), rightPWM);
    }
    
    // Back motor: positive = right movement, negative = left movement
    if (backSpeed > 0) {
        // When back motor is positive, it contributes to rightward movement
        backPWM = calculateDynamicSpeed(rightDist, backPWM);
    } else if (backSpeed < 0) {
        // When back motor is negative, it contributes to leftward movement
        backPWM = calculateDynamicSpeed(leftDist, backPWM);
    }
    
    // Apply direction based on sign
    Direction leftDir = (leftSpeed >= 0) ? FORWARD : BACKWARD;
    Direction rightDir = (rightSpeed >= 0) ? FORWARD : BACKWARD;
    Direction backDir = (backSpeed >= 0) ? FORWARD : BACKWARD;
    
    // Send commands to motors
    moveMotor(motorLeft, leftDir, leftPWM);
    moveMotor(motorRight, rightDir, rightPWM);
    moveMotor(motorBack, backDir, backPWM);
}

// Stop all motors
void stopAllMotors() {
    moveMotor(motorLeft, STOP, 0);
    moveMotor(motorRight, STOP, 0);
    moveMotor(motorBack, STOP, 0);
}

// Read distance from ultrasonic sensor
float readDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000);
    return (duration <= 0) ? 999.0 : duration * 0.034 / 2.0;
}

// Update all sensor distance readings
void updateDistances() {
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
                   
    if (emergencyStop) {
        Serial.println("EMERGENCY STOP - OBSTACLE DETECTED");
    }
}

// Global controller instance
AprilTagController tagController;

// === SETUP AND LOOP FUNCTIONS ===

void setup() {
    // Initialize serial with higher baud rate
    Serial.begin(SERIAL_BAUD_RATE);
    
    // Setup motors
    setupMotorPins(motorLeft);
    setupMotorPins(motorRight);
    setupMotorPins(motorBack);
    
    // Setup sensor pins
    int trigPins[] = {TRIG_FL, TRIG_F, TRIG_FR, TRIG_BL, TRIG_B, TRIG_BR};
    int echoPins[] = {ECHO_FL, ECHO_F, ECHO_FR, ECHO_BL, ECHO_B, ECHO_BR};
    
    for (int i = 0; i < 6; i++) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }
    
    Serial.println(F("Integrated Movement Controller Initialized"));
    Serial.println(F("April Tag tracking with obstacle avoidance"));
}

void loop() {
    // Process serial commands
    while (Serial.available() > 0) {
        processSerialInput();
    }
    
    // Update sensor readings at fixed intervals
    unsigned long now = millis();
    if (now - lastSensorUpdate >= REACTION_DELAY) {
        updateDistances();
        lastSensorUpdate = now;
        
        // Print distance readings periodically
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
    }
    
    // Run tag controller at fixed intervals
    if (now - lastControlLoop >= CONTROL_LOOP_INTERVAL) {
        lastControlLoop = now;
        
        if (emergencyStop) {
            stopAllMotors();
        } else {
            tagController.processMovement();
        }
    }
}

void processSerialInput() {
    static char buffer[32];
    static uint8_t index = 0;
    
    char c = Serial.read();
    
    if (c == '\n') {
        buffer[index] = '\0';
        parseCommand(buffer);
        index = 0;
    } else if (index < sizeof(buffer) - 1) {
        buffer[index++] = c;
    }
}

void parseCommand(const char* cmd) {
    if (strncmp(cmd, "TAG:", 4) == 0) {
        // Original format: TAG:x,y,yaw
        float x = 0, y = 0, yaw = 0;
        if (sscanf(cmd + 4, "%f,%f,%f", &x, &y, &yaw) == 3) {
            tagController.updateTagData(x, y, yaw);
            return;
        }
        
        // Alternative format from apriltag_communication.py: TAG:tag_id,distance,direction
        int tagId;
        float distance;
        char direction;
        if (sscanf(cmd + 4, "%d,%f,%c", &tagId, &distance, &direction) == 3) {
            // Convert direction character to x,y,yaw coordinates
            float x = 0, y = 0, yaw = 0;
            
            // Set forward direction as +Y, with distance as magnitude
            switch (direction) {
                case 'F': // Forward
                    y = distance;
                    break;
                case 'B': // Backward
                    y = -distance;
                    break;
                case 'L': // Left
                    x = -distance;
                    yaw = PI/2; // 90 degrees
                    break;
                case 'R': // Right
                    x = distance;
                    yaw = -PI/2; // -90 degrees
                    break;
                case 'S': // Stop
                    // All zeros, no movement
                    break;
                default:
                    return; // Invalid direction
            }
            
            tagController.updateTagData(x, y, yaw);
        }
    } else if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "CLEAR") == 0) {
        // Stop robot and reset controller
        tagController.reset();
        stopAllMotors();
    } else if (strncmp(cmd, "TEST", 4) == 0) {
        // Run sensor tests
        Serial.println("Running sensor diagnostics:");
        updateDistances();
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
    }
}