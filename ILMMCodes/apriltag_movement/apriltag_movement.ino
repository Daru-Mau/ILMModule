#include <Arduino.h>
#include <Wire.h>

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
const float ACCEL_RATE = 10.0f;  // Units per control loop

// Ring buffer for commands
char cmdBuffer[COMMAND_BUFFER_SIZE];
uint8_t bufferHead = 0;
uint8_t bufferTail = 0;

// Timing variables
unsigned long lastControlLoop = 0;
unsigned long lastTagUpdate = 0;
const unsigned long TAG_TIMEOUT = 1000;  // 1 second timeout

enum TrackingState
{
    IDLE,
    MOVING_TO_TAG,
    ALIGNING_WITH_TAG,
    MAINTAINING_POSITION
};

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

    void processMovement() {
        if (!tagVisible || millis() - lastTagUpdate > TAG_TIMEOUT) {
            moveRobot(0, 0, 0);
            currentState = IDLE;
            return;
        }

        float distance = sqrt(sq(targetX - currentX) + sq(targetY - currentY));
        float rotationError = targetYaw - currentYaw;
        
        // Normalize rotation error to [-π, π]
        while (rotationError > PI) rotationError -= 2 * PI;
        while (rotationError < -PI) rotationError += 2 * PI;

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
                
                moveRobot(vy, vx, calculateRotationSpeed(rotationError));
                break;
            }
            
            case ALIGNING_WITH_TAG: {
                if (abs(rotationError) <= ROTATION_TOLERANCE) {
                    currentState = MAINTAINING_POSITION;
                    break;
                }
                moveRobot(0, 0, calculateRotationSpeed(rotationError));
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
                moveRobot(vy, vx, vrot);
                break;
            }
            
            default:
                moveRobot(0, 0, 0);
                break;
        }
    }
};

// Motor pin definitions
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

// Motor structures
struct Motor {
    int RPWM;
    int LPWM;
    int REN;
    int LEN;
};

// Define motors
Motor motorRight = {RPWM_RIGHT, LPWM_RIGHT, REN_RIGHT, LEN_RIGHT};
Motor motorLeft = {RPWM_LEFT, LPWM_LEFT, REN_LEFT, LEN_LEFT};
Motor motorBack = {RPWM_BACK, LPWM_BACK, REN_BACK, LEN_BACK};

enum Direction {
    FORWARD,
    BACKWARD,
    STOP
};

// Optimized motor control functions
void setupMotors() {
    // Setup individual motor pins
    pinMode(motorRight.RPWM, OUTPUT);
    pinMode(motorRight.LPWM, OUTPUT);
    pinMode(motorRight.REN, OUTPUT);
    pinMode(motorRight.LEN, OUTPUT);
    digitalWrite(motorRight.REN, HIGH);
    digitalWrite(motorRight.LEN, HIGH);
    
    pinMode(motorLeft.RPWM, OUTPUT);
    pinMode(motorLeft.LPWM, OUTPUT);
    pinMode(motorLeft.REN, OUTPUT);
    pinMode(motorLeft.LEN, OUTPUT);
    digitalWrite(motorLeft.REN, HIGH);
    digitalWrite(motorLeft.LEN, HIGH);
    
    pinMode(motorBack.RPWM, OUTPUT);
    pinMode(motorBack.LPWM, OUTPUT);
    pinMode(motorBack.REN, OUTPUT);
    pinMode(motorBack.LEN, OUTPUT);
    digitalWrite(motorBack.REN, HIGH);
    digitalWrite(motorBack.LEN, HIGH);
}

// Helper function to control individual motors
void moveMotor(const Motor &motor, Direction dir, int speed) {
    speed = constrain(speed, 0, 255);
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
            break;
    }
}

// Stop all motors
void stopAllMotors() {
    moveMotor(motorLeft, STOP, 0);
    moveMotor(motorRight, STOP, 0);
    moveMotor(motorBack, STOP, 0);
}

// Main robot movement function (used by the controller)
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
    
    // Convert to motor commands
    int leftPWM = abs(leftSpeed) * speedScale;
    int rightPWM = abs(rightSpeed) * speedScale;
    int backPWM = abs(backSpeed) * speedScale;
    
    // Apply direction based on sign
    Direction leftDir = (leftSpeed >= 0) ? FORWARD : BACKWARD;
    Direction rightDir = (rightSpeed >= 0) ? FORWARD : BACKWARD;
    Direction backDir = (backSpeed >= 0) ? FORWARD : BACKWARD;
    
    // Send commands to motors
    moveMotor(motorLeft, leftDir, leftPWM);
    moveMotor(motorRight, rightDir, rightPWM);
    moveMotor(motorBack, backDir, backPWM);
}

// Global controller instance
AprilTagController tagController;

void setup() {
    // Initialize serial with higher baud rate
    Serial.begin(SERIAL_BAUD_RATE);
    
    // Motor setup with fast digital writes
    setupMotors();
    
    Serial.println(F("AprilTag Movement Controller Initialized"));
}

void loop() {
    // Process any available serial commands
    while (Serial.available() > 0) {
        processSerialInput();
    }

    // Run control loop at fixed interval
    unsigned long now = millis();
    if (now - lastControlLoop >= CONTROL_LOOP_INTERVAL) {
        lastControlLoop = now;
        tagController.processMovement();
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
        tagController.reset();
    }
}
