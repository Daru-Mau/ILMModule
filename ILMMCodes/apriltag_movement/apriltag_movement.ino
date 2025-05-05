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
        float x, y, yaw;
        if (sscanf(cmd + 4, "%f,%f,%f", &x, &y, &yaw) == 3) {
            tagController.updateTagData(x, y, yaw);
        }
    } else if (strcmp(cmd, "CLEAR") == 0) {
        tagController.reset();
    }
}

// Optimized motor control functions
void setupMotors() {
    // Direct port manipulation for faster digital writes
    DDRD |= B11111100;  // Set pins 2-7 as outputs
    DDRB |= B00111111;  // Set pins 8-13 as outputs
}
