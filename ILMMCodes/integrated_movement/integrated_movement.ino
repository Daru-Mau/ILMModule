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
// Changed from #define to global variables so they can be modified at runtime
int MAX_SPEED = 50;  // Increased from 50 to overcome motor stall torque
int MIN_SPEED = 40;  // Increased from 40 for more noticeable movement
#define DEBUG_MODE true          // Set to true for verbose output, false for reduced output

// Message framing characters for UART communication
const char START_MARKER = '<';
const char END_MARKER = '>';
const char ESCAPE_CHAR = '\\';

// Optimized movement parameters
const float PID_KP = 2.0f;
const float PID_KI = 0.1f;
const float PID_KD = 0.5f;
const float ACCEL_RATE = 0.15f;  // Speed change per cycle (0-1)

// === Pin Definitions === (Combined from both sketches)

/* // Motor Driver Pins
#define RPWM_RIGHT 2  // FIXED: Swapped to match basic_moveset.ino
#define LPWM_RIGHT 3  // FIXED: Swapped to match basic_moveset.ino
#define REN_RIGHT 39
#define LEN_RIGHT 38
#define RPWM_LEFT 5   // FIXED: Swapped to match basic_moveset.ino
#define LPWM_LEFT 4   // FIXED: Swapped to match basic_moveset.ino
#define REN_LEFT 44
#define LEN_LEFT 45
#define RPWM_BACK 7
#define LPWM_BACK 6
#define REN_BACK 51
#define LEN_BACK 50 */

// Motor Driver Pins CWW
#define RPWM_RIGHT 6 
#define LPWM_RIGHT 7 
#define REN_RIGHT 51
#define LEN_RIGHT 50
#define RPWM_LEFT 3 
#define LPWM_LEFT 2     
#define REN_LEFT 39
#define LEN_LEFT 38
#define RPWM_BACK 5
#define LPWM_BACK 4
#define REN_BACK 44
#define LEN_BACK 45

// Encoder Pins
#define ENC_RIGHT_C1 40
#define ENC_RIGHT_C2 41
#define ENC_LEFT_C1 46
#define ENC_LEFT_C2 47
#define ENC_BACK_C1 52
#define ENC_BACK_C2 53

/* // Ultrasonic Sensor Pins - Normal Setting
#define TRIG_BL 26
#define ECHO_BL 27
#define TRIG_B 24
#define ECHO_B 25
#define TRIG_BR 28
#define ECHO_BR 29
#define TRIG_FR 34
#define ECHO_FR 35 
#define TRIG_F 32
#define ECHO_F 33
#define TRIG_FL 30
#define ECHO_FL 31 
*/

// Ultrasonic Sensor Pins - Rotated Front
#define TRIG_F 28   
#define ECHO_F 29   
#define TRIG_FL 34  
#define ECHO_FL 35  
#define TRIG_FR 24  
#define ECHO_FR 25  
#define TRIG_BL 32 
#define ECHO_BL 33  
#define TRIG_B 30   
#define ECHO_B 31   
#define TRIG_BR 26  
#define ECHO_BR 27  

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
bool emergencyStop = false;

struct Motor {
    int RPWM;
    int LPWM;
    int REN;
    int LEN;
    float currentSpeed;
};

Motor motorRight = {RPWM_RIGHT, LPWM_RIGHT, REN_RIGHT, LEN_RIGHT, 0};
Motor motorLeft = {RPWM_LEFT, LPWM_LEFT, REN_LEFT, LEN_LEFT, 0};
Motor motorBack = {RPWM_BACK, LPWM_BACK, REN_BACK, LEN_BACK, 0};

enum Direction {
    FORWARD,
    BACKWARD,
    STOP
};

// Setup motor pins
void setupMotorPins(Motor &motor) {
    pinMode(motor.RPWM, OUTPUT);
    pinMode(motor.LPWM, OUTPUT);
    pinMode(motor.REN, OUTPUT);
    pinMode(motor.LEN, OUTPUT);
    digitalWrite(motor.REN, HIGH);
    digitalWrite(motor.LEN, HIGH);
}

// Move a single motor
void moveMotor(Motor &motor, Direction dir, float targetSpeed) {
    targetSpeed = constrain(targetSpeed, 0, MAX_SPEED);
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

void stopAllMotors() {
    moveMotor(motorLeft, STOP, 0);
    moveMotor(motorRight, STOP, 0);
    moveMotor(motorBack, STOP, 0);
}

// Dynamic speed calculation based on obstacle distance
float calculateDynamicSpeed(float distance, float targetSpeed) {
    if (distance <= CRITICAL_DISTANCE) {
        return 0; // Emergency stop
    } else if (distance <= SLOW_DOWN_DISTANCE) {
        float factor = (distance - CRITICAL_DISTANCE) / (SLOW_DOWN_DISTANCE - CRITICAL_DISTANCE);
        return MIN_SPEED + (targetSpeed - MIN_SPEED) * factor;
    }
    return targetSpeed;
}

// Execute movement with obstacle avoidance
void executeMovement(int direction, int speed) {
    // Read distances
    float forwardDist = min(min(distFL, distF), distFR);
    float backwardDist = min(min(distBL, distB), distBR);
    float leftDist = min(distFL, distBL);
    float rightDist = min(distFR, distBR);

    // Obstacle avoidance: if too close, stop or override
    if (direction == 1 && forwardDist < CRITICAL_DISTANCE) { // FORWARD
        stopAllMotors();
        return;
    } else if (direction == 2 && backwardDist < CRITICAL_DISTANCE) { // BACKWARD
        stopAllMotors();
        return;
    } else if (direction == 3 && leftDist < CRITICAL_DISTANCE) { // LEFT
        stopAllMotors();
        return;
    } else if (direction == 4 && rightDist < CRITICAL_DISTANCE) { // RIGHT
        stopAllMotors();
        return;
    }

    // Dynamic speed adjustment
    if (direction == 1) speed = calculateDynamicSpeed(forwardDist, speed);
    else if (direction == 2) speed = calculateDynamicSpeed(backwardDist, speed);
    else if (direction == 3) speed = calculateDynamicSpeed(leftDist, speed);
    else if (direction == 4) speed = calculateDynamicSpeed(rightDist, speed);

    switch (direction) {
        case 1: // FORWARD
            moveMotor(motorLeft, FORWARD, speed);
            moveMotor(motorRight, FORWARD, speed);
            moveMotor(motorBack, STOP, 0);
            break;
        case 2: // BACKWARD
            moveMotor(motorLeft, BACKWARD, speed);
            moveMotor(motorRight, BACKWARD, speed);
            moveMotor(motorBack, STOP, 0);
            break;
        case 3: // LEFT
            moveMotor(motorLeft, BACKWARD, speed);
            moveMotor(motorRight, FORWARD, speed);
            moveMotor(motorBack, FORWARD, speed);
            break;
        case 4: // RIGHT
            moveMotor(motorLeft, FORWARD, speed);
            moveMotor(motorRight, BACKWARD, speed);
            moveMotor(motorBack, BACKWARD, speed);
            break;
        case 0: // STOP
        default:
            stopAllMotors();
            break;
    }
}

// Add this function after the motor control functions but before setup()

float readUltrasonicDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    float duration = pulseIn(echoPin, HIGH);
    return (duration * 0.0343) / 2.0;  // Convert to centimeters
}

void updateDistances() {
    // Read all sensor distances
    distFL = readUltrasonicDistance(TRIG_FL, ECHO_FL);
    distF = readUltrasonicDistance(TRIG_F, ECHO_F);
    distFR = readUltrasonicDistance(TRIG_FR, ECHO_FR);
    distBL = readUltrasonicDistance(TRIG_BL, ECHO_BL);
    distB = readUltrasonicDistance(TRIG_B, ECHO_B);
    distBR = readUltrasonicDistance(TRIG_BR, ECHO_BR);

    // Constrain readings to valid range
    distFL = constrain(distFL, 1, 400);
    distF = constrain(distF, 1, 400);
    distFR = constrain(distFR, 1, 400);
    distBL = constrain(distBL, 1, 400);
    distB = constrain(distB, 1, 400);
    distBR = constrain(distBR, 1, 400);
}

// Ring buffer for commands
char cmdBuffer[COMMAND_BUFFER_SIZE];
uint8_t bufferHead = 0;
uint8_t bufferTail = 0;

// Timing variables
unsigned long lastControlLoop = 0;
unsigned long lastSensorUpdate = 0;
const unsigned long TAG_TIMEOUT = 1000;  // 1 second timeout

// === Globals ===
float distFL, distF, distFR, distBL, distB, distBR;
bool emergencyStop = false;

// Global controller instance

// === SETUP AND LOOP FUNCTIONS ===

void setup() {
    // 1. Hardware Initialization First
    // Setup motors 
    setupMotorPins(motorLeft);
    setupMotorPins(motorRight);
    setupMotorPins(motorBack);
    
    // Setup ultrasonic sensors
    int trigPins[] = {TRIG_FL, TRIG_F, TRIG_FR, TRIG_BL, TRIG_B, TRIG_BR};
    int echoPins[] = {ECHO_FL, ECHO_F, ECHO_FR, ECHO_BL, ECHO_B, ECHO_BR};
    for (int i = 0; i < 6; i++) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
        digitalWrite(trigPins[i], LOW); // Ensure clean start
    }

    // 2. Serial Communication Setup
    Serial.begin(115200); // Start serial FIRST without resetting
    delay(100); // Short stabilization delay
    
    // 3. Clear garbage data (critical for Raspberry Pi)
    while (Serial.available()) {
        Serial.read(); // Flush any noise
    }

    // 5. Motor Test (optional)
    if (DEBUG_MODE) {
        testMotors(); // Brief motor pulse test
        Serial.println("Motor test complete");
    }

    Serial.println("READY"); // Vital ready signal
}

// Quick motor test function
void testMotors() {
    // Very brief pulse on each motor to confirm connections
    const int testSpeed = 40;  // Lower speed for safety
    const int testDuration = 100;  // Very short duration (ms)
    
    // Test left motor
    Serial.println(F("Testing left motor..."));
    moveMotor(motorLeft, FORWARD, testSpeed);
    delay(testDuration);
    moveMotor(motorLeft, STOP, 0);
    
    // Test right motor
    Serial.println(F("Testing right motor..."));
    moveMotor(motorRight, FORWARD, testSpeed);
    delay(testDuration);
    moveMotor(motorRight, STOP, 0);
    
    // Test back motor
    Serial.println(F("Testing back motor..."));
    moveMotor(motorBack, FORWARD, testSpeed);
    delay(testDuration);
    moveMotor(motorBack, STOP, 0);
    
    Serial.println(F("Motor test complete."));
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
        
        // Print distance readings periodically (only in debug mode)
        static unsigned long lastPrintTime = 0;
        if (DEBUG_MODE && (now - lastPrintTime >= 1000)) {  // 1 second interval for debug output
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
            
            // Print current state information
            Serial.print("State: ");
            if (tagController.isActive()) {
                Serial.println("TAG TRACKING ACTIVE");
            } else {
                Serial.println("IDLE - Waiting for commands");
            }
            
            lastPrintTime = now;
        }
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
    static char buffer[COMMAND_BUFFER_SIZE];
    static uint8_t index = 0;
    static boolean messageStarted = false;
    static boolean escapeNext = false;
    
    while (Serial.available()) {
        char c = Serial.read();
        
        // Handle traditional newline-terminated commands for backward compatibility
        if (!messageStarted && (c == '\n' || c == '\r')) {
            if (index > 0) {  // Only process non-empty commands
                buffer[index] = '\0';
                parseCommand(buffer);
                index = 0;
            }
        }
        // Handle framed commands with start/end markers
        else if (c == START_MARKER && !escapeNext) {
            // Start of new message
            messageStarted = true;
            index = 0;
        }
        else if (c == END_MARKER && !escapeNext && messageStarted) {
            // End of message
            buffer[index] = '\0';
            parseCommand(buffer);
            messageStarted = false;
            index = 0;
        }
        else if (c == ESCAPE_CHAR && !escapeNext) {
            // Escape character
            escapeNext = true;
        }
        else {
            // Regular character or escaped special character
            if (index < COMMAND_BUFFER_SIZE - 1) {
                if (escapeNext) {
                    // Store the character after escape directly
                    buffer[index++] = c;
                    escapeNext = false;
                } 
                else if (messageStarted) {
                    // Only store if we're inside a message
                    buffer[index++] = c;
                }
                else if (!messageStarted && index == 0 && c != ' ') {
                    // For backward compatibility, start a non-framed command
                    buffer[index++] = c;
                }
                else if (!messageStarted && index > 0) {
                    // Continue a non-framed command
                    buffer[index++] = c;
                }
            }
        }
    }
}

void parseCommand(const char* cmd) {
    // Debug: Echo received command if in debug mode
    if (DEBUG_MODE) {
        Serial.print("Received command: ");
        Serial.println(cmd);
    }
    
    // Check for command type and parameters
    char command[16] = {0};
    char params[COMMAND_BUFFER_SIZE - 16] = {0};
    
    // Extract command and parameters (split on first colon)
    int colonPos = -1;
    for (int i = 0; cmd[i] != '\0'; i++) {
        if (cmd[i] == ':') {
            colonPos = i;
            break;
        }
    }
    
    if (colonPos >= 0) {
        // Copy command part
        strncpy(command, cmd, min(colonPos, 15));
        command[min(colonPos, 15)] = '\0';
        
        // Copy parameters part
        strncpy(params, cmd + colonPos + 1, COMMAND_BUFFER_SIZE - 17);
        params[COMMAND_BUFFER_SIZE - 17] = '\0';
    } else {
        // No parameters, just command
        strncpy(command, cmd, 15);
        command[15] = '\0';
        params[0] = '\0';
    }
    
    // Handle different command types
    if (strcmp(command, "TEST") == 0) {
        Serial.println("<ACK:TEST>");
        Serial.println("=== RUNNING DIAGNOSTICS ===");
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
        
        // Test motors with quick pulses
        Serial.println("Testing LEFT motor...");
        moveMotor(motorLeft, FORWARD, 150);
        delay(100);
        moveMotor(motorLeft, STOP, 0);
        
        Serial.println("Testing RIGHT motor...");
        moveMotor(motorRight, FORWARD, 150);
        delay(100);
        moveMotor(motorRight, STOP, 0);
        
        Serial.println("Testing BACK motor...");
        moveMotor(motorBack, FORWARD, 150);
        delay(100);
        moveMotor(motorBack, STOP, 0);
        
        Serial.println("=== DIAGNOSTIC COMPLETE ===");
        return;
    }
    
    // Check for PING command
    if (strcmp(command, "PING") == 0) {
        Serial.println("<ACK:PING>");
        Serial.println("PONG");
        return;
    }
    
    // Check for STOP/CLEAR commands
    if (strcmp(command, "STOP") == 0 || strcmp(command, "CLEAR") == 0) {
        // Stop robot and reset controller
        // tagController.reset();
        stopAllMotors();
        Serial.println("<ACK:STOP>");
        return;
    }
    
    // Handle SPEED command
    if (strcmp(command, "SPEED") == 0) {
        // Command format: SPEED:max_speed,min_speed
        int newMaxSpeed, newMinSpeed;
        if (sscanf(params, "%d,%d", &newMaxSpeed, &newMinSpeed) == 2) {
            // Apply constraints to ensure valid values
            newMaxSpeed = constrain(newMaxSpeed, 50, 255);
            newMinSpeed = constrain(newMinSpeed, 30, newMaxSpeed);
            
            // Update the global speed parameters
            MAX_SPEED = newMaxSpeed;
            MIN_SPEED = newMinSpeed;
            
            Serial.print("<ACK:SPEED>");
            Serial.print(" MAX_SPEED=");
            Serial.print(MAX_SPEED);
            Serial.print(", MIN_SPEED=");
            Serial.println(MIN_SPEED);
        } else {
            Serial.println("<ERR:Invalid SPEED format>");
        }
        return;
    }
    
    // Handle SENSOR command - return current sensor values
    if (strcmp(command, "SENSOR") == 0 || strcmp(command, "SENS") == 0) {
        updateDistances(); // Ensure fresh readings
        
        // Format: SENS:front,front_left,front_right,back,back_left,back_right
        Serial.print("<SENS:");
        Serial.print(distF);
        Serial.print(",");
        Serial.print(distFL);
        Serial.print(",");
        Serial.print(distFR);
        Serial.print(",");
        Serial.print(distB);
        Serial.print(",");
        Serial.print(distBL);
        Serial.print(",");
        Serial.print(distBR);
        Serial.println(">");
        return;
    }
    
    // Handle MOVE command
    if (strcmp(command, "MOVE") == 0) {
        // Command format: MOVE:direction[,speed]
        int direction, speed = 50; // Default speed
        
        // Parse parameters
        if (sscanf(params, "%d,%d", &direction, &speed) < 1) {
            Serial.println("<ERR:Invalid MOVE format>");
            return;
        }
        
        // Validate direction (0=STOP, 1=FORWARD, 2=BACKWARD, 3=LEFT, 4=RIGHT)
        if (direction < 0 || direction > 4) {
            Serial.println("<ERR:Invalid direction>");
            return;
        }
        
        // Execute movement
        executeMovement(direction, speed);
        Serial.println("<ACK:MOVE>");
        return;
    }
    
    // Handle TAG or MOV command (both use same simplified format)
    if (strcmp(command, "TAG") == 0 || strcmp(command, "MOV") == 0) {
        // Format: TAG:direction,speed or MOV:direction,speed
        int direction = 0;
        int speed = 0;
        
        // Parse parameters
        if (sscanf(params, "%d,%d", &direction, &speed) != 2) {
            Serial.print("<ERR:Invalid ");
            Serial.print(command);
            Serial.println(" format>");
            return;
        }
        
        // Validate direction (0=STOP, 1=FORWARD, 2=BACKWARD, 3=LEFT, 4=RIGHT)
        if (direction < 0 || direction > 4) {
            Serial.println("<ERR:Invalid direction>");
            return;
        }
        
        // Constrain speed to valid range
        speed = constrain(speed, MIN_SPEED, MAX_SPEED);
        
        if (DEBUG_MODE) {
            Serial.print("Processing movement command: Direction=");
            Serial.print(direction);
            Serial.print(", Speed=");
            Serial.println(speed);
        }
        
        // Execute movement based on direction and speed
        executeMovement(direction, speed);
        Serial.print("<ACK:");
        Serial.print(command);
        Serial.println(">");
        return;
    }
    
    // Handle MOTORS/MCTL command for direct motor control
    if (strcmp(command, "MOTORS") == 0 || strcmp(command, "MCTL") == 0) {
        // Command format: MCTL:left_speed,right_speed,back_speed
        int leftSpeed = 0, rightSpeed = 0, backSpeed = 0;
        
        if (sscanf(params, "%d,%d,%d", &leftSpeed, &rightSpeed, &backSpeed) == 3) {
            // Constrain speeds to valid range (-255 to 255)
            leftSpeed = constrain(leftSpeed, -255, 255);
            rightSpeed = constrain(rightSpeed, -255, 255);
            backSpeed = constrain(backSpeed, -255, 255);
            
            // Apply motor speeds directly
            if (leftSpeed > 0) 
                moveMotor(motorLeft, FORWARD, leftSpeed);
            else if (leftSpeed < 0)
                moveMotor(motorLeft, BACKWARD, -leftSpeed);
            else
                moveMotor(motorLeft, STOP, 0);
                
            if (rightSpeed > 0)
                moveMotor(motorRight, FORWARD, rightSpeed);
            else if (rightSpeed < 0)
                moveMotor(motorRight, BACKWARD, -rightSpeed);
            else
                moveMotor(motorRight, STOP, 0);
                
            if (backSpeed > 0)
                moveMotor(motorBack, FORWARD, backSpeed);
            else if (backSpeed < 0)
                moveMotor(motorBack, BACKWARD, -backSpeed);
            else
                moveMotor(motorBack, STOP, 0);
                
            Serial.println("<ACK:MCTL>");
        } else {
            Serial.println("<ERR:Invalid MCTL format>");
        }
        return;
    }
    
    // Unknown command
    Serial.print("<ERR:Unknown command: ");
    Serial.print(command);
    Serial.println(">");
}

void executeMovement(int direction, int speed) {
    // For tag_id=99, we're doing rotation
    // For tag_id=0, we're doing regular movement
    bool isRotation = (tagId == 99);
    
    // Direction mapping:
    // 0 = STOP
    // 1 = FORWARD
    // 2 = BACKWARD
    // 3 = LEFT
    // 4 = RIGHT
    
    switch (direction) {
        case 1: // FORWARD
            if (!isRotation) {
                if (DEBUG_MODE) Serial.println("Moving FORWARD");
                moveMotor(motorLeft, FORWARD, speed);
                moveMotor(motorRight, FORWARD, speed);
                moveMotor(motorBack, STOP, 0);
            }
            break;
            
        case 2: // BACKWARD
            if (!isRotation) {
                if (DEBUG_MODE) Serial.println("Moving BACKWARD");
                moveMotor(motorLeft, BACKWARD, speed);
                moveMotor(motorRight, BACKWARD, speed);
                moveMotor(motorBack, STOP, 0);
            }
            break;
            
        case 3: // LEFT
            if (isRotation) {
                if (DEBUG_MODE) Serial.println("Rotating COUNTERCLOCKWISE");
                moveMotor(motorLeft, BACKWARD, speed);
                moveMotor(motorRight, FORWARD, speed);
                moveMotor(motorBack, FORWARD, speed);
            } else {
                if (DEBUG_MODE) Serial.println("Moving LEFT");
                moveMotor(motorLeft, BACKWARD, speed);
                moveMotor(motorRight, FORWARD, speed);
                moveMotor(motorBack, FORWARD, speed);
            }
            break;
            
        case 4: // RIGHT
            if (isRotation) {
                if (DEBUG_MODE) Serial.println("Rotating CLOCKWISE");
                moveMotor(motorLeft, FORWARD, speed);
                moveMotor(motorRight, BACKWARD, speed);
                moveMotor(motorBack, BACKWARD, speed);
            } else {
                if (DEBUG_MODE) Serial.println("Moving RIGHT");
                moveMotor(motorLeft, FORWARD, speed);
                moveMotor(motorRight, BACKWARD, speed);
                moveMotor(motorBack, BACKWARD, speed);
            }
            break;
            
        case 0: // STOP
            if (DEBUG_MODE) Serial.println("Stopping all motors");
            stopAllMotors();
            break;
            
        default:
            if (DEBUG_MODE) Serial.println("Unknown direction code");
            stopAllMotors();
            break;
    }
}