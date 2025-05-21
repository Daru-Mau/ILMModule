/*
 * Integrated Movement Controller
 *
 * This sketch focuses on:
 * 1. UART communication with Raspberry Pi
 * 2. Motor control for omni-directional movement
 * 3. Obstacle detection and avoidance using ultrasonic sensors
 *
 * Note: All AprilTag processing is handled by the Raspberry Pi
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// === Performance Settings ===
#define SERIAL_BAUD_RATE 115200
#define COMMAND_BUFFER_SIZE 64
#define CONTROL_LOOP_INTERVAL 50 // 20Hz control loop
// Changed from #define to global variables so they can be modified at runtime
int MAX_SPEED = 50;
int MIN_SPEED = 40;
#define DEBUG_MODE false // Keep this false to prevent debug messages interfering with UART communication
// Message framing characters for UART communication
const char START_MARKER = '<';
const char END_MARKER = '>';
const char ESCAPE_CHAR = '\\';

// Optimized movement parameters
const float PID_KP = 2.0f;
const float PID_KI = 0.1f;
const float PID_KD = 0.5f;
const float ACCEL_RATE = 0.15f; // Speed change per cycle (0-1)

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

// Motor Driver Pins - Using COUNTER-CLOCKWISE arrangement from basic_moveset
// The motors have been positioned as follows:
// - LEFT wheel (pins 2, 3, 38, 39)
// - RIGHT wheel (pins 7, 6, 51, 50)
// - BACK wheel (pins 4, 5, 44, 45)
#define RPWM_RIGHT 7 
#define LPWM_RIGHT 6 
#define REN_RIGHT 51
#define LEN_RIGHT 50

#define RPWM_LEFT 2 
#define LPWM_LEFT 3 
#define REN_LEFT 38
#define LEN_LEFT 39

#define RPWM_BACK 4 
#define LPWM_BACK 5 
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
const float CRITICAL_DISTANCE = 15.0;  // Emergency stop distance (cm)
const float SLOW_DOWN_DISTANCE = 30.0; // Start slowing down distance (cm)
const float SAFE_DISTANCE = 60.0;     // Safe operating distance (cm)
const int REACTION_DELAY = 50;         // Milliseconds between updates

// Ring buffer for commands
char cmdBuffer[COMMAND_BUFFER_SIZE];
uint8_t bufferHead = 0;
uint8_t bufferTail = 0;

// Timing variables
unsigned long lastControlLoop = 0;
unsigned long lastTagUpdate = 0;
unsigned long lastSensorUpdate = 0;
const unsigned long COMMAND_TIMEOUT = 1000; // 1 second timeout

// === Globals ===
bool emergencyStop = false;
float distFL, distF, distFR, distBL, distB, distBR;
int movementMode = 0;        // 0=Normal, 1=Rotation
bool useThreeWheels = false; // Flag to select between 2-wheel (false) and 3-wheel (true) configuration

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

// Setup motor pins
void setupMotorPins(Motor &motor)
{
    pinMode(motor.RPWM, OUTPUT);
    pinMode(motor.LPWM, OUTPUT);
    pinMode(motor.REN, OUTPUT);
    pinMode(motor.LEN, OUTPUT);
    digitalWrite(motor.REN, HIGH);
    digitalWrite(motor.LEN, HIGH);
}

// Move a single motor
void moveMotor(Motor &motor, Direction dir, float targetSpeed)
{
    targetSpeed = constrain(targetSpeed, 0, MAX_SPEED);
    if (targetSpeed > motor.currentSpeed)
    {
        motor.currentSpeed = min(targetSpeed, motor.currentSpeed + MAX_SPEED * ACCEL_RATE);
    }
    else if (targetSpeed < motor.currentSpeed)
    {
        motor.currentSpeed = max(targetSpeed, motor.currentSpeed - MAX_SPEED * ACCEL_RATE);
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

void stopAllMotors()
{
    moveMotor(motorLeft, STOP, 0);
    moveMotor(motorRight, STOP, 0);
    moveMotor(motorBack, STOP, 0);
}

// Dynamic speed calculation based on obstacle distance
float calculateDynamicSpeed(float distance, float targetSpeed)
{
    if (distance <= CRITICAL_DISTANCE)
    {
        return 0; // Emergency stop
    }
    else if (distance <= SLOW_DOWN_DISTANCE)
    {
        float factor = (distance - CRITICAL_DISTANCE) / (SLOW_DOWN_DISTANCE - CRITICAL_DISTANCE);
        return MIN_SPEED + (targetSpeed - MIN_SPEED) * factor;
    }
    return targetSpeed;
}

// Rotation functions
void rotateLeft(int speed = MIN_SPEED)
{
    if (DEBUG_MODE)
        Serial.println("Rotating LEFT (CCW)");

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    
    // Both wheels spin in the same direction to rotate in place
    moveMotor(motorLeft, FORWARD, speed*1.30);
    moveMotor(motorRight, FORWARD, speed*0.25);

    // Use back wheel if in 3-wheel configuration
    if (useThreeWheels)
    {
        moveMotor(motorBack, BACKWARD, speed);
    }
    else
    {
        moveMotor(motorBack, STOP, 0); // Back wheel disabled in 2-wheel mode
    }
}

void rotateRight(int speed = MIN_SPEED)
{
    if (DEBUG_MODE)
        Serial.println("Rotating RIGHT (CW)");

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    
    // Both wheels spin in the same direction to rotate in place
    moveMotor(motorLeft, BACKWARD, speed*0.25);
    moveMotor(motorRight, BACKWARD, speed*1.30);

    // Use back wheel if in 3-wheel configuration
    if (useThreeWheels)
    {
        moveMotor(motorBack, FORWARD, speed);
    }
    else
    {
        moveMotor(motorBack, STOP, 0); // Back wheel disabled in 2-wheel mode
    }
}

// Execute movement with obstacle avoidance
void executeMovement(int direction, int speed)
{
    // Read distances
    float forwardDist = min(min(distFL, distF), distFR);
    float backwardDist = min(min(distBL, distB), distBR);
    float leftDist = min(distFL, distBL);
    float rightDist = min(distFR, distBR);

    // Obstacle avoidance: if too close, stop or override
    if (direction == 1 && forwardDist < CRITICAL_DISTANCE)
    { // FORWARD
        stopAllMotors();
        return;
    }
    else if (direction == 2 && backwardDist < CRITICAL_DISTANCE)
    { // BACKWARD
        stopAllMotors();
        return;
    }
    else if (direction == 3 && leftDist < CRITICAL_DISTANCE)
    { // LEFT
        stopAllMotors();
        return;
    }
    else if (direction == 4 && rightDist < CRITICAL_DISTANCE)
    { // RIGHT
        stopAllMotors();
        return;
    }
    else if (direction == 5 && leftDist < CRITICAL_DISTANCE)
    { // ROTATE LEFT
        stopAllMotors();
        return;
    }
    else if (direction == 6 && rightDist < CRITICAL_DISTANCE)
    { // ROTATE RIGHT
        stopAllMotors();
        return;
    }

    // Dynamic speed adjustment
    if (direction == 1)
        speed = calculateDynamicSpeed(forwardDist, speed);
    else if (direction == 2)
        speed = calculateDynamicSpeed(backwardDist, speed);
    else if (direction == 3)
        speed = calculateDynamicSpeed(leftDist, speed);
    else if (direction == 4)
        speed = calculateDynamicSpeed(rightDist, speed);
    else if (direction == 5)
        speed = calculateDynamicSpeed(leftDist, speed);
    else if (direction == 6)
        speed = calculateDynamicSpeed(rightDist, speed);
    switch (direction)
    {
    case 1: // FORWARD 
        if (DEBUG_MODE)
            Serial.println("Moving FORWARD");
        // Forward movement logic from basic_moveset
        moveMotor(motorLeft, BACKWARD, speed);
        moveMotor(motorRight, FORWARD, speed);
        moveMotor(motorBack, STOP, 0); // Back motor disabled for forward movement
        break;
        
    case 2: // BACKWARD
        if (DEBUG_MODE)
            Serial.println("Moving BACKWARD");
        // Backward movement logic from basic_moveset
        moveMotor(motorLeft, FORWARD, speed);
        moveMotor(motorRight, BACKWARD, speed);
        moveMotor(motorBack, STOP, 0); // Back motor disabled for backward movement
        break;
    case 3: // LEFT LATERAL MOVEMENT
        if (DEBUG_MODE)
            Serial.println("Moving LEFT");
        if (useThreeWheels)
        {
            // Three-wheel configuration for lateral movement
            moveMotor(motorLeft, STOP, 0);
            moveMotor(motorRight, BACKWARD, speed);
            moveMotor(motorBack, FORWARD, speed);
        }
        else
        {
            // Two-wheel strafe approximation
            moveMotor(motorLeft, FORWARD, speed);
            moveMotor(motorRight, FORWARD, speed);
            moveMotor(motorBack, STOP, 0);
        }
        break;
    case 4: // RIGHT LATERAL MOVEMENT
        if (DEBUG_MODE)
            Serial.println("Moving RIGHT");
        if (useThreeWheels)
        {
            // Three-wheel configuration for lateral movement
            moveMotor(motorLeft, BACKWARD, speed);
            moveMotor(motorRight, STOP, 0);
            moveMotor(motorBack, BACKWARD, speed);
        }
        else
        {
            // Two-wheel strafe approximation
            moveMotor(motorLeft, BACKWARD, speed);
            moveMotor(motorRight, BACKWARD, speed);
            moveMotor(motorBack, STOP, 0);
        }
        break;
    case 5: // ROTATE LEFT
        if (DEBUG_MODE)
            Serial.println("Rotating LEFT");
        rotateLeft(speed);
        break;
    case 6: // ROTATE RIGHT
        if (DEBUG_MODE)
            Serial.println("Rotating RIGHT");
        rotateRight(speed);
        break;
    case 0: // STOP
        if (DEBUG_MODE)
            Serial.println("Stopping all motors");
        stopAllMotors();
        break;
    default:
        if (DEBUG_MODE)
            Serial.println("Unknown direction code");
        stopAllMotors();
        break;
    }
}

// Function to read distance from an ultrasonic sensor
float readUltrasonicDistance(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    float duration = pulseIn(echoPin, HIGH);
    return (duration * 0.0343) / 2.0; // Convert to centimeters
}

void updateDistances()
{
    // Read all sensor distances
    distFL = readUltrasonicDistance(TRIG_FL, ECHO_FL);
    distF = readUltrasonicDistance(TRIG_F, ECHO_F);
    distFR = readUltrasonicDistance(TRIG_FR, ECHO_FR);
    distBL = readUltrasonicDistance(TRIG_BL, ECHO_BL);
    distB = readUltrasonicDistance(TRIG_B, ECHO_B);
    distBR = readUltrasonicDistance(TRIG_BR, ECHO_BR);
}
// Global controller instance

// === SETUP AND LOOP FUNCTIONS ===

void setup()
{
    // 1. Hardware Initialization First
    // Setup motors
    setupMotorPins(motorLeft);
    setupMotorPins(motorRight);
    setupMotorPins(motorBack);

    // Setup ultrasonic sensors
    int trigPins[] = {TRIG_FL, TRIG_F, TRIG_FR, TRIG_BL, TRIG_B, TRIG_BR};
    int echoPins[] = {ECHO_FL, ECHO_F, ECHO_FR, ECHO_BL, ECHO_B, ECHO_BR};
    for (int i = 0; i < 6; i++)
    {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
        digitalWrite(trigPins[i], LOW); // Ensure clean start
    }

    // 2. Serial Communication Setup
    Serial.begin(115200); // Start serial FIRST without resetting
    delay(100);           // Short stabilization delay

    // 3. Clear garbage data (critical for Raspberry Pi)
    while (Serial.available())
    {
        Serial.read(); // Flush any noise
    }

    // 5. Motor Test (optional)
    if (DEBUG_MODE)
    {
        testMotors(); // Brief motor pulse test
        Serial.println("Motor test complete");
    } // Always send a ready signal regardless of debug mode
    Serial.println("<READY>");

    // Initialize with 2-wheel configuration by default
    useThreeWheels = false;
    if (DEBUG_MODE)
    {
        Serial.println("Initialized in TWO_WHEEL mode");
    }
}

// Quick motor test function
void testMotors()
{
    // Very brief pulse on each motor to confirm connections
    const int testSpeed = 60;     // Higher speed for better visibility
    const int testDuration = 300; // Longer duration for better visibility

    Serial.println("<TESTING LEFT MOTOR FORWARD>");
    // Test left motor forward with direct pin control
    digitalWrite(motorLeft.REN, HIGH);
    digitalWrite(motorLeft.LEN, HIGH);
    analogWrite(motorLeft.RPWM, testSpeed);  // FORWARD
    analogWrite(motorLeft.LPWM, 0);
    delay(testDuration);
    analogWrite(motorLeft.RPWM, 0);
    analogWrite(motorLeft.LPWM, 0);
    delay(500);

    Serial.println("<TESTING LEFT MOTOR BACKWARD>");
    // Test left motor backward with direct pin control
    digitalWrite(motorLeft.REN, HIGH);
    digitalWrite(motorLeft.LEN, HIGH);
    analogWrite(motorLeft.RPWM, 0);
    analogWrite(motorLeft.LPWM, testSpeed);  // BACKWARD
    delay(testDuration);
    analogWrite(motorLeft.RPWM, 0);
    analogWrite(motorLeft.LPWM, 0);
    delay(500);

    // Test using the moveMotor function
    Serial.println("<TESTING LEFT MOTOR WITH MOVEMOTOR>");
    moveMotor(motorLeft, FORWARD, testSpeed);
    delay(testDuration);
    moveMotor(motorLeft, STOP, 0);
    delay(200);
    moveMotor(motorLeft, BACKWARD, testSpeed);
    delay(testDuration);
    moveMotor(motorLeft, STOP, 0);
    delay(500);

    // Test right motor
    Serial.println("<TESTING RIGHT MOTOR>");
    moveMotor(motorRight, FORWARD, testSpeed);
    delay(testDuration);
    moveMotor(motorRight, STOP, 0);
    delay(200);
    moveMotor(motorRight, BACKWARD, testSpeed);
    delay(testDuration);
    moveMotor(motorRight, STOP, 0);
    delay(500);

    // Test back motor
    Serial.println("<TESTING BACK MOTOR>");
    moveMotor(motorBack, FORWARD, testSpeed);
    delay(testDuration);
    moveMotor(motorBack, STOP, 0);
    delay(200);
    moveMotor(motorBack, BACKWARD, testSpeed);
    delay(testDuration);
    moveMotor(motorBack, STOP, 0);

    Serial.println("<MOTOR TEST COMPLETE>");
}

void loop()
{
    // Process serial commands
    while (Serial.available() > 0)
    {
        processSerialInput();
    }

    // Update sensor readings at fixed intervals
    unsigned long now = millis();
    if (now - lastSensorUpdate >= REACTION_DELAY)
    {
        updateDistances();
        lastSensorUpdate = now;
    } // Check for emergency stop condition at fixed intervals
    if (now - lastControlLoop >= CONTROL_LOOP_INTERVAL)
    {
        lastControlLoop = now;

        // Check emergency stop conditions
        if ((distF < CRITICAL_DISTANCE || distB < CRITICAL_DISTANCE) && !emergencyStop)
        {
            emergencyStop = true;
            stopAllMotors();
            Serial.println("<EMERGENCY STOP: OBSTACLE DETECTED>");
        }
        else if (distFL < CRITICAL_DISTANCE && !emergencyStop)
        {
            emergencyStop = true;
            stopAllMotors();
            Serial.println("<EMERGENCY STOP: LEFT FRONT OBSTACLE>");
        }
        else if (distFR < CRITICAL_DISTANCE && !emergencyStop)
        {
            emergencyStop = true;
            stopAllMotors();
            Serial.println("<EMERGENCY STOP: RIGHT FRONT OBSTACLE>");
        }
        else if (emergencyStop && (distF > SAFE_DISTANCE) && (distFL > SAFE_DISTANCE) && (distFR > SAFE_DISTANCE))
        {
            emergencyStop = false;
            Serial.println("<EMERGENCY STOP RELEASED>");
        }

        // Only print status message in DEBUG_MODE
        if (DEBUG_MODE && (now - lastTagUpdate > 5000))
        {
            Serial.print("State: ");
            Serial.println(emergencyStop ? "EMERGENCY STOP" : "ACTIVE - Waiting for commands");
        }
    }
}

void processSerialInput()
{
    static char buffer[COMMAND_BUFFER_SIZE];
    static uint8_t index = 0;
    static boolean messageStarted = false;
    static boolean escapeNext = false;

    while (Serial.available())
    {
        char c = Serial.read();

        // Handle traditional newline-terminated commands for backward compatibility
        if (!messageStarted && (c == '\n' || c == '\r'))
        {
            if (index > 0)
            { // Only process non-empty commands
                buffer[index] = '\0';
                parseCommand(buffer);
                index = 0;
            }
        }
        // Handle framed commands with start/end markers
        else if (c == START_MARKER && !escapeNext)
        {
            // Start of new message
            messageStarted = true;
            index = 0;
        }
        else if (c == END_MARKER && !escapeNext && messageStarted)
        {
            // End of message
            buffer[index] = '\0';
            parseCommand(buffer);
            messageStarted = false;
            index = 0;
        }
        else if (c == ESCAPE_CHAR && !escapeNext)
        {
            // Escape character
            escapeNext = true;
        }
        else
        {
            // Regular character or escaped special character
            if (index < COMMAND_BUFFER_SIZE - 1)
            {
                if (escapeNext)
                {
                    // Store the character after escape directly
                    buffer[index++] = c;
                    escapeNext = false;
                }
                else if (messageStarted)
                {
                    // Only store if we're inside a message
                    buffer[index++] = c;
                }
                else if (!messageStarted && index == 0 && c != ' ')
                {
                    // For backward compatibility, start a non-framed command
                    buffer[index++] = c;
                }
                else if (!messageStarted && index > 0)
                {
                    // Continue a non-framed command
                    buffer[index++] = c;
                }
            }
        }
    }
}

void parseCommand(const char *cmd)
{
    // Debug: Echo received command if in debug mode
    if (DEBUG_MODE)
    {
        Serial.print("Received command: ");
        Serial.println(cmd);
    }

    // Check for command type and parameters
    char command[16] = {0};
    char params[COMMAND_BUFFER_SIZE - 16] = {0};

    // Extract command and parameters (split on first colon)
    int colonPos = -1;
    for (int i = 0; cmd[i] != '\0'; i++)
    {
        if (cmd[i] == ':')
        {
            colonPos = i;
            break;
        }
    }

    if (colonPos >= 0)
    {
        strncpy(command, cmd, colonPos);
        command[colonPos] = '\0';
        strcpy(params, cmd + colonPos + 1);
    }
    else
    {
        strcpy(command, cmd);
    }

    // Handle different command types
    if (strcmp(command, "TEST") == 0)
    {
        testMotors();
        Serial.println("<ACK:TEST>");
        return;
    }

    // Check for PING command
    if (strcmp(command, "PING") == 0)
    {
        Serial.println("<ACK:PING>");
        return;
    } 
    
    // Check for STOP/CLEAR commands
    if (strcmp(command, "STOP") == 0 || strcmp(command, "CLEAR") == 0)
    {
        stopAllMotors();
        emergencyStop = false;
        Serial.println("<ACK:STOP>");
        return;
    } 
    
    // Handle MOV command
    if (strcmp(command, "MOV") == 0)
    {
        // Parse movement parameters: direction,speed
        int direction = 0;
        int speed = 0;

        if (sscanf(params, "%d,%d", &direction, &speed) == 2)
        {
            executeMovement(direction, speed);
            Serial.println("<ACK:MOV>");
        }
        else
        {
            Serial.println("<ERR:Invalid MOV params>");
        }
        return;
    }

    // Handle ROT command for rotation
    if (strcmp(command, "ROT") == 0)
    {
        // Parse rotation parameters: direction,speed (1 = LEFT, 2 = RIGHT)
        int direction = 0;
        int speed = 0;

        if (sscanf(params, "%d,%d", &direction, &speed) == 2)
        {
            if (direction == 1)
            {
                rotateLeft(speed);
                Serial.println("<ACK:ROT:LEFT>");
            }
            else if (direction == 2)
            {
                rotateRight(speed);
                Serial.println("<ACK:ROT:RIGHT>");
            }
            else
            {
                stopAllMotors();
                Serial.println("<ERR:Invalid rotation direction>");
            }
        }
        else
        {
            Serial.println("<ERR:Invalid ROT params>");
        }
        return;
    } // Handle SPEED command
    if (strcmp(command, "SPEED") == 0)
    {
        // Parse speed parameters: max_speed,min_speed
        int max_speed = 0;
        int min_speed = 0;

        if (sscanf(params, "%d,%d", &max_speed, &min_speed) == 2)
        {
            // Validate and apply speed limits
            MAX_SPEED = constrain(max_speed, 50, 150);
            MIN_SPEED = constrain(min_speed, 40, MAX_SPEED - 10);

            Serial.println("<ACK:SPEED>");
        }
        else
        {
            Serial.println("<ERR:Invalid SPEED params>");
        }
        return;
    } // Handle SENS command
    if (strcmp(command, "SENS") == 0)
    {
        // Output sensor distances with proper message framing
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

    // Handle MODE command to toggle between 2-wheel and 3-wheel configuration
    if (strcmp(command, "MODE") == 0)
    {
        // Parse mode parameter: 0 = two wheels, 1 = three wheels
        int mode = 0;

        if (sscanf(params, "%d", &mode) == 1)
        {
            if (mode == 0 || mode == 1)
            {
                useThreeWheels = (mode == 1);
                Serial.print("<ACK:MODE:");
                Serial.print(useThreeWheels ? "THREE_WHEEL" : "TWO_WHEEL");
                Serial.println(">");
            }
            else
            {
                Serial.println("<ERR:Invalid mode value (use 0 or 1)>");
            }
        }
        else
        {
            Serial.println("<ERR:Invalid MODE param>");
        }
        return;
    }

    // If we get here, command wasn't recognized
    Serial.print("<ERR:Unknown command ");
    Serial.print(command);
    Serial.println(">");
}