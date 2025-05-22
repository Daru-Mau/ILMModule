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

// === I2C Configuration ===
#define I2C_SLAVE_ADDRESS 0x08        // Slave address for this Arduino
#define I2C_MASTER_OVERRIDE_PIN 12    // Digital pin that master can use to override
bool masterOverrideActive = false;    // Flag to indicate if master override is active
bool prevMasterOverrideState = false; // To detect changes in override state

// === Performance Settings ===
#define SERIAL_BAUD_RATE 115200
#define COMMAND_BUFFER_SIZE 64
#define CONTROL_LOOP_INTERVAL 50 // 20Hz control loop
// Changed from #define to global variables so they can be modified at runtime
int MAX_SPEED = 100;
int MIN_SPEED = 50;
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

#define RPWM_LEFT 9
#define LPWM_LEFT 10
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
const float SAFE_DISTANCE = 60.0;      // Safe operating distance (cm)
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
bool frontEmergencyStop = false; // Blocks forward movement
bool backEmergencyStop = false;  // Blocks backward movement
bool leftEmergencyStop = false;  // Blocks left movement
bool rightEmergencyStop = false; // Blocks right movement
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

// Function to ensure all motor enable pins are set to HIGH
void ensureMotorEnablePins()
{
    // Left motor
    digitalWrite(motorLeft.REN, HIGH);
    digitalWrite(motorLeft.LEN, HIGH);

    // Right motor
    digitalWrite(motorRight.REN, HIGH);
    digitalWrite(motorRight.LEN, HIGH);

    // Back motor
    digitalWrite(motorBack.REN, HIGH);
    digitalWrite(motorBack.LEN, HIGH);
}

// Move a single motor
void moveMotor(Motor &motor, Direction dir, float targetSpeed)
{
    // Always ensure enable pins are HIGH before sending motor commands
    digitalWrite(motor.REN, HIGH);
    digitalWrite(motor.LEN, HIGH);

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

    // Debug output for left motor operations
    if (&motor == &motorLeft && DEBUG_MODE)
    {
        Serial.print("Left motor command: Dir=");
        Serial.print(dir);
        Serial.print(" Speed=");
        Serial.println(speed);
    }

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

    if (masterOverrideActive && DEBUG_MODE)
    {
        Serial.println("<STOPPED:MASTER_OVERRIDE>");
    }
}

// Modified dynamic speed calculation to prioritize commanded speed
float calculateDynamicSpeed(float distance, float targetSpeed)
{
    // Emergency stop for critical distances only
    if (distance <= CRITICAL_DISTANCE)
    {
        return 0; // Emergency stop
    }
    // For slow down range, apply a gentler reduction that doesn't override commanded speed as much
    else if (distance <= SLOW_DOWN_DISTANCE)
    {
        // Calculate reduction factor (0.5-1.0) based on how close to critical distance
        float reductionFactor = 0.5 + (0.5 * (distance - CRITICAL_DISTANCE) / (SLOW_DOWN_DISTANCE - CRITICAL_DISTANCE));

        // Apply reduction but ensure we never go below MIN_SPEED
        float reducedSpeed = targetSpeed * reductionFactor;
        return max(MIN_SPEED, reducedSpeed);
    }

    // Outside of slow down range, use commanded speed directly
    return targetSpeed;
}

// Rotation functions
void rotateLeft(int speed = MIN_SPEED)
{
    // Check for leftEmergencyStop or rightEmergencyStop since rotation uses both sides
    if (leftEmergencyStop || rightEmergencyStop)
    {
        if (DEBUG_MODE)
            Serial.println("<ROTATION_BLOCKED:EMERGENCY_STOP>");
        return;
    }

    if (DEBUG_MODE)
        Serial.println("Rotating LEFT (CCW)");

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    // Both wheels spin in the same direction to rotate in place
    moveMotor(motorLeft, FORWARD, speed * 1.30);
    moveMotor(motorRight, FORWARD, speed * 0.25);

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
    // Check for leftEmergencyStop or rightEmergencyStop since rotation uses both sides
    if (leftEmergencyStop || rightEmergencyStop)
    {
        if (DEBUG_MODE)
            Serial.println("<ROTATION_BLOCKED:EMERGENCY_STOP>");
        return;
    }

    if (DEBUG_MODE)
        Serial.println("Rotating RIGHT (CW)");

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    // Both wheels spin in the same direction to rotate in place
    moveMotor(motorLeft, BACKWARD, speed * 0.25);
    moveMotor(motorRight, BACKWARD, speed * 1.30);

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

void executeMovement(int direction, int speed)
{
    // Check if master override active
    if (masterOverrideActive)
    {
        if (DEBUG_MODE)
        {
            Serial.println("<MOVEMENT_BLOCKED:MASTER_OVERRIDE>");
        }
        return;
    }

    // Check direction-specific emergency stops
    if ((direction == 1 && frontEmergencyStop) ||
        (direction == 2 && backEmergencyStop) ||
        (direction == 3 && leftEmergencyStop) ||
        (direction == 4 && rightEmergencyStop) ||
        (direction == 5 && (leftEmergencyStop || rightEmergencyStop)) ||
        (direction == 6 && (leftEmergencyStop || rightEmergencyStop)))
    {
        if (DEBUG_MODE)
        {
            Serial.print("<MOVEMENT_BLOCKED:EMERGENCY_STOP_");
            switch (direction)
            {
            case 1:
                Serial.println("FORWARD>");
                break;
            case 2:
                Serial.println("BACKWARD>");
                break;
            case 3:
                Serial.println("LEFT>");
                break;
            case 4:
                Serial.println("RIGHT>");
                break;
            case 5:
                Serial.println("ROTATE_LEFT>");
                break;
            case 6:
                Serial.println("ROTATE_RIGHT>");
                break;
            }
        }
        return;
    }

    // Make sure all motor enable pins are HIGH
    ensureMotorEnablePins();

    // Read distances for dynamic speed adjustment
    float forwardDist = min(min(distFL, distF), distFR);
    float backwardDist = min(min(distBL, distB), distBR);
    float leftDist = min(distFL, distBL);
    float rightDist = min(distFR, distBR);

    // Only apply dynamic speed adjustment for really close obstacles
    // Otherwise, let the Raspberry Pi control the speed directly
    if (direction == 1 && forwardDist < SLOW_DOWN_DISTANCE)
        speed = calculateDynamicSpeed(forwardDist, speed);
    else if (direction == 2 && backwardDist < SLOW_DOWN_DISTANCE)
        speed = calculateDynamicSpeed(backwardDist, speed);
    else if (direction == 3 && leftDist < SLOW_DOWN_DISTANCE)
        speed = calculateDynamicSpeed(leftDist, speed);
    else if (direction == 4 && rightDist < SLOW_DOWN_DISTANCE)
        speed = calculateDynamicSpeed(rightDist, speed);
    else if (direction == 5 && leftDist < SLOW_DOWN_DISTANCE)
        speed = calculateDynamicSpeed(leftDist, speed);
    else if (direction == 6 && rightDist < SLOW_DOWN_DISTANCE)
        speed = calculateDynamicSpeed(rightDist, speed);

    switch (direction)
    {
    case 1: // FORWARD
        if (DEBUG_MODE)
            Serial.println("Moving FORWARD");
        moveMotor(motorLeft, BACKWARD, speed);
        moveMotor(motorRight, FORWARD, speed);
        moveMotor(motorBack, STOP, 0);
        break;

    case 2: // BACKWARD
        if (DEBUG_MODE)
            Serial.println("Moving BACKWARD");
        moveMotor(motorLeft, FORWARD, speed);
        moveMotor(motorRight, BACKWARD, speed);
        moveMotor(motorBack, STOP, 0);
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

// Function to read distance with improved reliability
float readUltrasonicDistance(int trigPin, int echoPin)
{
    // Try up to 3 readings to get a valid value
    for (int attempt = 0; attempt < 3; attempt++)
    {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        // Use timeout to avoid hanging
        unsigned long startTime = micros();
        float duration = pulseIn(echoPin, HIGH, 23200); // 4m max range timeout

        // Check if reading is valid (not 0 and not timeout)
        if (duration > 0)
        {
            float distance = (duration * 0.0343) / 2.0; // Convert to centimeters

            // Filter out unreasonable values (>400cm or <2cm)
            if (distance >= 2.0 && distance <= 400.0)
            {
                return distance;
            }
        }

        // Brief delay before retry
        delayMicroseconds(50);
    }

    // If all attempts failed, return a safe high value
    return SAFE_DISTANCE * 1.5; // Return a value that won't trigger slowdown
}

void updateDistances()
{
    // Read all sensor distances with filtering to prevent erratic behavior
    // Save previous readings
    static float prevFL = SAFE_DISTANCE, prevF = SAFE_DISTANCE, prevFR = SAFE_DISTANCE;
    static float prevBL = SAFE_DISTANCE, prevB = SAFE_DISTANCE, prevBR = SAFE_DISTANCE;

    // Read new values
    float newFL = readUltrasonicDistance(TRIG_FL, ECHO_FL);
    float newF = readUltrasonicDistance(TRIG_F, ECHO_F);
    float newFR = readUltrasonicDistance(TRIG_FR, ECHO_FR);
    float newBL = readUltrasonicDistance(TRIG_BL, ECHO_BL);
    float newB = readUltrasonicDistance(TRIG_B, ECHO_B);
    float newBR = readUltrasonicDistance(TRIG_BR, ECHO_BR);

    // In debug mode, periodically check for critically close readings
    static unsigned long lastCriticalCheck = 0;
    if (DEBUG_MODE)
    {
        unsigned long now = millis();
        if (now - lastCriticalCheck > 1000)
        { // Check once per second
            if (newF < CRITICAL_DISTANCE || newFL < CRITICAL_DISTANCE || newFR < CRITICAL_DISTANCE ||
                newB < CRITICAL_DISTANCE || newBL < CRITICAL_DISTANCE || newBR < CRITICAL_DISTANCE)
            {
                Serial.print("<DEBUG:CRITICAL_DISTANCE:");
                Serial.print(newFL < CRITICAL_DISTANCE ? "FL " : "");
                Serial.print(newF < CRITICAL_DISTANCE ? "F " : "");
                Serial.print(newFR < CRITICAL_DISTANCE ? "FR " : "");
                Serial.print(newBL < CRITICAL_DISTANCE ? "BL " : "");
                Serial.print(newB < CRITICAL_DISTANCE ? "B " : "");
                Serial.print(newBR < CRITICAL_DISTANCE ? "BR" : "");
                Serial.println(">");
            }
            lastCriticalCheck = now;
        }
    }

    // Apply simple filtering - if new reading is drastically different,
    // verify with additional reading before accepting
    distFL = filterReading(prevFL, newFL);
    distF = filterReading(prevF, newF);
    distFR = filterReading(prevFR, newFR);
    distBL = filterReading(prevBL, newBL);
    distB = filterReading(prevB, newB);
    distBR = filterReading(prevBR, newBR);

    // Update previous values for next iteration
    prevFL = distFL;
    prevF = distF;
    prevFR = distFR;
    prevBL = distBL;
    prevB = distB;
    prevBR = distBR;
}

// Helper function to filter unreliable readings
float filterReading(float prevValue, float newValue)
{
    // If reading jumps by more than 50% and is less than the safe distance,
    // be conservative and use the smaller value
    if (abs(newValue - prevValue) > (prevValue * 0.5) && newValue < SAFE_DISTANCE)
    {
        // Verify with an additional reading
        float verifyValue = readUltrasonicDistance(TRIG_FL, ECHO_FL);
        if (abs(verifyValue - newValue) < abs(verifyValue - prevValue))
        {
            return newValue; // New reading confirmed
        }
        else
        {
            return prevValue; // Keep previous reading
        }
    }
    return newValue; // Accept new reading
}

// === I2C Functions ===

// Function called when data is received from the master
void receiveEvent(int howMany)
{
    if (Wire.available())
    {
        char command = Wire.read();

        // Process command from master
        switch (command)
        {
        case 'S': // Stop/Suspend operations
            masterOverrideActive = true;
            stopAllMotors(); // Immediately stop all motors
            if (DEBUG_MODE)
            {
                Serial.println("<I2C:OVERRIDE_ACTIVE>");
            }
            break;

        case 'R': // Resume operations
            masterOverrideActive = false;
            if (DEBUG_MODE)
            {
                Serial.println("<I2C:OVERRIDE_RELEASED>");
            }
            break;

        default:
            // Unknown command
            if (DEBUG_MODE)
            {
                Serial.print("<I2C:UNKNOWN_CMD:");
                Serial.print(command);
                Serial.println(">");
            }
            break;
        }
    }
}

// Function to handle master's request for data
void requestEvent()
{
    // Send current status to master when requested
    if (masterOverrideActive)
    {
        Wire.write('S'); // Suspended
    }
    else if (emergencyStop)
    {
        Wire.write('E'); // Emergency stop
    }
    else
    {
        Wire.write('A'); // Active
    }
}

// Function to check if override pin is active (alternative to I2C communication)
void checkOverridePin()
{
    bool currentState = (digitalRead(I2C_MASTER_OVERRIDE_PIN) == HIGH);

    // Detect changes in override state
    if (currentState != prevMasterOverrideState)
    {
        if (currentState)
        {
            // Pin went high - activate override
            masterOverrideActive = true;
            stopAllMotors();
            if (DEBUG_MODE)
            {
                Serial.println("<PIN:OVERRIDE_ACTIVE>");
            }
        }
        else
        {
            // Pin went low - release override
            masterOverrideActive = false;
            if (DEBUG_MODE)
            {
                Serial.println("<PIN:OVERRIDE_RELEASED>");
            }
        }
        prevMasterOverrideState = currentState;
    }
}

// Global controller instance

// === SETUP AND LOOP FUNCTIONS ===

void setup()
{
    // Clear any existing serial data
    Serial.end();
    delay(100);

    // Start serial with proper baud rate
    Serial.begin(115200);
    delay(1000); // Wait for serial to fully initialize

    // Clear any initial data
    while (Serial.available())
    {
        Serial.read();
    }

    // 1. Hardware Initialization First (moved to top)
    // Setup motors - Initialize pins BEFORE anything else
    setupMotorPins(motorLeft);
    setupMotorPins(motorRight);
    setupMotorPins(motorBack);

    // Make sure enable pins are set properly
    ensureMotorEnablePins();

    // Setup I2C as slave
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    // ==== Setup override pin ====
    /* pinMode(I2C_MASTER_OVERRIDE_PIN, INPUT);
    prevMasterOverrideState = (digitalRead(I2C_MASTER_OVERRIDE_PIN) == HIGH);
    masterOverrideActive = prevMasterOverrideState;
     */

    // === TEMPORAL MEASURE TO DEACTIVATE MASTER OVERRIDE ===
    pinMode(I2C_MASTER_OVERRIDE_PIN, INPUT_PULLUP); // Using internal pullup resistor
    prevMasterOverrideState = false;                // Force this to false
    masterOverrideActive = false;                   // Force override to be inactive

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

    // Setup ultrasonic sensors
    int trigPins[] = {TRIG_FL, TRIG_F, TRIG_FR, TRIG_BL, TRIG_B, TRIG_BR};
    int echoPins[] = {ECHO_FL, ECHO_F, ECHO_FR, ECHO_BL, ECHO_B, ECHO_BR};
    for (int i = 0; i < 6; i++)
    {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
        digitalWrite(trigPins[i], LOW); // Ensure clean start
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
    analogWrite(motorLeft.RPWM, testSpeed); // FORWARD
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
    analogWrite(motorLeft.LPWM, testSpeed); // BACKWARD
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
    // Check if master override is active (via pin)
    /*     checkOverridePin();
     */
    // If master override is active, only process I2C but not serial or motor commands
    if (masterOverrideActive)
    {
        // Only report status periodically if in debug mode
        unsigned long now = millis();
        static unsigned long lastOverrideStatus = 0;
        if (DEBUG_MODE && (now - lastOverrideStatus > 5000))
        {
            Serial.println("<MASTER_OVERRIDE_ACTIVE>");
            lastOverrideStatus = now;
        }
        return; // Skip the rest of the loop function
    }

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

        // Check emergency stop conditions directionally
        bool frontDanger = (distF < CRITICAL_DISTANCE || distFL < CRITICAL_DISTANCE || distFR < CRITICAL_DISTANCE);
        bool backDanger = (distB < CRITICAL_DISTANCE || distBL < CRITICAL_DISTANCE || distBR < CRITICAL_DISTANCE);
        bool leftDanger = (distFL < CRITICAL_DISTANCE || distBL < CRITICAL_DISTANCE);
        bool rightDanger = (distFR < CRITICAL_DISTANCE || distBR < CRITICAL_DISTANCE);

        // Update directional emergency flags
        if (frontDanger != frontEmergencyStop)
        {
            frontEmergencyStop = frontDanger;
            if (frontEmergencyStop && DEBUG_MODE)
            {
                Serial.println("<EMERGENCY_STOP:FRONT>");
            }
            else if (!frontEmergencyStop && DEBUG_MODE)
            {
                Serial.println("<EMERGENCY_RELEASED:FRONT>");
            }
        }

        if (backDanger != backEmergencyStop)
        {
            backEmergencyStop = backDanger;
            if (backEmergencyStop && DEBUG_MODE)
            {
                Serial.println("<EMERGENCY_STOP:BACK>");
            }
            else if (!backEmergencyStop && DEBUG_MODE)
            {
                Serial.println("<EMERGENCY_RELEASED:BACK>");
            }
        }

        if (leftDanger != leftEmergencyStop)
        {
            leftEmergencyStop = leftDanger;
            if (leftEmergencyStop && DEBUG_MODE)
            {
                Serial.println("<EMERGENCY_STOP:LEFT>");
            }
            else if (!leftEmergencyStop && DEBUG_MODE)
            {
                Serial.println("<EMERGENCY_RELEASED:LEFT>");
            }
        }

        if (rightDanger != rightEmergencyStop)
        {
            rightEmergencyStop = rightDanger;
            if (rightEmergencyStop && DEBUG_MODE)
            {
                Serial.println("<EMERGENCY_STOP:RIGHT>");
            }
            else if (!rightEmergencyStop && DEBUG_MODE)
            {
                Serial.println("<EMERGENCY_RELEASED:RIGHT>");
            }
        }

        // Set overall emergencyStop flag (for compatibility with existing code)
        bool prevEmergencyStop = emergencyStop;
        emergencyStop = frontEmergencyStop || backEmergencyStop || leftEmergencyStop || rightEmergencyStop;

        // Don't stop all motors when an emergency stop is detected - instead let the
        // direction-specific emergency stops control movement in executeMovement().
        // Just log the sensor information in debug mode
        if (emergencyStop && !prevEmergencyStop && DEBUG_MODE)
        {
            Serial.print("Distances: FL=");
            Serial.print(distFL);
            Serial.print(" F=");
            Serial.print(distF);
            Serial.print(" FR=");
            Serial.print(distFR);
            Serial.print(" BL=");
            Serial.print(distBL);
            Serial.print(" B=");
            Serial.print(distB);
            Serial.print(" BR=");
            Serial.println(distBR);
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

    // Special test command just for left motor debugging
    if (strcmp(command, "TESTLEFT") == 0)
    {
        int speed = 80;
        int direction = 0;

        // Parse optional parameters if provided
        if (params[0] != '\0')
        {
            sscanf(params, "%d,%d", &direction, &speed);
        }

        Serial.println("<DIRECT LEFT MOTOR TEST>");
        // Direct pin control with verbose output
        digitalWrite(motorLeft.REN, HIGH);
        digitalWrite(motorLeft.LEN, HIGH);

        Serial.print("Testing left motor: Direction=");
        Serial.print(direction);
        Serial.print(" Speed=");
        Serial.println(speed);

        if (direction == 0)
        {
            // Forward
            Serial.println("LEFT MOTOR FORWARD");
            analogWrite(motorLeft.RPWM, speed);
            analogWrite(motorLeft.LPWM, 0);
        }
        else
        {
            // Backward
            Serial.println("LEFT MOTOR BACKWARD");
            analogWrite(motorLeft.RPWM, 0);
            analogWrite(motorLeft.LPWM, speed);
        }

        delay(1000); // Run for 1 second

        // Stop
        Serial.println("LEFT MOTOR STOPPING");
        analogWrite(motorLeft.RPWM, 0);
        analogWrite(motorLeft.LPWM, 0);

        Serial.println("<ACK:TESTLEFT>");
        return;
    }

    // Check for STOP/CLEAR commands
    if (strcmp(command, "STOP") == 0 || strcmp(command, "CLEAR") == 0)
    {
        stopAllMotors();

        // Check for direction-specific clear parameter
        if (params[0] != '\0')
        {
            // Direction specific clear requested
            if (strcmp(params, "FRONT") == 0 && distF > SAFE_DISTANCE && distFL > SAFE_DISTANCE && distFR > SAFE_DISTANCE)
            {
                frontEmergencyStop = false;
                if (DEBUG_MODE)
                    Serial.println("<CLEARED:FRONT_EMERGENCY>");
            }
            else if (strcmp(params, "BACK") == 0 && distB > SAFE_DISTANCE && distBL > SAFE_DISTANCE && distBR > SAFE_DISTANCE)
            {
                backEmergencyStop = false;
                if (DEBUG_MODE)
                    Serial.println("<CLEARED:BACK_EMERGENCY>");
            }
            else if (strcmp(params, "LEFT") == 0 && distFL > SAFE_DISTANCE && distBL > SAFE_DISTANCE)
            {
                leftEmergencyStop = false;
                if (DEBUG_MODE)
                    Serial.println("<CLEARED:LEFT_EMERGENCY>");
            }
            else if (strcmp(params, "RIGHT") == 0 && distFR > SAFE_DISTANCE && distBR > SAFE_DISTANCE)
            {
                rightEmergencyStop = false;
                if (DEBUG_MODE)
                    Serial.println("<CLEARED:RIGHT_EMERGENCY>");
            }
            else if (strcmp(params, "ALL") == 0)
            {
                // Check each direction individually
                bool clearedAny = false;

                if (frontEmergencyStop && distF > SAFE_DISTANCE && distFL > SAFE_DISTANCE && distFR > SAFE_DISTANCE)
                {
                    frontEmergencyStop = false;
                    clearedAny = true;
                    if (DEBUG_MODE)
                        Serial.println("<CLEARED:FRONT_EMERGENCY>");
                }

                if (backEmergencyStop && distB > SAFE_DISTANCE && distBL > SAFE_DISTANCE && distBR > SAFE_DISTANCE)
                {
                    backEmergencyStop = false;
                    clearedAny = true;
                    if (DEBUG_MODE)
                        Serial.println("<CLEARED:BACK_EMERGENCY>");
                }

                if (leftEmergencyStop && distFL > SAFE_DISTANCE && distBL > SAFE_DISTANCE)
                {
                    leftEmergencyStop = false;
                    clearedAny = true;
                    if (DEBUG_MODE)
                        Serial.println("<CLEARED:LEFT_EMERGENCY>");
                }

                if (rightEmergencyStop && distFR > SAFE_DISTANCE && distBR > SAFE_DISTANCE)
                {
                    rightEmergencyStop = false;
                    clearedAny = true;
                    if (DEBUG_MODE)
                        Serial.println("<CLEARED:RIGHT_EMERGENCY>");
                }

                // Send appropriate acknowledgment for clearing all directions
                if (clearedAny)
                {
                    Serial.println("<ACK:STOP:EMERGENCY_PARTLY_CLEARED>");
                }
                else
                {
                    Serial.println("<ACK:STOP:NO_CHANGE>");
                }
            }
            else
            {
                Serial.println("<ERR:Invalid CLEAR direction>");
            }
        }
        else
        {
            // Default behavior: try to clear all emergency stops if obstacles are no longer present
            bool clearedAny = false;

            if (frontEmergencyStop && distF > SAFE_DISTANCE && distFL > SAFE_DISTANCE && distFR > SAFE_DISTANCE)
            {
                frontEmergencyStop = false;
                clearedAny = true;
                if (DEBUG_MODE)
                    Serial.println("<CLEARED:FRONT_EMERGENCY>");
            }

            if (backEmergencyStop && distB > SAFE_DISTANCE && distBL > SAFE_DISTANCE && distBR > SAFE_DISTANCE)
            {
                backEmergencyStop = false;
                clearedAny = true;
                if (DEBUG_MODE)
                    Serial.println("<CLEARED:BACK_EMERGENCY>");
            }

            if (leftEmergencyStop && distFL > SAFE_DISTANCE && distBL > SAFE_DISTANCE)
            {
                leftEmergencyStop = false;
                clearedAny = true;
                if (DEBUG_MODE)
                    Serial.println("<CLEARED:LEFT_EMERGENCY>");
            }

            if (rightEmergencyStop && distFR > SAFE_DISTANCE && distBR > SAFE_DISTANCE)
            {
                rightEmergencyStop = false;
                clearedAny = true;
                if (DEBUG_MODE)
                    Serial.println("<CLEARED:RIGHT_EMERGENCY>");
            }

            // Send appropriate acknowledgment
            if (clearedAny)
            {
                Serial.println("<ACK:STOP:EMERGENCY_PARTLY_CLEARED>");
            }
            else if (emergencyStop)
            {
                Serial.println("<ACK:STOP:EMERGENCY_ACTIVE>");
            }
            else
            {
                Serial.println("<ACK:STOP>");
            }
        }

        // Update the overall emergency flag
        emergencyStop = frontEmergencyStop || backEmergencyStop || leftEmergencyStop || rightEmergencyStop;
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
            MAX_SPEED = constrain(max_speed, 50, 255);
            MIN_SPEED = constrain(min_speed, 30, MAX_SPEED - 5);

            Serial.print("<ACK:SPEED:");
            Serial.print(MAX_SPEED);
            Serial.print(",");
            Serial.print(MIN_SPEED);
            Serial.println(">");
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

    // Handle I2C status command
    if (strcmp(command, "I2C") == 0)
    {
        // Report I2C status
        Serial.print("<ACK:I2C:");
        Serial.print(masterOverrideActive ? "OVERRIDE_ACTIVE" : "NORMAL");
        Serial.println(">");
        return;
    }

    // Handle STATUS command to report current state
    if (strcmp(command, "STATUS") == 0)
    {
        // Report robot status including emergency flags and sensor readings
        Serial.print("<ACK:STATUS:");

        if (frontEmergencyStop || backEmergencyStop || leftEmergencyStop || rightEmergencyStop)
        {
            Serial.print("EMERGENCY:");
            Serial.print(frontEmergencyStop ? "F" : "");
            Serial.print(backEmergencyStop ? "B" : "");
            Serial.print(leftEmergencyStop ? "L" : "");
            Serial.print(rightEmergencyStop ? "R" : "");
        }
        else
        {
            Serial.print("NORMAL");
        }

        Serial.print(":");
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

    // If we get here, command wasn't recognized
    Serial.print("<ERR:Unknown command ");
    Serial.print(command);
    Serial.println(">");
}