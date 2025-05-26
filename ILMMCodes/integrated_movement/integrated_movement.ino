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
// Addresses for I2C communication
#define MY_ADDR 0b100     // Localization module's 3-bit address (4 in decimal)
#define MASTER_ADDR 0b001 // Master's 3-bit address (1 in decimal)

// Commands from Full_Master.ino
#define ASK_READY_CMD 0b00011  // Master asks if slaves are ready
#define TELL_READY_CMD 0b11100 // Slaves tell master they are ready
#define GO_IDLE_CMD 0b10111    // Master tells slaves to go into idle mode
#define GO_ACTIVE_CMD 0b11000  // Master tells slaves to go into active mode

// Legacy I2C configuration
#define I2C_SLAVE_ADDRESS 0x04        // Slave address for this Arduino (same as MY_ADDR but in hex)
#define I2C_MASTER_OVERRIDE_PIN 12    // Digital pin that master can use to override
bool masterOverrideActive = false;    // Flag to indicate if master override is active
bool prevMasterOverrideState = false; // To detect changes in override state
bool moduleReady = false;             // Flag to indicate if module is ready

// === Performance Settings ===
#define SERIAL_BAUD_RATE 115200
#define COMMAND_BUFFER_SIZE 64
#define CONTROL_LOOP_INTERVAL 50 // 20Hz control loop
// Changed from #define to global variables so they can be modified at runtime
int MAX_SPEED = 100;
int MIN_SPEED = 50;
int MAX_ROTATION_SPEED = 65; // Maximum speed for rotation to prevent escalation
#define DEBUG_MODE false     // Keep this false to prevent debug messages interfering with UART communication
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
#define RPWM_LEFT 5   // FIXED: Swapped to match basic_moveset.ino
#define LPWM_LEFT 4   // FIXED: Swapped to match basic_moveset.ino
#define REN_LEFT 44
#define LEN_LEFT 45
#define RPWM_RIGHT 2  // FIXED: Swapped to match basic_moveset.ino
#define LPWM_RIGHT 3  // FIXED: Swapped to match basic_moveset.ino
#define REN_RIGHT 39
#define LEN_RIGHT 38
#define RPWM_BACK 7
#define LPWM_BACK 6
#define REN_BACK 51
#define LEN_BACK 50 */

// Motor Driver Pins - Using COUNTER-CLOCKWISE arrangement from basic_moveset
// The motors have been positioned as follows:
// - LEFT wheel (pins 2, 3, 38, 39)
// - RIGHT wheel (pins 7, 6, 51, 50)
// - BACK wheel (pins 4, 5, 44, 45)

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

    // Store the previous direction to detect changes
    static Direction prevDirection[3] = {STOP, STOP, STOP};
    Direction *prevDir = nullptr;

    // Determine which motor we're operating on to track its previous direction
    if (&motor == &motorLeft)
        prevDir = &prevDirection[0];
    else if (&motor == &motorRight)
        prevDir = &prevDirection[1];
    else if (&motor == &motorBack)
        prevDir = &prevDirection[2];

    // If direction changed, reset speed accumulation for smoother transitions
    if (prevDir && *prevDir != dir && *prevDir != STOP)
    {
        motor.currentSpeed = 0;
    }

    // Update the previous direction
    if (prevDir)
        *prevDir = dir;

    targetSpeed = constrain(targetSpeed, MIN_SPEED, MAX_SPEED);
    if (targetSpeed > motor.currentSpeed)
    {
        motor.currentSpeed = min(targetSpeed, motor.currentSpeed + MAX_SPEED * ACCEL_RATE);
    }
    else if (targetSpeed < motor.currentSpeed)
    {
        motor.currentSpeed = max(targetSpeed, motor.currentSpeed - MAX_SPEED * ACCEL_RATE);
    }

    int speed = (int)motor.currentSpeed;
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

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

// Safety-focused dynamic speed calculation to scale down desired speed when obstacles detected
float calculateDynamicSpeed(float distance, float desiredSpeed)
{
    // Emergency stop for critical distances only
    if (distance <= CRITICAL_DISTANCE)
    {
        return 0; // Emergency stop - safety first!
    }
    // For slow down range, apply a progressive speed reduction based on proximity
    else if (distance <= SLOW_DOWN_DISTANCE)
    {
        // Calculate safety factor (0.5-1.0) based on distance from obstacle
        // Closer to obstacle = lower safety factor
        float safetyFactor = 0.5 + (0.5 * (distance - CRITICAL_DISTANCE) / (SLOW_DOWN_DISTANCE - CRITICAL_DISTANCE));

        // Apply safety reduction to desired speed but ensure we never go below MIN_SPEED
        float safeSpeed = desiredSpeed * safetyFactor;
        return max(MIN_SPEED, safeSpeed);
    }

    // Outside of slow down range, the desired speed is already safe to use
    return desiredSpeed;
}

// Rotation functions
void rotateLeft(int speed = MIN_SPEED)
{
    if (DEBUG_MODE)
        Serial.println("Rotating LEFT (CCW)");

    // Apply rotation-specific speed limit to prevent escalation
    speed = constrain(speed, MIN_SPEED, MAX_ROTATION_SPEED);

    // Reset current speeds to prevent accumulation from previous movements
    motorLeft.currentSpeed = 0;
    motorRight.currentSpeed = 0;

    // Use back wheel if in 3-wheel configuration
    if (useThreeWheels)
    {
        // Both wheels spin in the same direction to rotate in place
        moveMotor(motorLeft, FORWARD, speed);
        moveMotor(motorRight, FORWARD, speed);
        moveMotor(motorBack, BACKWARD, speed);
    }
    else
    {
        // In 2-wheel mode, use balanced speed ratios without escalating multipliers
        int leftSpeed = speed;
        int rightSpeed = speed;

        moveMotor(motorLeft, FORWARD, leftSpeed);
        moveMotor(motorRight, FORWARD, rightSpeed);
        moveMotor(motorBack, STOP, 0);
    }
}

void rotateRight(int speed = MIN_SPEED)
{
    if (DEBUG_MODE)
        Serial.println("Rotating RIGHT (CW)");

    // Apply rotation-specific speed limit to prevent escalation
    speed = constrain(speed, MIN_SPEED, MAX_ROTATION_SPEED);

    // Reset current speeds to prevent accumulation from previous movements
    motorLeft.currentSpeed = 0;
    motorRight.currentSpeed = 0;

    // Use back wheel if in 3-wheel configuration
    if (useThreeWheels)
    {
        // Both wheels spin in the same direction to rotate in place
        moveMotor(motorLeft, BACKWARD, speed);
        moveMotor(motorRight, BACKWARD, speed);
        moveMotor(motorBack, FORWARD, speed);
    }
    else
    {
        // In 2-wheel mode, use balanced speed ratios without escalating multipliers
        int leftSpeed = speed;
        int rightSpeed = speed;

        moveMotor(motorLeft, BACKWARD, leftSpeed);
        moveMotor(motorRight, BACKWARD, rightSpeed);
        moveMotor(motorBack, STOP, 0);
    }
}

void executeMovement(int direction, int desiredSpeed)
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

    // Prepare a safety-adjusted speed value starting with the desired speed
    int safeSpeed = desiredSpeed;

    // Read distances for dynamic speed adjustment
    float forwardDist = min(min(distFL, distF), distFR);
    float backwardDist = min(min(distBL, distB), distBR);
    float leftDist = min(distFL, distBL);
    float rightDist = min(distFR, distBR);

    // Only apply dynamic speed adjustment for really close obstacles
    // This adjusts the desired speed for safety when obstacles are detected
    if (direction == 1 && forwardDist < SLOW_DOWN_DISTANCE)
        safeSpeed = calculateDynamicSpeed(forwardDist, desiredSpeed);
    else if (direction == 2 && backwardDist < SLOW_DOWN_DISTANCE)
        safeSpeed = calculateDynamicSpeed(backwardDist, desiredSpeed);
    else if (direction == 3 && leftDist < SLOW_DOWN_DISTANCE)
        safeSpeed = calculateDynamicSpeed(leftDist, desiredSpeed);
    else if (direction == 4 && rightDist < SLOW_DOWN_DISTANCE)
        safeSpeed = calculateDynamicSpeed(rightDist, desiredSpeed);
    else if (direction == 5 && leftDist < SLOW_DOWN_DISTANCE)
        safeSpeed = calculateDynamicSpeed(leftDist, desiredSpeed);
    else if (direction == 6 && rightDist < SLOW_DOWN_DISTANCE)
        safeSpeed = calculateDynamicSpeed(rightDist, desiredSpeed);
    else if (direction == 7 && forwardDist < SLOW_DOWN_DISTANCE)
        safeSpeed = calculateDynamicSpeed(forwardDist, desiredSpeed);
    else if (direction == 8 && forwardDist < SLOW_DOWN_DISTANCE)
        safeSpeed = calculateDynamicSpeed(forwardDist, desiredSpeed);
    else if (direction == 9 && backwardDist < SLOW_DOWN_DISTANCE)
        safeSpeed = calculateDynamicSpeed(backwardDist, desiredSpeed);
    else if (direction == 10 && backwardDist < SLOW_DOWN_DISTANCE)
        safeSpeed = calculateDynamicSpeed(backwardDist, desiredSpeed);

    // Debug for safety adjustments
    if (DEBUG_MODE && safeSpeed != desiredSpeed)
    {
        Serial.print("<SPEED_ADJUSTED:desired=");
        Serial.print(desiredSpeed);
        Serial.print(",safe=");
        Serial.print(safeSpeed);
        Serial.println(">");
    }

    switch (direction)
    {
    case 1: // FORWARD
        moveForward(safeSpeed);
        break;

    case 2: // BACKWARD
        moveBackward(safeSpeed);
        break;

    case 3: // ARC TURN LEFT
        turnLeft(safeSpeed);
        break;

    case 4: // ARC TURN RIGHT
        turnRight(safeSpeed);
        break;

    case 5: // ROTATE LEFT
        // Apply rotation-specific speed limit for safety
        rotateLeft(constrain(safeSpeed, MIN_SPEED, MAX_ROTATION_SPEED));
        break;

    case 6: // ROTATE RIGHT
        // Apply rotation-specific speed limit for safety
        rotateRight(constrain(safeSpeed, MIN_SPEED, MAX_ROTATION_SPEED));
        break;

    case 7: // LEFT LATERAL MOVEMENT
        slideLeft(safeSpeed);
        break;

    case 8: // RIGHT LATERAL MOVEMENT
        slideRight(safeSpeed);
        break;

    case 9: // DIAGONAL FORWARD-LEFT
        moveDiagonalForwardLeft(safeSpeed);
        break;

    case 10: // DIAGONAL FORWARD-RIGHT
        moveDiagonalForwardRight(safeSpeed);
        break;

    case 11: // DIAGONAL BACKWARD-LEFT
        moveDiagonalBackwardLeft(safeSpeed);
        break;

    case 12: // DIAGONAL BACKWARD-RIGHT
        moveDiagonalBackwardRight(safeSpeed);
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
        uint8_t message = Wire.read();

        // Extract destination address (top 3 bits) and command (lower 5 bits)
        uint8_t dest = message >> 5;
        uint8_t cmd = message & 0b00011111;

        // Only process messages addressed to this module
        if (dest != MY_ADDR)
            return;

        // Handle commands from master
        switch (cmd)
        {
        case ASK_READY_CMD:
            // Master is asking if we're ready
            if (moduleReady)
            {
                // Send ready response
                uint8_t response = (MASTER_ADDR << 5) | TELL_READY_CMD;
                Wire.beginTransmission(MASTER_ADDR);
                Wire.write(response);
                Wire.endTransmission();

                if (DEBUG_MODE)
                {
                    Serial.println("<I2C:SENT_READY_STATUS>");
                }
            }
            break;

        case GO_IDLE_CMD:
            // Master wants us to enter idle mode (stop all operations)
            masterOverrideActive = true;
            stopAllMotors();
            if (DEBUG_MODE)
            {
                Serial.println("<LOCALIZATION_MODULE:ENTERED_IDLE_MODE>");
            }
            break;

        case GO_ACTIVE_CMD:
            // Master wants us to enter active mode (resume operations)
            masterOverrideActive = false;
            if (DEBUG_MODE)
            {
                Serial.println("<LOCALIZATION_MODULE:ENTERED_ACTIVE_MODE>");
            }
            break;

        default:
            // Unknown command
            if (DEBUG_MODE)
            {
                Serial.print("<LOCALIZATION_MODULE:UNKNOWN_CMD:");
                Serial.print(cmd);
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
        Wire.write('S'); // Suspended/Idle
    }
    else if (emergencyStop)
    {
        Wire.write('E'); // Emergency stop
    }
    else
    {
        Wire.write('A'); // Active
    }

    // Add a debug log for the request event
    if (DEBUG_MODE)
    {
        Serial.println("<LOCALIZATION_MODULE:REQUEST_EVENT_HANDLED>");
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
    ensureMotorEnablePins(); // Setup I2C as slave - Using address 0x04 for the localization module
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    // Log the module type to serial
    Serial.println("<MODULE_TYPE:LOCALIZATION>");
    Serial.print("<I2C_ADDRESS:0x");
    Serial.print(I2C_SLAVE_ADDRESS, HEX);
    Serial.println(">");

    // ==== Setup override pin ====
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
    } // Mark module as ready after all initialization is complete
    moduleReady = true;

    // Log that the localization module is ready
    Serial.println("<LOCALIZATION_MODULE:INITIALIZATION_COMPLETE>");
    Serial.println("<LOCALIZATION_MODULE:READY_FOR_I2C_COMMUNICATION>");

    // Send ready status to master if it's already asking
    uint8_t response = (MASTER_ADDR << 5) | TELL_READY_CMD;
    Wire.beginTransmission(MASTER_ADDR);
    Wire.write(response);
    Wire.endTransmission();

    if (DEBUG_MODE)
    {
        Serial.println("<I2C:MARKED_READY>");
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

// Function to implement forward movement with 2/3 wheel mode support
void moveForward(int speed)
{
    if (DEBUG_MODE)
        Serial.println("Moving FORWARD");

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    // Regular movement
    moveMotor(motorLeft, BACKWARD, speed);
    moveMotor(motorRight, FORWARD, speed);

    if (useThreeWheels)
    {
        moveMotor(motorBack, FORWARD, speed);
    }
    else
    {
        moveMotor(motorBack, STOP, 0);
    }
}

// Function to implement backward movement with 2/3 wheel mode support
void moveBackward(int speed)
{
    if (DEBUG_MODE)
        Serial.println("Moving BACKWARD");

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    // Regular movement
    moveMotor(motorLeft, FORWARD, speed);
    moveMotor(motorRight, BACKWARD, speed);

    if (useThreeWheels)
    {
        moveMotor(motorBack, BACKWARD, speed);
    }
    else
    {
        moveMotor(motorBack, STOP, 0);
    }
}

// Turn left (arc left)
void turnLeft(int speed)
{
    if (DEBUG_MODE)
        Serial.println("Arc Turning LEFT");

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    moveMotor(motorLeft, BACKWARD, speed * 0.70);
    moveMotor(motorRight, FORWARD, speed);

    if (useThreeWheels)
    {
        moveMotor(motorBack, FORWARD, speed * 0.75);
    }
    else
    {
        moveMotor(motorBack, STOP, 0);
    }
}

// Turn right (arc right)
void turnRight(int speed)
{
    if (DEBUG_MODE)
        Serial.println("Arc Turning RIGHT");

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    moveMotor(motorLeft, BACKWARD, speed);
    moveMotor(motorRight, FORWARD, speed * 0.70);

    if (useThreeWheels)
    {
        moveMotor(motorBack, FORWARD, speed * 0.75);
    }
    else
    {
        moveMotor(motorBack, STOP, 0);
    }
}

// Slide left (strafe) - Updated with integrated movement logic
void slideLeft(int speed)
{
    if (DEBUG_MODE)
        Serial.println("Sliding LEFT");

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    if (useThreeWheels)
    {
        // Three-wheel configuration for lateral movement
        moveMotor(motorLeft, STOP, 0);
        moveMotor(motorRight, BACKWARD, speed);
        moveMotor(motorBack, FORWARD, speed);
    }
    else
    {
        // In 2-wheel mode, fall back to rotation
        rotateLeft(speed);
    }
}

// Slide right (strafe) - Updated with integrated movement logic
void slideRight(int speed)
{
    if (DEBUG_MODE)
        Serial.println("Sliding RIGHT");

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    if (useThreeWheels)
    {
        // Three-wheel configuration for lateral movement
        moveMotor(motorLeft, BACKWARD, speed);
        moveMotor(motorRight, STOP, 0);
        moveMotor(motorBack, BACKWARD, speed);
    }
    else
    {
        // In 2-wheel mode, fall back to rotation
        rotateRight(speed);
    }
}

// Forward Left Diagonal movement
void moveDiagonalForwardLeft(int speed)
{
    if (DEBUG_MODE)
        Serial.println("Moving Diagonal Forward-Left");

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    // Left wheel minimal movement (reduced by 70%)
    moveMotor(motorLeft, BACKWARD, speed * 0.3);
    // Right wheel at full power
    moveMotor(motorRight, FORWARD, speed);

    if (useThreeWheels)
    {
        // Back wheel in between to achieve diagonal motion
        moveMotor(motorBack, FORWARD, speed * 0.6);
    }
    else
    {
        moveMotor(motorBack, STOP, 0); // Back wheel disabled in 2-wheel mode
    }
}

// Forward Right Diagonal movement
void moveDiagonalForwardRight(int speed)
{
    if (DEBUG_MODE)
        Serial.println("Moving Diagonal Forward-Right");

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    moveMotor(motorLeft, BACKWARD, speed);
    moveMotor(motorRight, FORWARD, speed * 0.3);

    if (useThreeWheels)
    {
        // Back wheel in between to achieve diagonal motion
        moveMotor(motorBack, FORWARD, speed * 0.6);
    }
    else
    {
        moveMotor(motorBack, STOP, 0); // Back wheel disabled in 2-wheel mode
    }
}

// Backward Left Diagonal movement
void moveDiagonalBackwardLeft(int speed)
{
    if (DEBUG_MODE)
        Serial.println("Moving Diagonal Backward-Left");

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    moveMotor(motorLeft, FORWARD, speed * 0.3);
    moveMotor(motorRight, BACKWARD, speed);

    if (useThreeWheels)
    {
        // Back wheel in between to achieve diagonal motion
        moveMotor(motorBack, BACKWARD, speed * 0.6);
    }
    else
    {
        moveMotor(motorBack, STOP, 0); // Back wheel disabled in 2-wheel mode
    }
}

// Backward Right Diagonal movement
void moveDiagonalBackwardRight(int speed)
{
    if (DEBUG_MODE)
        Serial.println("Moving Diagonal Backward-Right");

    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    moveMotor(motorLeft, FORWARD, speed);
    moveMotor(motorRight, BACKWARD, speed * 0.3);

    if (useThreeWheels)
    {
        // Back wheel in between to achieve diagonal motion
        moveMotor(motorBack, BACKWARD, speed * 0.6);
    }
    else
    {
        moveMotor(motorBack, STOP, 0); // Back wheel disabled in 2-wheel mode
    }
}
void loop()
{
    // ==== Check if master override is active (via pin or I2C) ====
    checkOverridePin();

    // If master override is active, ONLY handle I2C commands, stop all other functionality
    if (masterOverrideActive)
    {
        // Only report status periodically if in debug mode
        unsigned long now = millis();
        static unsigned long lastOverrideStatus = 0;
        if (DEBUG_MODE && (now - lastOverrideStatus > 5000))
        {
            Serial.println("<LOCALIZATION_MODULE:IDLE_MODE>");
            lastOverrideStatus = now;
        }

        // When in master override mode, ensure all motors are stopped
        stopAllMotors();

        // Skip all sensor readings, serial commands, and other operations
        return;
    } // Periodically send ready status to master if needed
    static unsigned long lastReadyCheck = 0;
    unsigned long currentTime = millis();
    if (moduleReady && (currentTime - lastReadyCheck > 5000)) // Check every 5 seconds
    {
        lastReadyCheck = currentTime;

        // The master should request this through I2C, but we can also proactively send it
        if (DEBUG_MODE)
        {
            Serial.println("<LOCALIZATION_MODULE:READY>");
        }

        // Can optionally send a ready status via I2C here if needed
        // uint8_t response = (MASTER_ADDR << 5) | TELL_READY_CMD;
        // Wire.beginTransmission(MASTER_ADDR);
        // Wire.write(response);
        // Wire.endTransmission();
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
    } // Handle MOV command
    if (strcmp(command, "MOV") == 0)
    {
        // Parse movement parameters: direction,speed
        // Direction codes:
        // 1 = FORWARD
        // 2 = BACKWARD
        // 3 = ARC TURN LEFT
        // 4 = ARC TURN RIGHT
        // 5 = ROTATE LEFT
        // 6 = ROTATE RIGHT
        // 7 = SLIDE LEFT (lateral movement)
        // 8 = SLIDE RIGHT (lateral movement)
        // 9 = DIAGONAL FORWARD-LEFT
        // 10 = DIAGONAL FORWARD-RIGHT
        // 11 = DIAGONAL BACKWARD-LEFT
        // 12 = DIAGONAL BACKWARD-RIGHT
        // 0 = STOP
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
            // Apply rotation speed limit for safety
            speed = constrain(speed, MIN_SPEED, MAX_ROTATION_SPEED);

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

    // Handle TAG command for AprilTag tracking with separate distance and speed
    // Format: TAG:id,distance,direction,speed
    if (strcmp(command, "TAG") == 0)
    {
        int tagId;
        float distance;
        int direction;
        int desiredSpeed;

        if (sscanf(params, "%d,%f,%d,%d", &tagId, &distance, &direction, &desiredSpeed) == 4)
        {
            // Update our last tag update time
            lastTagUpdate = millis();

            if (DEBUG_MODE)
            {
                Serial.print("<TAG:ID=");
                Serial.print(tagId);
                Serial.print(",D=");
                Serial.print(distance);
                Serial.print(",desiredSpeed=");
                Serial.print(desiredSpeed);
                Serial.println(">");
            }

            // Execute the movement using the direction and speed from the command
            // The executeMovement function will apply obstacle avoidance scaling
            // to the desired speed value when needed
            executeMovement(direction, desiredSpeed);

            Serial.println("<ACK:TAG>");
        }
        else
        {
            Serial.println("<ERR:Invalid TAG params>");
        }
        return;
    }

    // Handle single-character movement commands for direct control (matching basic_moveset.ino)
    if (strlen(command) == 1)
    {
        switch (command[0])
        {
        case 'W': // Forward
            executeMovement(1, MAX_SPEED);
            Serial.println("<ACK:FORWARD>");
            return;

        case 'S': // Backward
            executeMovement(2, MAX_SPEED);
            Serial.println("<ACK:BACKWARD>");
            return;

        case 'A': // Arc Turn Left
            executeMovement(3, MAX_SPEED);
            Serial.println("<ACK:ARC_LEFT>");
            return;

        case 'D': // Arc Turn Right
            executeMovement(4, MAX_SPEED);
            Serial.println("<ACK:ARC_RIGHT>");
            return;

        case 'Q': // Rotate Left
            executeMovement(5, MAX_SPEED);
            Serial.println("<ACK:ROTATE_LEFT>");
            return;

        case 'E': // Rotate Right
            executeMovement(6, MAX_SPEED);
            Serial.println("<ACK:ROTATE_RIGHT>");
            return;

        case '4': // Slide Left
            executeMovement(7, MAX_SPEED);
            Serial.println("<ACK:SLIDE_LEFT>");
            return;

        case '6': // Slide Right
            executeMovement(8, MAX_SPEED);
            Serial.println("<ACK:SLIDE_RIGHT>");
            return;

        case '7': // Diagonal Forward-Left
            executeMovement(9, MAX_SPEED);
            Serial.println("<ACK:DIAG_FORWARD_LEFT>");
            return;

        case '9': // Diagonal Forward-Right
            executeMovement(10, MAX_SPEED);
            Serial.println("<ACK:DIAG_FORWARD_RIGHT>");
            return;

        case '1': // Diagonal Backward-Left
            executeMovement(11, MAX_SPEED);
            Serial.println("<ACK:DIAG_BACKWARD_LEFT>");
            return;

        case '3': // Diagonal Backward-RIGHT
            executeMovement(12, MAX_SPEED);
            Serial.println("<ACK:DIAG_BACKWARD_RIGHT>");
            return;

        case '5': // Stop
            executeMovement(0, 0);
            Serial.println("<ACK:STOP>");
            return;
        }
    }

    // If we get here, command wasn't recognized
    Serial.print("<ERR:Unknown command ");
    Serial.print(command);
    Serial.println(">");
}