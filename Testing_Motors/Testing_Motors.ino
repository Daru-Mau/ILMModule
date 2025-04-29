// Motor Driver Pin Definitions
#define FRONT_RPWM 37
#define FRONT_LPWM 36
#define FRONT_REN 39
#define FRONT_LEN 38

#define LEFT_RPWM 43
#define LEFT_LPWM 42
#define LEFT_REN 45
#define LEFT_LEN 44

#define RIGHT_RPWM 49
#define RIGHT_LPWM 48
#define RIGHT_REN 51
#define RIGHT_LEN 50

// Test Configuration
#define TEST_SPEED_LOW 100  // Low speed for testing
#define TEST_SPEED_HIGH 200 // High speed for testing
#define MOVEMENT_TIME 1500  // Time for each movement (ms)
#define PAUSE_TIME 1000     // Pause between movements (ms)

// Motor Control Structure
struct MotorPins
{
    const int rpwm;
    const int lpwm;
    const int ren;
    const int len;
    const char *name;
};

// Define motors
const MotorPins frontMotor = {FRONT_RPWM, FRONT_LPWM, FRONT_REN, FRONT_LEN, "Front"};
const MotorPins leftMotor = {LEFT_RPWM, LEFT_LPWM, LEFT_REN, LEFT_LEN, "Left"};
const MotorPins rightMotor = {RIGHT_RPWM, RIGHT_LPWM, RIGHT_REN, RIGHT_LEN, "Right"};

void setupMotorPins(const MotorPins &motor)
{
    pinMode(motor.rpwm, OUTPUT);
    pinMode(motor.lpwm, OUTPUT);
    pinMode(motor.ren, OUTPUT);
    pinMode(motor.len, OUTPUT);
    digitalWrite(motor.ren, HIGH);
    digitalWrite(motor.len, HIGH);
}

void setMotorSpeed(const MotorPins &motor, int speed)
{
    speed = constrain(speed, -255, 255);
    if (speed > 0)
    {
        analogWrite(motor.rpwm, speed);
        analogWrite(motor.lpwm, 0);
    }
    else
    {
        analogWrite(motor.rpwm, 0);
        analogWrite(motor.lpwm, -speed);
    }
}

void stopMotor(const MotorPins &motor)
{
    analogWrite(motor.rpwm, 0);
    analogWrite(motor.lpwm, 0);
}

void setup()
{
    Serial.begin(115200);

    // Setup all motors
    setupMotorPins(frontMotor);
    setupMotorPins(leftMotor);
    setupMotorPins(rightMotor);

    Serial.println(F("=== Motor Testing System ==="));
    Serial.println(F("Type commands in Serial Monitor:"));
    Serial.println(F("1: Test individual motors"));
    Serial.println(F("2: Test basic movements"));
    Serial.println(F("3: Test complex movements"));
    Serial.println(F("4: Speed calibration test"));
}

void testIndividualMotors()
{
    Serial.println(F("\n=== Testing Individual Motors ==="));

    // Test front motor
    Serial.println(F("Testing Front Motor - Forward"));
    setMotorSpeed(frontMotor, TEST_SPEED_LOW);
    delay(MOVEMENT_TIME);
    Serial.println(F("Testing Front Motor - Backward"));
    setMotorSpeed(frontMotor, -TEST_SPEED_LOW);
    delay(MOVEMENT_TIME);
    stopMotor(frontMotor);
    delay(PAUSE_TIME);

    // Test left motor
    Serial.println(F("Testing Left Motor - Forward"));
    setMotorSpeed(leftMotor, TEST_SPEED_LOW);
    delay(MOVEMENT_TIME);
    Serial.println(F("Testing Left Motor - Backward"));
    setMotorSpeed(leftMotor, -TEST_SPEED_LOW);
    delay(MOVEMENT_TIME);
    stopMotor(leftMotor);
    delay(PAUSE_TIME);

    // Test right motor
    Serial.println(F("Testing Right Motor - Forward"));
    setMotorSpeed(rightMotor, TEST_SPEED_LOW);
    delay(MOVEMENT_TIME);
    Serial.println(F("Testing Right Motor - Backward"));
    setMotorSpeed(rightMotor, -TEST_SPEED_LOW);
    delay(MOVEMENT_TIME);
    stopMotor(rightMotor);
}

void testBasicMovements()
{
    Serial.println(F("\n=== Testing Basic Movements ==="));

    // Forward
    Serial.println(F("Moving Forward"));
    setMotorSpeed(frontMotor, TEST_SPEED_LOW);
    setMotorSpeed(leftMotor, TEST_SPEED_LOW);
    setMotorSpeed(rightMotor, TEST_SPEED_LOW);
    delay(MOVEMENT_TIME);

    // Stop
    stopMotor(frontMotor);
    stopMotor(leftMotor);
    stopMotor(rightMotor);
    delay(PAUSE_TIME);

    // Backward
    Serial.println(F("Moving Backward"));
    setMotorSpeed(frontMotor, -TEST_SPEED_LOW);
    setMotorSpeed(leftMotor, -TEST_SPEED_LOW);
    setMotorSpeed(rightMotor, -TEST_SPEED_LOW);
    delay(MOVEMENT_TIME);

    // Stop
    stopMotor(frontMotor);
    stopMotor(leftMotor);
    stopMotor(rightMotor);
    delay(PAUSE_TIME);
}

void testComplexMovements()
{
    Serial.println(F("\n=== Testing Complex Movements ==="));

    // Slide left
    Serial.println(F("Sliding Left"));
    setMotorSpeed(frontMotor, 0);
    setMotorSpeed(leftMotor, -TEST_SPEED_LOW);
    setMotorSpeed(rightMotor, TEST_SPEED_LOW);
    delay(MOVEMENT_TIME);

    // Stop and pause
    stopMotor(frontMotor);
    stopMotor(leftMotor);
    stopMotor(rightMotor);
    delay(PAUSE_TIME);

    // Slide right
    Serial.println(F("Sliding Right"));
    setMotorSpeed(frontMotor, 0);
    setMotorSpeed(leftMotor, TEST_SPEED_LOW);
    setMotorSpeed(rightMotor, -TEST_SPEED_LOW);
    delay(MOVEMENT_TIME);

    // Stop and pause
    stopMotor(frontMotor);
    stopMotor(leftMotor);
    stopMotor(rightMotor);
    delay(PAUSE_TIME);

    // Rotate clockwise
    Serial.println(F("Rotating Clockwise"));
    setMotorSpeed(frontMotor, TEST_SPEED_LOW);
    setMotorSpeed(leftMotor, TEST_SPEED_LOW);
    setMotorSpeed(rightMotor, -TEST_SPEED_LOW);
    delay(MOVEMENT_TIME);

    // Stop and pause
    stopMotor(frontMotor);
    stopMotor(leftMotor);
    stopMotor(rightMotor);
    delay(PAUSE_TIME);

    // Rotate counter-clockwise
    Serial.println(F("Rotating Counter-clockwise"));
    setMotorSpeed(frontMotor, -TEST_SPEED_LOW);
    setMotorSpeed(leftMotor, -TEST_SPEED_LOW);
    setMotorSpeed(rightMotor, TEST_SPEED_LOW);
    delay(MOVEMENT_TIME);

    // Final stop
    stopMotor(frontMotor);
    stopMotor(leftMotor);
    stopMotor(rightMotor);
}

void speedCalibrationTest()
{
    Serial.println(F("\n=== Speed Calibration Test ==="));

    // Test different speeds
    const int speeds[] = {50, 100, 150, 200, 250};
    const int numSpeeds = sizeof(speeds) / sizeof(speeds[0]);

    for (int i = 0; i < numSpeeds; i++)
    {
        int speed = speeds[i];
        Serial.print(F("Testing speed: "));
        Serial.println(speed);

        // Move all motors forward at current speed
        setMotorSpeed(frontMotor, speed);
        setMotorSpeed(leftMotor, speed);
        setMotorSpeed(rightMotor, speed);
        delay(MOVEMENT_TIME);

        // Stop and pause
        stopMotor(frontMotor);
        stopMotor(leftMotor);
        stopMotor(rightMotor);
        delay(PAUSE_TIME);
    }
}

void loop()
{
    if (Serial.available() > 0)
    {
        char command = Serial.read();

        switch (command)
        {
        case '1':
            testIndividualMotors();
            break;
        case '2':
            testBasicMovements();
            break;
        case '3':
            testComplexMovements();
            break;
        case '4':
            speedCalibrationTest();
            break;
        default:
            if (command != '\n' && command != '\r')
            {
                Serial.println(F("Invalid command"));
            }
            break;
        }
    }
}
