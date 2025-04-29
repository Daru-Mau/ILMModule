// === HolonomicDrive Class ===
class HolonomicDrive
{
public:
    HolonomicDrive(int rF, int lF, int enF, int rL, int lL, int enL, int rR, int lR, int enR)
    {
        // Assign motor pins
        RPWM_RIGHT = rF;
        LPWM_RIGHT = lF;
        EN_RIGHT = enF;
        RPWM_LEFT = rL;
        LPWM_LEFT = lL;
        EN_LEFT = enL;
        RPWM_BACK = rR;
        LPWM_BACK = lR;
        EN_BACK = enR;

        // Set motor pins as outputs
        int pins[] = {RPWM_RIGHT, LPWM_RIGHT, EN_RIGHT,
                      RPWM_LEFT, LPWM_LEFT, EN_LEFT,
                      RPWM_BACK, LPWM_BACK, EN_BACK};
        for (int i = 0; i < 9; i++)
        {
            pinMode(pins[i], OUTPUT);
        }

        enableMotors();
    }

    void enableMotors()
    {
        digitalWrite(EN_RIGHT, HIGH);
        digitalWrite(EN_LEFT, HIGH);
        digitalWrite(EN_BACK, HIGH);
    }

    void moveForward(int speed)
    {
        moveMotor(RPWM_RIGHT, LPWM_RIGHT, speed);
        moveMotor(RPWM_LEFT, LPWM_LEFT, speed);
        moveMotor(RPWM_BACK, LPWM_BACK, speed);
    }

    void moveBackward(int speed)
    {
        moveMotorBackward(RPWM_RIGHT, LPWM_RIGHT, speed);
        moveMotorBackward(RPWM_LEFT, LPWM_LEFT, speed);
        moveMotorBackward(RPWM_BACK, LPWM_BACK, speed);
    }

    void slideLeft(int speed)
    {
        moveMotor(RPWM_BACK, LPWM_BACK, speed);
        moveMotorBackward(RPWM_LEFT, LPWM_LEFT, speed);
        stopMotor(RPWM_RIGHT, LPWM_RIGHT);
    }

    void slideRight(int speed)
    {
        moveMotorBackward(RPWM_BACK, LPWM_BACK, speed);
        moveMotor(RPWM_LEFT, LPWM_LEFT, speed);
        stopMotor(RPWM_RIGHT, LPWM_RIGHT);
    }

    void rotateLeft(int speed)
    {
        moveMotorBackward(RPWM_RIGHT, LPWM_RIGHT, speed);
        moveMotor(RPWM_LEFT, LPWM_LEFT, speed);
        moveMotorBackward(RPWM_BACK, LPWM_BACK, speed);
    }

    void rotateRight(int speed)
    {
        moveMotor(RPWM_RIGHT, LPWM_RIGHT, speed);
        moveMotorBackward(RPWM_LEFT, LPWM_LEFT, speed);
        moveMotor(RPWM_BACK, LPWM_BACK, speed);
    }

    void stopAll()
    {
        stopMotor(RPWM_RIGHT, LPWM_RIGHT);
        stopMotor(RPWM_LEFT, LPWM_LEFT);
        stopMotor(RPWM_BACK, LPWM_BACK);
    }

private:
    int RPWM_RIGHT, LPWM_RIGHT, EN_RIGHT;
    int RPWM_LEFT, LPWM_LEFT, EN_LEFT;
    int RPWM_BACK, LPWM_BACK, EN_BACK;

    void moveMotor(int rpwm, int lpwm, int speed)
    {
        analogWrite(rpwm, speed);
        analogWrite(lpwm, 0);
    }

    void moveMotorBackward(int rpwm, int lpwm, int speed)
    {
        analogWrite(rpwm, 0);
        analogWrite(lpwm, speed);
    }

    void stopMotor(int rpwm, int lpwm)
    {
        analogWrite(rpwm, 0);
        analogWrite(lpwm, 0);
    }
};

// === Pin Definitions ===
// Ultrasonic Sensors
const int TP_FRONT = 22, EP_FRONT = 23;
const int TP_FRONT_LEFT = 24, EP_FRONT_LEFT = 25;
const int TP_FRONT_RIGHT = 26, EP_FRONT_RIGHT = 27;
const int TP_LEFT = 28, EP_LEFT = 29;
const int TP_RIGHT = 30, EP_RIGHT = 31;
const int TP_BACK = 32, EP_BACK = 33;

// Motor Driver Pins
const int RPWM_RIGHT = 37, LPWM_RIGHT = 36, REN_RIGHT = 39, LEN_RIGHT = 38;
const int RPWM_LEFT = 43, LPWM_LEFT = 42, REN_LEFT = 45, LEN_LEFT = 44;
const int RPWM_BACK = 49, LPWM_BACK = 48, REN_BACK = 51, LEN_BACK = 50;

// Encoder Pins
const int ENCODER_FRONT_A = 33;
const int ENCODER_FRONT_B = 32;

// === Globals ===
float distFront, distFrontLeft, distFrontRight, distLeft, distRight, distBack;
const float OBSTACLE_DISTANCE = 10.0; // cm

//== AprilTag Input ==
int tagID = -1;
float tagX = 0.0;
float tagZ = 0.0;
bool tagDetected = false;

// === Setup ===
void setup()
{
    Serial.begin(9600);

    int trigPins[] = {TP_FRONT, TP_FRONT_LEFT, TP_FRONT_RIGHT, TP_LEFT, TP_RIGHT, TP_BACK};
    int echoPins[] = {EP_FRONT, EP_FRONT_LEFT, EP_FRONT_RIGHT, EP_LEFT, EP_RIGHT, EP_BACK};
    for (int i = 0; i < 6; i++)
    {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }

    int motorPins[] = {
        RPWM_RIGHT, LPWM_RIGHT, REN_RIGHT, LEN_RIGHT,
        RPWM_LEFT, LPWM_LEFT, REN_LEFT, LEN_LEFT,
        RPWM_BACK, LPWM_BACK, REN_BACK, LEN_BACK};
    for (int i = 0; i < 12; i++)
        pinMode(motorPins[i], OUTPUT);

    digitalWrite(REN_RIGHT, HIGH);
    digitalWrite(LEN_RIGHT, HIGH);
    digitalWrite(REN_LEFT, HIGH);
    digitalWrite(LEN_LEFT, HIGH);
    digitalWrite(REN_BACK, HIGH);
    digitalWrite(LEN_BACK, HIGH);

    pinMode(ENCODER_FRONT_A, INPUT);
    pinMode(ENCODER_FRONT_B, INPUT);

    Serial.println("ROBOT READY");
}

// === Distance Sensing ===
float readDistance(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000);
    return (duration <= 0) ? 999.0 : duration * 0.034 / 2;
}

void updateDistances()
{
    distFront = readDistance(TP_FRONT, EP_FRONT);
    distFrontLeft = readDistance(TP_FRONT_LEFT, EP_FRONT_LEFT);
    distFrontRight = readDistance(TP_FRONT_RIGHT, EP_FRONT_RIGHT);
    distLeft = readDistance(TP_LEFT, EP_LEFT);
    distRight = readDistance(TP_RIGHT, EP_RIGHT);
    distBack = readDistance(TP_BACK, EP_BACK);
}

// === Motor Logic ===
void setMotor(int rpwm, int lpwm, float motorSpeed)
{
    motorSpeed = constrain(motorSpeed, -255, 255);
    if (motorSpeed > 0)
    {
        analogWrite(rpwm, motorSpeed);
        analogWrite(lpwm, 0);
    }
    else
    {
        analogWrite(rpwm, 0);
        analogWrite(lpwm, -motorSpeed);
    }
}

// === Triskar Movement Logic ===
void moveRobot(float vx, float vy, float omega, int speed = 180)
{
    // Triskar drive geometry adjustment
    // vx: Forward/backward velocity
    // vy: Left/right velocity
    // omega: Rotation velocity

    // Triskar kinematic equations
    float v_front = -vx + omega;                   // Front motor adjusts for both vx and omega
    float v_left = 0.5 * vx - 0.866 * vy + omega;  // Left motor adjusted for movement and rotation
    float v_right = 0.5 * vx + 0.866 * vy + omega; // Right motor adjusted for movement and rotation

    // Normalize velocities so they stay within -1 to 1 range
    float maxVal = max(max(abs(v_front), abs(v_left)), abs(v_right));
    if (maxVal > 1.0)
    {
        v_front /= maxVal;
        v_left /= maxVal;
        v_right /= maxVal;
    }

    // Apply the normalized velocities to motors
    setMotor(RPWM_RIGHT, LPWM_RIGHT, v_front * speed);
    setMotor(RPWM_LEFT, LPWM_LEFT, v_left * speed);
    setMotor(RPWM_BACK, LPWM_BACK, v_right * speed);
}

void stopAllMotors()
{
    analogWrite(RPWM_RIGHT, 0);
    analogWrite(LPWM_RIGHT, 0);
    analogWrite(RPWM_LEFT, 0);
    analogWrite(LPWM_LEFT, 0);
    analogWrite(RPWM_BACK, 0);
    analogWrite(LPWM_BACK, 0);
}

void checkSerialForTag()
{
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        input.trim();
        int sep1 = input.indexOf(' ');
        int sep2 = input.lastIndexOf(' ');

        if (sep1 > 0 && sep2 > sep1)
        {
            tagID = input.substring(0, sep1).toInt();
            tagX = input.substring(sep1 + 1, sep2).toFloat();
            tagZ = input.substring(sep2 + 1).toFloat();
            tagDetected = true;

            Serial.print("Received Tag - ID: ");
            Serial.print(tagID);
            Serial.print(" X: ");
            Serial.print(tagX);
            Serial.print(" Z: ");
            Serial.println(tagZ);
        }
    }
}

// === Main Loop ===
void loop()
{
    updateDistances();
    checkSerialForTag();

    Serial.print("F: ");
    Serial.print(distFront);
    Serial.print(" FL: ");
    Serial.print(distFrontLeft);
    Serial.print(" FR: ");
    Serial.print(distFrontRight);
    Serial.print(" L: ");
    Serial.print(distLeft);
    Serial.print(" R: ");
    Serial.print(distRight);
    Serial.print(" B: ");
    Serial.println(distBack);

    if (tagDetected)
    {
        float desiredDistance = 0.3; // Stop 30 cm away from tag
        float linearSpeed = 180;
        float angularCorrection = tagX * 2.0; // Proportional rotation

        // If the tag is far enough, move toward it
        if (tagZ > desiredDistance + 0.05)
        {
            Serial.println("Following AprilTag...");

            // vx = forward, vy = strafe, omega = rotation
            moveRobot(-0.8, 0.0, angularCorrection);
        }
        else
        {
            Serial.print("Tag ");
            Serial.print(tagID);
            Serial.println(" reached. Performing action...");

            if (tagID == 1)
            {
                Serial.println("Reached charging station.");
                // e.g., stop, play sound, start charging logic
            }
            else if (tagID == 2)
            {
                Serial.println("Reached final destination.");
                // e.g., stop, signal completion
            }

            stopAllMotors();
            tagDetected = false;
        }

        delay(200);
        return; // Skip obstacle logic if tag is detected
    }

    // Priority: Front obstacle (hard stop and rotate to find open space)
    if (distFront < OBSTACLE_DISTANCE || distFrontLeft < OBSTACLE_DISTANCE || distFrontRight < OBSTACLE_DISTANCE)
    {
        Serial.println("Obstacle in front - rotating to avoid");

        if (distLeft > distRight)
        {
            // More space on the left, rotate counter-clockwise
            moveRobot(0.0, 0.0, -1.0); // negative = CCW
        }
        else
        {
            // More space on the right, rotate clockwise
            moveRobot(0.0, 0.0, 1.0); // positive = CW
        }

        delay(500);
        stopAllMotors();
    }

    // Side obstacle - strafe away from the obstacle
    else if (distLeft < OBSTACLE_DISTANCE)
    {
        Serial.println("Obstacle on left - sliding right");
        moveRobot(1.0, 0.0, 0.0); // x=1.0 = rightward movement
        delay(400);
        stopAllMotors();
    }
    else if (distRight < OBSTACLE_DISTANCE)
    {
        Serial.println("Obstacle on right - sliding left");
        moveRobot(-1.0, 0.0, 0.0); // x=-1.0 = leftward movement
        delay(400);
        stopAllMotors();
    }

    // No obstacles - move forward
    else
    {
        Serial.println("Path is clear - moving forward");
        moveRobot(-1.0, 1.0, 0.0); // Forward movement (as per your logic)
    }

    delay(200);
}
