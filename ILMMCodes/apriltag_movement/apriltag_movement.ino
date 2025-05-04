#include <Arduino.h>
#include <Wire.h>

// Communication settings
#define SERIAL_BAUD_RATE 115200

// Movement thresholds
#define POSITION_TOLERANCE 5.0   // mm
#define ROTATION_TOLERANCE 0.05  // radians
#define MIN_MOVE_SPEED 50        // PWM
#define MAX_MOVE_SPEED 255       // PWM
#define MIN_ROTATION_SPEED 40    // PWM
#define SLOW_DOWN_DISTANCE 100.0 // mm

// Motor Driver Pin Definitions
struct Motor
{
    int RPWM;
    int LPWM;
    int REN;
    int LEN;
    float currentSpeed;
};

enum Direction
{
    FORWARD,
    BACKWARD,
    STOP
};

// Motor configurations
Motor motorRight = {3, 2, 39, 38, 0};
Motor motorLeft = {4, 5, 44, 45, 0};
Motor motorBack = {7, 6, 51, 50, 0};

void setupMotorPins(Motor &motor)
{
    pinMode(motor.RPWM, OUTPUT);
    pinMode(motor.LPWM, OUTPUT);
    pinMode(motor.REN, OUTPUT);
    pinMode(motor.LEN, OUTPUT);
    digitalWrite(motor.REN, HIGH);
    digitalWrite(motor.LEN, HIGH);
}

void moveMotor(Motor &motor, Direction dir, float targetSpeed)
{
    targetSpeed = constrain(targetSpeed, 0, MAX_MOVE_SPEED);

    // Smooth acceleration/deceleration
    if (targetSpeed > motor.currentSpeed)
    {
        motor.currentSpeed = min(targetSpeed, motor.currentSpeed + MAX_MOVE_SPEED * 0.15);
    }
    else if (targetSpeed < motor.currentSpeed)
    {
        motor.currentSpeed = max(targetSpeed, motor.currentSpeed - MAX_MOVE_SPEED * 0.15);
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
    // Scale inputs to motor speeds
    float speed = baseSpeed * max(max(abs(vy), abs(vx)), abs(omega));

    if (abs(vy) > 0.1)
    {
        if (vy > 0)
        {
            moveMotor(motorLeft, FORWARD, speed);
            moveMotor(motorRight, FORWARD, speed);
            moveMotor(motorBack, STOP, 0);
        }
        else
        {
            moveMotor(motorLeft, BACKWARD, speed);
            moveMotor(motorRight, BACKWARD, speed);
            moveMotor(motorBack, STOP, 0);
        }
    }
    else if (abs(vx) > 0.1)
    {
        if (vx > 0)
        {
            moveMotor(motorLeft, FORWARD, speed);
            moveMotor(motorRight, STOP, 0);
            moveMotor(motorBack, FORWARD, speed);
        }
        else
        {
            moveMotor(motorLeft, STOP, 0);
            moveMotor(motorRight, FORWARD, speed);
            moveMotor(motorBack, BACKWARD, speed);
        }
    }
    else if (abs(omega) > 0.1)
    {
        if (omega > 0)
        {
            moveMotor(motorLeft, FORWARD, speed);
            moveMotor(motorRight, BACKWARD, speed);
            moveMotor(motorBack, BACKWARD, speed);
        }
        else
        {
            moveMotor(motorLeft, BACKWARD, speed);
            moveMotor(motorRight, FORWARD, speed);
            moveMotor(motorBack, FORWARD, speed);
        }
    }
    else
    {
        moveMotor(motorLeft, STOP, 0);
        moveMotor(motorRight, STOP, 0);
        moveMotor(motorBack, STOP, 0);
    }
}

// AprilTag tracking states
enum TrackingState
{
    IDLE,
    MOVING_TO_TAG,
    ALIGNING_WITH_TAG,
    MAINTAINING_POSITION
};

class AprilTagController
{
private:
    TrackingState currentState;
    float targetX;
    float targetY;
    float targetYaw;
    float currentX;
    float currentY;
    float currentYaw;
    bool tagVisible;

public:
    AprilTagController()
    {
        currentState = IDLE;
        tagVisible = false;
        targetX = targetY = targetYaw = 0.0;
        currentX = currentY = currentYaw = 0.0;
    }

    void updateTagData(float x, float y, float yaw)
    {
        tagVisible = true;
        targetX = x;
        targetY = y;
        targetYaw = yaw;
    }

    void updateCurrentPosition(float x, float y, float yaw)
    {
        currentX = x;
        currentY = y;
        currentYaw = yaw;
    }

    void clearTagData()
    {
        tagVisible = false;
    }

    void processMovement()
    {
        if (!tagVisible)
        {
            // If tag is lost, stop movement
            moveRobot(0, 0, 0);
            currentState = IDLE;
            return;
        }

        float dx = targetX - currentX;
        float dy = targetY - currentY;
        float distance = sqrt(dx * dx + dy * dy);
        float angleToTarget = atan2(dy, dx);
        float rotationError = angleToTarget - currentYaw;

        // Normalize rotation error to [-PI, PI]
        while (rotationError > PI)
            rotationError -= 2 * PI;
        while (rotationError < -PI)
            rotationError += 2 * PI;

        // State machine for movement control
        switch (currentState)
        {
        case IDLE:
            if (tagVisible)
            {
                currentState = MOVING_TO_TAG;
            }
            break;

        case MOVING_TO_TAG:
            if (distance < POSITION_TOLERANCE)
            {
                currentState = ALIGNING_WITH_TAG;
                break;
            }

            // Calculate movement speeds
            float moveSpeed = calculateMoveSpeed(distance);
            float rotSpeed = calculateRotationSpeed(rotationError);

            if (abs(rotationError) > ROTATION_TOLERANCE * 2)
            {
                // First align with target
                moveRobot(0, 0, rotSpeed);
            }
            else
            {
                // Move towards target
                float vx = moveSpeed * cos(angleToTarget);
                float vy = moveSpeed * sin(angleToTarget);
                moveRobot(vy, vx, rotSpeed * 0.2); // Small rotation correction while moving
            }
            break;

        case ALIGNING_WITH_TAG:
            if (abs(rotationError) < ROTATION_TOLERANCE)
            {
                currentState = MAINTAINING_POSITION;
                break;
            }
            moveRobot(0, 0, calculateRotationSpeed(rotationError));
            break;

        case MAINTAINING_POSITION:
            // Small corrections to maintain position
            if (distance > POSITION_TOLERANCE || abs(rotationError) > ROTATION_TOLERANCE)
            {
                currentState = MOVING_TO_TAG;
            }
            break;
        }
    }

private:
    float calculateMoveSpeed(float distance)
    {
        if (distance < POSITION_TOLERANCE)
        {
            return 0;
        }
        float speed = map(constrain(distance, 0, SLOW_DOWN_DISTANCE),
                          0, SLOW_DOWN_DISTANCE,
                          MIN_MOVE_SPEED, MAX_MOVE_SPEED);
        return speed / MAX_MOVE_SPEED; // Normalize to 0-1 range
    }

    float calculateRotationSpeed(float error)
    {
        float absError = abs(error);
        if (absError < ROTATION_TOLERANCE)
        {
            return 0;
        }
        float speed = map(constrain(absError * RAD_TO_DEG, 0, 90),
                          0, 90,
                          MIN_ROTATION_SPEED, MAX_MOVE_SPEED);
        return (error > 0 ? 1 : -1) * speed / MAX_MOVE_SPEED; // Normalize and maintain sign
    }
};

// Global controller instance
AprilTagController tagController;

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    // Initialize motors
    setupMotorPins(motorRight);
    setupMotorPins(motorLeft);
    setupMotorPins(motorBack);

    Serial.println("AprilTag Movement Controller Initialized");
}

void loop()
{
    if (Serial.available())
    {
        String data = Serial.readStringUntil('\n');
        if (data.startsWith("TAG:"))
        {
            parseTagData(data.substring(4));
        }
        else if (data.startsWith("POS:"))
        {
            parsePositionData(data.substring(4));
        }
        else if (data == "CLEAR")
        {
            tagController.clearTagData();
        }
    }

    // Update movement every cycle
    tagController.processMovement();
}

void parseTagData(String data)
{
    // Format: "TAG:id,x,y,yaw"
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);
    int thirdComma = data.indexOf(',', secondComma + 1);

    if (firstComma > 0 && secondComma > firstComma && thirdComma > secondComma)
    {
        float x = data.substring(firstComma + 1, secondComma).toFloat();
        float y = data.substring(secondComma + 1, thirdComma).toFloat();
        float yaw = data.substring(thirdComma + 1).toFloat();
        tagController.updateTagData(x, y, yaw);
    }
}

void parsePositionData(String data)
{
    // Format: "POS:x,y,yaw"
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > firstComma)
    {
        float x = data.substring(0, firstComma).toFloat();
        float y = data.substring(firstComma + 1, secondComma).toFloat();
        float yaw = data.substring(secondComma + 1).toFloat();
        tagController.updateCurrentPosition(x, y, yaw);
    }
}
