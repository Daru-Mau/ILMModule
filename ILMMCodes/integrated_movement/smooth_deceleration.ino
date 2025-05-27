// filepath: c:\Users\Estudios\Documents\VC_Projects\12vo_Semestre\R&D\LMModule\ILMMCodes\integrated_movement\smooth_deceleration.ino

// This file contains functions for implementing smooth deceleration

// External declarations for variables from integrated_movement.ino
extern Direction leftDir;
extern Direction rightDir;
extern Direction backDir;

// Global variables for smooth deceleration
bool decelerationActive = false;
unsigned long decelerationTimer = 0;
const int DECELERATION_STEPS = 10;     // Number of steps to slow down
const int DECELERATION_STEP_TIME = 30; // Time between deceleration steps (ms)
float decelerationRate = 0.15;         // Rate of deceleration (0-1)

// Perform smooth deceleration on a motor
// Returns true when deceleration is complete
bool smoothDecelerateMotor(Motor &motor, Direction currentDir)
{
    // If the motor is already at zero speed, nothing to do
    if (motor.currentSpeed <= 0)
    {
        // Reset motor outputs to ensure it's fully stopped
        analogWrite(motor.RPWM, 0);
        analogWrite(motor.LPWM, 0);
        return true;
    }

    // Calculate new speed with smooth deceleration
    float newSpeed = max(0, motor.currentSpeed - (MAX_SPEED * decelerationRate));
    motor.currentSpeed = newSpeed;

    // Round to integer for PWM
    int speedInt = (int)newSpeed;

    // If we've reached a very low speed, stop completely
    if (speedInt <= MIN_SPEED * 0.5)
    {
        analogWrite(motor.RPWM, 0);
        analogWrite(motor.LPWM, 0);
        motor.currentSpeed = 0;
        return true;
    }

    // Otherwise, apply the reduced speed
    switch (currentDir)
    {
    case FORWARD:
        analogWrite(motor.RPWM, speedInt);
        analogWrite(motor.LPWM, 0);
        break;
    case BACKWARD:
        analogWrite(motor.RPWM, 0);
        analogWrite(motor.LPWM, speedInt);
        break;
    case STOP:
        // In case of STOP direction, we need to know which direction the motor was moving
        // This function assumes we already know the direction from the caller
        break;
    }

    return false;
}

// Main function to handle smooth deceleration of all motors
// Call this from the loop function
void handleSmoothDeceleration()
{
    // Skip if deceleration is not active
    if (!decelerationActive)
        return;

    // Only update at specific intervals for smooth deceleration
    unsigned long currentTime = millis();
    if (currentTime - decelerationTimer < DECELERATION_STEP_TIME)
        return;

    // Track if all motors are stopped
    bool allStopped = true;

    // Get current direction of each motor
    static Direction leftDir = STOP;
    static Direction rightDir = STOP;
    static Direction backDir = STOP;

    // Store previous speed to detect direction
    static float prevLeftSpeed = 0;
    static float prevRightSpeed = 0;
    static float prevBackSpeed = 0;

    // Update deceleration timer
    decelerationTimer = currentTime;

    // Apply deceleration to each motor
    allStopped &= smoothDecelerateMotor(motorLeft, leftDir);
    allStopped &= smoothDecelerateMotor(motorRight, rightDir);
    allStopped &= smoothDecelerateMotor(motorBack, backDir);

    // If all motors have stopped, deactivate deceleration mode
    if (allStopped)
    {
        decelerationActive = false;

        // Force zero outputs for safety
        analogWrite(motorLeft.RPWM, 0);
        analogWrite(motorLeft.LPWM, 0);
        analogWrite(motorRight.RPWM, 0);
        analogWrite(motorRight.LPWM, 0);
        analogWrite(motorBack.RPWM, 0);
        analogWrite(motorBack.LPWM, 0);

        // Reset cached direction
        leftDir = STOP;
        rightDir = STOP;
        backDir = STOP;

        if (DEBUG_MODE)
            debugPrint("DECELERATION:COMPLETE");
    }
}

// Initialize deceleration process
// This captures the current states and starts the deceleration
void startSmoothDeceleration()
{
    // Only start if not already active
    if (!decelerationActive)
    {
        decelerationActive = true;
        decelerationTimer = millis();

        // Detect motor directions based on current speeds and PWM output values
        // Access the static direction variables declared in handleSmoothDeceleration
        extern Direction leftDir;
        extern Direction rightDir;
        extern Direction backDir;

        // Left motor direction
        if (motorLeft.currentSpeed > 0)
        {
            // Check which pin is active to determine direction
            if (digitalRead(motorLeft.RPWM) > 0)
                leftDir = FORWARD;
            else
                leftDir = BACKWARD;
        }

        // Right motor direction
        if (motorRight.currentSpeed > 0)
        {
            if (digitalRead(motorRight.RPWM) > 0)
                rightDir = FORWARD;
            else
                rightDir = BACKWARD;
        }

        // Back motor direction
        if (motorBack.currentSpeed > 0)
        {
            if (digitalRead(motorBack.RPWM) > 0)
                backDir = FORWARD;
            else
                backDir = BACKWARD;
        }

        // If in debug mode, log that we started deceleration
        if (DEBUG_MODE)
            debugPrint("DECELERATION:STARTED");
    }
}

// Emergency stop - bypasses smooth deceleration for safety
void emergencyStopAllMotors()
{
    // Immediately set all PWM outputs to zero
    analogWrite(motorLeft.RPWM, 0);
    analogWrite(motorLeft.LPWM, 0);
    motorLeft.currentSpeed = 0;

    analogWrite(motorRight.RPWM, 0);
    analogWrite(motorRight.LPWM, 0);
    motorRight.currentSpeed = 0;

    analogWrite(motorBack.RPWM, 0);
    analogWrite(motorBack.LPWM, 0);
    motorBack.currentSpeed = 0;

    // Reset deceleration state
    decelerationActive = false;

    if (DEBUG_MODE)
        debugPrint("EMERGENCY_STOP:ALL_MOTORS");
}
