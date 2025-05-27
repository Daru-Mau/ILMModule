// Function to navigate around obstacles
// This function is called when the robot needs to move forward but has detected an obstacle
// It will check which direction (left or right) has more space, rotate in that direction,
// move forward a small distance, then rotate back to continue on the original path
//
// Note: All state-related variables and the AvoidanceState enum are defined in the main file

// Forward declarations of variables and functions used from integrated_movement.ino
// This ensures the compiler knows these exist even though they're defined in another file

// Enum declaration - using forward reference since it's already defined in integrated_movement.ino
enum AvoidanceState;
extern AvoidanceState avoidanceState;
// The enum values aren't variables, so we don't need to declare them extern
// We'll use the enum values directly

// External variables from integrated_movement.ino
extern float distFL, distF, distFR, distBL, distB, distBR;
extern bool leftEmergencyStop, rightEmergencyStop;
extern bool DEBUG_MODE;
extern int MIN_SPEED;
extern const float SAFE_DISTANCE;
extern const float MIN_AVOIDANCE_DISTANCE;
extern const float OPTIMAL_AVOIDANCE_DISTANCE;
extern int originalSpeed;
extern bool avoidanceLeftDirection;
extern float avoidanceRotationAngle;
extern int avoidanceAttempts;
extern bool avoidanceSuccessful;
extern unsigned long avoidanceTimer;
extern const unsigned long ROTATION_TIMEOUT;
extern const unsigned long MOVEMENT_TIMEOUT;
extern const float ROTATION_ANGLE_SMALL;
extern const float ROTATION_ANGLE_LARGE;
extern const int MAX_AVOIDANCE_ATTEMPTS;

// External functions from integrated_movement.ino
extern void debugPrint(const char *message);
extern void stopAllMotors();
extern void rotateLeft(int speed);
extern void rotateRight(int speed);
extern void moveForward(int speed);

// Standard Arduino functions and libraries needed
// These are automatically included in Arduino but need to be declared for the compiler when files are split
#include <Arduino.h>

// These are built-in Arduino functions so we don't need to redeclare them
// Remove the min() and constrain() declarations as they're already defined in Arduino's core
extern int sprintf(char *str, const char *format, ...);

// Local variable for storing the starting angle (not used in current implementation)
float avoidanceStartingAngle = 0.0; // Starting angle before avoidance (for future enhancement)

// This function starts the obstacle avoidance procedure
// Returns: true if avoidance started, false if already in progress or not needed
bool startObstacleAvoidance(int speed)
{
    if (avoidanceState != IDLE)
    {
        return false; // Already in the middle of avoiding an obstacle
    }

    originalSpeed = speed;
    avoidanceAttempts = 0;
    avoidanceSuccessful = false;

    // Calculate average distances on each side
    float leftDistAvg = (distFL + distBL) / 2.0;
    float rightDistAvg = (distFR + distBR) / 2.0;

    // Check which side has more space - use weighted average that prioritizes forward sensors
    float leftDist = (distFL * 0.7) + (distBL * 0.3);
    float rightDist = (distFR * 0.7) + (distBR * 0.3);

    // If both sides are blocked, cannot avoid
    if (leftDist < MIN_AVOIDANCE_DISTANCE && rightDist < MIN_AVOIDANCE_DISTANCE)
    {
        if (DEBUG_MODE)
            debugPrint("AVOIDANCE_FAILED:BOTH_SIDES_BLOCKED");
        return false;
    }

    // Decide which way to turn - use larger threshold for tight spaces
    avoidanceLeftDirection = (leftDist > rightDist);

    if (DEBUG_MODE)
    {
        char buffer[100];
        sprintf(buffer, "STARTING_AVOIDANCE:%s (Left=%.1f, Right=%.1f)",
                avoidanceLeftDirection ? "LEFT" : "RIGHT", leftDist, rightDist);
        debugPrint(buffer);
    }

    // Determine rotation angle based on how tight the space is
    float minDistance = min(leftDist, rightDist);
    bool isTightSpace = (minDistance < OPTIMAL_AVOIDANCE_DISTANCE);

    // Use larger rotation angle for tighter spaces
    avoidanceRotationAngle = isTightSpace ? ROTATION_ANGLE_LARGE : ROTATION_ANGLE_SMALL;

    avoidanceState = ROTATING_AWAY;
    avoidanceTimer = millis();
    return true;
}

// This function continues the obstacle avoidance procedure
// It should be called repeatedly until it returns true (avoidance complete)
// or false (avoidance failed)
// Returns: true if avoidance is complete, false if still in progress or failed
bool continueObstacleAvoidance()
{
    unsigned long currentTime = millis();

    // Safety timeout check
    if (currentTime - avoidanceTimer > 10000)
    { // 10-second total timeout
        if (DEBUG_MODE)
            debugPrint("AVOIDANCE_FAILED:TIMEOUT");
        avoidanceState = IDLE;
        stopAllMotors();
        return false;
    }

    // Check for any critical emergency stops that would prevent movement
    if ((avoidanceLeftDirection && leftEmergencyStop) ||
        (!avoidanceLeftDirection && rightEmergencyStop))
    {
        if (DEBUG_MODE)
            debugPrint("AVOIDANCE_FAILED:EMERGENCY_STOP");
        avoidanceState = IDLE;
        stopAllMotors();

        // Try the other direction if this is our first attempt
        if (avoidanceAttempts < MAX_AVOIDANCE_ATTEMPTS)
        {
            avoidanceAttempts++;
            avoidanceLeftDirection = !avoidanceLeftDirection; // Try opposite direction

            if (DEBUG_MODE)
            {
                debugPrint("AVOIDANCE:TRYING_OPPOSITE_DIRECTION");
            }

            avoidanceState = ROTATING_AWAY;
            avoidanceTimer = currentTime;
            return false; // Still in progress
        }

        return false;
    }

    // Calculate dynamic speed based on proximity to obstacles
    float nearestObstacle;
    if (avoidanceLeftDirection)
    {
        nearestObstacle = min(distFL, distBL);
    }
    else
    {
        nearestObstacle = min(distFR, distBR);
    }

    // Dynamic speed calculation: slower when closer to obstacles
    float speedFactor = constrain(nearestObstacle / OPTIMAL_AVOIDANCE_DISTANCE, 0.5, 1.0);
    int avoidanceSpeed = constrain((int)(originalSpeed * speedFactor), MIN_SPEED, originalSpeed);

    // Execute the current state of the avoidance procedure
    switch (avoidanceState)
    {
    case ROTATING_AWAY:
        // Check if path ahead is clear before rotating more
        if (distF > SAFE_DISTANCE && ((avoidanceLeftDirection && distFL > SAFE_DISTANCE) ||
                                      (!avoidanceLeftDirection && distFR > SAFE_DISTANCE)))
        {
            // Path is clear enough, proceed to moving past
            avoidanceState = MOVING_PAST;
            avoidanceTimer = currentTime;
            if (DEBUG_MODE)
                debugPrint("AVOIDANCE:PATH_CLEAR_PROCEEDING");
            break;
        }

        // Rotate in the chosen direction with dynamic speed
        if (avoidanceLeftDirection)
        {
            rotateLeft(avoidanceSpeed);
        }
        else
        {
            rotateRight(avoidanceSpeed);
        }

        // Check if we've rotated far enough
        unsigned long rotationTime = (avoidanceRotationAngle / 90.0) * ROTATION_TIMEOUT;
        if (currentTime - avoidanceTimer > rotationTime)
        {
            avoidanceState = MOVING_PAST;
            avoidanceTimer = currentTime;
            if (DEBUG_MODE)
            {
                char buffer[80];
                sprintf(buffer, "AVOIDANCE:ROTATION_COMPLETE (%.1f°)", avoidanceRotationAngle);
                debugPrint(buffer);
            }
        }
        break;

    case MOVING_PAST:
        // Check if we need to adjust course due to new obstacles
        if ((avoidanceLeftDirection && distFL < MIN_AVOIDANCE_DISTANCE) ||
            (!avoidanceLeftDirection && distFR < MIN_AVOIDANCE_DISTANCE))
        {
            // Need to rotate more to avoid the obstacle
            avoidanceState = ROTATING_AWAY;
            avoidanceTimer = currentTime;
            if (DEBUG_MODE)
                debugPrint("AVOIDANCE:ADJUSTING_ROTATION");
            break;
        }

        // Move forward to pass the obstacle with dynamic speed
        moveForward(avoidanceSpeed);

        // Advance for a set time or until we've moved far enough
        // Use dynamic timing based on obstacle size
        float passingTime = (nearestObstacle < OPTIMAL_AVOIDANCE_DISTANCE) ? MOVEMENT_TIMEOUT * 1.5 : MOVEMENT_TIMEOUT;

        if (currentTime - avoidanceTimer > passingTime)
        {
            avoidanceState = ROTATING_BACK;
            avoidanceTimer = currentTime;
            if (DEBUG_MODE)
                debugPrint("AVOIDANCE:PASSING_COMPLETE");
        }
        break;

    case ROTATING_BACK:
        // Rotate back to the original direction (opposite of first rotation)
        if (avoidanceLeftDirection)
        {
            rotateRight(avoidanceSpeed);
        }
        else
        {
            rotateLeft(avoidanceSpeed);
        }

        // Check if we've rotated back enough or if front path is clear
        if ((currentTime - avoidanceTimer > (avoidanceRotationAngle / 90.0) * ROTATION_TIMEOUT) ||
            (distF > SAFE_DISTANCE && distFL > SAFE_DISTANCE && distFR > SAFE_DISTANCE))
        {
            avoidanceState = RETURNING_TO_PATH;
            avoidanceTimer = currentTime;
            if (DEBUG_MODE)
                debugPrint("AVOIDANCE:ROTATION_BACK_COMPLETE");
        }
        break;

    case RETURNING_TO_PATH:
        // Check if path ahead is clear before proceeding
        if (distF < MIN_AVOIDANCE_DISTANCE)
        {
            if (avoidanceAttempts < MAX_AVOIDANCE_ATTEMPTS)
            {
                // Try again if we still have obstacles ahead
                avoidanceAttempts++;
                avoidanceState = ROTATING_AWAY;
                avoidanceTimer = currentTime;
                if (DEBUG_MODE)
                    debugPrint("AVOIDANCE:NEW_OBSTACLE_DETECTED_RETRYING");
                break;
            }
            else
            {
                // Give up after multiple attempts
                avoidanceState = IDLE;
                if (DEBUG_MODE)
                    debugPrint("AVOIDANCE:FAILED_AFTER_MULTIPLE_ATTEMPTS");
                return false;
            }
        }

        // Move forward to return to the original path
        moveForward(avoidanceSpeed);

        // Advance for a set time or until we've returned to the path
        if (currentTime - avoidanceTimer > MOVEMENT_TIMEOUT / 2)
        {
            avoidanceState = IDLE;
            avoidanceSuccessful = true;
            if (DEBUG_MODE)
                debugPrint("AVOIDANCE:COMPLETED_SUCCESSFULLY");
            return true; // Obstacle avoidance complete
        }
        break;

    case IDLE:
    default:
        return true; // Nothing to do
    }

    return false; // Still in progress
}

// Main obstacle avoidance function - integrates with the current movement system
// Call this from the moveForward function
bool navigateAroundObstacle(int speed)
{
    // Check if we're already in an avoidance procedure
    if (avoidanceState != IDLE)
    {
        // Continue the avoidance procedure
        bool result = continueObstacleAvoidance();

        // If avoidance is complete, reset the system for the next obstacle
        if (result)
        {
            if (DEBUG_MODE)
            {
                debugPrint("AVOIDANCE:COMPLETED_OBSTACLE_NAVIGATION");
            }
            // Reset state to prepare for next obstacle
            avoidanceState = IDLE;
            avoidanceAttempts = 0;
            // Note: We don't reset avoidanceSuccessful here because moveForward needs to read it
        }

        return result;
    }
    else
    {
        // Start a new avoidance procedure
        return startObstacleAvoidance(speed);
    }
}

// Check if we need to update the emergency stop status and adjust avoidance parameters
void checkEmergencyStatus()
{
    // Calculate average distances and set warning flags
    float frontAvg = (distF + distFL + distFR) / 3.0;
    float backAvg = (distB + distBL + distBR) / 3.0;
    float leftAvg = (distFL + distBL) / 2.0;
    float rightAvg = (distFR + distBR) / 2.0;

    // Determine the tightness of space using the average distances
    float minAvgDistance = min(min(frontAvg, backAvg), min(leftAvg, rightAvg));

    // Adjust the avoidance rotation angle based on how tight the space is
    // Tighter spaces require larger rotation angles to successfully navigate
    if (minAvgDistance < MIN_AVOIDANCE_DISTANCE * 1.5)
    {
        // Very tight space - use large angle
        avoidanceRotationAngle = ROTATION_ANGLE_LARGE;

        // In very tight spaces, we may also need to adjust other parameters
        // For example, reduce speed further for safer navigation
        MAX_SPEED = constrain(MAX_SPEED, MIN_SPEED, 80);

        if (DEBUG_MODE)
        {
            debugPrint("TIGHT_SPACE:USING_LARGE_ROTATION_ANGLE");
        }
    }
    else if (minAvgDistance < OPTIMAL_AVOIDANCE_DISTANCE)
    {
        // Moderately tight space - use medium angle
        avoidanceRotationAngle = (ROTATION_ANGLE_SMALL + ROTATION_ANGLE_LARGE) / 2.0;

        // Restore normal speed limits for moderate spaces
        MAX_SPEED = 100;

        if (DEBUG_MODE)
        {
            debugPrint("MODERATE_SPACE:USING_MEDIUM_ROTATION_ANGLE");
        }
    }
    else
    {
        // Open space - use smaller angle for efficiency
        avoidanceRotationAngle = ROTATION_ANGLE_SMALL;

        // Restore normal speed limits for open spaces
        MAX_SPEED = 100;

        if (DEBUG_MODE && avoidanceState != IDLE)
        {
            debugPrint("OPEN_SPACE:USING_SMALL_ROTATION_ANGLE");
        }
    }
}
