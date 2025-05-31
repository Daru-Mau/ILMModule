# Troubleshooting Guide

This guide provides solutions to common issues encountered when working with the Localization Module.

## Compilation Errors

### Undefined Reference to Direction Variables

**Error:**
```
undefined reference to `leftDir'
undefined reference to `rightDir'
undefined reference to `backDir'
```

**Solution:**
This error occurs when the smooth deceleration system doesn't have access to the motor direction tracking variables. To fix it:

1. Make sure the direction variables are globally defined in `integrated_movement.ino`:
   ```cpp
   // Motor direction tracking variables
   Direction leftDir = STOP;  // Direction tracking for left motor
   Direction rightDir = STOP; // Direction tracking for right motor
   Direction backDir = STOP;  // Direction tracking for back motor
   ```

2. Add proper extern declarations in `smooth_deceleration.ino`:
   ```cpp
   // External declarations for variables from integrated_movement.ino
   extern Direction leftDir;
   extern Direction rightDir;
   extern Direction backDir;
   ```

3. Verify that the Direction enum is accessible to all files:
   ```cpp
   enum Direction
   {
       FORWARD,
       BACKWARD,
       STOP
   };
   ```

## Movement Control Issues

### Jerky Stops or Missing Deceleration

**Symptoms:**
- Robot stops abruptly instead of gradually
- Motors whine during stops
- Direction changes are jarring

**Solution:**
1. Check that `handleSmoothDeceleration()` is being called in the main loop:
   ```cpp
   void loop() {
       // Other code...
       handleSmoothDeceleration();
       // Other code...
   }
   ```

2. Ensure the `stopAllMotors()` function is starting deceleration properly:
   ```cpp
   void stopAllMotors() {
       // Start the smooth deceleration process
       startSmoothDeceleration();
       // Rest of function...
   }
   ```

3. Verify that `moveMotor()` respects the deceleration process:
   ```cpp
   case STOP:
       if (!decelerationActive) {
           analogWrite(motor.RPWM, 0);
           analogWrite(motor.LPWM, 0);
           motor.currentSpeed = 0;
       }
       break;
   ```

## System Integration

### Obstacle Avoidance Not Working with Deceleration

**Symptoms:**
- Robot ignores obstacles during deceleration
- Emergency stops don't function during deceleration

**Solution:**
1. Make sure emergency stop checks occur before deceleration handling:
   ```cpp
   void loop() {
       // Check for emergency conditions first
       checkEmergencyStatus();
       
       // Then handle deceleration
       handleSmoothDeceleration();
       
       // Rest of loop...
   }
   ```

2. Ensure emergency stop can bypass smooth deceleration:
   ```cpp
   void emergencyStopAllMotors() {
       // Immediately set all PWM outputs to zero
       analogWrite(motorLeft.RPWM, 0);
       analogWrite(motorLeft.LPWM, 0);
       motorLeft.currentSpeed = 0;
       
       // Same for other motors...
       
       // Reset deceleration state
       decelerationActive = false;
   }
   ```

## Performance Tuning

### Deceleration Too Slow or Fast

**Solution:**
Adjust the deceleration rate in `smooth_deceleration.ino`:
```cpp
// Increase for faster stops, decrease for smoother stops
float decelerationRate = 0.15;  // Rate of deceleration (0-1)
```
