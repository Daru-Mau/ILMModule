#!/bin/bash
# This script verifies that the Arduino code compiles correctly

ARDUINO_CODE="/home/ILMModule/ILMModule/ILMMCodes/integrated_movement/integrated_movement.ino"
ARDUINO_CLI="/home/ILMModule/bin/arduino-cli"

# Check if arduino-cli exists
if [ ! -f "$ARDUINO_CLI" ]; then
    echo "Error: arduino-cli not found at $ARDUINO_CLI"
    echo "Please ensure that arduino-cli is installed at the correct location."
    exit 1
fi

# Verify the code compilation
echo "Verifying Arduino code compilation..."
$ARDUINO_CLI compile --fqbn arduino:avr:mega "$ARDUINO_CODE"

# Check the result
if [ $? -eq 0 ]; then
    echo "Success: Arduino code compiled without errors."
    exit 0
else
    echo "Error: Arduino code compilation failed."
    exit 1
fi
