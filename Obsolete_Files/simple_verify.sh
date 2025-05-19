#!/bin/bash
# Simple script to verify Arduino code without all the test infrastructure

echo "Verifying Arduino code..."
# Try to use arduino-cli to verify the code
if command -v arduino-cli &> /dev/null; then
    arduino-cli compile --fqbn arduino:avr:mega /home/ILMModule/ILMModule/ILMMCodes/integrated_movement/integrated_movement.ino
    if [ $? -eq 0 ]; then
        echo "Success: Arduino code compiles!"
    else
        echo "Error: Arduino code still has compilation issues."
    fi
else
    echo "Warning: arduino-cli not found in PATH. Can't verify code."
    echo "You'll need to verify the code in the Arduino IDE."
fi
