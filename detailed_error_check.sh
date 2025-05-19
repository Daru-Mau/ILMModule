#!/bin/bash
ARDUINO_CODE="/home/ILMModule/ILMModule/ILMMCodes/integrated_movement/integrated_movement.ino"
OUTPUT_FILE="/tmp/arduino_compile_output.txt"

# Run the compilation and capture all output
cd /home/ILMModule
arduino-cli compile --fqbn arduino:avr:mega "$ARDUINO_CODE" &> "$OUTPUT_FILE"

# Display the output
echo "======= COMPILATION RESULTS ======="
cat "$OUTPUT_FILE"
echo "=================================="

# Check for specific error patterns
echo "Looking for specific errors..."
if grep -q "redefinition of 'void sendSensorData()'" "$OUTPUT_FILE"; then
    echo "FOUND: Duplicate sendSensorData() function definition"
fi

if grep -q "redefinition of 'void setup()'" "$OUTPUT_FILE"; then
    echo "FOUND: Duplicate setup() function definition"
fi

if grep -q "redefinition of 'void loop()'" "$OUTPUT_FILE"; then
    echo "FOUND: Duplicate loop() function definition"
fi

if grep -q "was not declared in this scope" "$OUTPUT_FILE"; then
    echo "FOUND: Function or variable used before declaration"
    grep "was not declared in this scope" "$OUTPUT_FILE"
fi

if grep -q "error:" "$OUTPUT_FILE"; then
    echo "FOUND: General compilation errors"
    grep "error:" "$OUTPUT_FILE"
fi

# Clean up
echo "Detailed output saved to $OUTPUT_FILE"
