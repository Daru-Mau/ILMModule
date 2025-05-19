#!/bin/bash
# Comprehensive Test Suite for Arduino Ultrasonic Sensor Integration

echo "===== ARDUINO ULTRASONIC SENSOR INTEGRATION TEST SUITE ====="
echo "This script will run a series of tests to verify the integration"
echo "between the Arduino and Raspberry Pi for ultrasonic sensor data."
echo ""

# Define colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

# Default settings
ARDUINO_PORT="/dev/ttyACM0"
BAUD_RATE=115200

# Check command line arguments
if [ "$1" != "" ]; then
    ARDUINO_PORT="$1"
    echo -e "${YELLOW}Using custom Arduino port: $ARDUINO_PORT${NC}"
fi

# Function to check if Arduino is connected
check_arduino_connection() {
    echo -e "${YELLOW}Checking Arduino connection on $ARDUINO_PORT...${NC}"
    
    if [ ! -e "$ARDUINO_PORT" ]; then
        echo -e "${RED}ERROR: Arduino port $ARDUINO_PORT does not exist${NC}"
        echo "Available ports:"
        ls -la /dev/tty*
        return 1
    fi
    
    # Try to read from the port
    echo "Attempting to open serial port..."
    stty -F "$ARDUINO_PORT" $BAUD_RATE raw -echo
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Arduino port accessible${NC}"
        return 0
    else
        echo -e "${RED}Could not access Arduino port. Check permissions or if another process is using it.${NC}"
        return 1
    fi
}

# Function to verify Arduino code
verify_arduino_code() {
    echo -e "\n${YELLOW}=== TEST 1: Verifying Arduino Code ===${NC}"
    
    if [ -x "$(command -v arduino-cli)" ]; then
        echo "Using arduino-cli to verify code..."
        ARDUINO_CODE_PATH="/home/ILMModule/ILMModule/ILMMCodes/integrated_movement/integrated_movement.ino"
        
        arduino-cli compile --fqbn arduino:avr:mega "$ARDUINO_CODE_PATH"
        
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}Arduino code verification successful${NC}"
            return 0
        else
            echo -e "${RED}Arduino code verification failed${NC}"
            return 1
        fi
    else
        echo -e "${YELLOW}arduino-cli not found. Skipping code verification.${NC}"
        echo "Please run the verification script separately:"
        echo "    ./verify_arduino_code.sh"
        return 0
    fi
}

# Function to test basic serial communication
test_basic_communication() {
    echo -e "\n${YELLOW}=== TEST 2: Basic Serial Communication Test ===${NC}"
    echo "Running quick_arduino_test.py to test basic communication..."
    
    python3 quick_arduino_test.py --port "$ARDUINO_PORT" --baud "$BAUD_RATE"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Basic communication test passed${NC}"
        return 0
    else
        echo -e "${RED}Basic communication test failed${NC}"
        return 1
    fi
}

# Function to test sensor data format
test_sensor_format() {
    echo -e "\n${YELLOW}=== TEST 3: Sensor Data Format Test ===${NC}"
    echo "Running test_sensor_format.py to validate sensor data format..."
    
    python3 test_sensor_format.py --port "$ARDUINO_PORT" --baud "$BAUD_RATE" --duration 10
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Sensor format test completed${NC}"
        return 0
    else
        echo -e "${RED}Sensor format test failed${NC}"
        return 1
    fi
}

# Function to test monitor script
test_sensor_monitoring() {
    echo -e "\n${YELLOW}=== TEST 4: Sensor Data Monitoring Test ===${NC}"
    echo "Running monitor_sensor_data_fixed.py to test sensor monitoring..."
    
    python3 monitor_sensor_data_fixed.py --port "$ARDUINO_PORT" --baud "$BAUD_RATE" --duration 10
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Sensor monitoring test completed${NC}"
        return 0
    else
        echo -e "${RED}Sensor monitoring test failed${NC}"
        return 1
    fi
}

# Function to test integration
test_integrated_functionality() {
    echo -e "\n${YELLOW}=== TEST 5: Integrated Functionality Test ===${NC}"
    echo "Running integrated_sensor_test.py to test full integration..."
    
    python3 integrated_sensor_test.py --port "$ARDUINO_PORT" --baud "$BAUD_RATE" --duration 15
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Integration test completed${NC}"
        return 0
    else
        echo -e "${RED}Integration test failed${NC}"
        return 1
    fi
}

# Main test sequence
main() {
    echo -e "\n${YELLOW}Starting comprehensive test suite...${NC}"
    
    # Check Arduino connection
    check_arduino_connection
    if [ $? -ne 0 ]; then
        echo -e "${RED}Arduino connection check failed. Aborting tests.${NC}"
        exit 1
    fi
    
    # Run all tests
    echo -e "\n${YELLOW}Running all tests in sequence...${NC}"
    
    local all_tests_passed=true
    
    verify_arduino_code
    if [ $? -ne 0 ]; then all_tests_passed=false; fi
    
    test_basic_communication
    if [ $? -ne 0 ]; then all_tests_passed=false; fi
    
    test_sensor_format
    if [ $? -ne 0 ]; then all_tests_passed=false; fi
    
    test_sensor_monitoring
    if [ $? -ne 0 ]; then all_tests_passed=false; fi
    
    test_integrated_functionality
    if [ $? -ne 0 ]; then all_tests_passed=false; fi
    
    # Final report
    echo -e "\n${YELLOW}===== TEST SUMMARY =====${NC}"
    if $all_tests_passed; then
        echo -e "${GREEN}All tests completed successfully!${NC}"
        echo "The Arduino is properly sending ultrasonic sensor data that can be"
        echo "integrated with the AprilTag recognition system."
    else
        echo -e "${RED}Some tests failed. Please review the output above.${NC}"
        echo "Check the specific error messages for each failed test."
    fi
    
    echo -e "\n${YELLOW}===== NEXT STEPS =====${NC}"
    echo "1. Update the apriltag_recognition.py script to utilize sensor data"
    echo "2. Test with real obstacles while running the full system"
    echo "3. Fine-tune timing parameters if needed"
    
    return 0
}

# Run the main function
main
exit $?
