/* // Ultrasonic Sensor Pins - Normal Setting
#define TRIG_BL 26
#define ECHO_BL 27
#define TRIG_B 24
#define ECHO_B 25
#define TRIG_BR 28
#define ECHO_BR 29
#define TRIG_FR 34
#define ECHO_FR 35 
#define TRIG_F 32
#define ECHO_F 33
#define TRIG_FL 30
#define ECHO_FL 31 
*/

// Ultrasonic Sensor Pins - Rotated Front
#define TRIG_F 28   
#define ECHO_F 29   
#define TRIG_FL 34  
#define ECHO_FL 35  
#define TRIG_FR 24  
#define ECHO_FR 25  
#define TRIG_BL 32 
#define ECHO_BL 33  
#define TRIG_B 30   
#define ECHO_B 31   
#define TRIG_BR 26  
#define ECHO_BR 27  

// Test Configuration
#define MAX_DISTANCE 400    // Maximum reliable reading distance in cm
#define WARNING_DISTANCE 10 // Distance for proximity warnings in cm
#define READING_DELAY 500   // Delay between readings in ms

// Sensor Structure
struct UltrasonicSensor
{
    const int trigPin;
    const int echoPin;
    const char *name;
};

// Define sensors
const UltrasonicSensor frontSensor = {TRIG_F, ECHO_F, "Front"};
const UltrasonicSensor frontLeftSensor = {TRIG_FL, ECHO_FL, "Front-Left"};
const UltrasonicSensor frontRightSensor = {TRIG_FR, ECHO_FR, "Front-Right"};
const UltrasonicSensor backLeftSensor = {TRIG_BL, ECHO_BL, "Back-Left"};
const UltrasonicSensor backSensor = {TRIG_B, ECHO_B, "Back"};
const UltrasonicSensor backRightSensor = {TRIG_BR, ECHO_BR, "Back-Right"};

void setupSensorPins(const UltrasonicSensor &sensor)
{
    pinMode(sensor.trigPin, OUTPUT);
    pinMode(sensor.echoPin, INPUT);
}

long readDistance(const UltrasonicSensor &sensor)
{
    digitalWrite(sensor.trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sensor.trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensor.trigPin, LOW);

    long duration = pulseIn(sensor.echoPin, HIGH, 30000); // 30ms timeout
    if (duration == 0)
    {
        return -1; // No reading (timeout)
    }

    return duration * 0.034 / 2; // Calculate distance in centimeters
}

void setup()
{
    Serial.begin(9600); // Changed to 9600 baud rate for better compatibility

    // Setup all sensors
    setupSensorPins(frontSensor);
    setupSensorPins(frontLeftSensor);
    setupSensorPins(frontRightSensor);
    setupSensorPins(backLeftSensor);
    setupSensorPins(backRightSensor);
    setupSensorPins(backSensor);

    Serial.println("===============================");
    Serial.println("Ultrasonic Sensors Testing System");
    Serial.println("===============================");
    Serial.println("Commands:");
    Serial.println("1: Test individual sensors");
    Serial.println("2: Test all sensors continuously");
    Serial.println("3: Test proximity warnings");
    Serial.println("4: Test sensor reliability");
    Serial.println("===============================");
}

void testIndividualSensors()
{
    const UltrasonicSensor *sensors[] = {&frontSensor, &frontLeftSensor, &frontRightSensor,
                                         &backLeftSensor, &backRightSensor, &backSensor};

    Serial.println(F("\n=== Testing Individual Sensors ==="));

    for (int i = 0; i < 6; i++)
    {
        Serial.print(F("Testing "));
        Serial.print(sensors[i]->name);
        Serial.println(F(" sensor:"));

        long distance = readDistance(*sensors[i]);
        if (distance >= 0)
        {
            Serial.print(F("Distance: "));
            Serial.print(distance);
            Serial.println(F(" cm"));
        }
        else
        {
            Serial.println(F("No reading (timeout)"));
        }
        delay(READING_DELAY);
    }
}

void testAllSensorsContinuously()
{
    const UltrasonicSensor *sensors[] = {&frontSensor, &frontLeftSensor, &frontRightSensor,
                                         &backLeftSensor, &backRightSensor, &backSensor};

    Serial.println("\n=== All Sensors Reading ===");

    for (int i = 0; i < 6; i++)
    {
        long distance = readDistance(*sensors[i]);
        Serial.print(sensors[i]->name);
        Serial.print(": ");
        if (distance >= 0)
        {
            Serial.print(distance);
            Serial.print(" cm");
            // Add visual indicator for close objects
            if (distance < WARNING_DISTANCE)
            {
                Serial.print(" [!]");
            }
            Serial.println();
        }
        else
        {
            Serial.println("No reading");
        }
    }
    Serial.println("-------------------");
}

void testProximityWarnings()
{
    const UltrasonicSensor *sensors[] = {&frontSensor, &frontLeftSensor, &frontRightSensor,
                                         &backLeftSensor, &backRightSensor, &backSensor};

    Serial.println("\n=== Proximity Warnings Test ===");
    bool warningDetected = false;

    for (int i = 0; i < 6; i++)
    {
        long distance = readDistance(*sensors[i]);
        if (distance > 0 && distance < WARNING_DISTANCE)
        {
            Serial.print("WARNING! ");
            Serial.print(sensors[i]->name);
            Serial.println(" object detected under 10cm");
            warningDetected = true;
        }
    }

    if (!warningDetected)
    {
        Serial.println("No proximity warnings");
    }
}

void testSensorReliability()
{
    const UltrasonicSensor *sensors[] = {&frontSensor, &frontLeftSensor, &frontRightSensor,
                                         &backLeftSensor, &backRightSensor, &backSensor};

    Serial.println("\n=== Sensor Reliability Test ===");

    for (int i = 0; i < 6; i++)
    {
        Serial.print(sensors[i]->name);
        Serial.println(" sensor reliability test:");
        Serial.print("Progress: ");

        int validReadings = 0;
        int totalReadings = 10;

        for (int j = 0; j < totalReadings; j++)
        {
            long distance = readDistance(*sensors[i]);
            if (distance > 0 && distance <= MAX_DISTANCE)
            {
                validReadings++;
            }
            Serial.print(".");
            delay(100);
        }
        Serial.println();

        float reliability = (validReadings * 100.0) / totalReadings;
        Serial.print("Reliability: ");
        Serial.print(reliability, 1); // Show one decimal place
        Serial.println("%");
        delay(500);
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
            testIndividualSensors();
            break;
        case '2':
            testAllSensorsContinuously();
            break;
        case '3':
            testProximityWarnings();
            break;
        case '4':
            testSensorReliability();
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
