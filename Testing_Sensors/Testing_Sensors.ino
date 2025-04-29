// Ultrasonic sensor pins
// FRONT sensor
#define TRIG_FRONT 22
#define ECHO_FRONT 23

// FRONT-LEFT sensor
#define TRIG_FRONT_LEFT 24
#define ECHO_FRONT_LEFT 25

// FRONT-RIGHT sensor
#define TRIG_FRONT_RIGHT 26
#define ECHO_FRONT_RIGHT 27

// LEFT sensor
#define TRIG_LEFT 28
#define ECHO_LEFT 29

// RIGHT sensor
#define TRIG_RIGHT 30
#define ECHO_RIGHT 31

// REAR sensor
#define TRIG_REAR 2
#define ECHO_REAR 3

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
const UltrasonicSensor frontSensor = {TRIG_FRONT, ECHO_FRONT, "Front"};
const UltrasonicSensor frontLeftSensor = {TRIG_FRONT_LEFT, ECHO_FRONT_LEFT, "Front-Left"};
const UltrasonicSensor frontRightSensor = {TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT, "Front-Right"};
const UltrasonicSensor leftSensor = {TRIG_LEFT, ECHO_LEFT, "Left"};
const UltrasonicSensor rightSensor = {TRIG_RIGHT, ECHO_RIGHT, "Right"};
const UltrasonicSensor rearSensor = {TRIG_REAR, ECHO_REAR, "Rear"};

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
    setupSensorPins(leftSensor);
    setupSensorPins(rightSensor);
    setupSensorPins(rearSensor);

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
                                         &leftSensor, &rightSensor, &rearSensor};

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
                                         &leftSensor, &rightSensor, &rearSensor};

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
                                         &leftSensor, &rightSensor, &rearSensor};

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
                                         &leftSensor, &rightSensor, &rearSensor};

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
