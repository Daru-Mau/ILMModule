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
#define TRIG_REAR 32
#define ECHO_REAR 33

void setup()
{
    Serial.begin(115200);

    // Set all trig pins as OUTPUT and echo pins as INPUT
    pinMode(TRIG_FRONT, OUTPUT);
    pinMode(ECHO_FRONT, INPUT);

    pinMode(TRIG_FRONT_LEFT, OUTPUT);
    pinMode(ECHO_FRONT_LEFT, INPUT);

    pinMode(TRIG_FRONT_RIGHT, OUTPUT);
    pinMode(ECHO_FRONT_RIGHT, INPUT);

    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);

    pinMode(TRIG_RIGHT, OUTPUT);
    pinMode(ECHO_RIGHT, INPUT);

    pinMode(TRIG_REAR, OUTPUT);
    pinMode(ECHO_REAR, INPUT);
}

long readDistanceCM(int trigPin, int echoPin)
{
    // Send trigger pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read echo pulse
    long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
    if (duration == 0)
    {
        return -1; // No reading (timeout)
    }

    // Calculate distance in centimeters
    long distance = duration * 0.034 / 2;
    return distance;
}

void loop()
{
    long distanceFront = readDistanceCM(TRIG_FRONT, ECHO_FRONT);
    long distanceFrontLeft = readDistanceCM(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
    long distanceFrontRight = readDistanceCM(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
    long distanceLeft = readDistanceCM(TRIG_LEFT, ECHO_LEFT);
    long distanceRight = readDistanceCM(TRIG_RIGHT, ECHO_RIGHT);
    long distanceRear = readDistanceCM(TRIG_REAR, ECHO_REAR);

    Serial.println("--- Sensor Readings (cm) ---");
    Serial.print("Front: ");
    Serial.println(distanceFront);

    Serial.print("Front-Left: ");
    Serial.println(distanceFrontLeft);

    Serial.print("Front-Right: ");
    Serial.println(distanceFrontRight);

    Serial.print("Left: ");
    Serial.println(distanceLeft);

    Serial.print("Right: ");
    Serial.println(distanceRight);

    Serial.print("Rear: ");
    Serial.println(distanceRear);

    Serial.println("----------------------------");
    delay(500); // Half a second between readings
}
