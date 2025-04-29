// Basic motor test code
// Motor pins (example)
#define FRONT_RPWM 37
#define FRONT_LPWM 36
#define FRONT_REN 39
#define FRONT_LEN 38

#define LEFT_RPWM 43
#define LEFT_LPWM 42
#define LEFT_REN 45
#define LEFT_LEN 44

#define RIGHT_RPWM 49
#define RIGHT_LPWM 48
#define RIGHT_REN 51
#define RIGHT_LEN 50

void setup()
{
    // Set all pins as output
    pinMode(FRONT_RPWM, OUTPUT);
    pinMode(FRONT_LPWM, OUTPUT);
    pinMode(FRONT_REN, OUTPUT);
    pinMode(FRONT_LEN, OUTPUT);

    pinMode(LEFT_RPWM, OUTPUT);
    pinMode(LEFT_LPWM, OUTPUT);
    pinMode(LEFT_REN, OUTPUT);
    pinMode(LEFT_LEN, OUTPUT);

    pinMode(RIGHT_RPWM, OUTPUT);
    pinMode(RIGHT_LPWM, OUTPUT);
    pinMode(RIGHT_REN, OUTPUT);
    pinMode(RIGHT_LEN, OUTPUT);

    // Enable all motor drivers
    digitalWrite(FRONT_REN, HIGH);
    digitalWrite(FRONT_LEN, HIGH);

    digitalWrite(LEFT_REN, HIGH);
    digitalWrite(LEFT_LEN, HIGH);

    digitalWrite(RIGHT_REN, HIGH);
    digitalWrite(RIGHT_LEN, HIGH);
}

void loop()
{
    // Move motors forward
    analogWrite(FRONT_RPWM, 150); // Speed 0-255
    analogWrite(FRONT_LPWM, 0);

    analogWrite(LEFT_RPWM, 150);
    analogWrite(LEFT_LPWM, 0);

    analogWrite(RIGHT_RPWM, 150);
    analogWrite(RIGHT_LPWM, 0);

    delay(2000); // Run for 2 seconds

    // Stop motors
    analogWrite(FRONT_RPWM, 0);
    analogWrite(FRONT_LPWM, 0);

    analogWrite(LEFT_RPWM, 0);
    analogWrite(LEFT_LPWM, 0);

    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, 0);

    delay(2000); // Pause for 2 seconds

    // Move motors backward
    analogWrite(FRONT_RPWM, 0);
    analogWrite(FRONT_LPWM, 150);

    analogWrite(LEFT_RPWM, 0);
    analogWrite(LEFT_LPWM, 150);

    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, 150);

    delay(2000); // Run for 2 seconds

    // Stop motors
    analogWrite(FRONT_RPWM, 0);
    analogWrite(FRONT_LPWM, 0);

    analogWrite(LEFT_RPWM, 0);
    analogWrite(LEFT_LPWM, 0);

    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, 0);

    delay(2000); // Pause before repeating
}
