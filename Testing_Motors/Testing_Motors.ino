// Motor Driver Pin Definitions
struct Motor
{
    int RPWM;
    int LPWM;
    int REN;
    int LEN;
};

Motor motorRight = {3, 2, 39, 38};
Motor motorLeft = {4, 5, 44, 45};
Motor motorBack = {7, 6, 51, 50};

const int MOTOR_SPEED = 100; // Adjust speed from 0 to 255

void setupMotor(Motor m)
{
    pinMode(m.RPWM, OUTPUT);
    pinMode(m.LPWM, OUTPUT);
    pinMode(m.REN, OUTPUT);
    pinMode(m.LEN, OUTPUT);
    digitalWrite(m.REN, HIGH);
    digitalWrite(m.LEN, HIGH);
    analogWrite(m.RPWM, MOTOR_SPEED); // Forward
    analogWrite(m.LPWM, 0);
}

void setup()
{
    setupMotor(motorLeft);
    setupMotor(motorRight);
    setupMotor(motorBack);
}

void loop()
{
    // All motors run forward continuously
}
