#include <Arduino.h>

// Motor pins
const int motorLeftForward  = 3;
const int motorLeftBackward = 4;
const int motorRightForward = 5;
const int motorRightBackward = 6;

// PWM ramping
const int maxForwardSpeed = 255;
const int maxBackwardSpeed = 180;
const int rampStep = 3;        // how fast speed ramps up/down
const int rampInterval = 20;   // ms delay between PWM steps

// -------------------- ULTRASONIC SENSOR --------------------
const int trigPin = 7;
const int echoPin = 8;

float distanceCM = 999.0;
unsigned long lastUSread = 0;           // timestamp
const unsigned long USinterval = 80;    // only sample every 80 ms
const float safeDistance = 35.0;        // cm, threshold for backing up
// ------------------------------------------------------------

// Current motor state
int motorSign = 1;   // 1 = forward, -1 = backward
int motorSpeed = 0;  // current PWM

bool movingBackward = false; // state flag

void setup() {
    Serial.begin(115200);

    pinMode(motorLeftForward, OUTPUT);
    pinMode(motorLeftBackward, OUTPUT);
    pinMode(motorRightForward, OUTPUT);
    pinMode(motorRightBackward, OUTPUT);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    Serial.println("Ready.");
}

// -------------------- MOTOR CONTROL --------------------
void applyMotor(int sign, int speed) {
    speed = constrain(speed, 0, 255);

    if (sign < 0) { // forward
        analogWrite(motorLeftForward, speed);
        analogWrite(motorLeftBackward, 0);
        analogWrite(motorRightForward, speed);
        analogWrite(motorRightBackward, 0);
    } else if (sign > 0) { // backward
        analogWrite(motorLeftForward, 0);
        analogWrite(motorLeftBackward, speed);
        analogWrite(motorRightForward, 0);
        analogWrite(motorRightBackward, speed);
    } else { // stop
        analogWrite(motorLeftForward, 0);
        analogWrite(motorLeftBackward, 0);
        analogWrite(motorRightForward, 0);
        analogWrite(motorRightBackward, 0);
    }
}

// -------------------- ULTRASONIC FUNCTION --------------------
float getDistanceCM() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(3);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(12);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return 999.0;
    return duration * 0.0343 / 2.0;
}

// -------------------- SPEED MAPPING --------------------
// change return values for modifying speed
int getSpeedFromDistance(float distance) {
    // Adjust these thresholds and speeds as needed
    if (distance > 50) return maxForwardSpeed;
    if (distance > 40) return 200;
    if (distance > 30) return 100;
    if (distance > 25) return 70; 
    return 50; // extremely close
}

// -------------------- MAIN LOOP --------------------
void loop() {
    // Read ultrasonic sensor every 80ms
    if (millis() - lastUSread > USinterval) {
        distanceCM = getDistanceCM();
        lastUSread = millis();
    }

    int targetSign = 1;
    int targetSpeed;

    // Check if we should start moving backward
    if (!movingBackward && distanceCM < safeDistance) {
        movingBackward = true; // lock into backward motion
        Serial.println("Obstacle detected → switching to backward mode");
    }

    if (movingBackward) {
        targetSign = -1;
        targetSpeed = maxBackwardSpeed; // full backward speed
    } else {
        targetSign = 1;
        targetSpeed = getSpeedFromDistance(distanceCM); // dynamic forward speed
    }

    // Smooth acceleration/deceleration when changing direction
    if (motorSign != targetSign) {
        while (motorSpeed > 0) {
            motorSpeed -= rampStep;
            if (motorSpeed < 0) motorSpeed = 0;
            applyMotor(motorSign, motorSpeed);
            delay(rampInterval);
        }
        motorSign = targetSign;
    }

    // Ramp up/down to target speed
    while (motorSpeed < targetSpeed) {
        motorSpeed += rampStep;
        if (motorSpeed > targetSpeed) motorSpeed = targetSpeed;
        applyMotor(motorSign, motorSpeed);
        delay(rampInterval);
    }
    while (motorSpeed > targetSpeed) {
        motorSpeed -= rampStep;
        if (motorSpeed < targetSpeed) motorSpeed = targetSpeed;
        applyMotor(motorSign, motorSpeed);
        delay(rampInterval);
    }

    // Optional: debug
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        Serial.print("DistanceCM: "); Serial.print(distanceCM);
        Serial.print(", MotorSign: "); Serial.print(motorSign);
        Serial.print(", MotorSpeed: "); Serial.print(motorSpeed);
        Serial.print(", MovingBackward: "); Serial.println(movingBackward);
        lastPrint = millis();
    }
}
