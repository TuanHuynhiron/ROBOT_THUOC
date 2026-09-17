#include "HCSR04.h"

HCSR04::HCSR04(uint8_t trig, uint8_t echo) : trigPin(trig), echoPin(echo) {}

void HCSR04::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
}

float HCSR04::getDistanceCm() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000); // Timeout 30ms
    if (duration == 0) return -1.0; // Out of range
    return (duration * 0.0343) / 2.0;
}