#include "EncoderDriver.h"

EncoderDriver::EncoderDriver(uint8_t a, uint8_t b) : pinA(a), pinB(b), pulseCount(0) {}

void EncoderDriver::begin(void (*ISR_func)()) {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinA), ISR_func, RISING);
}

void IRAM_ATTR EncoderDriver::handleInterrupt() {
    if (digitalRead(pinB) == HIGH) {
        pulseCount++;
    } else {
        pulseCount--;
    }
}

long EncoderDriver::getCount() {
    return pulseCount;
}

void EncoderDriver::reset() {
    pulseCount = 0;
}