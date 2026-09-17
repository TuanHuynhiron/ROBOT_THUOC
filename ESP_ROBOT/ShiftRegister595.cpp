#include "ShiftRegister595.h"

ShiftRegister595::ShiftRegister595(uint8_t data, uint8_t clock, uint8_t latch)
    : dataPin(data), clockPin(clock), latchPin(latch), pinStates(0) {}

void ShiftRegister595::begin() {
    pinMode(dataPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(latchPin, OUTPUT);
    clearAll();
}

void ShiftRegister595::setPin(uint8_t pinIndex, bool state) {
    if (pinIndex > 15) return;
    if (state) {
        pinStates |= (1 << pinIndex);
    } else {
        pinStates &= ~(1 << pinIndex);
    }
}

void ShiftRegister595::write() {
    digitalWrite(latchPin, LOW);
    // Xuất Byte cao trước (IC2), Byte thấp sau (IC1)
    shiftOut(dataPin, clockPin, MSBFIRST, (pinStates >> 8) & 0xFF);
    shiftOut(dataPin, clockPin, MSBFIRST, pinStates & 0xFF);
    digitalWrite(latchPin, HIGH);
}

void ShiftRegister595::clearAll() {
    pinStates = 0x0000;
    write();
}