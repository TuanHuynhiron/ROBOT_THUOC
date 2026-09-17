#ifndef SHIFT_REGISTER_595_H
#define SHIFT_REGISTER_595_H

#include <Arduino.h>

class ShiftRegister595 {
private:
    uint8_t dataPin;
    uint8_t clockPin;
    uint8_t latchPin;
    uint16_t pinStates; // Lưu trạng thái 16-bit cho 2 IC cascade

public:
    ShiftRegister595(uint8_t data, uint8_t clock, uint8_t latch);
    void begin();
    void setPin(uint8_t pinIndex, bool state);
    void write();
    void clearAll();
};

#endif