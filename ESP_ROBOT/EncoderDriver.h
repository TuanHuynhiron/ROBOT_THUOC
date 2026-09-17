#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>

class EncoderDriver {
private:
    uint8_t pinA;
    uint8_t pinB;
    volatile long pulseCount;

public:
    EncoderDriver(uint8_t a, uint8_t b);
    void begin(void (*ISR_func)());
    void handleInterrupt();
    long getCount();
    void reset();
};

#endif