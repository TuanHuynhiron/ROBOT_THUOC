#ifndef HCSR04_H
#define HCSR04_H

#include <Arduino.h>

class HCSR04 {
private:
    uint8_t trigPin;
    uint8_t echoPin;

public:
    HCSR04(uint8_t trig, uint8_t echo);
    void begin();
    float getDistanceCm();
};

#endif