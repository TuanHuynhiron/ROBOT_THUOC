#ifndef SIM7680_MODULE_H
#define SIM7680_MODULE_H

#include <Arduino.h>

class SIM7680Module {
private:
    HardwareSerial& simSerial;
    uint8_t rxPin;
    uint8_t txPin;

public:
    SIM7680Module(HardwareSerial& serial, uint8_t rx, uint8_t tx);
    void begin(unsigned long baud = 115200);
    bool sendSMS(const String &phoneNumber, const String &message);
};

#endif