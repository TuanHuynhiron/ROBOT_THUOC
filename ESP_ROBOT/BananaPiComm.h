#ifndef BANANA_PI_COMM_H
#define BANANA_PI_COMM_H

#include <Arduino.h>

class BananaPiComm {
private:
    HardwareSerial& serialPort;
    uint8_t rxPin;
    uint8_t txPin;

public:
    BananaPiComm(HardwareSerial& serial, uint8_t rx, uint8_t tx);
    void begin(unsigned long baud = 115200);
    bool readCommand(String &outCmd);
    void sendData(const String &data);
};

#endif