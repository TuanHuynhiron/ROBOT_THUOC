#include "BananaPiComm.h"

BananaPiComm::BananaPiComm(HardwareSerial& serial, uint8_t rx, uint8_t tx)
    : serialPort(serial), rxPin(rx), txPin(tx) {}

void BananaPiComm::begin(unsigned long baud) {
    serialPort.begin(baud, SERIAL_8N1, rxPin, txPin);
}

bool BananaPiComm::readCommand(String &outCmd) {
    if (serialPort.available() > 0) {
        outCmd = serialPort.readStringUntil('\n');
        outCmd.trim();
        return true;
    }
    return false;
}

void BananaPiComm::sendData(const String &data) {
    serialPort.println(data);
}