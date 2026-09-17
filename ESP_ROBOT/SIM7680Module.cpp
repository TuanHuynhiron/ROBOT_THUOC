#include "SIM7680Module.h"

SIM7680Module::SIM7680Module(HardwareSerial& serial, uint8_t rx, uint8_t tx)
    : simSerial(serial), rxPin(rx), txPin(tx) {}

void SIM7680Module::begin(unsigned long baud) {
    simSerial.begin(baud, SERIAL_8N1, rxPin, txPin);
}

bool SIM7680Module::sendSMS(const String &phoneNumber, const String &message) {
    simSerial.println("AT+CMGF=1"); 
    delay(200);
    simSerial.print("AT+CMGS=\"");
    simSerial.print(phoneNumber);
    simSerial.println("\"");
    delay(200);
    simSerial.print(message);
    delay(100);
    simSerial.write(26); // Ctrl+Z
    delay(3000);
    return true;
}