#ifndef COMPASS_QMC5883P_H
#define COMPASS_QMC5883P_H

#include <Arduino.h>
#include <Wire.h>

class CompassQMC5883P {
private:
    uint8_t i2cAddr;
    
    // Biến lưu Min/Max phục vụ Calib Elip
    int16_t xMin, xMax;
    int16_t yMin, yMax;
    
    // Hệ số Bù Offset & Scale Factor
    float offsetX, offsetY;
    float scaleX, scaleY;
    
    // Trạng thái & Biến lọc mượt EMA
    bool isCalibrating;
    float smoothedHeading;

    void writeReg(uint8_t reg, uint8_t val);

public:
    // Mặc định địa chỉ I2C là 0x2C theo code chuẩn của bạn
    CompassQMC5883P(uint8_t addr = 0x2C);
    
    void begin(uint8_t sdaPin = 21, uint8_t sclPin = 22);
    float readHeading();
    
    void startCalibration();
    void stopCalibration();
    bool getCalibratingStatus();
};

#endif