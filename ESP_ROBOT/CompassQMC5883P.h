#ifndef COMPASS_QMC5883P_H
#define COMPASS_QMC5883P_H

#include <Arduino.h>
#include <Wire.h>

class CompassQMC5883P {
private:
    uint8_t i2cAddr;
    int16_t x_min, x_max;
    int16_t y_min, y_max;
    float offset_x, offset_y;
    float scale_x, scale_y;
    float smoothed_heading;

    void writeReg(uint8_t reg, uint8_t val);

public:
    CompassQMC5883P(uint8_t addr = 0x2C); // Địa chỉ 0x2C theo code mẫu của bạn
    void begin(uint8_t sdaPin = 21, uint8_t sclPin = 22);
    void calibrate(unsigned long durationMs = 10000); // Calib elip tự động
    float getHeading(); // Đọc góc hướng đã lọc EMA
};

#endif