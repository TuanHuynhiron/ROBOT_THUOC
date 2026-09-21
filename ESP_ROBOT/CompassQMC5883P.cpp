#include "CompassQMC5883P.h"

CompassQMC5883P::CompassQMC5883P(uint8_t addr) {
    i2cAddr = addr;
    xMin = 32767; xMax = -32768;
    yMin = 32767; yMax = -32768;
    offsetX = 0.0; offsetY = 0.0;
    scaleX = 1.0;  scaleY = 1.0;
    isCalibrating = false;
    smoothedHeading = 0.0;
}

void CompassQMC5883P::writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(i2cAddr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void CompassQMC5883P::begin(uint8_t sdaPin, uint8_t sclPin) {
    Wire.begin(sdaPin, sclPin);
    delay(200);

    // Khởi tạo QMC5883P theo cấu hình chuẩn của bạn
    writeReg(0x0B, 0x80); delay(50); // Soft Reset
    writeReg(0x0B, 0x00); delay(50);
    writeReg(0x0A, 0x1D); delay(50); // Continuous 200Hz, 8G
}

void CompassQMC5883P::startCalibration() {
    xMin = 32767; xMax = -32768;
    yMin = 32767; yMax = -32768;
    isCalibrating = true;
}

void CompassQMC5883P::stopCalibration() {
    isCalibrating = false;

    // 1. Tính Offset (Tâm Elip)
    offsetX = (xMax + xMin) / 2.0;
    offsetY = (yMax + yMin) / 2.0;

    // 2. Tính Scale Factor (Nắn Elip thành Hình Tròn)
    float xRange = (xMax - xMin) / 2.0;
    float yRange = (yMax - yMin) / 2.0;
    float avgRange = (xRange + yRange) / 2.0;

    if (xRange > 0) scaleX = avgRange / xRange;
    if (yRange > 0) scaleY = avgRange / yRange;
}

float CompassQMC5883P::readHeading() {
    Wire.beginTransmission(i2cAddr);
    Wire.write(0x01); // Bắt đầu đọc từ thanh ghi dữ liệu 0x01
    Wire.endTransmission(false);

    Wire.requestFrom((uint8_t)i2cAddr, (uint8_t)6);

    if (Wire.available() >= 6) {
        int16_t rawX = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t rawY = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t rawZ = (int16_t)(Wire.read() | (Wire.read() << 8));

        // Nếu đang trong quá trình xoay calib 360 độ
        if (isCalibrating) {
            if (rawX < xMin) xMin = rawX;
            if (rawX > xMax) xMax = rawX;
            if (rawY < yMin) yMin = rawY;
            if (rawY > yMax) yMax = rawY;
        }

        // Nắn chỉnh tọa độ Elip chuẩn hình tròn
        float normX = (rawX - offsetX) * scaleX;
        float normY = (rawY - offsetY) * scaleY;

        // Tính góc chuẩn
        float rawHeading = atan2(normY, normX) * 180.0 / PI;
        if (rawHeading < 0) rawHeading += 360.0;

        // Bộ lọc mượt EMA (80% cũ + 20% mới) chống rung đồ thị
        smoothedHeading = (smoothedHeading * 0.8) + (rawHeading * 0.2);

        return smoothedHeading;
    }

    return smoothedHeading;
}

bool CompassQMC5883P::getCalibratingStatus() {
    return isCalibrating;
}