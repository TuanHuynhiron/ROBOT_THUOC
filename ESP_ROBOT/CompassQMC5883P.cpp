#include "CompassQMC5883P.h"

CompassQMC5883P::CompassQMC5883P(uint8_t addr) 
    : i2cAddr(addr), x_min(32767), x_max(-32768), y_min(32767), y_max(-32768),
      offset_x(0), offset_y(0), scale_x(1.0), scale_y(1.0), smoothed_heading(0) {}

void CompassQMC5883P::writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(i2cAddr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void CompassQMC5883P::begin(uint8_t sdaPin, uint8_t sclPin) {
    Wire.begin(sdaPin, sclPin);
    delay(200);

    // Khởi tạo theo đúng thiết lập mẫu
    writeReg(0x0B, 0x80); delay(50); // Soft Reset
    writeReg(0x0B, 0x00); delay(50);
    writeReg(0x0A, 0x1D); delay(50); // Continuous 200Hz, 8G
}

void CompassQMC5883P::calibrate(unsigned long durationMs) {
    x_min = 32767; x_max = -32768;
    y_min = 32767; y_max = -32768;

    Serial.println("==============================================");
    Serial.println(">>> KHỞI ĐỘNG CALIB ELIP <<<");
    Serial.println("HÃY XOAY CẢM BIẾN TRÒN 360 ĐỘ LIÊN TỤC!");
    Serial.println("==============================================");

    unsigned long startTime = millis();
    while (millis() - startTime < durationMs) {
        Wire.beginTransmission(i2cAddr);
        Wire.write(0x01);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)i2cAddr, (uint8_t)6);

        if (Wire.available() >= 6) {
            int16_t x = (int16_t)(Wire.read() | (Wire.read() << 8));
            int16_t y = (int16_t)(Wire.read() | (Wire.read() << 8));
            int16_t z = (int16_t)(Wire.read() | (Wire.read() << 8));

            if (x < x_min) x_min = x;
            if (x > x_max) x_max = x;
            if (y < y_min) y_min = y;
            if (y > y_max) y_max = y;
        }
        delay(30);
    }

    // Tính toán Offset & Scale Factor
    offset_x = (x_max + x_min) / 2.0;
    offset_y = (y_max + y_min) / 2.0;

    float x_range = (x_max - x_min) / 2.0;
    float y_range = (y_max - y_min) / 2.0;
    float avg_range = (x_range + y_range) / 2.0;

    if (x_range != 0) scale_x = avg_range / x_range;
    if (y_range != 0) scale_y = avg_range / y_range;

    Serial.println("=== CALIB HOÀN TẤT ===");
    Serial.print("Offset X: "); Serial.print(offset_x); Serial.print(" | Scale X: "); Serial.println(scale_x);
    Serial.print("Offset Y: "); Serial.print(offset_y); Serial.print(" | Scale Y: "); Serial.println(scale_y);
}

float CompassQMC5883P::getHeading() {
    Wire.beginTransmission(i2cAddr);
    Wire.write(0x01);
    Wire.endTransmission(false);

    Wire.requestFrom((uint8_t)i2cAddr, (uint8_t)6);

    if (Wire.available() >= 6) {
        int16_t raw_x = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t raw_y = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t raw_z = (int16_t)(Wire.read() | (Wire.read() << 8));

        // Nắn chỉnh tọa độ
        float norm_x = (raw_x - offset_x) * scale_x;
        float norm_y = (raw_y - offset_y) * scale_y;

        // Tính góc
        float raw_heading = atan2(norm_y, norm_x) * 180.0 / PI;
        if (raw_heading < 0) raw_heading += 360.0;

        // Bộ lọc mượt EMA (Alpha = 0.2)
        smoothed_heading = (smoothed_heading * 0.8) + (raw_heading * 0.2);
    }
    return smoothed_heading;
}