#ifndef MOTOR_L298N_H
#define MOTOR_L298N_H

#include <Arduino.h>
#include "ShiftRegister595.h" // Thêm thư viện ShiftRegister595

// Định nghĩa các bit trên 74HC595 điều khiển hướng L298N
#define IN1_BIT 10 // Bánh trái tiến
#define IN2_BIT 9// Bánh trái lùi
#define IN3_BIT 12 // Bánh phải tiến
#define IN4_BIT 11 // Bánh phải lùi

class MotorL298N {
private:
    uint8_t pwmLeftPin;
    uint8_t pwmRightPin;
    ShiftRegister595& shiftReg; // Đã sửa từ ShiftRegister16 -> ShiftRegister595

public:
    // Constructor nhận tham chiếu ShiftRegister595
    MotorL298N(uint8_t pwmL, uint8_t pwmR, ShiftRegister595& sr);

    void begin();
    void moveForward(uint8_t speedL, uint8_t speedR);
    void moveBackward(uint8_t speedL, uint8_t speedR);
    void turnLeft(uint8_t speedL, uint8_t speedR);
    void turnRight(uint8_t speedL, uint8_t speedR);
    void stop();
};

#endif