#ifndef MOTOR_L298N_H
#define MOTOR_L298N_H

#include <Arduino.h>
#include "ShiftRegister16.h"

// Giả định gán 4 chân IN1-4 của L298N bánh di chuyển vào 4 bit đầu (0..3) của 74HC595
#define IN1_BIT 0  // Bánh trái tiến
#define IN2_BIT 1  // Bánh trái lùi
#define IN3_BIT 2  // Bánh phải tiến
#define IN4_BIT 3  // Bánh phải lùi

class MotorL298N {
private:
    uint8_t pwmLeftPin;
    uint8_t pwmRightPin;
    ShiftRegister16& shiftReg;

public:
    MotorL298N(uint8_t pwmL, uint8_t pwmR, ShiftRegister16& sr);
    void begin();
    
    // Các hàm điều khiển hướng di chuyển
    void moveForward(uint8_t speedL, uint8_t speedR);
    void moveBackward(uint8_t speedL, uint8_t speedR);
    void turnLeft(uint8_t speedL, uint8_t speedR);  // Xoay tại chỗ hoặc rẽ trái
    void turnRight(uint8_t speedL, uint8_t speedR); // Xoay tại chỗ hoặc rẽ phải
    void stop();
};

#endif