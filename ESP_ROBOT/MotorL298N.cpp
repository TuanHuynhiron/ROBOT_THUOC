#include "MotorL298N.h"

MotorL298N::MotorL298N(uint8_t pwmL, uint8_t pwmR, ShiftRegister16& sr) 
    : pwmLeftPin(pwmL), pwmRightPin(pwmR), shiftReg(sr) {}

void MotorL298N::begin() {
    ledcAttach(pwmLeftPin, 5000, 8);  // Tần số 5kHz, độ phân giải 8-bit (0-255)
    ledcAttach(pwmRightPin, 5000, 8);
    stop();
}

// Đi thẳng (Cả 2 bánh cùng tiến)
void MotorL298N::moveForward(uint8_t speedL, uint8_t speedR) {
    shiftReg.setPin(IN1_BIT, true);
    shiftReg.setPin(IN2_BIT, false);
    shiftReg.setPin(IN3_BIT, true);
    shiftReg.setPin(IN4_BIT, false);
    shiftReg.write();

    ledcWrite(pwmLeftPin, speedL);
    ledcWrite(pwmRightPin, speedR);
}

// Đi lùi (Cả 2 bánh cùng lùi)
void MotorL298N::moveBackward(uint8_t speedL, uint8_t speedR) {
    shiftReg.setPin(IN1_BIT, false);
    shiftReg.setPin(IN2_BIT, true);
    shiftReg.setPin(IN3_BIT, false);
    shiftReg.setPin(IN4_BIT, true);
    shiftReg.write();

    ledcWrite(pwmLeftPin, speedL);
    ledcWrite(pwmRightPin, speedR);
}

// Rẽ trái (Bánh trái lùi/dừng, bánh phải tiến)
void MotorL298N::turnLeft(uint8_t speedL, uint8_t speedR) {
    shiftReg.setPin(IN1_BIT, false);
    shiftReg.setPin(IN2_BIT, true);  // Bánh trái lùi
    shiftReg.setPin(IN3_BIT, true);  // Bánh phải tiến
    shiftReg.setPin(IN4_BIT, false);
    shiftReg.write();

    ledcWrite(pwmLeftPin, speedL);
    ledcWrite(pwmRightPin, speedR);
}

// Rẽ phải (Bánh trái tiến, bánh phải lùi/dừng)
void MotorL298N::turnRight(uint8_t speedL, uint8_t speedR) {
    shiftReg.setPin(IN1_BIT, true);   // Bánh trái tiến
    shiftReg.setPin(IN2_BIT, false);
    shiftReg.setPin(IN3_BIT, false);
    shiftReg.setPin(IN4_BIT, true);  // Bánh phải lùi
    shiftReg.write();

    ledcWrite(pwmLeftPin, speedL);
    ledcWrite(pwmRightPin, speedR);
}

// Dừng robot
void MotorL298N::stop() {
    shiftReg.setPin(IN1_BIT, false);
    shiftReg.setPin(IN2_BIT, false);
    shiftReg.setPin(IN3_BIT, false);
    shiftReg.setPin(IN4_BIT, false);
    shiftReg.write();

    ledcWrite(pwmLeftPin, 0);
    ledcWrite(pwmRightPin, 0);
}