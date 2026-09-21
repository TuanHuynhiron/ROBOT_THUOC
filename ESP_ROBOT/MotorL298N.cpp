#include "MotorL298N.h"

// Constructor gán chính xác kiểu ShiftRegister595
MotorL298N::MotorL298N(uint8_t pwmL, uint8_t pwmR, ShiftRegister595& sr)
    : pwmLeftPin(pwmL), pwmRightPin(pwmR), shiftReg(sr) {}

void MotorL298N::begin() {
    pinMode(pwmLeftPin, OUTPUT);
    pinMode(pwmRightPin, OUTPUT);
    stop();
}

void MotorL298N::moveForward(uint8_t speedL, uint8_t speedR) {
    shiftReg.setPin(IN1_BIT, HIGH);
    shiftReg.setPin(IN2_BIT, LOW);
    shiftReg.setPin(IN3_BIT, HIGH);
    shiftReg.setPin(IN4_BIT, LOW);
    shiftReg.write();

    analogWrite(pwmLeftPin, speedL);
    analogWrite(pwmRightPin, speedR);
}

void MotorL298N::moveBackward(uint8_t speedL, uint8_t speedR) {
    shiftReg.setPin(IN1_BIT, LOW);
    shiftReg.setPin(IN2_BIT, HIGH);
    shiftReg.setPin(IN3_BIT, LOW);
    shiftReg.setPin(IN4_BIT, HIGH);
    shiftReg.write();

    analogWrite(pwmLeftPin, speedL);
    analogWrite(pwmRightPin, speedR);
}

void MotorL298N::turnLeft(uint8_t speedL, uint8_t speedR) {
    shiftReg.setPin(IN1_BIT, LOW);
    shiftReg.setPin(IN2_BIT, HIGH);
    shiftReg.setPin(IN3_BIT, HIGH);
    shiftReg.setPin(IN4_BIT, LOW);
    shiftReg.write();

    analogWrite(pwmLeftPin, speedL);
    analogWrite(pwmRightPin, speedR);
}

void MotorL298N::turnRight(uint8_t speedL, uint8_t speedR) {
    shiftReg.setPin(IN1_BIT, HIGH);
    shiftReg.setPin(IN2_BIT, LOW);
    shiftReg.setPin(IN3_BIT, LOW);
    shiftReg.setPin(IN4_BIT, HIGH);
    shiftReg.write();

    analogWrite(pwmLeftPin, speedL);
    analogWrite(pwmRightPin, speedR);
}

void MotorL298N::stop() {
    shiftReg.setPin(IN1_BIT, LOW);
    shiftReg.setPin(IN2_BIT, LOW);
    shiftReg.setPin(IN3_BIT, LOW);
    shiftReg.setPin(IN4_BIT, LOW);
    shiftReg.write();

    analogWrite(pwmLeftPin, 0);
    analogWrite(pwmRightPin, 0);
}