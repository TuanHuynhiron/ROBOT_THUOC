#include <Arduino.h>
#include "BananaPiComm.h"
#include "HCSR04.h"
#include "EncoderDriver.h"
#include "MotorL298N.h"
#include "ShiftRegister16.h"
#include "CompassQMC5883P.h"
#include "SIM7680Module.h"

// Định nghĩa Chân GPIO ESP32 theo Sơ đồ Khối
#define PI_RX_PIN        15  // Pi TX (G17/16) -> ESP RX (G15)
#define PI_TX_PIN        14  // ESP TX (G14) -> Pi RX (G15/14)

#define HC04_TRIG_PIN    5
#define HC04_ECHO_PIN    34

#define ENC_L_A          35
#define ENC_L_B          32
#define ENC_R_A          36
#define ENC_R_B          33

#define MOTOR_PWM_L      26
#define MOTOR_PWM_R      25

#define SR_DATA_PIN      19
#define SR_CLOCK_PIN     18
#define SR_LATCH_PIN     23

#define QMC_SDA_PIN      21
#define QMC_SCL_PIN      22

#define SIM_RX_PIN       14
#define SIM_TX_PIN       13

// Khởi tạo các Đối tượng
BananaPiComm piComm(Serial2, PI_RX_PIN, PI_TX_PIN);
HCSR04 ultrasonic(HC04_TRIG_PIN, HC04_ECHO_PIN);
EncoderDriver encLeft(ENC_L_A, ENC_L_B);
EncoderDriver encRight(ENC_R_A, ENC_R_B);
MotorL298N motors(MOTOR_PWM_L, MOTOR_PWM_R);
ShiftRegister16 shiftReg(SR_DATA_PIN, SR_CLOCK_PIN, SR_LATCH_PIN);
CompassQMC5883P compass(QMC_SDA_PIN, QMC_SCL_PIN);
SIM7680Module sim4g(Serial1, SIM_RX_PIN, SIM_TX_PIN);

// Hàm ngắt cho Encoder
void IRAM_ATTR leftEncoderISR() { encLeft.handleInterrupt(); }
void IRAM_ATTR rightEncoderISR() { encRight.handleInterrupt(); }

void setup() {
    Serial.begin(115200);

    // Khởi tạo các module
    piComm.begin(115200);
    ultrasonic.begin();
    encLeft.begin(leftEncoderISR);
    encRight.begin(rightEncoderISR);
    motors.begin();
    shiftReg.begin();
    compass.begin();
    sim4g.begin(115200);

    // VÍ DỤ MAPPING CHÂN 74HC595 (16 bit Output):
    // Bit 0..3: In1-4 L298N (Hướng bánh chạy)
    // Bit 4..9: Điều khiển 3 motor đẩy khay thuốc qua L293
    // Bit 10, 11: 2 Relay
    // Bit 12: Còi Buzzer
    
    shiftReg.setPin(12, true); // Bật Buzzer kêu ngắn báo hiệu khởi động
    shiftReg.write();
    delay(100);
    shiftReg.setPin(12, false);
    shiftReg.write();
}

void loop() {
    // 1. Nhận lệnh từ Banana Pi
    String command = "";
    if (piComm.readCommand(command)) {
        if (command == "MEDICINE_DRAWER_1") {
            // Điều khiển motor đẩy khay 1 qua 74HC595
            shiftReg.setPin(4, true);  
            shiftReg.setPin(5, false);
            shiftReg.write();
            delay(2000);
            shiftReg.setPin(4, false); // Dừng motor
            shiftReg.write();
        } else if (command.startsWith("SMS:")) {
            // Lệnh gửi tin nhắn SMS: format SMS:090xxxxxxx:Noidung
            int firstColon = command.indexOf(':');
            int secondColon = command.indexOf(':', firstColon + 1);
            String phone = command.substring(firstColon + 1, secondColon);
            String msg = command.substring(secondColon + 1);
            sim4g.sendSMS(phone, msg);
        }
    }

    // 2. Định kỳ gửi dữ liệu cảm biến về Banana Pi để xử lý
    static unsigned long lastSend = 0;
    if (millis() - lastSend > 200) { // Mỗi 200ms
        lastSend = millis();
        float dist = ultrasonic.getDistanceCm();
        float heading = compass.getHeading();
        long encL = encLeft.getCount();
        long encR = encRight.getCount();

        String telemetry = "DATA:" + String(dist) + "," + String(heading) + "," + String(encL) + "," + String(encR);
        piComm.sendData(telemetry);
    }
}