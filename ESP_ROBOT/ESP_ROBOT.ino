#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>

#include "ShiftRegister595.h"
#include "MotorL298N.h"
#include "EncoderDriver.h"
#include "CompassQMC5883P.h"
#include "HCSR04.h"

// Số xung trên 1 vòng quay bánh xe
#define ENCODER_PULSES_PER_REV  495.0 

// ===================================================================
// BỘ LỌC KALMAN 1D CHO LA BÀN
// ===================================================================
class KalmanFilterAngle {
private:
  float err_measure;
  float err_estimate;
  float q;
  float current_estimate;
  float last_estimate;
  bool initialized;

public:
  KalmanFilterAngle(float mea_e, float est_e, float q_val) {
    err_measure = mea_e;
    err_estimate = est_e;
    q = q_val;
    current_estimate = 0;
    last_estimate = 0;
    initialized = false;
  }

  float updateEstimate(float mea) {
    if (!initialized) {
      current_estimate = mea;
      last_estimate = mea;
      initialized = true;
      return current_estimate;
    }

    float diff = mea - last_estimate;
    if (diff < -180.0) diff += 360.0;
    else if (diff > 180.0) diff -= 360.0;

    float adjusted_mea = last_estimate + diff;

    float kalman_gain = err_estimate / (err_estimate + err_measure);
    current_estimate = last_estimate + kalman_gain * (adjusted_mea - last_estimate);
    err_estimate = (1.0 - kalman_gain) * err_estimate + fabs(last_estimate - current_estimate) * q;
    
    while (current_estimate >= 360.0) current_estimate -= 360.0;
    while (current_estimate < 0.0) current_estimate += 360.0;

    last_estimate = current_estimate;
    return current_estimate;
  }
};

KalmanFilterAngle kalmanCompass(2.0, 2.0, 0.05);

// ===================================================================
// [PHẦN 1] BẢNG BIT 74HC595 (16 BIT CASCADE)
// ===================================================================
#define BIT_MED1_IN1   0  
#define BIT_MED1_IN2   1  
#define BIT_MED2_IN1   2  
#define BIT_MED2_IN2   3  
#define BIT_MED3_IN1   4  
#define BIT_MED3_IN2   5  

#define BIT_RELAY_1    6  
#define BIT_RELAY_2    7  
#define BIT_BUZZER     8  

#define BIT_L298_IN1   9  
#define BIT_L298_IN2   10 
#define BIT_L298_IN3   11 
#define BIT_L298_IN4   12 

// ===================================================================
// [PHẦN 2] CẤU HÌNH CHÂN ESP32
// ===================================================================
#define PIN_595_DATA   23
#define PIN_595_CLOCK  19
#define PIN_595_LATCH  18

#define PIN_MOTOR_PWM_L 25
#define PIN_MOTOR_PWM_R 26

#define PIN_ENC_L_A    36  
#define PIN_ENC_L_B    33
#define PIN_ENC_R_A    35  
#define PIN_ENC_R_B    32

#define PIN_US_TRIG    5
#define PIN_US_ECHO    17

#define PIN_I2C_SDA    21
#define PIN_I2C_SCL    22

#define PIN_BTN_CALIB  4

// ===================================================================
// [PHẦN 3] KHỞI TẠO ĐỐI TƯỢNG & BIẾN PID / TỐC ĐỘ / XOAY GÓC
// ===================================================================
ShiftRegister595  shiftReg(PIN_595_DATA, PIN_595_CLOCK, PIN_595_LATCH);
MotorL298N        motors(PIN_MOTOR_PWM_L, PIN_MOTOR_PWM_R, shiftReg);
EncoderDriver     encoderLeft(PIN_ENC_L_A, PIN_ENC_L_B);
EncoderDriver     encoderRight(PIN_ENC_R_A, PIN_ENC_R_B);
CompassQMC5883P   compass;
HCSR04            ultrasonic(PIN_US_TRIG, PIN_US_ECHO);
Preferences       flashMem;
WebServer         server(80);

const char* ssid     = "pc";
const char* password = "1234567890";

float currentHeadingRaw    = 0.0;
float currentHeadingKalman = 0.0;
float distanceCM           = 0.0;

// Biến hỗ trợ xoay đúng 90 độ
float startTurnHeading = 0.0;
float lastTurnHeading  = 0.0;
float accumulatedTurn  = 0.0;

// Biến tính tốc độ RPM
float speedRPM_L = 0.0;
float speedRPM_R = 0.0;
long lastEncCountL = 0;
long lastEncCountR = 0;

// Tham số PID cho đi thẳng
float Kp = 1.5;
float Ki = 0.05;
float Kd = 0.2;

float pidError = 0;
float pidIntegral = 0;
float pidLastError = 0;
float pidOutput = 0;

// Trạng thái xe
String currentDir = "stop";
int baseSpeed = 160;

// Trạng thái Relay / Buzzer / Door
bool stateRelay1 = false;
bool stateRelay2 = false;
bool stateBuzzer = false;
int stateDoor[3] = {0, 0, 0};

bool isCalibrating         = false;
unsigned long calibStartMs = 0;
int calibCountdown         = 10;

void IRAM_ATTR isrLeftEncoder()  { encoderLeft.handleInterrupt(); }
void IRAM_ATTR isrRightEncoder() { encoderRight.handleInterrupt(); }

TaskHandle_t WebServerTask;

// Hàm kích hoạt chế độ rẽ
void startTurnMode(String dir) {
  currentDir = dir;
  startTurnHeading = currentHeadingKalman;
  lastTurnHeading  = currentHeadingKalman;
  accumulatedTurn  = 0.0;
}

// ===================================================================
// [PHẦN 4] ĐIỀU KHIỂN ĐỘNG CƠ TÍCH HỢP PID & KIỂM SOÁT XOAY 90 DEG
// ===================================================================
void applyPIDControl() {
  if (currentDir == "forward" || currentDir == "backward") {
    pidError = speedRPM_L - speedRPM_R;
    pidIntegral += pidError;
    pidIntegral = constrain(pidIntegral, -100, 100);
    
    float derivative = pidError - pidLastError;
    pidLastError = pidError;

    pidOutput = (Kp * pidError) + (Ki * pidIntegral) + (Kd * derivative);

    int pwmL = constrain(baseSpeed - pidOutput, 0, 255);
    int pwmR = constrain(baseSpeed + pidOutput, 0, 255);

    if (currentDir == "forward") {
      motors.moveForward(pwmL, pwmR);
    } else if (currentDir == "backward") {
      motors.moveBackward(pwmL, pwmR);
    }
  } else if (currentDir == "turn_left") {
    // Đã đảo ngược chức năng động cơ theo yêu cầu
    motors.turnRight(baseSpeed, baseSpeed);
    pidIntegral = 0; pidLastError = 0;
  } else if (currentDir == "turn_right") {
    // Đã đảo ngược chức năng động cơ theo yêu cầu
    motors.turnLeft(baseSpeed, baseSpeed);
    pidIntegral = 0; pidLastError = 0;
  } else {
    motors.stop();
    pidIntegral = 0; pidLastError = 0;
  }
}

void checkTurnAngle90() {
  if (currentDir == "turn_left" || currentDir == "turn_right") {
    float diff = currentHeadingKalman - lastTurnHeading;
    
    // Xử lý lệch góc khi quay qua 0/360
    if (diff > 180.0) diff -= 360.0;
    if (diff < -180.0) diff += 360.0;
    
    accumulatedTurn += fabs(diff);
    lastTurnHeading = currentHeadingKalman;

    // Ngắt chuyển động khi đủ 90 độ
    if (accumulatedTurn >= 90.0) {
      currentDir = "stop";
      motors.stop();
    }
  }
}

void controlMedicineDoor(uint8_t compartment, int action) {
  uint8_t bitIn1, bitIn2;
  if (compartment == 1)      { bitIn1 = BIT_MED1_IN1; bitIn2 = BIT_MED1_IN2; }
  else if (compartment == 2) { bitIn1 = BIT_MED2_IN1; bitIn2 = BIT_MED2_IN2; }
  else if (compartment == 3) { bitIn1 = BIT_MED3_IN1; bitIn2 = BIT_MED3_IN2; }
  else return;

  if (action == 1) {        
    shiftReg.setPin(bitIn1, HIGH); shiftReg.setPin(bitIn2, LOW);
  } else if (action == -1) { 
    shiftReg.setPin(bitIn1, LOW);  shiftReg.setPin(bitIn2, HIGH);
  } else {                  
    shiftReg.setPin(bitIn1, LOW);  shiftReg.setPin(bitIn2, LOW);
  }
  shiftReg.write();
}

void setRelay(uint8_t relayNum, bool enable) {
  if (relayNum == 1) shiftReg.setPin(BIT_RELAY_1, enable ? HIGH : LOW);
  if (relayNum == 2) shiftReg.setPin(BIT_RELAY_2, enable ? HIGH : LOW);
  shiftReg.write();
}

void setBuzzer(bool enable) {
  shiftReg.setPin(BIT_BUZZER, enable ? HIGH : LOW);
  shiftReg.write();
}

// ===================================================================
// [PHẦN 5] XỬ LÝ CALIB LA BÀN
// ===================================================================
void startCalibrationProcess() {
  if (isCalibrating) return;
  isCalibrating = true;
  calibStartMs = millis();
  calibCountdown = 10;
  compass.startCalibration();
}

void handleCalibrationTask() {
  if (!isCalibrating) return;

  unsigned long elapsedSec = (millis() - calibStartMs) / 1000;
  calibCountdown = 10 - elapsedSec;

  compass.readHeading();

  if (elapsedSec >= 10) {
    compass.stopCalibration();
    isCalibrating = false;
    setBuzzer(true); delay(200); setBuzzer(false);

    flashMem.begin("compass", false);
    flashMem.putBool("calib_done", true);
    flashMem.end();
  }
}

// ===================================================================
// [PHẦN 6] GIAO DIỆN WEB DASHBOARD HTML
// ===================================================================
const char HTML_WEB_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32 Robot Dashboard</title>
  <style>
    :root { --bg: #0d1117; --card: #161b22; --accent: #58a6ff; --green: #2ea043; --red: #da3633; --border: #30363d; --text: #c9d1d9; --orange: #d97706; }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: system-ui, sans-serif; user-select: none; }
    body { background: var(--bg); color: var(--text); padding: 15px; }
    .header { text-align: center; padding-bottom: 10px; border-bottom: 1px solid var(--border); margin-bottom: 15px; }
    .header h2 { color: var(--accent); }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 15px; }
    .card { background: var(--card); border: 1px solid var(--border); border-radius: 10px; padding: 15px; }
    .card h3 { font-size: 15px; margin-bottom: 10px; border-bottom: 1px solid var(--border); padding-bottom: 5px; color: #fff; }
    .sensor-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; }
    .sensor-box { background: #0d1117; border: 1px solid var(--border); border-radius: 6px; padding: 10px; text-align: center; }
    .sensor-box .val { font-size: 20px; font-weight: bold; color: var(--accent); margin-top: 4px; }
    
    .btn { 
      background: #21262d; border: 2px solid var(--border); color: #fff; padding: 10px; border-radius: 6px; 
      cursor: pointer; font-weight: bold; font-size: 12px; outline: none; touch-action: manipulation;
      transition: background 0.15s, border-color 0.15s;
    }
    .btn-green { background: var(--green) !important; border-color: #3fb950 !important; }
    .btn-red { background: var(--red) !important; border-color: #f85149 !important; }
    .btn-orange { background: var(--orange) !important; border-color: #f59e0b !important; }
    .btn-active { background: #1f6feb !important; border-color: #58a6ff !important; box-shadow: 0 0 8px rgba(88,166,255,0.6); }

    .btn-group { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 6px; }
    .btn-group .btn { flex: 1; min-width: 80px; }
    .dpad { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; max-width: 220px; margin: 0 auto; }
    
    .pid-inputs { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin-top: 8px; }
    .pid-inputs input { background: #0d1117; border: 1px solid var(--border); color: #fff; padding: 8px; border-radius: 4px; text-align: center; width: 100%; font-weight: bold; }

    .compass-container { display: flex; flex-direction: column; align-items: center; justify-content: center; margin-bottom: 12px; }
    .compass-disc {
      width: 110px; height: 110px; border-radius: 50%; border: 3px solid var(--accent);
      background: radial-gradient(circle, #161b22 55%, #0d1117 100%);
      position: relative; transition: transform 0.25s cubic-bezier(0.1, 0.9, 0.2, 1);
      box-shadow: 0 0 12px rgba(88,166,255,0.25);
    }
    .compass-mark { position: absolute; font-size: 11px; font-weight: bold; }
    .mark-n { top: 4px; left: 48px; color: var(--red); }
    .mark-s { bottom: 4px; left: 49px; color: var(--text); }
    .mark-e { top: 47px; right: 6px; color: var(--text); }
    .mark-w { top: 47px; left: 6px; color: var(--text); }
    .compass-pointer {
      position: absolute; top: 18px; left: 51px; width: 8px; height: 74px;
    }
    .compass-pointer::before {
      content: ''; position: absolute; top: 0; left: 0;
      width: 0; height: 0;
      border-left: 4px solid transparent; border-right: 4px solid transparent; border-bottom: 37px solid var(--red);
    }
    .compass-pointer::after {
      content: ''; position: absolute; bottom: 0; left: 0;
      width: 0; height: 0;
      border-left: 4px solid transparent; border-right: 4px solid transparent; border-top: 37px solid #c9d1d9;
    }
  </style>
</head>
<body>
  <div class="header">
    <h2>🤖 ESP32 ROBOT DASHBOARD</h2>
  </div>

  <div class="grid">
    <div class="card">
      <h3>📊 THÔNG SỐ CẢM BIẾN & TỐC ĐỘ</h3>
      
      <div class="compass-container">
        <div class="compass-disc" id="compassDisc">
          <span class="compass-mark mark-n">N</span>
          <span class="compass-mark mark-e">E</span>
          <span class="compass-mark mark-s">S</span>
          <span class="compass-mark mark-w">W</span>
          <div class="compass-pointer"></div>
        </div>
        <div style="margin-top:6px; font-size:14px; font-weight:bold; color:var(--accent);">
          🧭 Kalman: <span id="valHeading" style="color:#fff;">0.0°</span>
        </div>
      </div>

      <div class="sensor-grid">
        <div class="sensor-box" style="grid-column: span 2;">
          <div style="font-size:11px; color:#8b949e;">📏 Khoảng Cách Siêu Âm</div>
          <div class="val" id="valDist">0.0 cm</div>
        </div>
        <div class="sensor-box">
          <div style="font-size:11px; color:#8b949e;">⚡ Tốc Độ Trái</div>
          <div class="val" id="valRpmL">0 RPM</div>
        </div>
        <div class="sensor-box">
          <div style="font-size:11px; color:#8b949e;">⚡ Tốc Độ Phải</div>
          <div class="val" id="valRpmR">0 RPM</div>
        </div>
      </div>
      <button class="btn btn-orange" style="width:100%; margin-top:10px;" onclick="send('/api/calib')">🔄 Calib La Bàn (10s)</button>
      <div id="calibStatus" style="text-align:center; color:var(--orange); font-size:12px; margin-top:5px;"></div>
    </div>

    <div class="card">
      <h3>🚗 ĐIỀU KHIỂN BÁNH XE (TÍCH HỢP PID)</h3>
      <div class="dpad">
        <div></div>
        <button class="btn btn-green" onclick="send('/api/move?dir=forward')">▲ TIẾN</button>
        <div></div>
        <button class="btn" onclick="send('/api/move?dir=turn_left')">↰ TRÁI (90°)</button>
        <button class="btn btn-red" onclick="send('/api/move?dir=stop')">🛑 DỪNG</button>
        <button class="btn" onclick="send('/api/move?dir=turn_right')">↱ PHẢI (90°)</button>
        <div></div>
        <button class="btn btn-green" onclick="send('/api/move?dir=backward')">▼ LÙI</button>
        <div></div>
      </div>

      <div style="margin-top: 15px; border-top: 1px solid var(--border); padding-top: 10px;">
        <label style="font-size:12px; font-weight:bold; color:var(--accent);">⚙️ Cấu Hình PID Cân Bằng 2 Bánh:</label>
        <div class="pid-inputs">
          <div><span style="font-size:10px;">Kp:</span><input type="number" step="0.1" id="inpKp" value="1.5"></div>
          <div><span style="font-size:10px;">Ki:</span><input type="number" step="0.01" id="inpKi" value="0.05"></div>
          <div><span style="font-size:10px;">Kd:</span><input type="number" step="0.05" id="inpKd" value="0.2"></div>
        </div>
        <button class="btn btn-green" style="width:100%; margin-top:8px;" onclick="updatePID()">💾 Cập Nhật Tham Số PID</button>
      </div>
    </div>

    <div class="card">
      <h3>💊 QUẢN LÝ 3 NGĂN THUỐC (TOGGLE)</h3>
      
      <div style="margin-bottom:10px;">
        <label style="font-size:12px; font-weight:bold;">Ngăn 1:</label>
        <div class="btn-group">
          <button class="btn" id="d1_1" onclick="send('/api/door?id=1&act=1')">Đẩy Ra</button>
          <button class="btn" id="d1_0" onclick="send('/api/door?id=1&act=0')">Dừng</button>
          <button class="btn" id="d1_-1" onclick="send('/api/door?id=1&act=-1')">Thu Về</button>
        </div>
      </div>

      <div style="margin-bottom:10px;">
        <label style="font-size:12px; font-weight:bold;">Ngăn 2:</label>
        <div class="btn-group">
          <button class="btn" id="d2_1" onclick="send('/api/door?id=2&act=1')">Đẩy Ra</button>
          <button class="btn" id="d2_0" onclick="send('/api/door?id=2&act=0')">Dừng</button>
          <button class="btn" id="d2_-1" onclick="send('/api/door?id=2&act=-1')">Thu Về</button>
        </div>
      </div>

      <div>
        <label style="font-size:12px; font-weight:bold;">Ngăn 3:</label>
        <div class="btn-group">
          <button class="btn" id="d3_1" onclick="send('/api/door?id=3&act=1')">Đẩy Ra</button>
          <button class="btn" id="d3_0" onclick="send('/api/door?id=3&act=0')">Dừng</button>
          <button class="btn" id="d3_-1" onclick="send('/api/door?id=3&act=-1')">Thu Về</button>
        </div>
      </div>
    </div>

    <div class="card">
      <h3>🔌 RELAY & BUZZER</h3>
      <div style="margin-bottom:10px;">
        <button class="btn" id="btnRelay1" style="width:100%;" onclick="send('/api/relay?id=1&st=toggle')">RELAY PI/M: OFF</button>
      </div>
      <div style="margin-bottom:10px;">
        <button class="btn" id="btnRelay2" style="width:100%;" onclick="send('/api/relay?id=2&st=toggle')">RELAY SV: OFF</button>
      </div>
      <div style="margin-top:10px;">
        <button class="btn" id="btnBuzzer" style="width:100%;" onclick="send('/api/buzzer?st=toggle')">🔔 CÒI BUZZER: OFF</button>
      </div>
    </div>
  </div>

  <script>
    let isFetching = false;
    let currentCompassAngle = 0;

    function send(url) { 
      fetch(url, { cache: 'no-store' }).then(() => {
        setTimeout(updateSensors, 60); 
      });
    }

    function updatePID() {
      let p = document.getElementById('inpKp').value;
      let i = document.getElementById('inpKi').value;
      let d = document.getElementById('inpKd').value;
      send('/api/pid?p=' + p + '&i=' + i + '&d=' + d);
      alert('Đã gửi cấu hình PID mới xuống ESP32!');
    }

    function renderDoorUI(id, state) {
      const btnOut = document.getElementById(`d${id}_1`);
      const btnStop = document.getElementById(`d${id}_0`);
      const btnIn = document.getElementById(`d${id}_-1`);

      if(btnOut && btnStop && btnIn) {
        btnOut.className = 'btn ' + (state === 1 ? 'btn-green' : '');
        btnStop.className = 'btn ' + (state === 0 ? 'btn-red' : '');
        btnIn.className = 'btn ' + (state === -1 ? 'btn-orange' : '');
      }
    }

    function updateSensors() {
      if (isFetching) return;
      isFetching = true;

      fetch('/api/sensors', { cache: 'no-store' })
        .then(res => res.json())
        .then(data => {
          let deg = data.heading;
          document.getElementById('valHeading').innerText = deg.toFixed(1) + '°';

          let disc = document.getElementById('compassDisc');
          if(disc) {
            let diff = deg - (currentCompassAngle % 360);
            if (diff < -180) diff += 360;
            if (diff > 180) diff -= 360;
            currentCompassAngle += diff;
            disc.style.transform = `rotate(${-currentCompassAngle}deg)`;
          }

          document.getElementById('valDist').innerText = data.dist.toFixed(1) + ' cm';
          document.getElementById('valRpmL').innerText = Math.round(data.rpmL) + ' RPM';
          document.getElementById('valRpmR').innerText = Math.round(data.rpmR) + ' RPM';
          
          const r1 = document.getElementById('btnRelay1');
          r1.innerText = 'RELAY PI/M: ' + (data.r1 ? 'ON' : 'OFF');
          r1.className = 'btn ' + (data.r1 ? 'btn-active' : '');

          const r2 = document.getElementById('btnRelay2');
          r2.innerText = 'RELAY SV: ' + (data.r2 ? 'ON' : 'OFF');
          r2.className = 'btn ' + (data.r2 ? 'btn-active' : '');

          const bz = document.getElementById('btnBuzzer');
          bz.innerText = '🔔 CÒI BUZZER: ' + (data.bz ? 'ON' : 'OFF');
          bz.className = 'btn ' + (data.bz ? 'btn-orange' : '');

          if(data.isCalib) {
            document.getElementById('calibStatus').innerText = 'Đang Calib... Còn: ' + data.countdown + 's';
          } else {
            document.getElementById('calibStatus').innerText = '';
          }

          if(data.doors) {
            renderDoorUI(1, data.doors[0]);
            renderDoorUI(2, data.doors[1]);
            renderDoorUI(3, data.doors[2]);
          }
        })
        .catch(e => {})
        .finally(() => { isFetching = false; });
    }

    setInterval(updateSensors, 200);
  </script>
</body>
</html>
)rawliteral";

// ===================================================================
// [PHẦN 7] ROUTING SERVER & DUAL CORE TASK
// ===================================================================
void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", HTML_WEB_PAGE);
  });

  server.on("/api/sensors", HTTP_GET, []() {
    String json = "{";
    json += "\"heading\":" + String(currentHeadingKalman) + ",";
    json += "\"dist\":" + String(distanceCM) + ",";
    json += "\"rpmL\":" + String(speedRPM_L) + ",";
    json += "\"rpmR\":" + String(speedRPM_R) + ",";
    json += "\"r1\":" + String(stateRelay1 ? "true" : "false") + ",";
    json += "\"r2\":" + String(stateRelay2 ? "true" : "false") + ",";
    json += "\"bz\":" + String(stateBuzzer ? "true" : "false") + ",";
    json += "\"isCalib\":" + String(isCalibrating ? "true" : "false") + ",";
    json += "\"countdown\":" + String(calibCountdown) + ",";
    json += "\"doors\":[" + String(stateDoor[0]) + "," + String(stateDoor[1]) + "," + String(stateDoor[2]) + "]";
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/api/move", HTTP_GET, []() {
    String dir = server.arg("dir");
    if (dir == "turn_left" || dir == "turn_right") {
      startTurnMode(dir);
    } else {
      currentDir = dir;
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/pid", HTTP_GET, []() {
    if (server.hasArg("p")) Kp = server.arg("p").toFloat();
    if (server.hasArg("i")) Ki = server.arg("i").toFloat();
    if (server.hasArg("d")) Kd = server.arg("d").toFloat();
    pidIntegral = 0;
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/door", HTTP_GET, []() {
    int id = server.arg("id").toInt();
    int act = server.arg("act").toInt();
    if (id >= 1 && id <= 3) {
      stateDoor[id - 1] = act;
      controlMedicineDoor(id, act);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/relay", HTTP_GET, []() {
    int id = server.arg("id").toInt();
    String st = server.arg("st");
    if (id == 1) {
      stateRelay1 = (st == "toggle") ? !stateRelay1 : st.toInt();
      setRelay(1, stateRelay1);
    } else if (id == 2) {
      stateRelay2 = (st == "toggle") ? !stateRelay2 : st.toInt();
      setRelay(2, stateRelay2);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/buzzer", HTTP_GET, []() {
    String st = server.arg("st");
    stateBuzzer = (st == "toggle") ? !stateBuzzer : st.toInt();
    setBuzzer(stateBuzzer);
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/calib", HTTP_GET, []() {
    startCalibrationProcess();
    server.send(200, "text/plain", "OK");
  });

  server.begin();
}

void TaskWebServerCode(void * pvParameters) {
  for(;;) {
    server.handleClient();
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

// ===================================================================
// [PHẦN 8] SETUP & LOOP CHÍNH
// ===================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_BTN_CALIB, INPUT_PULLUP);

  pinMode(PIN_ENC_L_A, INPUT);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  shiftReg.begin();
  motors.begin();

  encoderLeft.begin(isrLeftEncoder);
  encoderRight.begin(isrRightEncoder);

  ultrasonic.begin();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  compass.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(300); }

  setupWebServer();

  xTaskCreatePinnedToCore(
    TaskWebServerCode, 
    "WebServerTask",   
    8192,              
    NULL,              
    1,                 
    &WebServerTask,    
    0                  
  );

  setBuzzer(true); delay(150); setBuzzer(false);
}

void loop() {
  // 1. VÒNG LẶP PID & TỐC ĐỘ (MỖI 30MS)
  static unsigned long lastFastLoopMs = 0;
  if (millis() - lastFastLoopMs >= 30) {
    unsigned long dt = millis() - lastFastLoopMs;
    lastFastLoopMs = millis();

    long currEncL = encoderLeft.getCount();
    long currEncR = encoderRight.getCount();

    long dL = currEncL - lastEncCountL;
    long dR = currEncR - lastEncCountR;

    lastEncCountL = currEncL;
    lastEncCountR = currEncR;

    speedRPM_L = abs((float)dL / ENCODER_PULSES_PER_REV) * (60000.0 / dt);
    speedRPM_R = abs((float)dR / ENCODER_PULSES_PER_REV) * (60000.0 / dt);

    applyPIDControl();
  }

  // 2. VÒNG LẶP ĐỌC CẢM BIẾN, LỌC KALMAN & KIỂM TRA TỰ DỪNG GÓC (MỖI 80MS)
  static unsigned long lastSensorLoopMs = 0;
  if (millis() - lastSensorLoopMs >= 80) {
    lastSensorLoopMs = millis();

    currentHeadingRaw    = compass.readHeading();
    currentHeadingKalman = kalmanCompass.updateEstimate(currentHeadingRaw);

    // Kiểm tra góc đã xoay để tự dừng khi đủ 90 độ
    checkTurnAngle90();

    distanceCM           = ultrasonic.getDistanceCm();

    if (distanceCM > 2.0 && distanceCM < 15.0) {
      currentDir = "stop";
      motors.stop();
    }
  }

  handleCalibrationTask();

  if (digitalRead(PIN_BTN_CALIB) == LOW && !isCalibrating) {
    delay(50);
    if (digitalRead(PIN_BTN_CALIB) == LOW) {
      startCalibrationProcess();
    }
  }
}