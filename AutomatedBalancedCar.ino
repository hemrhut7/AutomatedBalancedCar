#include <BalanceSystem.h>
#include <Arduino.h>
#include "SensorReader.h"
#include "BluetoothSerial.h"


enum {
  WHEEL,
  BYTE,
  MSG,
  IMU,
  TIME,
  NONE
};

BalanceSystem my_bs;
Nano33Sensor sensor(Serial2);
MotorPID motorL(MOTOR_L_PWM_PIN, MOTOR_L_DIR1_PIN, MOTOR_L_DIR2_PIN, MOTOR_L_DTBY_PIN, MOTOR_L_E1A_PIN, MOTOR_L_E1B_PIN);
MotorPID motorR(MOTOR_R_PWM_PIN, MOTOR_R_DIR1_PIN, MOTOR_R_DIR2_PIN, MOTOR_R_DTBY_PIN, MOTOR_R_E2A_PIN, MOTOR_R_E2B_PIN);
// 藍芽Serial initialization
BluetoothSerial SerialBT;

unsigned long last_time = 0;      // 上次計算時間
const unsigned long interval = 50; // 計算間隔 (ms)

float target_speed = 0;
float target_speed2 = 0.1;
uint8_t read_type = MSG;

float pitch = 0, wx = 0, wz = 0, vel = 0;
bool enable_PID = false;


// 編碼器1中斷處理函數
void IRAM_ATTR handleEncoder1() {
  motorL.encoderISR();
}

// 編碼器2中斷處理函數
void IRAM_ATTR handleEncoder2() {
  motorR.encoderISR();
}


void setup() {
  // 初始化串口
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  SerialBT.begin("ABC_BT"); // Bluetooth 裝置名稱

  // 初始化電機
  motorR.setDirection(-1);
  pinMode(MOTOR_R_DTBY_PIN, OUTPUT);
  digitalWrite(MOTOR_R_DTBY_PIN, HIGH); // 啟動電機
  attachInterrupt(digitalPinToInterrupt(MOTOR_L_E1A_PIN), handleEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_R_E2A_PIN), handleEncoder2, RISING);
  
  // 初始化計時
  last_time = millis();
}

void loop() {
  unsigned long current_time = millis();
  if (Serial2.available()){
    sensor.readData(); 
    if (sensor.isNewData()){
      motorL.updateCurrentSpeed();
      motorR.updateCurrentSpeed();
      vel = (motorL.getSpeed() + motorR.getSpeed()) / 2;

      my_data_3f omg = sensor.getcali_gyro();
      pitch = sensor.getEuler().float_val[0];
      wx = 0.2*omg.float_val[0]+0.8*wx;
      wz = omg.float_val[2];

      if (enable_PID) {
        my_bs.updateState(current_time, vel, pitch, omg.float_val[0], omg.float_val[2]);
        float output_L = my_bs.getOutputLeft();
        float output_R = my_bs.getOutputRight();

        motorL.setTargetSpeed(output_L);
        motorR.setTargetSpeed(output_R);
        
        if (read_type == TIME) {
          Serial.println((current_time - last_time) / 1000.0);
          last_time = current_time;
        }
      }
    }
  }

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "0") {
      digitalWrite(MOTOR_R_DTBY_PIN, LOW); // 停止電機
      motorL.pwm = 0;
      motorR.pwm = 0;
      motorL.setTargetSpeed(0);
      motorR.setTargetSpeed(0);
      my_bs.reset();
      enable_PID = false;
    } 
    else if (command == "1") {
      digitalWrite(MOTOR_R_DTBY_PIN, HIGH); 
      enable_PID = true;
    }
    else if (command == "+")    {      target_speed += 0.01;          } 
    else if (command == "-")    {      target_speed -= 0.01;          } 
    else if (command == "++")   {      target_speed += 0.05;           } 
    else if (command == "--")   {      target_speed -= 0.05;           } 
    else if (command == "a")    {      target_speed = 0;              } 
    else if (command == "b")    {      target_speed = target_speed2;  } 

    else if (command == "byte")   { read_type = BYTE;   } 
    else if (command == "msg")    { read_type = MSG;    } 
    else if (command == "wheel")  { read_type = WHEEL;  } 
    else if (command == "imu1")   { Serial2.print('1'); } 
    else if (command == "imu0")   { Serial2.print('2'); } 
    else if (command == "imu")    { read_type = IMU;    } 
    else if (command == "none")   { read_type = NONE;   }
    else if (command == "time")   {read_type = TIME;    }

    else if (command.startsWith("PID_")) {
      checkPIDSettings(command);
    }
  }

  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    SerialBT.print("Received: ");
    SerialBT.println(command);
    if (command == "0") {
      digitalWrite(MOTOR_R_DTBY_PIN, LOW); // 停止電機
      motorL.pwm = 0;
      motorR.pwm = 0;
      motorL.setTargetSpeed(0);
      motorR.setTargetSpeed(0);
      my_bs.reset();
      enable_PID = false;
    } 
    else if (command == "1") {
      digitalWrite(MOTOR_R_DTBY_PIN, HIGH); 
      enable_PID = true;
    }
    else if (command.startsWith("PID_")) { checkPIDSettings(command); }
  }

  // print current state every 100ms
  if (current_time - last_time >= interval) {
    if (read_type == WHEEL){
      Serial.print(target_speed);
      Serial.print("\t");
      Serial.print(motorL.getSpeed(), 4);
      Serial.print("\t");
      Serial.print(motorR.getSpeed(), 4);
      Serial.print("\t");
      Serial.print(motorL.pwm, 0);
      Serial.print("\t");
      Serial.println(motorR.pwm, 0);
    } 
    
    else if (read_type == IMU){
        my_data_3f euler = sensor.getEuler();
        Serial.print(sensor.getTime());
        Serial.print("\t");
        Serial.print(euler.float_val[0]);
        Serial.print("\t");
        Serial.print(euler.float_val[1]);
        Serial.print("\t");
        Serial.println(euler.float_val[2]);
    }

    else if (read_type == MSG){
      Serial.print(pitch);
      Serial.print("\t");
      Serial.print(wx);
      Serial.print("\t");
      Serial.print(wz);
      Serial.print("\t");
      Serial.print(vel);
      Serial.print("\t");
      Serial.print(my_bs.getOutputLeft());
      Serial.print("\t");
      Serial.println(my_bs.getOutputRight());
    }

    // chagne target speed here
    // motorL.setTargetSpeed(target_speed);
    // motorR.setTargetSpeed(target_speed);

    last_time = current_time;
  }  
}

void checkPIDSettings(String command) {
  String data = command.substring(4);
  
  if (data.length() < 2 || data[1] != '_') {
    Serial.println("Error: Invalid PID format (expected PID_A_XX.XXX_YY.YYY_ZZ.ZZZ)");
    return;
  }

  char pidType = data[0]; // 獲取 PID 類型 (A 或 R)
  String pidValues = data.substring(2); // 獲取 XX.XXX_YY.YYY_ZZ.ZZZ
  float values[3]; // 儲存三個浮點數
  int valueIndex = 0;
  char *ptr = strtok((char *)pidValues.c_str(), "_"); // 以 _ 分割

  while (ptr != nullptr && valueIndex < 3) {
    values[valueIndex] = atof(ptr); // 轉換為浮點數
    valueIndex++;
    ptr = strtok(nullptr, "_");
  }

  // 驗證是否成功解析三個浮點數
  if (valueIndex == 3) {
    if (pidType == 'A') {
      my_bs.setAnglePID(values[0], values[1], values[2]);
    }
    else if (pidType == 'R') {
      my_bs.setRatePID(values[0], values[1], values[2]);
    } 
    else if (pidType == 'M') {
      motorL.setPID(values[0], values[1], values[2]);
      motorR.setPID(values[0], values[1], values[2]);
    } 
    else if (pidType == 'S') {
      my_bs.setSpeedPID(values[0], values[1], values[2]);
    } 
    else if (pidType == 'T0') {
      my_bs.setTurnPID(values[0], values[1], values[2]);
    } 
    else {
      Serial.println("Error: Invalid PID type (expected A or R)");
      return;
    }
    
  } else {
    Serial.println("Error: Invalid PID data");
  }
}