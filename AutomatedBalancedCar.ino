#include <Arduino.h>
#include "BluetoothSerial.h"
#include <driver/pcnt.h>
#include "src/BalanceSystem.h"
#include "src/SensorReader.h"

#define ENALBE_BT

enum OUTPUT_MODE{
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

#ifdef ENALBE_BT
// 藍芽Serial initialization
BluetoothSerial SerialBT;
#endif

float imu_time = 0, last_output_time = 0, last_reset_taget_rate_time = 0;
float target_speed = 0;
OUTPUT_MODE read_type = MSG;
SYSTEM_STATE sys_state = INITIALIZING;

float pitch = 0, wz = 0, vel = 0;
bool enable_PID = false, enable_motor_tunning = false;


void setupPCNT() {
    // 配置左電機 (Unit 0)
    pcnt_config_t pcnt_config_l = {
        .pulse_gpio_num = MOTOR_L_E1A_PIN, // A 相
        .ctrl_gpio_num = MOTOR_L_E1B_PIN,  // B 相
        .lctrl_mode = PCNT_MODE_REVERSE,   // B 相低電平反轉計數（減）
        .hctrl_mode = PCNT_MODE_KEEP,      // B 相高電平保持計數（增）
        .pos_mode = PCNT_COUNT_INC,        // A 相上升沿增計數
        .neg_mode = PCNT_COUNT_DIS,        // A 相下降沿禁用
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = PCNT_UNIT_0,
        .channel = PCNT_CHANNEL_0
    };
    pcnt_unit_config(&pcnt_config_l);
    pcnt_set_filter_value(PCNT_UNIT_0, 10); // 濾波 10 個 APB 週期 (~125ns @ 80MHz)
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);

    // 配置右電機 (Unit 1)
    pcnt_config_t pcnt_config_r = {
        .pulse_gpio_num = MOTOR_R_E2A_PIN,
        .ctrl_gpio_num = MOTOR_R_E2B_PIN,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = PCNT_UNIT_1,
        .channel = PCNT_CHANNEL_0
    };
    pcnt_unit_config(&pcnt_config_r);
    pcnt_set_filter_value(PCNT_UNIT_1, 10);
    pcnt_filter_enable(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
}
  
  // 讀取 PCNT 計數
int32_t getPCNTCount(pcnt_unit_t unit) {
    int16_t count;
    pcnt_get_counter_value(unit, &count);
    pcnt_counter_clear(unit); // 讀取後清零
    return (int32_t)count;
}

void setup() {
  // 初始化串口
  Serial.begin(115200);
  Serial2.begin(230400, SERIAL_8N1, 16, 17);  // rx:16, tx:17
  Serial2.print('1');

  #ifdef ENALBE_BT
  SerialBT.begin("ABC_BT"); // Bluetooth 裝置名稱
  #endif
  Serial.println("Serial Setting...");

  // 初始化電機
  motorR.setDirection(-1);
  setupPCNT();
  Serial.println("Motor Setting...");
  Serial.println("Start Working...");
}

void loop() {
  if (Serial2.available()){
    if (sensor.readData()){
      motorL.updateCurrentRate(getPCNTCount(PCNT_UNIT_0));
      motorR.updateCurrentRate(getPCNTCount(PCNT_UNIT_1));
      vel = (motorL.getRate() + motorR.getRate()) / 2;

      float dt = sensor.getTime() - imu_time;
      pitch = sensor.getEuler().float_val[0];
      wz = sensor.getGyro().float_val[2];
      imu_time = sensor.getTime();

      if (enable_PID) {
          my_bs.updateState(micros(), vel, pitch, wz);
          motorL.setTargetRate(my_bs.outputLeft);
          motorR.setTargetRate(my_bs.outputRight);
      }

      else if (enable_motor_tunning) {
      
        if (sensor.getTime() - last_reset_taget_rate_time >= 0.1) {
          last_reset_taget_rate_time = sensor.getTime();
        }
      
        else if (last_reset_taget_rate_time > sensor.getTime()) {
          last_reset_taget_rate_time = sensor.getTime();
        }
        
        motorL.setTargetRate(target_speed);
        motorR.setTargetRate(target_speed);
      }

      if (read_type == IMU){ sensor.printBuffer(); }
      else if (read_type == TIME) { Serial.println(dt); }
    }

    if (sys_state == INITIALIZING) sys_state = IMU_MEASURING;
    blinkLED(sys_state);
  }

  if (Serial.available()) { 
    checkCommand(readCurrentBytes(Serial)); 
  }

  #ifdef ENALBE_BT
  if (SerialBT.available()) {
    ParseResult result = readSerialPacket(SerialBT);
    if (result.success){
      if (result.type == 0){ 
        Serial.print(result.data[0]);
        Serial.print(", ");
        Serial.print(result.data[1]);
        Serial.print(", ");
        Serial.println(result.data[2]);
      }
      else if (result.type == 1){
        checkCommand(result.string_data);
      }
    }
  }
  #endif
  
  outputTask();
}

void outputTask(){
  if (last_output_time > imu_time) { last_output_time = imu_time; }

  // print current state every 100ms
  if (imu_time - last_output_time >= 0.05) {
    char buffer[100];
    int index = 0;
    last_output_time = imu_time;
    
    if (read_type == WHEEL){
      index = appendValue2Str(buffer, 100, index, -target_speed, 2);
      index = appendValue2Str(buffer, 100, index, motorL.getRate(), 2);
      index = appendValue2Str(buffer, 100, index, motorR.getRate(), 2);
      index = appendValue2Str(buffer, 100, index, motorL.pwm, 1);
      index = appendValue2Str(buffer, 100, index, motorR.pwm, 1);
    } 

    else if (read_type == MSG){
      index = appendValue2Str(buffer, 100, index, pitch, 1);
      index = appendValue2Str(buffer, 100, index, my_bs.target_angle, 1);
      index = appendValue2Str(buffer, 100, index, my_bs.current_rateX, 3);
      index = appendValue2Str(buffer, 100, index, my_bs.target_rateX, 3);
      index = appendValue2Str(buffer, 100, index, vel, 2);
      index = appendValue2Str(buffer, 100, index, my_bs.outputLeft, 2);
    }
    if (index > 0){ Serial.println(buffer); }

    #ifdef ENALBE_BT
    sendBTMessage(SerialBT, vel, 0, pitch, my_bs.target_angle, vel, -my_bs.outputLeft);
    #endif
  }  
}

void checkCommand(String command) {
  command.trim();
  Serial.print("Received: ");
  Serial.println(command);
  if (command == "0") {
    target_speed = 0; 
    my_bs.reset();
    motorL.reset();
    motorR.reset();
    digitalWrite(MOTOR_R_DTBY_PIN, LOW); // 停止電機
    enable_PID = false;
    enable_motor_tunning = false;

  } 
  else if (command == "1") {
    target_speed = 0; 
    my_bs.reset();
    motorL.reset();
    motorR.reset();
    digitalWrite(MOTOR_R_DTBY_PIN, HIGH); 
    enable_PID = true;
    enable_motor_tunning = false;
  }
  else if (command == "wheel")  { 
    read_type = WHEEL;  
    enable_PID = false;
    enable_motor_tunning = true;
    target_speed = 0; 
    my_bs.reset();
    motorL.reset();
    motorR.reset();
    digitalWrite(MOTOR_R_DTBY_PIN, HIGH);
  }
  else if (command == "+")      { target_speed += 3;  } 
  else if (command == "-")      { target_speed -= 3;  } 
  else if (command == "++")     { target_speed += 10; } 
  else if (command == "--")     { target_speed -= 10; } 
  else if (command == "a")      { target_speed = 0;   } 
  else if (command == "b")      { target_speed = 10;  } 
  else if (command == "byte")   { read_type = BYTE;   } 
  else if (command == "msg")    { read_type = MSG;    } 
  else if (command == "imu")    { read_type = IMU;    } 
  else if (command == "none")   { read_type = NONE;   }
  else if (command == "time")   { read_type = TIME;   }
  else if (command == "LC")     { Serial2.print('2'); }
  else if (command == "imu1")   { Serial2.print('1'); } 
  else if (command == "imu0")   { Serial2.print('0'); }
  else if (command.startsWith("PID_")) {
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
      Serial.println(command);
      motorL.pwm = 0;
      motorR.pwm = 0;
      if (pidType != 'M') { target_speed = 0; }      
      my_bs.reset();
      motorL.reset();
      motorR.reset();

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
      else if (pidType == 'V') {
        my_bs.setSpeedPID(values[0], values[1], values[2]);
      } 
      else if (pidType == 'T') {
        my_bs.setTurnPID(values[0], values[1], values[2]);
      } 
      else if (pidType == 'O') {
        my_bs.offset = values[0]; 
      }
      else {
        Serial.println("Error: Invalid PID type (expected A or R)");
        return;
      }
      
    } else {
      Serial.println("Error: Invalid PID data");
    }
  }
}