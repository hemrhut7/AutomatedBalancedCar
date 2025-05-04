# 1 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\AutomatedBalancedCar.ino"
# 2 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\AutomatedBalancedCar.ino" 2
# 3 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\AutomatedBalancedCar.ino" 2
# 4 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\AutomatedBalancedCar.ino" 2
# 5 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\AutomatedBalancedCar.ino" 2
# 6 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\AutomatedBalancedCar.ino" 2

// #define ENALBE_BT

enum {
  WHEEL,
  BYTE,
  MSG,
  IMU,
  NONE
};

BalanceSystem my_bs;
Nano33Sensor sensor(Serial2);
MotorPID motorL(19, 5, 18, 21, 32, 33);
MotorPID motorR(2, 4, 0, 21, 25, 26);






float last_imu_time = 0;
unsigned long last_time = 0; // 上次計算時間
const unsigned long interval = 50; // 列印間隔 (ms)

float target_speed = 0;
float target_speed2 = 3;
uint8_t read_type = MSG;

float pitch = 0, wx = 0, wz = 0, vel = 0;
bool enable_PID = false;
int fs = 0;


void setupPCNT() {
    // 配置左電機 (Unit 0)
    pcnt_config_t pcnt_config_l = {
        .pulse_gpio_num = 32, // A 相
        .ctrl_gpio_num = 33, // B 相
        .lctrl_mode = PCNT_CHANNEL_LEVEL_ACTION_INVERSE /*!< Control mode: invert counter mode(increase -> decrease, decrease -> increase) */, // B 相低電平反轉計數（減）
        .hctrl_mode = PCNT_CHANNEL_LEVEL_ACTION_KEEP /*!< Control mode: won't change counter mode*/, // B 相高電平保持計數（增）
        .pos_mode = PCNT_CHANNEL_EDGE_ACTION_INCREASE /*!< Counter mode: Increase counter value */, // A 相上升沿增計數
        .neg_mode = PCNT_CHANNEL_EDGE_ACTION_HOLD /*!< Counter mode: Inhibit counter(counter value will not change in this condition) */, // A 相下降沿禁用
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
        .pulse_gpio_num = 25,
        .ctrl_gpio_num = 26,
        .lctrl_mode = PCNT_CHANNEL_LEVEL_ACTION_INVERSE /*!< Control mode: invert counter mode(increase -> decrease, decrease -> increase) */,
        .hctrl_mode = PCNT_CHANNEL_LEVEL_ACTION_KEEP /*!< Control mode: won't change counter mode*/,
        .pos_mode = PCNT_CHANNEL_EDGE_ACTION_INCREASE /*!< Counter mode: Increase counter value */,
        .neg_mode = PCNT_CHANNEL_EDGE_ACTION_HOLD /*!< Counter mode: Inhibit counter(counter value will not change in this condition) */,
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
  Serial0.begin(115200);
  Serial2.begin(230400, SERIAL_8N1, 16, 17);





  // 初始化電機
  motorR.setDirection(-1);
  pinMode(21, 0x03);
  digitalWrite(21, 0x1); // 啟動電機
  setupPCNT();
  last_time = millis();
}

void loop() {
  unsigned long current_time = millis();
  if (Serial2.available()){
    if (sensor.readData()){
        motorL.updateCurrentSpeed(getPCNTCount(PCNT_UNIT_0));
        motorR.updateCurrentSpeed(getPCNTCount(PCNT_UNIT_1));
        vel = (motorL.getSpeed() + motorR.getSpeed()) / 2;

        my_data_3f omg = sensor.getcali_gyro();
        pitch = sensor.getEuler().float_val[0];
        wx = 0.2*omg.float_val[0]+0.8*wx;
        wz = omg.float_val[2];
        fs = int(1 / (sensor.getTime() - last_imu_time));
        last_imu_time = sensor.getTime();

        if (enable_PID) {
            my_bs.updateState(current_time, vel, pitch, wx, wz);
            motorL.setTargetSpeed(my_bs.outputLeft);
            motorR.setTargetSpeed(my_bs.outputRight);
        }
    }
  }

  if (Serial0.available()) {
    String command = Serial0.readStringUntil('\n');
    command.trim();
    if (command == "0") {
      digitalWrite(21, 0x0); // 停止電機
      motorL.pwm = 0;
      motorR.pwm = 0;
      target_speed = 0;
      motorL.driveMotor(0);
      motorR.driveMotor(0);
      my_bs.reset();
      enable_PID = false;
    }
    else if (command == "1") {
      digitalWrite(21, 0x1);
      enable_PID = true;
    }
    else if (command == "+") { target_speed += 3; }
    else if (command == "-") { target_speed -= 3; }
    else if (command == "++") { target_speed += 10; }
    else if (command == "--") { target_speed -= 10; }
    else if (command == "a") { target_speed = 0; }
    else if (command == "b") { target_speed = 10; }

    else if (command == "byte") { read_type = BYTE; }
    else if (command == "msg") { read_type = MSG; }
    else if (command == "wheel") { read_type = WHEEL; }
    else if (command == "imu1") { Serial2.print('1'); }
    else if (command == "imu0") { Serial2.print('2'); }
    else if (command == "imu") { read_type = IMU; }
    else if (command == "none") { read_type = NONE; }

    else if (command.startsWith("PID_")) {
      checkPIDSettings(command);
    }

    // chagne target speed here
    // motorL.setTargetSpeed(target_speed);
    // motorR.setTargetSpeed(target_speed);
  }
# 189 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\AutomatedBalancedCar.ino"
  // print current state every 100ms
  if (current_time - last_time >= interval) {
    // char buffer[256];
    // int index = 0;
    if (read_type == WHEEL){
      // index = appendValue2Str(buffer, 256, index, target_speed, 2);
      // index = appendValue2Str(buffer, 256, index, motorL.getSpeed(), 2);
      // index = appendValue2Str(buffer, 256, index, motorR.getSpeed(), 2);
      // index = appendValue2Str(buffer, 256, index, motorL.pwm, 0);
      // index = appendValue2Str(buffer, 256, index, motorR.pwm, 0);

      Serial0.print(target_speed);
      Serial0.print("\t");
      Serial0.print(motorL.getSpeed());
      Serial0.print("\t");
      Serial0.print(motorR.getSpeed());
      Serial0.print("\t");
      Serial0.print(motorL.pwm, 0);
      Serial0.print("\t");
      Serial0.println(motorR.pwm, 0);
    }

    else if (read_type == IMU){
        my_data_3f euler = sensor.getEuler();
        // my_data_3f gyro = sensor.getGyro();
        // index = appendValue2Str(buffer, 256, index, sensor.getTime(), 3);
        // index = appendValue2Str(buffer, 256, index, gyro.float_val[0], 3);
        // index = appendValue2Str(buffer, 256, index, gyro.float_val[2], 3);
        // index = appendValues2Str(buffer, 256, index, euler.float_val, 3, 1);

        Serial0.print(sensor.getTime());
        Serial0.print("\t");
        Serial0.print(euler.float_val[0]);
        Serial0.print("\t");
        Serial0.print(euler.float_val[1]);
        Serial0.print("\t");
        Serial0.println(euler.float_val[2]);
    }

    else if (read_type == MSG){
      // index = appendValue2Str(buffer, 256, index, pitch, 1);
      // index = appendValue2Str(buffer, 256, index, my_bs.target_angle, 1);
      // index = appendValue2Str(buffer, 256, index, my_bs.current_rateX, 3);
      // index = appendValue2Str(buffer, 256, index, my_bs.target_rateX, 3);
      // index = appendValue2Str(buffer, 256, index, vel, 2);
      // index = appendValue2Str(buffer, 256, index, my_bs.outputLeft, 2);
      Serial0.print(fs);
      Serial0.print("\t");
      Serial0.print(my_bs.current_rateX);
      Serial0.print("\t");
      Serial0.print(my_bs.target_rateX);
      Serial0.print("\t");
      Serial0.print(pitch);
      Serial0.print("\t");
      Serial0.print(my_bs.target_angle);
      Serial0.print("\t");
      Serial0.print(vel);
      Serial0.print("\t");
      Serial0.println(my_bs.outputLeft);
    }
    // Serial.println(buffer);
    last_time = current_time;
  }
}

void checkPIDSettings(String command) {
  String data = command.substring(4);

  if (data.length() < 2 || data[1] != '_') {
    Serial0.println("Error: Invalid PID format (expected PID_A_XX.XXX_YY.YYY_ZZ.ZZZ)");
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
      Serial0.println("Error: Invalid PID type (expected A or R)");
      return;
    }

  } else {
    Serial0.println("Error: Invalid PID data");
  }
}
