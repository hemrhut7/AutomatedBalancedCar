# 1 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\AutomatedBalancedCar.ino"
# 2 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\AutomatedBalancedCar.ino" 2
# 3 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\AutomatedBalancedCar.ino" 2
# 4 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\AutomatedBalancedCar.ino" 2
# 5 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\AutomatedBalancedCar.ino" 2


#define IS_OUTPUT_BIN true
#define TIME_SCALE 0.9891


void blinkLED();
void INS(uint8_t* buffer);
void ISR_readIMU();
void ISR_motor_timer();
// void ISR_MotorLeft();
// void ISR_MotorRight();

const unsigned char HEADER[2] = {0xFA, 0xFF};
unsigned long t0, pre_time;
my_data_3f omg, acc, ori, mag, new_omg, new_acc;
my_data_u4 imu_time, temp, bar;
volatile bool imu_ready = false, motor_ready = false, gesture_ready = false;

MyCRC myCRC;
Navigation::ComplementaryFilter my_cpf;
SYSTEM_STATE sys_state = INITIALIZING;

// BalanceSystem my_balance_system;
// MotorPID motorLeft(MOTOR_L_PWM_PIN, MOTOR_L_DIR1_PIN, MOTOR_L_DIR2_PIN, MOTOR_L_DTBY_PIN);
// MotorPID motorRight(MOTOR_R_PWM_PIN, MOTOR_R_DIR1_PIN, MOTOR_R_DIR2_PIN, MOTOR_R_DTBY_PIN);
NRF52_MBED_Timer ITimer(NRF_TIMER_1);


void setup()
{
    // Initialize LED pin as output
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);

 Serial.begin(115200);
    while(!Serial) {blinkLED();}

    // Initialize IMU
    if (!sensor.begin()){
        Serial.println("Failed to initialize IMU!");
        while (1) {blinkLED();}
    }
    sensor.onInterrupt(ISR_readIMU);
    Serial.println("Connect to IMU");

    // Initialize BARO
    if (!baro.begin()){
        Serial.println("Failed to initialize BARO!");
        while (1) {blinkLED();}
    }
    Serial.println("Connect to BARO");

    // Initialize Gesture Sensor
    if (!gesture.begin()){
        Serial.println("Failed to initialize Gesture Sensor!");
        while (1) {blinkLED();}
    }
    Serial.println("Connect to Gesture Sensor");
    gesture.gestureAvailable();
    attachInterrupt(digitalPinToInterrupt(gesture.getInterrputPin()), ISR_gesture, FALLING);

    // Initialize CPF Parameters
    my_cpf.setIMUError(Nano33, 100);
    my_cpf.setThresholdBySTD();
    my_cpf.setWindowSizeLC(100);
    my_cpf.setLevelingConstant(2);
    my_cpf.startLC();
    my_cpf.setEnableLC_Output(true);
    float pos[3] = {25.013332647853254, 121.22195612772002, 0};
    my_cpf.setPOS(pos);
    Serial.println("Initialize CPF");

    // // Initialize Motor
    // attachInterrupt(digitalPinToInterrupt(MOTOR_L_INT_PIN), ISR_MotorLeft, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(MOTOR_R_INT_PIN), ISR_MotorRight, CHANGE);
    if (!ITimer.attachInterruptInterval(1e6 / 2, ISR_motor_timer)){
          Serial.println("Can't set ITimer. Select another freq. or timer");
          while (1) {blinkLED();}
    }

    // Initialize variables
    t0 = micros();
    imu_time.ulong_val = micros() - t0;
    pre_time = micros() - t0;
    sys_state = IMU_MEASURING;
    Serial.println("Start measuring...");
}

void loop()
{
    uint8_t buffer[62]; // 根據需要的總長度來分配buffer
    if (imu_ready && sys_state == IMU_MEASURING) {
        imu_ready = false;
        INS(buffer);
    }

    if (gesture_ready){
        gesture_ready = false;
        gestureSensing();
    }

    // if (motor_ready){
    //     motor_ready = false;
        // Serial.println("motor ready");
        // motorLeft.updateSpeed(MOTOR_SPEED_UDR);
        // motorRight.updateSpeed(MOTOR_SPEED_UDR);
    // }

    // PLL
    // float mean_vel = (motorLeft.getSpeed() + motorRight.getSpeed()) / 2; 
    // my_balance_system.updateState(mean_vel, ori.float_val[0], new_omg.float_val);
    // motorLeft.update(my_balance_system.getOutputLeft());
    // motorRight.update(my_balance_system.getOutputRight());

    blinkLED();
}

void INS(uint8_t* buffer) {
    //  calculate time
    imu_time.ulong_val = (micros() - t0) * 0.9891;
    unsigned long dt = imu_time.ulong_val - pre_time;
    if (dt > 0) {
        pre_time = imu_time.ulong_val;
        sensor.getIMUData(omg.float_val, acc.float_val);
        sensor.getMAGData(mag.float_val);
        baro.getBARData(bar.float_val);

        // calculate attitude
        my_cpf.run(imu_time.ulong_val * 1e-6, omg.float_val, acc.float_val);
        my_cpf.getEularAngle(ori.float_val);
        my_cpf.getCaliRate(omg.float_val, new_omg.float_val);
        my_cpf.getCaliACC(acc.float_val, new_acc.float_val);

        // transport data by byte
        memcpy(buffer, HEADER, 2);
        memcpy(buffer + 2, imu_time.bin_val, 4);
        memcpy(buffer + 6, omg.bin_val, 12);
        memcpy(buffer + 18, acc.bin_val, 12);
        memcpy(buffer + 30, mag.bin_val, 12);
        memcpy(buffer + 42, ori.bin_val, 12);
        memcpy(buffer + 54, bar.bin_val, 4);
        myCRC.calCRC(buffer, 62);
        Serial.write(buffer, 62);
    }
}

void gestureSensing(){
    static int num_color = 0;
    if (gesture.gestureAvailable()){
        int gs = gesture.readGesture();
        switch (gs) {
            case GESTURE_UP:
                num_color++;
                if (abs(num_color) % 3 == 0){
                    digitalWrite(LEDB, LOW);
                }else if (abs(num_color) % 3 == 1){
                    digitalWrite(LEDG, LOW);
                }else{
                    digitalWrite(LEDR, LOW);
                }
                break;

            case GESTURE_DOWN:
                num_color--;
                if (abs(num_color) % 3 == 0){
                    digitalWrite(LEDB, LOW);
                }else if (abs(num_color) % 3 == 1){
                    digitalWrite(LEDG, LOW);
                }else{
                    digitalWrite(LEDR, LOW);
                }
                break;

            case GESTURE_LEFT:
                num_color--;
                if (abs(num_color) % 4 == 0){
                    digitalWrite(LEDR, LOW);
                    digitalWrite(LEDG, HIGH);
                    digitalWrite(LEDB, HIGH);
                }else if (abs(num_color) % 3 == 1){
                    digitalWrite(LEDG, LOW);
                    digitalWrite(LEDR, HIGH);
                    digitalWrite(LEDB, HIGH);
                }else if (abs(num_color) % 3 == 2){
                    digitalWrite(LEDB, LOW);
                    digitalWrite(LEDR, HIGH);
                    digitalWrite(LEDG, HIGH);
                }else{
                    digitalWrite(LEDB, HIGH);
                    digitalWrite(LEDR, HIGH);
                    digitalWrite(LEDG, HIGH);
                }
                break;

            case GESTURE_RIGHT:
                num_color++;
                if (abs(num_color) % 4 == 0){
                    digitalWrite(LEDR, LOW);
                    digitalWrite(LEDG, HIGH);
                    digitalWrite(LEDB, HIGH);
                }else if (abs(num_color) % 3 == 1){
                    digitalWrite(LEDG, LOW);
                    digitalWrite(LEDR, HIGH);
                    digitalWrite(LEDB, HIGH);
                }else if (abs(num_color) % 3 == 2){
                    digitalWrite(LEDB, LOW);
                    digitalWrite(LEDR, HIGH);
                    digitalWrite(LEDG, HIGH);
                }else{
                    digitalWrite(LEDB, HIGH);
                    digitalWrite(LEDR, HIGH);
                    digitalWrite(LEDG, HIGH);
                }

                break;

            default:
                break;
        }


    }
}

void blinkLED() {
    unsigned long current_time = micros();
    static unsigned long lastBlinkTime = 0;
    static bool ledState = false;

    switch (sys_state)
    {
    case INITIALIZING:
        if (current_time - lastBlinkTime >= 100000) { // Blink every 0.5 second
            ledState = !ledState;
            digitalWrite(LED_BUILTIN, ledState);
            lastBlinkTime = current_time;
        }
        break;
    case IMU_MEASURING:
        if (current_time - lastBlinkTime >= 1000000) { // Blink every second
            ledState = !ledState;
            digitalWrite(LED_BUILTIN, ledState);
            lastBlinkTime = current_time;
        }
        break;
    case CONFIGURING:
        if (current_time - lastBlinkTime >= 10000000) { // Blink every second
            ledState = true;
            digitalWrite(LED_BUILTIN, ledState);
            lastBlinkTime = current_time;
        }
        break;
    default:
        if (current_time - lastBlinkTime >= 10000000) { // Blink every second
            ledState = false;
            digitalWrite(LED_BUILTIN, ledState);
            lastBlinkTime = current_time;
        }
        break;
    }
}

// void ISR_MotorLeft() {
//     motorLeft.encoderISR();
// }

// void ISR_MotorRight() {
//     motorRight.encoderISR();
// }

void ISR_readIMU(){
    imu_ready = true;
}

void ISR_motor_timer() {
    motor_ready = true;
}

void ISR_gesture(){
    gesture_ready = true;
}
