// #include <ArduinoBLE.h>
#include "TimerInterrupt_Generic.h"
#include "src/sensor/myI2CSensor.h"
#include "src/Navigation/MyNavigation.h"
#include "src/BalanceSystem.h"

#define MOTOR_L_PWM_PIN     5
#define MOTOR_L_DIR1_PIN    4
#define MOTOR_L_DIR2_PIN    4
#define MOTOR_L_INT_PIN     2
#define MOTOR_L_DTBY_PIN    2
#define MOTOR_R_PWM_PIN     6
#define MOTOR_R_DIR1_PIN    7
#define MOTOR_R_DIR2_PIN    7
#define MOTOR_R_INT_PIN     3
#define MOTOR_R_DTBY_PIN    2
#define MOTOR_SPEED_UDR     2

#define IS_OUTPUT_BIN true
#define LED_PIN 13 // Define the LED pin
#define TIME_SCALE 0.9891


void ISR_readIMU();
// void ISR_MotorLeft();
// void ISR_MotorRight();
void ISR_motor_timer();
// void BLE_onConnect();
void blinkLED();

const unsigned char HEADER[2] = {0xFA, 0xFF};
unsigned long t0, pre_time;
my_data_3f omg, acc, ori, mag, new_omg, new_acc;
my_data_u4 imu_time, temp, bar;
volatile bool imu_ready = false, motor_ready = false;

MyCRC myCRC; 
BoschSensorClass sensor(Wire1);
Navigation::ComplementaryFilter my_cpf;
SYSTEM_STATE sys_state = INITIALIZING;

// BalanceSystem my_balance_system;
// MotorPID motorLeft(MOTOR_L_PWM_PIN, MOTOR_L_DIR1_PIN, MOTOR_L_DIR2_PIN, MOTOR_L_DTBY_PIN);
// MotorPID motorRight(MOTOR_R_PWM_PIN, MOTOR_R_DIR1_PIN, MOTOR_R_DIR2_PIN, MOTOR_R_DTBY_PIN);
NRF52_MBED_Timer ITimer(NRF_TIMER_1);

// BLEService BLE_service("180D"); // 自定義服務 UUID
// BLECharacteristic BLE_chart("2A37", BLERead | BLEWrite, 50); // 自定義特徵 UUID




void setup()
{   
    pinMode(LED_PIN, OUTPUT); // Initialize LED pin as output
	Serial.begin(115200);
    while(!Serial) {blinkLED();}

    // Initialize IMU
    if (!sensor.begin()){
        Serial.println("Failed to initialize IMU!");
        while (1) {blinkLED();}
    }
    sensor.onInterrupt(ISR_readIMU);
    Serial.println("Connect to IMU");

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
    if (!ITimer.attachInterruptInterval(1e6 / MOTOR_SPEED_UDR, ISR_motor_timer)){
          Serial.println("Can't set ITimer. Select another freq. or timer");
          while (1) {blinkLED();}
    }

    // // Initialize BLE
    // if (!BLE.begin()) {
    //     Serial.println("Starting BLE failed!");
    //     while (1);
    // }
    // BLE.setLocalName("BalancedCar");
    // BLE.setAdvertisedService(BLE_service);
    // BLE_service.addCharacteristic(BLE_chart);
    // BLE_chart.writeValue("Ready");
    // BLE.advertise();
    // Serial.println("Bluetooth device active, waiting for connections...");
    
    // Initialize variables
    t0 = micros();
    imu_time.ulong_val = micros() - t0;
    pre_time = micros() - t0;
    sys_state = IMU_MEASURING;
    Serial.println("Start measuring...");
}

void loop()
{
    // BLE_onConnect();

    if (imu_ready && sys_state == IMU_MEASURING) {    
        imu_ready = false;

        //  calculate time
        imu_time.ulong_val = (micros() - t0) * TIME_SCALE;
        unsigned long dt = imu_time.ulong_val - pre_time;
        if (dt > 0) {
            pre_time = imu_time.ulong_val;
            sensor.getIMUData(omg.float_val, acc.float_val);
            sensor.getMAGData(mag.float_val);
            // bar.float_val = sensor.getBarData();

            // calculate attitude
            my_cpf.run(imu_time.ulong_val * 1e-6, omg.float_val, acc.float_val);  
            my_cpf.getEularAngle(ori.float_val);
            my_cpf.getCaliRate(omg.float_val, new_omg.float_val);
            my_cpf.getCaliACC(acc.float_val, new_acc.float_val);
            
            // PLL
            // float mean_vel = (motorLeft.getSpeed() + motorRight.getSpeed()) / 2; 
            // my_balance_system.updateState(mean_vel, ori.float_val[0], new_omg.float_val);
            // motorLeft.update(my_balance_system.getOutputLeft());
            // motorRight.update(my_balance_system.getOutputRight());

            // transport data by byte
            uint8_t buffer[62];  // 根據需要的總長度來分配buffer
            memcpy(buffer, HEADER, 2);
            memcpy(buffer + 2, imu_time.bin_val, 4);
            memcpy(buffer + 6, omg.bin_val, 12);
            memcpy(buffer + 18, acc.bin_val, 12);
            memcpy(buffer + 30, mag.bin_val, 12);
            memcpy(buffer + 42, ori.bin_val, 12);
            memcpy(buffer + 54, bar.bin_val, 4);
            myCRC.calCRC(buffer, 62);
            // Serial.write(buffer, 62);
        }
    }

    if (motor_ready){
        motor_ready = false;
        Serial.println("motor ready");
    }

    blinkLED();
}

void ISR_readIMU(){
    imu_ready = true;
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
            digitalWrite(LED_PIN, ledState);
            lastBlinkTime = current_time;
        }
        break;
    case IMU_MEASURING:
        if (current_time - lastBlinkTime >= 1000000) { // Blink every second
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState);
            lastBlinkTime = current_time;
        }
        break;
    case CONFIGURING:
        if (current_time - lastBlinkTime >= 10000000) { // Blink every second
            ledState = true;
            digitalWrite(LED_PIN, ledState);
            lastBlinkTime = current_time;
        }
        break;
    default:
        if (current_time - lastBlinkTime >= 10000000) { // Blink every second
            ledState = false;
            digitalWrite(LED_PIN, ledState);
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

void ISR_motor_timer()
{   
    motor_ready = true;
	// motorLeft.updateSpeed(MOTOR_SPEED_UDR);
    // motorRight.updateSpeed(MOTOR_SPEED_UDR);
}

// void BLE_onConnect() {
//     BLEDevice central = BLE.central();
//     unsigned long t00 = millis();
//     if (central) {
//         Serial.print("Connected to central: ");
//         Serial.println(central.address());

//         while (central.connected()) {
//             // read data
//             if (BLE_chart.written()) {
//                 const uint8_t* receivedData = BLE_chart.value();
//                 Serial.print("Received data: ");
//                 for (int i = 0; i < BLE_chart.valueLength(); i++) {
//                     Serial.print(receivedData[i]); Serial.print(' ');
//                 }
//                 Serial.println();
//             }

//             // send data
//             unsigned long currentTime = millis() - t00;
//             BLE_chart.writeValue(currentTime);

//             // int buffer_size = 50;
//             // uint8_t buffer[buffer_size];
//             // memcpy(buffer, HEADER, 2);
//             // memcpy(buffer + 2, imu_time.bin_val, 4);
//             // memcpy(buffer + 6, omg.bin_val, 12);
//             // memcpy(buffer + 18, acc.bin_val, 12);
//             // memcpy(buffer + 30, temp.bin_val, 4);
//             // memcpy(buffer + 34, ori.bin_val, 12);
//             // myCRC.calCRC(buffer, buffer_size);
//             // BLE_chart.writeValue(buffer);

//             delay(1000);
//         }

//         Serial.print("Disconnected from central: ");
//         Serial.println(central.address());
//     }
// }
