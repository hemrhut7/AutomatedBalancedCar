
#include <Arduino.h>
#include <ArduinoBLE.h>
#include "TimerInterrupt_Generic.h"
#include "src/myI2CSensor.h"
#include "src/Navigation/MyNavigation.h"
#include "src/BalanceSystem.h"

#define MOTOR_L_PWM_PIN     5
#define MOTOR_L_DIR1_PIN    4
#define MOTOR_L_DIR2_PIN    4
#define MOTOR_L_INT_PIN     2
#define MOTOR_L_DTBY_PIN     2

#define MOTOR_R_PWM_PIN     6
#define MOTOR_R_DIR1_PIN    7
#define MOTOR_R_DIR2_PIN    7
#define MOTOR_R_INT_PIN     3
#define MOTOR_R_DTBY_PIN     2

#define MOTOR_SPEED_UDR   2

void readIMU_ISR();
void ISR_MotorLeft();
void ISR_MotorRight();
void ISR_motor_timer();

const unsigned char HEADER[2] = {0xFA, 0xFF};
const bool is_output_bin = true;
unsigned long t0;
my_data_3f omg, acc, ori, mag, new_omg, new_acc;
my_data_u4 pre_time, temp, bar;

MyCRC myCRC; 
Nano33BLESensor sensor;
Navigation::ComplementaryFilter my_cpf;

BalanceSystem my_balance_system;
MotorPID motorLeft(MOTOR_L_PWM_PIN, MOTOR_L_DIR1_PIN, MOTOR_L_DIR2_PIN, MOTOR_L_DTBY_PIN);
MotorPID motorRight(MOTOR_R_PWM_PIN, MOTOR_R_DIR1_PIN, MOTOR_R_DIR2_PIN, MOTOR_R_DTBY_PIN);
NRF52_MBED_Timer ITimer(NRF_TIMER_1);

BLEService BLE_service("180D"); // 自定義服務 UUID
BLECharacteristic BLE_chart("2A37", BLERead | BLEWrite, 50); // 自定義特徵 UUID




void setup()
{
	Serial.begin(115200);
    if (Serial.available()) {
        Serial.println("Connect to PC");
        delay(2000);
    }

    // Initialize IMU
    if (!sensor.begin()){
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
    sensor.enableIMUInterrupt();
    attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), readIMU_ISR, RISING);
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
    Serial.println("Initialize to CPF");

    // Initialize Motor
    attachInterrupt(digitalPinToInterrupt(MOTOR_L_INT_PIN), ISR_MotorLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_R_INT_PIN), ISR_MotorRight, CHANGE);
	if (!ITimer.attachInterruptInterval(1e6 / MOTOR_SPEED_UDR, ISR_motor_timer)){
        Serial.println(F("Can't set ITimer. Select another freq. or timer"));
        while(1);
	}

    // Initialize BLE
    if (!BLE.begin()) {
        Serial.println("Starting BLE failed!");
        while (1);
    }
    BLE.setLocalName("BalancedCar");
    BLE.setAdvertisedService(BLE_service);
    BLE_service.addCharacteristic(BLE_chart);
    BLE_chart.writeValue("Ready");
    BLE.advertise();
    Serial.println("Bluetooth device active, waiting for connections...");
    
    // Initialize variables
    delay(200);
    t0 = micros();
    pre_time.ulong_val = micros() - t0;
}

void loop()
{
    BLE_onConnect();
    delay(100);
}

void readIMU_ISR(){
    if (sensor.readIMU_ISR(omg.float_val, acc.float_val)){
        pre_time.ulong_val = micros() - t0;
        
        // calculate attitude
        my_cpf.run(float(pre_time.ulong_val) * 1e-6, omg.float_val, acc.float_val);  
        my_cpf.getEularAngle(ori.float_val);
        my_cpf.getCaliRate(omg.float_val, new_omg.float_val);
        my_cpf.getCaliACC(acc.float_val, new_acc.float_val);
        
        // PLL
        float mean_vel = (motorLeft.getSpeed() + motorRight.getSpeed()) / 2; 
        my_balance_system.updateState(mean_vel, ori.float_val[0], new_omg.float_val);
        motorLeft.update(my_balance_system.getOutputLeft());
        motorRight.update(my_balance_system.getOutputRight());

        // print data
        if (is_output_bin){
            int buffer_size = 50;
            uint8_t buffer[buffer_size];
            memcpy(buffer, HEADER, 2);
            memcpy(buffer + 2, pre_time.bin_val, 4);
            memcpy(buffer + 6, omg.bin_val, 12);
            memcpy(buffer + 18, acc.bin_val, 12);
            memcpy(buffer + 30, temp.bin_val, 4);
            memcpy(buffer + 34, ori.bin_val, 12);
            myCRC.calCRC(buffer, buffer_size);
            Serial.write(buffer, buffer_size);
        }
        else{
            char buffer[256];
            int n = 0;
            n += appendDataToBuffer(buffer+n, float(pre_time.ulong_val) * 1e-6, 3);
            n += appendDataToBuffer(buffer+n, omg.float_val, 3, 4);
            n += appendDataToBuffer(buffer+n, acc.float_val, 3, 4);
            n += appendDataToBuffer(buffer+n, temp.float_val, 1);
            n += appendDataToBuffer(buffer+n, ori.float_val, 3, 2);
            Serial.println(buffer); 
        }
    }
}

void ISR_MotorLeft() {
    motorLeft.encoderISR();
}
  
void ISR_MotorRight() {
    motorRight.encoderISR();
}

void ISR_motor_timer()
{   
	motorLeft.updateSpeed(MOTOR_SPEED_UDR);
    motorRight.updateSpeed(MOTOR_SPEED_UDR);
}

void BLE_onConnect() {
    BLEDevice central = BLE.central();
    unsigned long t00 = millis();
    if (central) {
        Serial.print("Connected to central: ");
        Serial.println(central.address());

        while (central.connected()) {
            if (BLE_chart.written()) {
                const uint8_t* receivedData = BLE_chart.value();
                Serial.print("Received data: ");
                for (int i = 0; i < BLE_chart.valueLength(); i++) {
                    Serial.print(receivedData[i]); Serial.print(' ');
                }
                Serial.println();
            }

            // 傳輸數據
            unsigned long currentTime = millis() - t00;
            BLE_chart.writeValue(currentTime);

            // char buffer[256];
            // int n = 0;
            // n += appendDataToBuffer(buffer+n, float(pre_time.ulong_val) * 1e-6, 3);
            // n += appendDataToBuffer(buffer+n, omg.float_val, 3, 4);
            // n += appendDataToBuffer(buffer+n, acc.float_val, 3, 4);
            // n += appendDataToBuffer(buffer+n, temp.float_val, 1);
            // n += appendDataToBuffer(buffer+n, ori.float_val, 3, 2);
            // BLE_chart.writeValue(buffer);

            delay(1000);
        }

        Serial.print("Disconnected from central: ");
        Serial.println(central.address());
    }
}
