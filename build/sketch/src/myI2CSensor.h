#line 1 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\src\\myI2CSensor.h"
#ifndef MYI2CSENSOR_H
#define MYI2CSENSOR_H

#include <SparkFun_BMI270_Arduino_Library.h>
#include "myMessage.h"

#define IMU_ADDR 0x68
#define IMU_INT_PIN 32u

class Nano33BLESensor
{
private:
    BMI270 imu;
    volatile my_data_3f omg, acc;
    volatile bool gyroDataReady = false;
    volatile bool acclDataReady = false;
public:
    Nano33BLESensor();
    ~Nano33BLESensor();
    bool begin();
    void enableIMUInterrupt();
    void readIMU(float* gyro, float* accl);
    bool readIMU_ISR(float* gyro, float* accl);
};







#endif