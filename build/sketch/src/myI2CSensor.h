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
    volatile float omg[3] = {0, 0, 0}, acc[3] = {0, 0, 0};
    volatile bool gyroDataReady = false;
    volatile bool acclDataReady = false;
public:
    Nano33BLESensor();
    ~Nano33BLESensor();
    bool begin();
    void enableIMUInterrupt();
    void readIMU(float (&gyro)[3], float (&accl)[3]);
    bool readIMU_ISR(float (&gyro)[3], float (&accl)[3]);
};







#endif