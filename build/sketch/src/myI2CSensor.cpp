#line 1 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\src\\myI2CSensor.cpp"
#include "myI2CSensor.h"

Nano33BLESensor::Nano33BLESensor(/* args */){}

Nano33BLESensor::~Nano33BLESensor(){}

bool Nano33BLESensor::begin(){
    Wire1.begin();
    int counter = 0;
    while(imu.beginI2C(IMU_ADDR, Wire1) != BMI2_OK){
        Serial.println("IMU not found");
        delay(1000);
        counter++;
        if(counter > 5){
            return false;
        }
    }
    imu.setAccelODR(BMI2_ACC_ODR_100HZ);
    imu.setGyroODR(BMI2_GYR_ODR_100HZ);
    return true;
}

void Nano33BLESensor::enableIMUInterrupt(){
    imu.mapInterruptToPin(BMI2_DRDY_INT, BMI2_INT1);
    bmi2_int_pin_config intPinConfig;
    intPinConfig.pin_type = BMI2_INT1;
    intPinConfig.int_latch = BMI2_INT_NON_LATCH;
    intPinConfig.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    intPinConfig.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    intPinConfig.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    intPinConfig.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
    imu.setInterruptPinConfig(intPinConfig);
}

void Nano33BLESensor::readIMU(float (&gyro)[3], float (&accl)[3]){
    imu.getSensorData();
    accl[0] = imu.data.accelX;
    accl[1] = imu.data.accelY;
    accl[2] = imu.data.accelZ;
    gyro[0] = imu.data.gyroX;
    gyro[1] = imu.data.gyroY;
    gyro[2] = imu.data.gyroZ;
}

bool Nano33BLESensor::readIMU_ISR(float (&gyro)[3], float (&accl)[3]){
    uint16_t interruptStatus = 0;
    imu.getInterruptStatus(&interruptStatus);

    if (!gyroDataReady){
        if(interruptStatus & BMI2_GYR_DRDY_INT_MASK){
            imu.getSensorData();
            omg[0] = imu.data.gyroX;
            omg[1] = imu.data.gyroY;
            omg[2] = imu.data.gyroZ;
            gyroDataReady = true;
        }
    }
    
    if (!acclDataReady){
        if(interruptStatus & BMI2_ACC_DRDY_INT_MASK){
            imu.getSensorData();
            acc[0] = imu.data.accelX;
            acc[1] = imu.data.accelY;
            acc[2] = imu.data.accelZ;
            acclDataReady = true;
        }
    }
    
    if (gyroDataReady && acclDataReady){
        gyroDataReady = false;
        acclDataReady = false;
        for (int i=0;i<3;i++){
            accl[i] = acc[i];
            gyro[i] = omg[i];
        }
        return true;
    }
    

    if(!(interruptStatus & (BMI2_GYR_DRDY_INT_MASK | BMI2_ACC_DRDY_INT_MASK))){
        Serial.print("Unknown interrupt condition!");
    }
    return false;
}