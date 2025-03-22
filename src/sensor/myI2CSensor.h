#ifndef MYI2CSENSOR_H
#define MYI2CSENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "BMI270-Sensor-API/bmi270.h"
#include "BMM150-Sensor-API/bmm150.h"
#include "myMessage.h"

// default range is +-4G, so conversion factor is (((1 << 15)/4.0f))
#define INT16_to_G   (8192.0f)

// scale factor = (2^15) / range (postive to negative) = (2^15) / (2 * 2000) = 32768 / 4000 = 16.384
#define INT16_to_DPS   (16.384f * (2000/500)) // to 250dps

#define GRAVITY 9.7895f

typedef enum {
    BOSCH_ACCELEROMETER_ONLY,
    BOSCH_MAGNETOMETER_ONLY,
    BOSCH_ACCEL_AND_MAGN
} CfgBoshSensor_t;
  
struct dev_info {
    TwoWire* _wire;
    uint8_t dev_addr;
};
  
class BoschSensorClass {
    public:
        BoschSensorClass(TwoWire& wire = Wire);
        ~BoschSensorClass() {}

        void setContinuousMode();
        void oneShotMode();

        int begin(CfgBoshSensor_t cfg = BOSCH_ACCEL_AND_MAGN);
        void end();

        void debug(Stream&);
        #ifdef __MBED__
        void onInterrupt(mbed::Callback<void()>);
        void setInterruptPin(PinName irq_pin) {
        BMI270_INT1 = irq_pin;
        }
        void setInterruptPin(pin_size_t irq_pin) {
        BMI270_INT1 = digitalPinToPinName(irq_pin);
        }
        PinName BMI270_INT1 = NC;
        #endif

        virtual int getIMUData(float (&omg)[3], float (&acc)[3]);
        virtual int accelerationAvailable(); // Number of samples in the FIFO.
        virtual float accelerationSampleRate(); // Sampling rate of the sensor.
        virtual int gyroscopeAvailable(); // Number of samples in the FIFO.
        virtual float gyroscopeSampleRate(); // Sampling rate of the sensor.
        
        // Magnetometer
        virtual int getMAGData(float (&mag)[3]); // Results are in uT (micro Tesla).
        virtual int magneticFieldAvailable(); // Number of samples in the FIFO.
        virtual float magneticFieldSampleRate(); // Sampling rate of the sensor.
        

    protected:
        // can be modified by subclassing for finer configuration
        virtual int8_t configure_sensor(struct bmm150_dev *dev);
        virtual int8_t configure_sensor(struct bmi2_dev *dev);

    private:
        static int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
        static int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
        static void bmi2_delay_us(uint32_t period, void *intf_ptr);
        void interrupt_handler();
        void print_rslt(int8_t rslt);

    private:
        TwoWire* _wire;
        Stream* _debug = nullptr;
        #ifdef __MBED__
        mbed::Callback<void(void)> _cb;
        #endif
        bool _initialized = false;
        int _interrupts = 0;
        struct dev_info accel_gyro_dev_info;
        struct dev_info mag_dev_info;
        struct bmi2_dev bmi2;
        struct bmm150_dev bmm1;
        uint16_t _int_status;
    private:
        bool continuousMode;
};


// Barometer

#define LPS22HB_ADDRESS  0x5C
#define LPS22HB_WHO_AM_I_REG        0x0f
#define LPS22HB_CTRL2_REG           0x11
#define LPS22HB_STATUS_REG          0x27
#define LPS22HB_PRESS_OUT_XL_REG    0x28
#define LPS22HB_PRESS_OUT_L_REG     0x29
#define LPS22HB_PRESS_OUT_H_REG     0x2a
#define LPS22HB_TEMP_OUT_L_REG      0x2b
#define LPS22HB_TEMP_OUT_H_REG      0x2c

  
class LPS22HBClass {
    public:
    LPS22HBClass(TwoWire& wire);

    int begin();
    void end();

    bool getBARData(float &bar);
    float geTEMPData(void);

    private:
    int i2cRead(uint8_t reg);
    int i2cWrite(uint8_t reg, uint8_t val);

    private:
    TwoWire* _wire;
    bool _initialized;
};

#endif