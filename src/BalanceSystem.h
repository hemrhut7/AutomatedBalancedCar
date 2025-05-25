#ifndef BALANCE_SYSTEM_H
#define BALANCE_SYSTEM_H

#include <Arduino.h>
#include <deque>
#include "common/pid.h"
#include "common/lowpass_filter.h"
// #define TIME_SCALE 0.99998882
#define TIME_SCALE 1.0

#define MOTOR_L_DIR1_PIN    5
#define MOTOR_L_DIR2_PIN    18
#define MOTOR_L_PWM_PIN     19
#define MOTOR_L_DTBY_PIN    21
#define MOTOR_L_E1A_PIN     32
#define MOTOR_L_E1B_PIN     33

#define MOTOR_R_DIR1_PIN    4
#define MOTOR_R_DIR2_PIN    0
#define MOTOR_R_PWM_PIN     15
#define MOTOR_R_DTBY_PIN    MOTOR_L_DTBY_PIN
#define MOTOR_R_E2A_PIN     25
#define MOTOR_R_E2B_PIN     26

const int PPR = 500;           // 每轉脈衝數 (300線)
const float GEAR_RATIO = 30.0; // 減速比
const int QUADRATURE = 1;      // 1倍頻 (1Pin * RISING)
const float CPR = PPR * QUADRATURE * GEAR_RATIO; // 每轉計數 (500 * 1 * 30 = 15000)
const float WHEEL_RADIUS = 0.065 / 2; // 輪半徑 (m)
const float CONTER2RPM = 1 / CPR;
const float COUNTER2RAD = CONTER2RPM * (2 * PI);
const float COUNTER2LEN = COUNTER2RAD * WHEEL_RADIUS;
const float MAX_VEL = 30;


class MotorPID {
    public:
        MotorPID(int pwmPin, int dirPin1, int dirPin2, int STBY, int EAPin, int EBPin);
        ~MotorPID();

        void setPID(float kp, float ki, float kd);
        void setTargetRate(float target_rate);
        float getRate(){return rate;};
        void updateCurrentRate(int32_t count);
        void reset();

        // output is between -255 and 255
        void driveMotor(int target_PWM);

        // 1: forward, -1: backward
        void setDirection(int dir) { dir_scale = dir; }
        float pwm = 0;

    private:
        PID pid = PID(1.5, 0.0, 0.0, 250.0f);
        int dir_scale = 1;
        int pwmPin;
        int dirPin1;
        int dirPin2;
        int STBY;
        int EAPin;
        int EBPin;
        float rate = 0;
        float target_rate = 0;
        uint32_t last_time = 0;
};


class BalanceSystem {
    public:
        BalanceSystem(){};
        ~BalanceSystem(){};
        float current_angle = 0;
        float current_speed = 0;
        float current_rateX = 0;
        float current_rateZ = 0;
        float target_angle = 0;
        float target_speed = 0;
        float target_rateZ = 0;
        float target_rateX = 0;
        float outputLeft = 0;
        float outputRight = 0;
        float offset = 0;
        
        void updateState(uint32_t timestamp, float vel, float pitch, float wz);
        void setTargetAngle(float angle);
        void setTargetSpeed(float speed);
        void setTargetRateZ(float Rate);
        void setAnglePID(float kp, float ki, float kd);
        void setSpeedPID(float kp, float ki, float kd);
        void setRatePID(float kp, float ki, float kd);
        void setTurnPID(float kp, float ki, float kd);
        void reset();

        
    private:
        PID RatePID = PID(0.00, 0.0, 0.000);
        PID anglePID = PID(0.8, 0.0, 0.01);
        PID speedPID = PID(0.0, 0.000, 0.00);
        PID turnPID = PID(0.0, 0.000, 0.00);
        LowPassFilter lpf  = LowPassFilter(0.07, micros());
        float bias_angle = 0;

        unsigned long current_t = 0;
};

#endif