#ifndef BALANCE_SYSTEM_H
#define BALANCE_SYSTEM_H

#include <Arduino.h>
#include <deque>


#define MOTOR_L_DIR1_PIN    5
#define MOTOR_L_DIR2_PIN    18
#define MOTOR_L_PWM_PIN     19
#define MOTOR_L_DTBY_PIN    21
#define MOTOR_L_E1A_PIN     32
#define MOTOR_L_E1B_PIN     33

#define MOTOR_R_DIR1_PIN    4
#define MOTOR_R_DIR2_PIN    0
#define MOTOR_R_PWM_PIN     2
#define MOTOR_R_DTBY_PIN    MOTOR_L_DTBY_PIN
#define MOTOR_R_E2A_PIN     25
#define MOTOR_R_E2B_PIN     26

const int PPR = 500;           // 每轉脈衝數 (300線)
const float GEAR_RATIO = 30.0; // 減速比
const int QUADRATURE = 1;      // 1倍頻 (1Pin * RISING)
const float CPR = PPR * QUADRATURE * GEAR_RATIO; // 每轉計數 (500 * 1 * 30 = 15000)
const float WHEEL_RADIUS = 0.065 / 2; // 輪半徑 (m)
const float COUNTER2RAD = 1 / CPR * (2 * PI);
const float COUNTER2LEN = COUNTER2RAD * WHEEL_RADIUS;
const float MAX_VEL = 0.25;

class PID {
    public:
        PID(){};
        PID(float kp, float ki, float kd, size_t window_size=100) : kp(kp), ki(ki), kd(kd), window_size(window_size){}
        ~PID(){};

        void setTunings(float kp, float ki, float kd);
        float compute(float target, float current);
        float compute(unsigned long now, float target, float current);
        float compute(unsigned long now, float target, float current, float derivative);
        

    private:
        float kp = 1, ki = 1/200, kd = 0.1;
        size_t window_size = 100;
        std::deque<float> errorWindow;
        float lastError = 0;
        unsigned long lastTime = 0;
};

class MotorPID {
    public:
        MotorPID(int pwmPin, int dirPin1, int dirPin2, int STBY, int EAPin, int EBPin);
        ~MotorPID();

        void encoderISR();
        void setPID(float kp, float ki, float kd);
        void setTargetSpeed(float target_speed);
        float getSpeed(){return speed;};
        void updateCurrentSpeed();

        // output is between -255 and 255
        void driveMotor(int target_PWM);
        void driveMotor(float target_PWM);

        // 1: forward, -1: backward
        void setDirection(int dir) { dir_scale = dir; }
        float pwm = 0;

    private:
        PID pid = PID(250, 30, 12);
        int dir_scale = 1;
        int pwmPin;
        int dirPin1;
        int dirPin2;
        int STBY;
        int EAPin;
        int EBPin;
        float speed = 0;
        float target_speed = 0;
        volatile int32_t encoderCount = 0;
};


class BalanceSystem {
    public:
        BalanceSystem(){};
        ~BalanceSystem(){};
        
        void setTargetAngle(float angle);
        void setTargetSpeed(float speed);
        void setTargetRateZ(float Rate);
        void updateState(uint32_t time, float vel, float pitch, float wx, float wz);
        float getOutputLeft(){return outputLeft;};
        float getOutputRight(){return outputRight;};
        void setAnglePID(float kp, float ki, float kd);
        void setSpeedPID(float kp, float ki, float kd);
        void setRatePID(float kp, float ki, float kd);
        void setTurnPID(float kp, float ki, float kd);
        void reset();

    private:
        PID RatePID = PID(0.000125, 0, 0.00000);
        PID anglePID = PID(2.2, 0.000, 0.0);
        PID speedPID;
        PID turnPID;
        float bias_angle = 0;

        unsigned long current_t = 0;
        float current_angle = 0;
        float current_speed = 0;
        float current_rateX = 0;
        float current_rateZ = 0;

        float output_rateX = 0;

        float target_angle = 0;
        float target_speed = 0;
        float target_rateZ = 0;
        float target_rateX = 0;

        float outputLeft = 0;
        float outputRight = 0;

        void updateMotor();
        void updateAngle();
        void updateSpeed();
        void updateRate();
        void updateTurn();
};

#endif