#line 1 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\src\\BalanceSystem.h"
#ifndef BALANCE_SYSTEM_H
#define BALANCE_SYSTEM_H

#include <Arduino.h>
#include <deque>


class PID {
    public:
        PID(){};
        PID(float kp, float ki, float kd, size_t window_size) : kp(kp), ki(ki), kd(kd), window_size(window_size){}
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
        MotorPID(int pwmPin, int dirPin1, int dirPin2, int STBY);
        ~MotorPID();

        void encoderISR();
        void setPID(float kp, float ki, float kd);
        void update(float target);
        float getSpeed(){return speed;};
        void updateSpeed(float hz);

    private:
        PID pid;
        int pwmPin;
        int dirPin1;
        int dirPin2;
        int STBY;
        float speed = 0;
        volatile float encoderCount = 0;
        
        void driveMotor(float output);
};


class BalanceSystem {
    public:
        BalanceSystem(){};
        ~BalanceSystem(){};
        
        void setTargetAngle(float angle);
        void setTargetSpeed(float speed);
        void setTargetRateZ(float Rate);
        void updateState(float vel, float pitch, float* omg);
        float getOutputLeft(){return outputLeft;};
        float getOutputRight(){return outputRight;};

    private:
        PID anglePID;
        PID speedPID;
        PID RatePID;
        float bias_angle = 0;

        unsigned long current_t = 0;
        float current_angle = 0;
        float current_speed = 0;
        float current_RateX = 0;
        float current_RateZ = 0;

        float targetAngle = 0;
        float targetSpeed = 0;
        float targetRateZ = 0;

        float outputLeft = 0;
        float outputRight = 0;

        void updateMotor();
        void updateAngle();
        void updateSpeed();
        void updateRate();
};


#endif