#ifndef BALANCE_SYSTEM_H
#define BALANCE_SYSTEM_H

#include <Arduino.h>
#include <deque>


#define MOTOR_L_PWM_PIN     D8
#define MOTOR_L_DIR1_PIN    D7
#define MOTOR_L_DIR2_PIN    D6
#define MOTOR_L_DTBY_PIN    D5
#define MOTOR_L_ENC_PIN     D9
#define MOTOR_L_DIR_PIN     D10

#define MOTOR_R_PWM_PIN     D4
#define MOTOR_R_DIR1_PIN    D3
#define MOTOR_R_DIR2_PIN    D2
#define MOTOR_R_DTBY_PIN    D1
#define MOTOR_R_ENC_PIN     D11
#define MOTOR_R_DIR_PIN     D12



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
        MotorPID(int pwmPin, int dirPin1, int dirPin2, int STBY, int ENCODER_PIN, int DIR_PIN);
        ~MotorPID();

        void encoderISR();
        void setPID(float kp, float ki, float kd);
        void setTargetSpeed(float target_speed);
        float getSpeed(){return speed;};
        void updateCurrentSpeed(float hz);
        void driveMotor(float output);

    private:
        PID pid;
        int pwmPin;
        int dirPin1;
        int dirPin2;
        int STBY;
        int DIR_PIN;
        int ENCODER_PIN;
        float speed = 0;
        volatile float encoderCount = 0;
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

typedef enum {
    INITIALIZING,
    IMU_MEASURING,
    CONFIGURING,
    
}SYSTEM_STATE;


#endif