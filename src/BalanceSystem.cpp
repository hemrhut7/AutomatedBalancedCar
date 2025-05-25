#include "BalanceSystem.h"




MotorPID::MotorPID(int pwmPin, int dirPin1, int dirPin2, int STBY, int EAPin, int EBPin): pwmPin(pwmPin), dirPin1(dirPin1), dirPin2(dirPin2), STBY(STBY), EAPin(EAPin), EBPin(EBPin) {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    pinMode(STBY, OUTPUT);

    pinMode(EAPin, INPUT);
    pinMode(EBPin, INPUT);

    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    digitalWrite(STBY, LOW);
    analogWrite(pwmPin, 0);
}

MotorPID::~MotorPID(){}

void MotorPID::setPID(float kp, float ki, float kd) {
    pid.setTunings(kp, ki, kd);
}

void MotorPID::setTargetRate(float target_rate) {   
    this->target_rate = -constrain(target_rate, -MAX_VEL, MAX_VEL);
}

void MotorPID::updateCurrentRate(int32_t count) {
    static const int DEADZONE = 17;
    unsigned long current_time = micros();
    if (last_time > 0){
        float dt = (current_time - last_time) * 1e-6f;
        // rate += count / CPR;
        rate = count * COUNTER2RAD / dt * dir_scale;
        pwm = pid.compute(dt, target_rate, rate) + pwm;
        driveMotor(int(pwm));
        
        // dead zone
        // float output = 0;
        // if      (pwm > 0) {output = pwm + DEADZONE / 2;}
        // else if (pwm < 0) {output = pwm - DEADZONE / 2;}
        // output = constrain(output, -250.0f, 250.0f);
        // driveMotor(output);
    }
    
    last_time = current_time;
}

void MotorPID::reset() {
    rate = 0;
    target_rate = 0;
    pwm = 0;
    last_time = 0;
    pid.reset();
    driveMotor(0);
}

void MotorPID::driveMotor(int target_PWM) {
    // speed to PWM conversion needs to be added
    target_PWM = constrain(target_PWM, -250, 250);
    if(target_PWM >= 0) {
        digitalWrite(dirPin1, HIGH);
        digitalWrite(dirPin2, LOW);
        analogWrite(pwmPin, target_PWM);
    } else {
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, HIGH);
        analogWrite(pwmPin, -target_PWM);
    }
}

void BalanceSystem::updateState(uint32_t timestamp, float vel, float pitch, float wz){
    if (current_t > 0){
        float dt = (timestamp - current_t) * 1e-6f;
        current_speed = vel;
        current_angle = pitch;
        current_rateZ = wz;

        target_angle = -lpf(speedPID.compute(dt, target_speed, current_speed), timestamp);
        // target_angle = -speedPID.compute(dt, target_speed, current_speed);
        outputLeft = anglePID.compute(dt, target_angle - offset, current_angle);
        outputRight = outputLeft;
    }
    current_t = timestamp;
}

void BalanceSystem::setTargetAngle(float angle){
    target_angle = angle;
}

void BalanceSystem::setTargetSpeed(float speed){
    target_speed = speed;
}

void BalanceSystem::setTargetRateZ(float Rate){
    target_rateZ = Rate;
}

void BalanceSystem::setAnglePID(float kp, float ki, float kd){
    anglePID.setTunings(kp, ki, kd);
}

void BalanceSystem::setSpeedPID(float kp, float ki, float kd){
    speedPID.setTunings(kp, ki, kd);
}

void BalanceSystem::setRatePID(float kp, float ki, float kd){
    RatePID.setTunings(kp, ki, kd);
}

void BalanceSystem::setTurnPID(float kp, float ki, float kd){
    turnPID.setTunings(kp, ki, kd);
}


void BalanceSystem::reset(){
    current_t = 0;
    current_angle = 0;
    current_speed = 0;
    current_rateX = 0;
    current_rateZ = 0;

    target_angle = 0;
    target_speed = 0;
    target_rateZ = 0;
    target_rateX = 0;

    outputLeft = 0;
    outputRight = 0;
    
    RatePID.reset();
    anglePID.reset();
    speedPID.reset();
    turnPID.reset();
}
