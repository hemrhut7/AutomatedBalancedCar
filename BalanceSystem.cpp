#include "BalanceSystem.h"


void PID::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

float PID::compute(float target, float current) {
    unsigned long now = millis();
    float timeChange = (float)(now - lastTime) / 1000.0;
    lastTime = now;

    float error = target - current;
    errorWindow.push_back(error * timeChange);
    if (errorWindow.size() > window_size) {
        errorWindow.pop_front();
    }

    float integral = 0;
    for (float e : errorWindow) {
        integral += e;
    }

    float derivative = 0;
    if (timeChange > 0) {
        derivative = (error - lastError) / timeChange;
    }

    float output = kp * error + ki * integral + kd * derivative;
    lastError = error;

    return output;
}


float PID::compute(unsigned long now, float target, float current) {
    float timeChange = (float)(now - lastTime) / 1000.0;
    lastTime = now;

    float error = target - current;
    errorWindow.push_back(error * timeChange);
    if (errorWindow.size() > window_size) {
        errorWindow.pop_front();
    }

    float integral = 0;
    for (float e : errorWindow) {
        integral += e;
    }

    float derivative = (error - lastError) / timeChange;

    float output = kp * error + ki * integral + kd * derivative;
    lastError = error;

    return output;
}

float PID::compute(unsigned long now, float target, float current, float derivative) {
    float timeChange = (float)(now - lastTime) / 1000.0;
    lastTime = now;

    float error = target - current;
    errorWindow.push_back(error * timeChange);
    if (errorWindow.size() > window_size) {
        errorWindow.pop_front();
    }

    float integral = 0;
    for (float e : errorWindow) {
        integral += e;
    }

    float output = kp * error + ki * integral + kd * derivative;
    lastError = error;

    return output;
}


MotorPID::MotorPID(int pwmPin, int dirPin1, int dirPin2, int MotorPID, int EAPin, int EBPin): pwmPin(pwmPin), dirPin1(dirPin1), dirPin2(dirPin2), STBY(STBY), EAPin(EAPin), EBPin(EBPin) {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    pinMode(STBY, OUTPUT);

    pinMode(EAPin, INPUT);
    pinMode(EBPin, INPUT);

    //控制电机A的方向，(dirPin1, dirPin1)=(1, 0)为正转，(dirPin1, dirPin1)=(0, 1)为反转
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    digitalWrite(STBY, HIGH);
    analogWrite(pwmPin, 0);
}

MotorPID::~MotorPID(){}

void MotorPID::encoderISR() {
    if (digitalRead(EAPin) == digitalRead(EBPin)) { encoderCount++; }  // 正轉
    else { encoderCount--; } // 反轉
}

void MotorPID::setPID(float kp, float ki, float kd) {
    pid.setTunings(kp, ki, kd);
}

void MotorPID::setTargetSpeed(float target_speed) {   
    this->target_speed = max(min(target_speed, MAX_VEL), -MAX_VEL);
}

void MotorPID::updateCurrentSpeed() {
    noInterrupts();
    speed = encoderCount * COUNTER2LEN * 20.0 * dir_scale;
    encoderCount = 0;
    interrupts();
    
    pwm = max(min(pid.compute(target_speed, speed)+pwm, 255.0f), -255.0f);
    driveMotor(pwm);
}

void MotorPID::driveMotor(int target_PWM) {
    // speed to PWM conversion needs to be added
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

void MotorPID::driveMotor(float target_PWM) {
    driveMotor((int)target_PWM);
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

void BalanceSystem::updateState(uint32_t time, float vel, float pitch, float wx, float wz){
    current_t = time;
    current_speed = vel;
    current_angle = pitch;
    current_rateX = wx;
    current_rateZ = wz;
    
    target_rateX = anglePID.compute(current_t, target_angle, current_angle);
    output_rateX += RatePID.compute(current_t, target_rateX, current_rateX);
    outputLeft = output_rateX;
    outputRight = output_rateX;
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

    output_rateX = 0;

    target_angle = 0;
    target_speed = 0;
    target_rateZ = 0;
    target_rateX = 0;

    outputLeft = 0;
    outputRight = 0;
}

void BalanceSystem::updateAngle(){
    float output = anglePID.compute(current_t, target_angle, current_angle);
    outputLeft += output;
    outputRight += output;
}

void BalanceSystem::updateSpeed(){
    target_angle = 0;
    updateAngle();
}

void BalanceSystem::updateRate(){
    float output = RatePID.compute(current_t, 0, current_rateX);
    outputLeft += output;
    outputRight -= output;
}

void BalanceSystem::updateTurn(){
    float output = RatePID.compute(current_t, target_rateZ, current_rateZ);
    outputLeft += output;
    outputRight -= output;
}
