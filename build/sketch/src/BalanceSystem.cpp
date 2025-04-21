#line 1 "C:\\Users\\hemrh\\Documents\\GitHub\\AutomatedBalancedCar\\src\\BalanceSystem.cpp"
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

    float derivative = (error - lastError) / timeChange;

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


MotorPID::MotorPID(int pwmPin, int dirPin1, int dirPin2, int MotorPID): pwmPin(pwmPin), dirPin1(dirPin1), dirPin2(dirPin2), STBY(STBY) {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    pinMode(STBY, OUTPUT);

    //控制电机A的方向，(dirPin1, dirPin1)=(1, 0)为正转，(dirPin1, dirPin1)=(0, 1)为反转
    digitalWrite(dirPin1, 1);
    digitalWrite(dirPin1, 0);
    digitalWrite(STBY, 1);
    analogWrite(pwmPin, 0);
}

MotorPID::~MotorPID(){}

void MotorPID::encoderISR() {
    int pwm = digitalRead(pwmPin);
    int dir = digitalRead(dir);
    if (pwm == LOW) {
        if (dir == LOW){
            encoderCount--;
        }
        else{
            encoderCount++;
        }
    }
    else {
        if (dir == LOW){
            encoderCount++;
        }
        else{
            encoderCount--;
        }
    }
}

void MotorPID::setPID(float kp, float ki, float kd) {
    pid.setTunings(kp, ki, kd);
}

void MotorPID::setTargetSpeed(float target_speed) {   
    driveMotor(pid.compute(target_speed, speed));
}

void MotorPID::updateCurrentSpeed(float hz) {
    // noInterrupts();
    speed = encoderCount * hz;
    encoderCount = 0;
    // interrupts();
}

void MotorPID::driveMotor(float output) {
    // speed to PWM conversion needs to be added
    int pwmVal = abs((int)output);
    if(pwmVal > 255) pwmVal = 255;
    
    if(output >= 0) {
        digitalWrite(dirPin1, HIGH);
        digitalWrite(dirPin2, LOW);
        analogWrite(pwmPin, pwmVal);
    } else {
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, HIGH);
        analogWrite(pwmPin, pwmVal);
    }
}


void BalanceSystem::setTargetAngle(float angle){
    targetAngle = angle;
}

void BalanceSystem::setTargetSpeed(float speed){
    targetSpeed = speed;
}

void BalanceSystem::setTargetRateZ(float Rate){
    targetRateZ = Rate;
}

void BalanceSystem::updateState(float vel, float pitch, float* omg){
    current_speed = vel;
    current_angle = pitch;
    current_RateX = omg[0];
    current_RateZ = omg[2];
    updateMotor();
}

void BalanceSystem::updateMotor(){
    outputLeft = 0;
    outputRight = 0;
    current_t = millis();
    updateSpeed();
    updateRate();
}

void BalanceSystem::updateAngle(){
    float output = anglePID.compute(current_t, targetAngle, current_angle, current_RateX);
    outputLeft += output;
    outputRight += output;
}

void BalanceSystem::updateSpeed(){
    targetAngle = speedPID.compute(current_t, targetSpeed, current_speed);
    updateAngle();
}

void BalanceSystem::updateRate(){
    float output = RatePID.compute(current_t, targetRateZ, current_RateZ);
    outputLeft += output;
    outputRight -= output;
}
