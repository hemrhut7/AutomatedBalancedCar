#include "pid.h"


void PID::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

float PID::compute(unsigned long current_time, float target, float current) {
    float dt = (current_time - lastTime) * 1e-6f;
    lastTime = current_time;
    return compute(dt, target, current);
}


float PID::compute(float dt, float target, float current) {
    float error = target - current;
    integral += error * dt;
    float derivative = (dt > 0) ? (error - lastError) / dt : 0;
    float output = kp * error + ki * integral + kd * derivative;
    lastError = error;

    return constrain_pid(output, -limit, limit);
}

float PID::compute(float dt, float target, float current, float derivative) {
    float error = target - current;
    integral += error * dt;
    float output = kp * error + ki * integral + kd * derivative;
    lastError = error;
    return constrain_pid(output, -limit, limit);
}