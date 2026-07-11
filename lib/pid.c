#include "pid.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float integral_max, float integral_min) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;
}

float pid_update(pid_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    float derivative = 0.0f;

    pid->integral += error * dt;

    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }

    if (dt > 1e-6f) {
        derivative = (error - pid->prev_error) / dt;
    }

    pid->prev_error = error;

    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}