#pragma once

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_max;
    float integral_min;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float integral_max, float integral_min);
float pid_update(pid_t *pid, float setpoint, float measurement, float dt);