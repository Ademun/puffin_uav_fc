#pragma once

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd);
float pid_update(pid_t *pid, float setpoint, float measurement, float dt);