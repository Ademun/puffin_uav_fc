#include "pid.h"

void pid_init(pid_t *pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}

float pid_update(pid_t *pid, float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;

  float proportional = pid->kp * error;
  float derivative = pid->kd * ((error - pid->prev_error) / dt);

  pid->prev_error = error;
  return proportional + derivative;
}