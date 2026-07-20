#pragma once

#include <stddef.h>

#include "esp_err.h"

#define PARAM_NAME_LEN 16

typedef struct {
  float roll_gain_kp;
  float pitch_gain_kp;
  float yaw_gain_kp;
  //===
  float roll_angular_lim;
  float pitch_angular_lim;
  float yaw_angular_lim;
  //===
  float pitch_rate_kp;
  float pitch_rate_ki;
  float pitch_rate_kd;

  float roll_rate_kp;
  float roll_rate_ki;
  float roll_rate_kd;

  float yaw_rate_kp;
  float yaw_rate_ki;
  float yaw_rate_kd;
  //===
  float thrust;
} params_config_t;

typedef struct {
  const char name[PARAM_NAME_LEN];
  float *value_p;
  float default_value;
  float min_value;
  float max_value;
} params_entry_t;

extern params_config_t params_config;

extern const params_entry_t params_list[];

extern const size_t PARAMS_COUNT;

esp_err_t params_init(void);

bool param_set(const char *name, float value, params_entry_t *p, uint16_t *id);

bool param_get(const char *name, params_entry_t *p);
