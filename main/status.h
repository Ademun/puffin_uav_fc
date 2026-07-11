#pragma once

#include <stdatomic.h>
#include <stdint.h>
#include <stdbool.h>
#include "common/mavlink.h"

#define STATUS_SENSOR_REQUIRED_MASK \
  (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | \
  MAV_SYS_STATUS_SENSOR_BATTERY)

typedef enum {
  STATUS_FLAG_PARAMS_LOADED = 1u << 0,
} status_flag_t;

#define STATUS_FLAG_REQUIRED_MASK (STATUS_FLAG_PARAMS_LOADED)

typedef enum {
  ARM_STATE_DISARMED = 0,
  ARM_STATE_ARMED = 1,
} arm_state_t;

void status_sensor_set_healthy(uint32_t mav_sys_status_sensor_bit);

void status_sensor_set_unhealthy(uint32_t mav_sys_status_sensor_bit);

uint32_t status_sensor_health(void);

uint32_t status_sensors_unhealthy(void);

void status_flag_set(status_flag_t flag);
void status_flag_clear(status_flag_t flag);
uint32_t status_missing_flags(void);

bool arm_is_armed(void);
bool arm_is_ready_to_arm(void);
bool arm_toggle_armed(bool want_armed);