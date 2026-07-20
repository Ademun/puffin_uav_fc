#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef enum {
  SIGNAL_BOOT = 0,
  SIGNAL_CALIBRATION_START = 1,
  SIGNAL_CALIBRATION_END = 2,
  SIGNAL_READY_TO_ARM = 3,
  SIGNAL_ARMED = 4,
  SIGNAL_DISARMED = 5,
  SIGNAL_GCS_CONNECTION_LOST = 6,
  SIGNAL_CRITICAL_ERROR = 7,
  SIGNAL_ENUM_END = 8,
} signal_codes;

void signal_play(signal_codes code);

TaskHandle_t signal_init(void);