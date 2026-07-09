#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
  uint32_t timestamp_ms;
  float roll, pitch, yaw;
  float vroll, vpitch, vyaw;
  float temp;
} telemetry_data_t;

extern QueueHandle_t telemetry_queue;

void telemetry_init(void);

#endif