#include "telemetry.h"

QueueHandle_t telemetry_queue = NULL;

void telemetry_init(void) {
  telemetry_queue = xQueueCreate(1, sizeof(telemetry_data_t));
}