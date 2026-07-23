#include "../status.h"
#include "../telemetry.h"
#include "comms.h"
#include "esp_log.h"

static const char *TAG = "COMMS_TX";

typedef bool (*tx_handler_fn)(mavlink_message_t *msg);

typedef struct {
  uint16_t interval_ms;
  uint32_t last_sent_ms;
  tx_handler_fn handler_fn;
  const char *name;
} tx_handler_t;

static bool mav_pack_heartbeat(mavlink_message_t *msg) {
  uint32_t mode = 0;
  uint32_t state = 0;
  if (arm_is_armed()) {
    mode = MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_STABILIZE_ARMED;
    state = MAV_STATE_ACTIVE;
  } else if (arm_is_ready_to_arm()) {
    mode = MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    state = MAV_STATE_STANDBY;
  } else {
    state = MAV_STATE_BOOT;
  }
  mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, msg, MAV_DRONE_TYPE, MAV_AUTOPILOT_TYPE, mode, 0, state);
  return true;
}

static bool mav_pack_attitude(mavlink_message_t *msg) {
  telemetry_data_t data;
  if (xQueuePeek(telemetry_queue, &data, 0) != pdTRUE) {
    return false;
  }
  mavlink_msg_attitude_pack(MAV_SYSTEM_ID,
                            MAV_COMPONENT_ID,
                            msg,
                            data.timestamp_ms,
                            data.roll,
                            data.pitch,
                            data.yaw,
                            data.vroll,
                            data.vpitch,
                            data.vyaw);
  return true;
}

static bool mav_pack_sys_status(mavlink_message_t *msg) {
  uint32_t sensors_present = STATUS_SENSOR_REQUIRED_MASK;
  uint32_t sensors_enabled = STATUS_SENSOR_REQUIRED_MASK;
  uint32_t sensors_health = status_sensor_health();
  mavlink_msg_sys_status_pack(MAV_SYSTEM_ID,
                              MAV_COMPONENT_ID,
                              msg,
                              sensors_present,
                              sensors_enabled,
                              sensors_health,
                              0,
                              12000,
                              -1,
                              -1,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0);
  return true;
}

static tx_handler_t tx_handler_table[] = {
    {.interval_ms = 1000, .last_sent_ms = 0, .handler_fn = mav_pack_heartbeat, .name = "HEARTBEAT"},
    {.interval_ms = 100, .last_sent_ms = 0, .handler_fn = mav_pack_attitude, .name = "ATTITUDE"},
    {.interval_ms = 1000, .last_sent_ms = 0, .handler_fn = mav_pack_sys_status, .name = "SYS_STATUS"},
};

#define TX_HANDLER_COUNT (sizeof(tx_handler_table) / sizeof(tx_handler_t))

void tx_task(void *pvParameters) {
  uint32_t now_ms;
  mavlink_message_t msg;
  while (1) {
    now_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    for (size_t i = 0; i < TX_HANDLER_COUNT; i++) {
      tx_handler_t *h = &tx_handler_table[i];
      if (now_ms - h->last_sent_ms < h->interval_ms)
        continue;
      h->last_sent_ms = now_ms;
      if (h->handler_fn(&msg) != true) {
        ESP_LOGW(TAG, "send %s failed", h->name);
        continue;
      }
      mav_lock();
      mav_send(&msg);
      mav_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}