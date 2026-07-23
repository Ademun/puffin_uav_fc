#include "../status.h"
#include "../telemetry.h"
#include "comms.h"
#include "esp_log.h"

static const char *TAG = "COMMS_TX";

typedef bool (*tx_handler_fn)(mavlink_message_t *msg);

typedef struct {
  uint32_t mav_msg_id;
  uint16_t interval_ms;
  uint16_t default_interval_ms;
  uint32_t last_sent_ms;
  bool enabled;
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
  mavlink_msg_heartbeat_pack_chan(
      MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAVLINK_TX_CHAN, msg, MAV_DRONE_TYPE, MAV_AUTOPILOT_TYPE, mode, 0, state);
  return true;
}

static bool mav_pack_attitude(mavlink_message_t *msg) {
  telemetry_data_t data;
  if (xQueuePeek(telemetry_queue, &data, 0) != pdTRUE) {
    return false;
  }
  mavlink_msg_attitude_pack_chan(MAV_SYSTEM_ID,
                                 MAV_COMPONENT_ID,
                                 MAVLINK_TX_CHAN,
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
  mavlink_msg_sys_status_pack_chan(MAV_SYSTEM_ID,
                                   MAV_COMPONENT_ID,
                                   MAVLINK_TX_CHAN,
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
    {.mav_msg_id = MAVLINK_MSG_ID_HEARTBEAT,
     .interval_ms = 1000,
     .default_interval_ms = 1000,
     .last_sent_ms = 0,
     .enabled = true,
     .handler_fn = mav_pack_heartbeat,
     .name = "HEARTBEAT"},
    {.mav_msg_id = MAVLINK_MSG_ID_ATTITUDE,
     .interval_ms = 100,
     .default_interval_ms = 100,
     .last_sent_ms = 0,
     .enabled = true,
     .handler_fn = mav_pack_attitude,
     .name = "ATTITUDE"},
    {.mav_msg_id = MAVLINK_MSG_ID_SYS_STATUS,
     .interval_ms = 1000,
     .default_interval_ms = 1000,
     .last_sent_ms = 0,
     .enabled = true,
     .handler_fn = mav_pack_sys_status,
     .name = "SYS_STATUS"},
};

#define TX_HANDLER_COUNT (sizeof(tx_handler_table) / sizeof(tx_handler_t))

bool tx_set_message_interval(uint32_t mav_msg_id, int32_t interval_us) {
  for (size_t i = 0; i < TX_HANDLER_COUNT; i++) {
    tx_handler_t *h = &tx_handler_table[i];
    if (h->mav_msg_id != mav_msg_id) {
      continue;
    }

    mav_lock();
    if (interval_us < 0) {
      h->enabled = false;
    } else if (interval_us == 0) {
      h->enabled = true;
      h->interval_ms = h->default_interval_ms;
    } else {
      uint32_t interval_ms = (uint32_t)interval_us / 1000;
      if (interval_ms == 0) {
        interval_ms = 1;
      } else if (interval_ms > UINT16_MAX) {
        interval_ms = UINT16_MAX;
      }
      h->enabled = true;
      h->interval_ms = (uint16_t)interval_ms;
    }
    mav_unlock();
    return true;
  }
  return false;
}

tx_request_result_t tx_request_message(uint32_t mav_msg_id) {
  for (size_t i = 0; i < TX_HANDLER_COUNT; i++) {
    tx_handler_t *h = &tx_handler_table[i];
    if (h->mav_msg_id != mav_msg_id) {
      continue;
    }

    mavlink_message_t msg;
    mav_lock();
    bool packed = h->handler_fn(&msg);
    if (packed) {
      mav_send(&msg);
    }
    mav_unlock();
    return packed ? TX_REQUEST_OK : TX_REQUEST_NOT_READY;
  }
  return TX_REQUEST_NOT_FOUND;
}

void tx_task(void *pvParameters) {
  uint32_t now_ms;
  mavlink_message_t msg;
  while (1) {
    now_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    for (size_t i = 0; i < TX_HANDLER_COUNT; i++) {
      tx_handler_t *h = &tx_handler_table[i];
      if (!h->enabled) {
        continue;
      }
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