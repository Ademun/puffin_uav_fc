#include "comms.h"

#include "Config.h"
#include "common/mavlink.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "params.h"
#include "status.h"
#include "telemetry.h"
#include <stdio.h>

static const char *TAG = "COMMS";
#define MAVLINK_TX_CHAN MAVLINK_COMM_0
#define MAVLINK_RX_CHAN MAVLINK_COMM_1

static constexpr uint8_t MAV_SYSTEM_ID = 1;
static constexpr uint8_t MAV_COMPONENT_ID = MAV_COMP_ID_AUTOPILOT1;
static constexpr uint8_t MAV_DRONE_TYPE = MAV_TYPE_QUADROTOR;
static constexpr uint8_t MAV_AUTOPILOT_TYPE = MAV_AUTOPILOT_GENERIC;

static int udp_socket;
static struct sockaddr_in local_addr;
static struct sockaddr_in dest_addr;

static SemaphoreHandle_t mav_mutex;

static void mav_lock(void) { xSemaphoreTake(mav_mutex, portMAX_DELAY); };

static void mav_unlock(void) { xSemaphoreGive(mav_mutex); }

static void mav_send(const mavlink_message_t *msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  sendto(udp_socket, buf, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
}

static TaskHandle_t tx_task_handle;
static TaskHandle_t rx_task_handle;

typedef bool (*tx_handler_fn)(mavlink_message_t *msg);
typedef bool (*rx_handler_fn)(const mavlink_message_t *msg);
typedef bool (*rx_command_handler_fn)(const mavlink_command_long_t *cmd);

typedef struct {
  uint16_t interval_ms;
  uint32_t last_sent_ms;
  tx_handler_fn handler_fn;
} tx_handler_t;

typedef struct {
  uint32_t mav_msg_id;
  rx_handler_fn handler_fn;
} rx_handler_t;

typedef struct {
  uint16_t mav_cmd_id;
  rx_command_handler_fn handler_fn;
} rx_command_handler_t;

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
};

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

static bool mav_pack_status_text(mavlink_message_t *msg, const char *text) {
  mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, msg, MAV_SEVERITY_WARNING, text, 0, 0);
  return true;
}

static void handle_heartbeat(const mavlink_message_t *msg) {

}

static void send_param_value(const params_entry_t *p, const uint16_t idx) {
  mavlink_message_t msg;
  mav_lock();
  mavlink_msg_param_value_pack_chan(MAV_SYSTEM_ID,
                                    MAV_COMPONENT_ID,
                                    MAVLINK_RX_CHAN,
                                    &msg,
                                    p->name,
                                    *(p->value_p),
                                    MAV_PARAM_TYPE_REAL32,
                                    PARAMS_COUNT,
                                    idx);
  mav_send(&msg);
  mav_unlock();
}

static bool handle_param_request_list(const mavlink_message_t *msg) {
  for (uint16_t i = 0; i < PARAMS_COUNT; i++) {
    send_param_value(&params_list[i], i);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return true;
}

static bool handle_param_request_read(const mavlink_message_t *msg) {
  mavlink_param_request_read_t decoded;
  mavlink_msg_param_request_read_decode(msg, &decoded);
  params_entry_t param;
  if (param_get(decoded.param_id, &param) != true) {
    mavlink_message_t resp;
    char buf[50];
    snprintf(buf, sizeof(buf), "Unknown parameter: \'%s\'", decoded.param_id);
    mav_pack_status_text(&resp, buf);
    mav_lock();
    mav_send(&resp);
    mav_unlock();
    return false;
  }
  send_param_value(&param, decoded.param_index);
  return true;
}

static void send_command_ack(const uint16_t cmd_id, const uint8_t result, const uint8_t percentage) {
  mavlink_message_t msg;
  mav_lock();
  mavlink_msg_command_ack_pack_chan(
      MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAVLINK_RX_CHAN, &msg, cmd_id, result, percentage, 0, 0, 0);
  mav_send(&msg);
  mav_unlock();
}

static bool handle_param_set(const mavlink_message_t *msg) {
  mavlink_param_set_t decoded;
  mavlink_msg_param_set_decode(msg, &decoded);
  uint16_t idx = UINT16_MAX;
  params_entry_t param;
  bool result = param_set(decoded.param_id, decoded.param_value, &param, &idx);
  if (idx != UINT16_MAX) {
    send_param_value(&param, idx);
  }
  return result;
}

static bool handle_command_long(const mavlink_message_t *msg);

static bool handle_cmd_component_arm_disarm(const mavlink_command_long_t *cmd) {
  // ignore force arming for now
  bool result = arm_toggle_armed(cmd->param1 == 1);
  if (result) {
    send_command_ack(cmd->command, MAV_RESULT_ACCEPTED, 0);
  } else {
    send_command_ack(cmd->command, MAV_RESULT_DENIED, 0);
  }
  return result;
}

static tx_handler_t tx_handler_table[] = {
    {
        .interval_ms = 1000,
        .last_sent_ms = 0,
        .handler_fn = mav_pack_heartbeat,
    },
    {
        .interval_ms = 100,
        .last_sent_ms = 0,
        .handler_fn = mav_pack_attitude,
    },
    {
        .interval_ms = 1000,
        .last_sent_ms = 0,
        .handler_fn = mav_pack_sys_status,
    },
};

#define TX_HANDLER_COUNT sizeof(tx_handler_table) / sizeof(tx_handler_t)

static rx_handler_t rx_handler_table[] = {
    {
        .mav_msg_id = MAVLINK_MSG_ID_PARAM_REQUEST_LIST,
        .handler_fn = handle_param_request_list,
    },
    {
        .mav_msg_id = MAVLINK_MSG_ID_PARAM_REQUEST_READ,
        .handler_fn = handle_param_request_read,
    },
    {
        .mav_msg_id = MAVLINK_MSG_ID_PARAM_SET,
        .handler_fn = handle_param_set,
    },
    {
        .mav_msg_id = MAVLINK_MSG_ID_COMMAND_LONG,
        .handler_fn = handle_command_long,
    },
};

#define RX_HANDLER_COUNT sizeof(rx_handler_table) / sizeof(rx_handler_t)

static rx_command_handler_t rx_command_handler_table[] = {
    {
        .mav_cmd_id = MAV_CMD_COMPONENT_ARM_DISARM,
        .handler_fn = handle_cmd_component_arm_disarm,
    },
};

#define RX_COMMAND_HANDLER_COUNT sizeof(rx_command_handler_table) / sizeof(rx_command_handler_t)

static bool handle_command_long(const mavlink_message_t *msg) {
  mavlink_command_long_t decoded;
  mavlink_msg_command_long_decode(msg, &decoded);
  for (size_t i = 0; i < RX_COMMAND_HANDLER_COUNT; i++) {
    if (rx_command_handler_table[i].mav_cmd_id == decoded.command) {
      return rx_command_handler_table[i].handler_fn(&decoded);
    }
  }
  return false;
}

static void tx_task(void *pvParameters) {
  uint32_t now_ms;
  mavlink_message_t msg;
  while (1) {
    now_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    for (size_t i = 0; i < TX_HANDLER_COUNT; i++) {
      tx_handler_t *handler = &tx_handler_table[i];
      if (now_ms - handler->last_sent_ms < handler->interval_ms)
        continue;
      handler->last_sent_ms = now_ms;
      if (handler->handler_fn(&msg) != true) {
        ESP_LOGW(TAG, "Failed to send tx message: handler %d", i);
      };
      mav_lock();
      mav_send(&msg);
      mav_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

static void rx_task(void *pvParameters) {
  uint8_t rx_buf[512];
  mavlink_message_t msg;
  mavlink_status_t status;

  while (1) {
    struct sockaddr_in src_addr;
    socklen_t src_len = sizeof(src_addr);
    int n = recvfrom(udp_socket, rx_buf, sizeof(rx_buf), 0, (struct sockaddr *)&src_addr, &src_len);
    if (n < 0) {
      ESP_LOGW(TAG, "Failed to receive rx message: errno %d", errno);
      continue;
    }
    for (int i = 0; i < n; i++) {
      if (mavlink_parse_char(MAVLINK_RX_CHAN, rx_buf[i], &msg, &status)) {
        for (size_t j = 0; j < RX_HANDLER_COUNT; j++) {
          if (rx_handler_table[j].mav_msg_id == msg.msgid) {
            if (rx_handler_table[j].handler_fn(&msg) != true) {
              ESP_LOGW(TAG, "Failed to process rx message: msg_id %d", msg.msgid);
            };
            break;
          }
        }
      }
    }
  }
}

TaskHandle_t communications_start() {
  mav_mutex = xSemaphoreCreateMutex();
  if (mav_mutex == nullptr) {
    ESP_LOGE(TAG, "Failed to create mavlink mutex");
    return nullptr;
  }

  udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (udp_socket < 0) {
    ESP_LOGE(TAG, "UDP socket error");
    return nullptr;
  }

  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  local_addr.sin_port = htons(CFG_HOST_PORT);

  if (bind(udp_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) != 0) {
    ESP_LOGE(TAG, "UDP bind error: errno %d", errno);
    return nullptr;
  }

  dest_addr.sin_addr.s_addr = inet_addr(CFG_HOST_ADDR);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(CFG_HOST_PORT);

  BaseType_t ret = xTaskCreatePinnedToCore(tx_task, "mavlink_tx", 3072, nullptr, 20, &tx_task_handle, 0);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create TX task");
    return nullptr;
  }

  ret = xTaskCreatePinnedToCore(rx_task, "mavlink_rx", 4096, nullptr, 18, &rx_task_handle, 0);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create RX task");
    vTaskDelete(tx_task_handle);
    return nullptr;
  }

  return tx_task_handle;
}