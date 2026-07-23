#include "../params.h"
#include "../signal.h"
#include "comms.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "status.h"
#include <stdio.h>

static const char *TAG = "COMMS_RX";

typedef bool (*rx_handler_fn)(const mavlink_message_t *msg);
typedef bool (*rx_command_handler_fn)(const mavlink_command_long_t *cmd);

typedef struct {
  uint32_t mav_msg_id;
  rx_handler_fn handler_fn;
} rx_handler_t;

typedef struct {
  uint16_t mav_cmd_id;
  rx_command_handler_fn handler_fn;
} rx_command_handler_t;

static const char *get_cmd_name(uint16_t cmd) {
  switch (cmd) {
  case MAV_CMD_COMPONENT_ARM_DISARM:
    return "COMPONENT_ARM_DISARM";
  case MAV_CMD_REQUEST_MESSAGE:
    return "REQUEST_MESSAGE";
  case MAV_CMD_SET_MESSAGE_INTERVAL:
    return "SET_MESSAGE_INTERVAL";
  case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
    return "REQUEST_AUTOPILOT_CAPABILITIES";
  default:
    return nullptr;
  }
}

static bool pack_status_text(mavlink_message_t *msg, const char *text) {
  mavlink_msg_statustext_pack_chan(
      MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAVLINK_RX_CHAN, msg, MAV_SEVERITY_WARNING, text, 0, 0);
  return true;
}

static void send_param_value(const params_entry_t *p, uint16_t idx) {
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

static void send_command_ack(uint16_t cmd_id, uint8_t result, uint8_t percentage) {
  mavlink_message_t msg;
  mav_lock();
  mavlink_msg_command_ack_pack_chan(
      MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAVLINK_RX_CHAN, &msg, cmd_id, result, percentage, 0, 0, 0);
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
  if (!param_get(decoded.param_id, &param)) {
    mavlink_message_t resp;
    char buf[50];
    snprintf(buf, sizeof(buf), "Unknown parameter: '%s'", decoded.param_id);
    pack_status_text(&resp, buf);
    mav_lock();
    mav_send(&resp);
    mav_unlock();
    return false;
  }
  send_param_value(&param, decoded.param_index);
  return true;
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

static bool handle_cmd_component_arm_disarm(const mavlink_command_long_t *cmd) {
  bool result = arm_toggle_armed(cmd->param1 == 1);
  if (result) {
    send_command_ack(cmd->command, MAV_RESULT_ACCEPTED, 0);
    signal_play(cmd->param1 == 1 ? SIGNAL_ARMED : SIGNAL_DISARMED);
  } else {
    send_command_ack(cmd->command, MAV_RESULT_DENIED, 0);
  }
  return result;
}

static bool handle_cmd_request_autopilot_capabilities(const mavlink_command_long_t *cmd) {
  if (cmd->param1 != 1.0f && cmd->param1 != 0.0f) {
    send_command_ack(cmd->command, MAV_RESULT_DENIED, 0);
    return false;
  }

  send_command_ack(cmd->command, MAV_RESULT_ACCEPTED, 0);

  mavlink_message_t msg;
  uint64_t capabilities = 0;
  uint32_t flight_sw_version = 0;
  uint32_t middleware_sw_version = 0;
  uint32_t os_sw_version = 0;
  uint32_t board_version = 0;
  uint8_t flight_custom_version[8] = {0};
  uint8_t middleware_custom_version[8] = {0};
  uint8_t os_custom_version[8] = {0};
  uint16_t vendor_id = 0;
  uint16_t product_id = 0;
  uint64_t uid = 0;
  uint8_t uid2[18] = {0};

  mavlink_msg_autopilot_version_pack_chan(MAV_SYSTEM_ID,
                                          MAV_COMPONENT_ID,
                                          MAVLINK_RX_CHAN,
                                          &msg,
                                          capabilities,
                                          flight_sw_version,
                                          middleware_sw_version,
                                          os_sw_version,
                                          board_version,
                                          flight_custom_version,
                                          middleware_custom_version,
                                          os_custom_version,
                                          vendor_id,
                                          product_id,
                                          uid,
                                          uid2);
  mav_lock();
  mav_send(&msg);
  mav_unlock();

  return true;
}

static bool handle_cmd_set_message_interval(const mavlink_command_long_t *cmd) {
  uint32_t msg_id = (uint32_t)cmd->param1;
  int32_t interval_us = (int32_t)cmd->param2;

  if (!tx_set_message_interval(msg_id, interval_us)) {
    ESP_LOGW(TAG, "SET_MESSAGE_INTERVAL: unknown message id %lu", (unsigned long)msg_id);
    send_command_ack(cmd->command, MAV_RESULT_UNSUPPORTED, 0);
    return false;
  }

  send_command_ack(cmd->command, MAV_RESULT_ACCEPTED, 0);
  return true;
}

static bool handle_cmd_request_message(const mavlink_command_long_t *cmd) {
  uint32_t msg_id = (uint32_t)cmd->param1;

  switch (tx_request_message(msg_id)) {
  case TX_REQUEST_OK:
    send_command_ack(cmd->command, MAV_RESULT_ACCEPTED, 0);
    return true;
  case TX_REQUEST_NOT_READY:
    ESP_LOGW(TAG, "REQUEST_MESSAGE: message id %lu has no data yet", (unsigned long)msg_id);
    send_command_ack(cmd->command, MAV_RESULT_TEMPORARILY_REJECTED, 0);
    return false;
  case TX_REQUEST_NOT_FOUND:
  default:
    ESP_LOGW(TAG, "REQUEST_MESSAGE: unknown message id %lu", (unsigned long)msg_id);
    send_command_ack(cmd->command, MAV_RESULT_UNSUPPORTED, 0);
    return false;
  }
}

static rx_command_handler_t rx_command_handler_table[] = {
    {.mav_cmd_id = MAV_CMD_COMPONENT_ARM_DISARM, .handler_fn = handle_cmd_component_arm_disarm},
    {.mav_cmd_id = MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, .handler_fn = handle_cmd_request_autopilot_capabilities},
    {.mav_cmd_id = MAV_CMD_SET_MESSAGE_INTERVAL, .handler_fn = handle_cmd_set_message_interval},
    {.mav_cmd_id = MAV_CMD_REQUEST_MESSAGE, .handler_fn = handle_cmd_request_message},
};
#define RX_COMMAND_HANDLER_COUNT (sizeof(rx_command_handler_table) / sizeof(rx_command_handler_t))

static bool handle_command_long(const mavlink_message_t *msg) {
  mavlink_command_long_t decoded;
  mavlink_msg_command_long_decode(msg, &decoded);

  for (size_t i = 0; i < RX_COMMAND_HANDLER_COUNT; i++) {
    if (rx_command_handler_table[i].mav_cmd_id == decoded.command) {
      bool ok = rx_command_handler_table[i].handler_fn(&decoded);
      if (!ok) {
        const char *cmd_name = get_cmd_name(decoded.command);
        if (cmd_name) {
          ESP_LOGW(TAG, "COMMAND_LONG cmd %u (%s) failed", decoded.command, cmd_name);
        } else {
          ESP_LOGW(TAG, "COMMAND_LONG cmd %u failed", decoded.command);
        }
      }
      return true;
    }
  }

  const char *cmd_name = get_cmd_name(decoded.command);
  if (cmd_name) {
    ESP_LOGW(TAG, "COMMAND_LONG cmd %u (%s) unhandled", decoded.command, cmd_name);
  } else {
    ESP_LOGW(TAG, "COMMAND_LONG cmd %u unhandled", decoded.command);
  }
  return true;
}

static rx_handler_t rx_handler_table[] = {
    {.mav_msg_id = MAVLINK_MSG_ID_PARAM_REQUEST_LIST, .handler_fn = handle_param_request_list},
    {.mav_msg_id = MAVLINK_MSG_ID_PARAM_REQUEST_READ, .handler_fn = handle_param_request_read},
    {.mav_msg_id = MAVLINK_MSG_ID_PARAM_SET, .handler_fn = handle_param_set},
    {.mav_msg_id = MAVLINK_MSG_ID_COMMAND_LONG, .handler_fn = handle_command_long},
};
#define RX_HANDLER_COUNT (sizeof(rx_handler_table) / sizeof(rx_handler_t))

void rx_task(void *pvParameters) {
  uint8_t rx_buf[512];
  mavlink_message_t msg;
  mavlink_status_t status;

  while (1) {
    struct sockaddr_in src_addr;
    socklen_t src_len = sizeof(src_addr);
    int n = recvfrom(udp_socket, rx_buf, sizeof(rx_buf), 0, (struct sockaddr *)&src_addr, &src_len);
    if (n < 0) {
      ESP_LOGW(TAG, "recv error: %d", errno);
      continue;
    }
    for (int i = 0; i < n; i++) {
      if (mavlink_parse_char(MAVLINK_RX_CHAN, rx_buf[i], &msg, &status)) {
        for (size_t j = 0; j < RX_HANDLER_COUNT; j++) {
          if (rx_handler_table[j].mav_msg_id == msg.msgid) {
            if (!rx_handler_table[j].handler_fn(&msg)) {
              ESP_LOGW(TAG, "msg %u unhandled", msg.msgid);
            }
            break;
          }
        }
      }
    }
  }
}