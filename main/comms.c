#include "comms.h"

#include "Config.h"
#include "common/mavlink.h"
#include "esp_log.h"
#include "geom.h"
#include "lwip/sockets.h"
#include "params.h"
#include "telemetry.h"

#define MAVLINK_TX_CHAN MAVLINK_COMM_0
#define MAVLINK_RX_CHAN MAVLINK_COMM_1

static int udp_socket;
static struct sockaddr_in dest_addr;
static SemaphoreHandle_t mavlink_mutex;

static const uint8_t system_id = 1;
static const uint8_t component_id = MAV_COMP_ID_AUTOPILOT1;

static bool armed = false;

static TaskHandle_t s_tx_task_handle = NULL;
static TaskHandle_t s_rx_task_handle = NULL;

static void mavlink_lock() { xSemaphoreTake(mavlink_mutex, portMAX_DELAY); }

static void mavlink_unlock() { xSemaphoreGive(mavlink_mutex); }

static void mavlink_sendto(mavlink_message_t* msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  sendto(udp_socket, buf, len, 0, (struct sockaddr*)&dest_addr,
         sizeof(dest_addr));
}

static uint32_t now_ms() { return xTaskGetTickCount() * portTICK_PERIOD_MS; }

static bool pack_heartbeat(mavlink_message_t* msg) {
  uint8_t base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
  uint8_t sys_state = MAV_STATE_STANDBY;

  if (armed) {
    base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    sys_state = MAV_STATE_ACTIVE;
  }

  mavlink_msg_heartbeat_pack(system_id, component_id, msg, MAV_TYPE_QUADROTOR,
                             MAV_AUTOPILOT_GENERIC, base_mode, 0, sys_state);
  return true;
}

// TODO: No battery - no actual status. Stub data for now
static bool pack_sys_status(mavlink_message_t* msg) {
  uint32_t sensors_present =
      MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL;
  uint32_t sensors_enabled =
      MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL;
  uint32_t sensors_health =
      MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL;
  mavlink_msg_sys_status_pack(system_id, component_id, msg, sensors_present,
                              sensors_enabled, sensors_health, 0, 12000, -1, -1,
                              0, 0, 0, 0, 0, 0, 0, 0, 0);
  return true;
}

static bool pack_attitude(mavlink_message_t* msg) {
  telemetry_data_t d;
  if (xQueuePeek(telemetry_queue, &d, 0) != pdTRUE) return false;
  mavlink_msg_attitude_pack(system_id, component_id, msg, d.timestamp_ms,
                            d.roll, d.pitch, d.yaw, d.vroll * G_DEG_TO_RAD,
                            d.vpitch * G_DEG_TO_RAD, d.vyaw * G_DEG_TO_RAD);
  return true;
}

typedef struct {
  uint32_t interval_ms;
  uint32_t last_sent_ms;
  bool (*pack)(mavlink_message_t* msg);
} tx_stream_t;

static tx_stream_t tx_streams[] = {
    {
        .interval_ms = 1000,
        .last_sent_ms = 0,
        .pack = pack_heartbeat,
    },
    {
        .interval_ms = 1000,
        .last_sent_ms = 0,
        .pack = pack_sys_status,
    },
    {
        .interval_ms = 50,
        .last_sent_ms = 0,
        .pack = pack_attitude,
    },
};

#define TX_STREAM_COUNT (sizeof(tx_streams) / sizeof(tx_stream_t))

static void mavlink_tx_task(void* pvParameters) {
  uint32_t now;
  while (1) {
    now = now_ms();
    for (size_t i = 0; i < TX_STREAM_COUNT; i++) {
      tx_stream_t* s = &tx_streams[i];
      if (now - s->last_sent_ms < s->interval_ms) continue;
      mavlink_message_t msg;
      mavlink_lock();
      bool ready = s->pack(&msg);
      if (ready) mavlink_sendto(&msg);
      mavlink_unlock();
      s->last_sent_ms = now;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

static void send_param_value(params_entry_t p, uint16_t index) {
  mavlink_message_t msg;
  mavlink_lock();
  mavlink_msg_param_value_pack_chan(system_id, component_id, MAVLINK_TX_CHAN,
                                    &msg, p.name, *p.value_p,
                                    MAV_PARAM_TYPE_REAL32, params_count, index);
  mavlink_sendto(&msg);
  mavlink_unlock();
}

static void handle_param_request_list(mavlink_message_t* msg) {
  for (uint16_t i = 0; i < params_count; i++) {
    send_param_value(params_list[i], i);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void handle_param_set(mavlink_message_t* msg) {
  mavlink_param_set_t buf;
  mavlink_msg_param_set_decode(msg, &buf);
  uint16_t id = 0;
  params_entry_t p;
  bool ok = param_set(buf.param_id, buf.param_value, &p, &id);
  if (ok) {
    send_param_value(p, id);
  }
}

typedef void (*mavlink_handler_fn_t)(mavlink_message_t* msg);

typedef struct {
  uint32_t msg_id;
  mavlink_handler_fn_t handler;
} rx_dispatch_entry_t;

static const rx_dispatch_entry_t rx_dispatch_table[] = {
    {MAVLINK_MSG_ID_PARAM_REQUEST_LIST, handle_param_request_list},
    {MAVLINK_MSG_ID_PARAM_SET, handle_param_set}};
#define RX_DISPATCH_COUNT \
  (sizeof(rx_dispatch_table) / sizeof(rx_dispatch_table[0]))

static void dispatch_mavlink_message(mavlink_message_t* msg) {
  for (size_t i = 0; i < RX_DISPATCH_COUNT; i++) {
    if (rx_dispatch_table[i].msg_id == msg->msgid) {
      rx_dispatch_table[i].handler(msg);
      return;
    }
  }
}

static void mavlink_rx_task(void* pvParameters) {
  (void)pvParameters;
  uint8_t rx_buf[512];
  mavlink_message_t msg;
  mavlink_status_t status;

  while (1) {
    struct sockaddr_in src_addr;
    socklen_t src_len = sizeof(src_addr);
    int n = recvfrom(udp_socket, rx_buf, sizeof(rx_buf), 0,
                     (struct sockaddr*)&src_addr, &src_len);
    if (n < 0) {
      ESP_LOGW(CFG_LOG_TAG, "recvfrom error: errno %d", errno);
      continue;
    }
    for (int i = 0; i < n; i++) {
      if (mavlink_parse_char(MAVLINK_RX_CHAN, rx_buf[i], &msg, &status)) {
        dispatch_mavlink_message(&msg);
      }
    }
  }
}

TaskHandle_t communications_start() {
  mavlink_mutex = xSemaphoreCreateMutex();
  if (mavlink_mutex == NULL) {
    ESP_LOGE(CFG_LOG_TAG, "Failed to create mavlink mutex");
    return NULL;
  }

  udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (udp_socket < 0) {
    ESP_LOGE(CFG_LOG_TAG, "UDP socket error");
    return NULL;
  }

  struct sockaddr_in local_addr = {
      .sin_family = AF_INET,
      .sin_addr.s_addr = htonl(INADDR_ANY),
      .sin_port = htons(CFG_HOST_PORT),
  };
  if (bind(udp_socket, (struct sockaddr*)&local_addr, sizeof(local_addr)) !=
      0) {
    ESP_LOGE(CFG_LOG_TAG, "UDP bind error: errno %d", errno);
    return NULL;
  }

  dest_addr.sin_addr.s_addr = inet_addr(CFG_HOST_ADDR);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(CFG_HOST_PORT);

  BaseType_t ret = xTaskCreatePinnedToCore(mavlink_tx_task, "mavlink_tx", 3072,
                                           NULL, 20, &s_tx_task_handle, 1);
  if (ret != pdPASS) {
    ESP_LOGE(CFG_LOG_TAG, "Failed to create TX task");
    return NULL;
  }

  ret = xTaskCreatePinnedToCore(mavlink_rx_task, "mavlink_rx", 4096, NULL, 18,
                                &s_rx_task_handle, 1);
  if (ret != pdPASS) {
    ESP_LOGE(CFG_LOG_TAG, "Failed to create RX task");
    vTaskDelete(s_tx_task_handle);
    return NULL;
  }

  return s_tx_task_handle;
}