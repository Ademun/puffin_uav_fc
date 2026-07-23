#include "comms.h"
#include "../Config.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

static const char *TAG = "COMMS";

SemaphoreHandle_t mav_mutex;

int udp_socket;
static struct sockaddr_in local_addr;
static struct sockaddr_in dest_addr;

void mav_lock(void) { xSemaphoreTake(mav_mutex, portMAX_DELAY); }

void mav_unlock(void) { xSemaphoreGive(mav_mutex); }

void mav_send(const mavlink_message_t *msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  sendto(udp_socket, buf, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
}

extern void tx_task(void *pvParameters);
extern void rx_task(void *pvParameters);

esp_err_t communications_start() {
  mav_mutex = xSemaphoreCreateMutex();
  if (mav_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create mavlink mutex");
    return ESP_FAIL;
  }

  udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (udp_socket < 0) {
    ESP_LOGE(TAG, "UDP socket error");
    return ESP_FAIL;
  }

  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  local_addr.sin_port = htons(CFG_HOST_PORT);

  if (bind(udp_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) != 0) {
    ESP_LOGE(TAG, "UDP bind error: errno %d", errno);
    return ESP_FAIL;
  }

  dest_addr.sin_addr.s_addr = inet_addr(CFG_HOST_ADDR);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(CFG_HOST_PORT);

  TaskHandle_t tx_handle = nullptr;
  BaseType_t ret = xTaskCreatePinnedToCore(tx_task, "mavlink_tx", 3072, NULL, 20, &tx_handle, 0);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create TX task");
    return ESP_FAIL;
  }

  TaskHandle_t rx_handle = nullptr;
  ret = xTaskCreatePinnedToCore(rx_task, "mavlink_rx", 4096, NULL, 18, &rx_handle, 0);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create RX task");
    vTaskDelete(tx_handle);
    return ESP_FAIL;
  }

  return ESP_OK;
}