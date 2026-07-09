#include "wifi.h"

#include "Config.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

EventGroupHandle_t wifi_event_group;

static int s_wifi_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    xEventGroupClearBits(wifi_event_group,
                         (WIFI_FAIL_BIT | WIFI_CONNECTED_BIT));
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    if (s_wifi_retry_num < 30) {
      esp_wifi_connect();
      s_wifi_retry_num++;
      ESP_LOGW(CFG_LOG_TAG, "Retry to connect to WiFi (%d)", s_wifi_retry_num);
    } else {
      xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
      ESP_LOGE(CFG_LOG_TAG, "Failed to connect to WiFi");
    }
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    s_wifi_retry_num = 0;
    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
    ESP_LOGI(CFG_LOG_TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
  }
}

void wifi_init() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = CFG_NETWORK_SSID,
              .password = CFG_NETWORK_PASS,
          },
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGI(CFG_LOG_TAG, "WiFi started, waiting for connection...");

  EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);
  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(CFG_LOG_TAG, "Connected to AP");
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGE(CFG_LOG_TAG, "Failed to connect to AP");
  } else {
    ESP_LOGE(CFG_LOG_TAG, "Unexpected event bits: 0x%" PRIx32, bits);
  }
}