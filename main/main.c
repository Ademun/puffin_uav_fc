#include "Config.h"
#include "Mahony.h"
#include "comms.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "flight_control.h"
#include "imu.h"
#include "wifi.h"
#include "telemetry.h"
#include "params.h"

void app_main(void) {
  wifi_init();
  telemetry_init();
  params_init();
  i2c_master_bus_config_t i2c_cfg = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = I2C_NUM_0,
      .scl_io_num = CFG_SCL_PIN_ADDR,
      .sda_io_num = CFG_SLA_PIN_ADDR,
      .flags.enable_internal_pullup = 1,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
  };

  i2c_master_bus_handle_t bus_handle = NULL;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &bus_handle));
  ESP_LOGI(CFG_LOG_TAG, "I2C bus configured");

  i2c_master_dev_handle_t imu_handle = NULL;
  ESP_ERROR_CHECK(init_imu(bus_handle, &imu_handle));
  ESP_LOGI(CFG_LOG_TAG, "IMU device configured");

  ESP_LOGI(CFG_LOG_TAG, "IMU calibration, please wait");
  ESP_ERROR_CHECK(calibrate_imu(imu_handle));
  ESP_LOGI(CFG_LOG_TAG, "IMU calibration finished");

  init_mahony_filter(CFG_ATTITUDE_LOOP_HZ);

  TaskHandle_t fc_handle = flight_control_start(imu_handle);
  if (fc_handle == NULL) {
    ESP_LOGE(CFG_LOG_TAG, "Failed to start flight control");
    return;
  }

  TaskHandle_t comms_handle = communications_start();
  if (comms_handle == NULL) {
    ESP_LOGE(CFG_LOG_TAG, "Failed to start communications");
    return;
  }

  ESP_LOGI(CFG_LOG_TAG, "System started");
}