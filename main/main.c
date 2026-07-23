#include "Config.h"
#include "Mahony.h"
#include "communication/comms.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "flight_control.h"
#include "imu/imu.h"
#include "params.h"
#include "signal.h"
#include "status.h"
#include "telemetry.h"
#include "wifi.h"

static const i2c_device_config_t i2c_imu_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x68,
    .scl_speed_hz = 400000,
    .scl_wait_us = 0,
    .flags.disable_ack_check = 0,
};

void app_main(void) {
  esp_err_t err;
  TaskHandle_t signal_handle = signal_init();
  if (signal_handle == nullptr) {
    ESP_LOGE(CFG_LOG_TAG, "Failed to start signal");
    return;
  }
  signal_play(SIGNAL_BOOT);
  wifi_init();
  telemetry_init();
  params_init();
  status_flag_set(STATUS_FLAG_PARAMS_LOADED);
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

  i2c_master_bus_handle_t bus_handle = nullptr;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &bus_handle));
  ESP_LOGI(CFG_LOG_TAG, "I2C bus configured");

  i2c_master_dev_handle_t imu_handle = nullptr;
  ESP_ERROR_CHECK(imu_init(bus_handle, &i2c_imu_config, &imu_handle));
  ESP_LOGI(CFG_LOG_TAG, "IMU device configured");
  ESP_ERROR_CHECK(imu_configure_DLPF(DLPF_0, DLPF_1, imu_handle));

  ESP_LOGI(CFG_LOG_TAG, "IMU calibration, please wait");
  signal_play(SIGNAL_CALIBRATION_START);
  ESP_ERROR_CHECK(imu_calibrate(imu_handle));
  ESP_LOGI(CFG_LOG_TAG, "IMU calibration finished");
  signal_play(SIGNAL_CALIBRATION_END);
  status_sensor_set_healthy(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                            MAV_SYS_STATUS_SENSOR_BATTERY);

  init_mahony_filter(CFG_ATTITUDE_LOOP_HZ);

  TaskHandle_t fc_handle = flight_control_start(imu_handle);
  if (fc_handle == nullptr) {
    ESP_LOGE(CFG_LOG_TAG, "Failed to start flight control");
    return;
  }

  err = communications_start();
  if (err != ESP_OK) {
    ESP_ERROR_CHECK(err);
    ESP_LOGE(CFG_LOG_TAG, "Failed to start communications");
    return;
  }

  ESP_LOGI(CFG_LOG_TAG, "System started");
}