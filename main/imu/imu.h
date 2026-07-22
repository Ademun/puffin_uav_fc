#pragma once

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

typedef struct {
  float ax, ay, az;
  float gx, gy, gz;
  float temp;
} imu_data_t;

typedef enum {
  DLPF_0 = 0x00,
  DLPF_1 = 0x01,
  DLPF_2 = 0x02,
  DLPF_3 = 0x03,
  DLPF_4 = 0x04,
  DLPF_5 = 0x05,
  DLPF_6 = 0x06,
  DLPF_7 = 0x07,
} DLPF_LEVEL;

esp_err_t imu_init(i2c_master_bus_handle_t master, const i2c_device_config_t *imu_config, i2c_master_dev_handle_t *imu);

esp_err_t imu_configure_DLPF(DLPF_LEVEL gyro_dlpf, DLPF_LEVEL accel_dlpf, i2c_master_dev_handle_t imu);

esp_err_t imu_read_data(i2c_master_dev_handle_t imu, imu_data_t *data);

esp_err_t imu_calibrate(i2c_master_dev_handle_t imu);