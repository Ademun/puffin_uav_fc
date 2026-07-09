#ifndef LIB_IMU_H_
#define LIB_IMU_H_

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

#define MPU6500_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define AXEL_X_OUT_H 0x3B
#define CALIBRATION_SAMPLES 500

typedef struct {
  float ax, ay, az;
  float gx, gy, gz;
  float temp;
} imu_data_t;

static const i2c_device_config_t i2c_imu_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MPU6500_ADDR,
    .scl_speed_hz = 400000,
    .scl_wait_us = 0,
    .flags.disable_ack_check = 0,
};

// Configure IMU handle and wake it
esp_err_t init_imu(i2c_master_bus_handle_t bus_handle,
                   i2c_master_dev_handle_t* imu);

// Read imu_data_t
esp_err_t read_imu_data(i2c_master_dev_handle_t imu, imu_data_t* data);

// Calculate IMU offsets for further readings
esp_err_t calibrate_imu(i2c_master_dev_handle_t imu);

typedef struct {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
} imu_offsets_t;

#endif