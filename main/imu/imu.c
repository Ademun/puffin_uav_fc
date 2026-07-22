#include "imu.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG_2 0x1D
#define ACCEL_X_OUT_H 0x3B

#define EXPECTED_WHO_AM_I 0x70

#define ACCEL_SENSITIVITY 16384.0f
#define GYRO_SENSITIVITY 131.0f
#define TEMP_SENSITIVITY 333.87f
#define TEMP_OFFSET 21.0f

#define CALIBRATION_SAMPLES 500
#define GYRO_MOVEMENT_THRESHOLD 5.0f
#define ACCEL_MOVEMENT_THRESHOLD 0.5f

#define ESP_ERR_IMU_UNKNOWN_SENSOR 0x5000
#define ESP_ERR_IMU_CALIBRATION_MOVING 0x5001

struct imu_offsets_t {
  float ax, ay, az;
  float gx, gy, gz;
};

static struct imu_offsets_t imu_offsets = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

esp_err_t
imu_init(i2c_master_bus_handle_t master, const i2c_device_config_t *imu_config, i2c_master_dev_handle_t *imu) {
  esp_err_t err;
  err = i2c_master_bus_add_device(master, imu_config, imu);
  if (err != ESP_OK) {
    return err;
  }

  uint8_t wake_data[2] = {PWR_MGMT_1, 0x1};
  err = i2c_master_transmit(*imu, wake_data, sizeof(wake_data), 100);
  if (err != ESP_OK) {
    return err;
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  uint8_t gyro_cfg[2] = {GYRO_CONFIG, 0};
  err = i2c_master_transmit(*imu, gyro_cfg, sizeof(gyro_cfg), 100);
  if (err != ESP_OK) {
    return err;
  }

  uint8_t accel_cfg[2] = {ACCEL_CONFIG, 0};
  err = i2c_master_transmit(*imu, accel_cfg, sizeof(accel_cfg), 100);
  if (err != ESP_OK) {
    return err;
  }

  uint8_t reg = WHO_AM_I;
  uint8_t buf;
  err = i2c_master_transmit_receive(*imu, &reg, 1, &buf, 1, 20);
  if (err != ESP_OK) {
    return err;
  }

  if (buf != EXPECTED_WHO_AM_I) {
    return ESP_ERR_IMU_UNKNOWN_SENSOR;
  }

  return ESP_OK;
};

esp_err_t imu_configure_DLPF(DLPF_LEVEL gyro_dlpf, DLPF_LEVEL accel_dlpf, i2c_master_dev_handle_t imu) {
  esp_err_t err;
  uint8_t gyr_config[2] = {CONFIG, gyro_dlpf};
  err = i2c_master_transmit(imu, gyr_config, 2, 100);
  if (err != ESP_OK) {
    return err;
  }

  uint8_t accel_config[2] = {ACCEL_CONFIG_2, accel_dlpf};
  err = i2c_master_transmit(imu, accel_config, 2, 100);
  if (err != ESP_OK) {
    return err;
  }
  return ESP_OK;
};

esp_err_t imu_read_data(i2c_master_dev_handle_t imu, imu_data_t *data) {
  esp_err_t err;
  uint8_t raw[14];
  uint8_t reg = ACCEL_X_OUT_H;
  err = i2c_master_transmit_receive(imu, &reg, 1, raw, sizeof(raw), 20);
  if (err != ESP_OK) {
    return err;
  }

  data->ax = (float)(int16_t)(raw[0] << 8 | raw[1]) / ACCEL_SENSITIVITY - imu_offsets.ax;
  data->ay = (float)(int16_t)(raw[2] << 8 | raw[3]) / ACCEL_SENSITIVITY - imu_offsets.ay;
  data->az = (float)(int16_t)(raw[4] << 8 | raw[5]) / ACCEL_SENSITIVITY - imu_offsets.az;
  data->temp = (float)(int16_t)(raw[6] << 8 | raw[7]) / TEMP_SENSITIVITY + TEMP_OFFSET;
  data->gx = (float)(int16_t)(raw[8] << 8 | raw[9]) / GYRO_SENSITIVITY - imu_offsets.gx;
  data->gy = (float)(int16_t)(raw[10] << 8 | raw[11]) / GYRO_SENSITIVITY - imu_offsets.gy;
  data->gz = (float)(int16_t)(raw[12] << 8 | raw[13]) / GYRO_SENSITIVITY - imu_offsets.gz;

  return ESP_OK;
}

esp_err_t imu_calibrate(i2c_master_dev_handle_t imu) {
  esp_err_t err;
  imu_data_t data;

  float acc_ax = 0.0f, acc_ay = 0.0f, acc_az = 0.0f;
  float acc_gx = 0.0f, acc_gy = 0.0f, acc_gz = 0.0f;

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
    err = imu_read_data(imu, &data);
    if (err != ESP_OK) {
      return err;
    }

    acc_ax += data.ax;
    acc_ay += data.ay;
    acc_az += data.az;
    acc_gx += data.gx;
    acc_gy += data.gy;
    acc_gz += data.gz;
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  imu_offsets.ax = acc_ax / CALIBRATION_SAMPLES;
  imu_offsets.ay = acc_ay / CALIBRATION_SAMPLES;
  imu_offsets.az = acc_az / CALIBRATION_SAMPLES - 1.0f;
  imu_offsets.gx = acc_gx / CALIBRATION_SAMPLES;
  imu_offsets.gy = acc_gy / CALIBRATION_SAMPLES;
  imu_offsets.gz = acc_gz / CALIBRATION_SAMPLES;

  ESP_LOGI("TAG",
           "%f %f %f %f %f %f",
           imu_offsets.ax,
           imu_offsets.ay,
           imu_offsets.az,
           imu_offsets.gx,
           imu_offsets.gy,
           imu_offsets.gz);
  if (fabsf(imu_offsets.ax) > ACCEL_MOVEMENT_THRESHOLD || fabsf(imu_offsets.ay) > ACCEL_MOVEMENT_THRESHOLD ||
      fabsf(imu_offsets.az) > ACCEL_MOVEMENT_THRESHOLD || fabsf(imu_offsets.gx) > GYRO_MOVEMENT_THRESHOLD ||
      fabsf(imu_offsets.gy) > GYRO_MOVEMENT_THRESHOLD || fabsf(imu_offsets.gz) > GYRO_MOVEMENT_THRESHOLD) {
    return ESP_ERR_IMU_CALIBRATION_MOVING;
  }

  return ESP_OK;
};