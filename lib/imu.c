#include "imu.h"

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

imu_offsets_t imu_offsets = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

esp_err_t init_imu(i2c_master_bus_handle_t bus_handle,
                   i2c_master_dev_handle_t* imu) {
  esp_err_t err;
  err = i2c_master_bus_add_device(bus_handle, &i2c_imu_config, imu);
  if (err != ESP_OK) {
    return err;
  }
  uint8_t wake_data[2] = {PWR_MGMT_1, 0x00};
  err = i2c_master_transmit(*imu, wake_data, sizeof(wake_data), 100);
  if (err != ESP_OK) {
    return err;
  }
  vTaskDelay(pdMS_TO_TICKS(100));
  return ESP_OK;
}

esp_err_t calibrate_imu(i2c_master_dev_handle_t imu) {
  imu_data_t data;
  esp_err_t err;
  float ax_acc = 0.0f, ay_acc = 0.0f, az_acc = 0.0f;
  float gx_acc = 0.0f, gy_acc = 0.0f, gz_acc = 0.0f;

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
    err = read_imu_data(imu, &data);
    if (err != ESP_OK) {
      return err;
    }
    ax_acc += data.ax;
    ay_acc += data.ay;
    az_acc += data.az;
    gx_acc += data.gx;
    gy_acc += data.gy;
    gz_acc += data.gz;
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  imu_offsets.ax = ax_acc / CALIBRATION_SAMPLES;
  imu_offsets.ay = ay_acc / CALIBRATION_SAMPLES;
  imu_offsets.az = az_acc / CALIBRATION_SAMPLES - 1.0f;
  imu_offsets.gx = gx_acc / CALIBRATION_SAMPLES;
  imu_offsets.gy = gy_acc / CALIBRATION_SAMPLES;
  imu_offsets.gz = gz_acc / CALIBRATION_SAMPLES;

  return ESP_OK;
}

esp_err_t read_imu_data(i2c_master_dev_handle_t imu, imu_data_t* data) {
  uint8_t raw[14];
  uint8_t reg = AXEL_X_OUT_H;
  esp_err_t err;
  err = i2c_master_transmit_receive(imu, &reg, 1, raw, sizeof(raw), 20);
  if (err != ESP_OK) {
    return err;
  }
  data->ax = (int16_t)(raw[0] << 8 | raw[1]) / 16384.0f - imu_offsets.ax;
  data->ay = (int16_t)(raw[2] << 8 | raw[3]) / 16384.0f - imu_offsets.ay;
  data->az = (int16_t)(raw[4] << 8 | raw[5]) / 16384.0f - imu_offsets.az;
  data->temp = (int16_t)(raw[6] << 8 | raw[7]) / 333.87f + 21.0f;
  data->gx = (int16_t)(raw[8] << 8 | raw[9]) / 131.0f - imu_offsets.gx;
  data->gy = (int16_t)(raw[10] << 8 | raw[11]) / 131.0f - imu_offsets.gy;
  data->gz = (int16_t)(raw[12] << 8 | raw[13]) / 131.0f - imu_offsets.gz;
  return ESP_OK;
}