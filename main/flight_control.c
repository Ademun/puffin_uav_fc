#include "flight_control.h"

#include <esp_timer.h>
#include <stdint.h>

#include "Config.h"
#include "Mahony.h"
#include "esp_err.h"
#include "esp_log.h"
#include "math.h"
#include "telemetry.h"
#include "params.h"

static TaskHandle_t s_flight_control_task_handle = NULL;
static esp_timer_handle_t s_flight_control_task_timer = NULL;

static void flight_control_task_timer_callback(void* arg) {
  xTaskNotifyGive(s_flight_control_task_handle);
}

static esp_err_t flight_control_task_timer_init(void) {
  const esp_timer_create_args_t timer_args = {
      .callback = &flight_control_task_timer_callback,
      .name = "1kHz_timer",
  };
  esp_err_t ret = esp_timer_create(&timer_args, &s_flight_control_task_timer);
  if (ret != ESP_OK) return ret;
  return esp_timer_start_periodic(s_flight_control_task_timer, 1000);
}

static void get_correction_speed(quat_t* orientation,
                                 vec3_t* out_correction_speed) {
  quat_t desired_rotation = {.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
  quat_t orient_conj, orient_err;
  vec3_t angular_vel;
  float orient_err_angle, angular_vel_norm;

  q_conj(orientation, &orient_conj);
  q_mult(&orient_conj, &desired_rotation, &orient_err);

  if (orient_err.w < 0.0f) {
    q_multn(&orient_err, -1.0f);
  }

  q_vec(&orient_err, &angular_vel);
  angular_vel_norm = v_norm(&angular_vel);
  if (angular_vel_norm > 1e-6f) {
    orient_err_angle =
        2.0f * atan2f(angular_vel_norm, orient_err.w) / angular_vel_norm;
  } else {
    orient_err_angle = 2.0f;
  }

  v_mult(&angular_vel, orient_err_angle * params_config.pitch_gain_kp);
  printf("%f\n", params_config.pitch_gain_kp);
  *out_correction_speed = angular_vel;
}

static void flight_control_task(void* pvParameters) {
  i2c_master_dev_handle_t imu_handle = (i2c_master_dev_handle_t)pvParameters;
  uint8_t loop_count = 0;
  imu_data_t imu_data;
  telemetry_data_t telemetry_data;
  quat_t orientation;
  vec3_t orientation_euler;
  vec3_t angular_vel_cmd;

  while (1) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 0) continue;

    loop_count++;

    esp_err_t err = read_imu_data(imu_handle, &imu_data);
    if (err != ESP_OK) {
      ESP_LOGW(CFG_LOG_TAG, "IMU read error: 0x%x", err);
      continue;
    }

    if ((loop_count % CFG_ATTITUDE_LOOP_DIVIDER) == 0) {
      update_mahony_imu(imu_data.ax, imu_data.ay, imu_data.az, imu_data.gx,
                        imu_data.gy, imu_data.gz);
      get_orientation_quat(&orientation);
      get_correction_speed(&orientation, &angular_vel_cmd);
    }

    if ((loop_count % CFG_TELEMETRY_LOOP_DIVIDER) == 0) {
      q_euler(&orientation, &orientation_euler);
      telemetry_data.timestamp_ms = pdTICKS_TO_MS(xTaskGetTickCount());
      telemetry_data.roll = orientation_euler.x;
      telemetry_data.pitch = orientation_euler.y;
      telemetry_data.yaw = orientation_euler.z;
      telemetry_data.vroll = imu_data.gy;
      telemetry_data.vpitch = imu_data.gx;
      telemetry_data.vyaw = imu_data.gz;
      telemetry_data.temp = imu_data.temp;
      xQueueOverwrite(telemetry_queue, &telemetry_data);
      loop_count = 0;
    }
  }
}

TaskHandle_t flight_control_start(i2c_master_dev_handle_t imu_handle) {
  BaseType_t ret = xTaskCreatePinnedToCore(
      flight_control_task, "flight_control", 2048, (void*)imu_handle, 20,
      &s_flight_control_task_handle, 1);
  if (ret != pdPASS) {
    ESP_LOGE(CFG_LOG_TAG, "Failed to create flight control task");
    return NULL;
  }
  if (flight_control_task_timer_init() != ESP_OK) {
    ESP_LOGE(CFG_LOG_TAG, "Failed to start flight control task timer");
    vTaskDelete(s_flight_control_task_handle);
    return NULL;
  }
  return s_flight_control_task_handle;
}