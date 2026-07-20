#include "flight_control.h"

#include <esp_timer.h>
#include <stdint.h>
#include <stdio.h>

#include "Config.h"
#include "Mahony.h"
#include "esp_err.h"
#include "esp_log.h"
#include "math.h"
#include "params.h"
#include "pid.h"
#include "status.h"
#include "telemetry.h"

static pid_t pitch_pid;
static pid_t roll_pid;
static pid_t yaw_pid;

static TaskHandle_t s_flight_control_task_handle = nullptr;
static esp_timer_handle_t s_flight_control_task_timer = nullptr;

static void flight_control_task_timer_callback(void *arg) { xTaskNotifyGive(s_flight_control_task_handle); }

static esp_err_t flight_control_task_timer_init(void) {
  esp_timer_create_args_t timer_args = {
      .callback = &flight_control_task_timer_callback,
      .name = "1kHz_timer",
  };
  esp_err_t ret = esp_timer_create(&timer_args, &s_flight_control_task_timer);
  if (ret != ESP_OK)
    return ret;
  return esp_timer_start_periodic(s_flight_control_task_timer, 1000);
}

static void get_correction_speed(quat_t *orientation, vec3_t *out_correction_speed) {
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
    orient_err_angle = 2.0f * atan2f(angular_vel_norm, orient_err.w) / angular_vel_norm;
  } else {
    orient_err_angle = 2.0f;
  }

  angular_vel.x = clamp(angular_vel.x * orient_err_angle * params_config.pitch_gain_kp,
                        -params_config.pitch_angular_lim,
                        params_config.pitch_angular_lim);
  angular_vel.y = clamp(angular_vel.y * orient_err_angle * params_config.roll_gain_kp,
                        -params_config.roll_angular_lim,
                        params_config.roll_angular_lim);
  angular_vel.z = clamp(angular_vel.z * orient_err_angle * params_config.yaw_gain_kp,
                        -params_config.yaw_angular_lim,
                        params_config.yaw_angular_lim);
  *out_correction_speed = angular_vel;
}

static void get_correction_torque(const imu_data_t *imu_data, const vec3_t *angular_speed, vec3_t *torque) {
  float pitch_err = imu_data->gx - angular_speed->x * G_RAD_TO_DEG;
  float roll_err = imu_data->gy - angular_speed->y * G_RAD_TO_DEG;
  float yaw_err = imu_data->gz - angular_speed->z * G_RAD_TO_DEG;

  float torque_pitch = pid_update(&pitch_pid, 0, pitch_err, 0.001f);
  float torque_roll = pid_update(&roll_pid, 0, roll_err, 0.001f);
  float torque_yaw = pid_update(&yaw_pid, 0, yaw_err, 0.001f);

  torque->x = torque_pitch;
  torque->y = torque_roll;
  torque->z = torque_yaw;
}

static void motor_mixer(const vec3_t *torque, const float thrust) {
  float m1 = thrust + torque->x - torque->y + torque->z;
  float m2 = thrust - torque->x + torque->y + torque->z;
  float m3 = thrust + torque->x + torque->y - torque->z;
  float m4 = thrust - torque->x - torque->y - torque->z;

  float m1_diff = m1 - thrust;
  float m2_diff = m2 - thrust;
  float m3_diff = m3 - thrust;
  float m4_diff = m4 - thrust;

  float t_min = 0.0f - fmin(fmin(m1_diff, m2_diff), fmin(m3_diff, m4_diff));
  float t_max = 1.0f - fmax(fmax(m1_diff, m2_diff), fmax(m3_diff, m4_diff));

  if (t_min <= t_max) {
    float thrust_sat = fmax(fmin(thrust, t_max), t_min);
    m1 = m1_diff + thrust_sat;
    m2 = m2_diff + thrust_sat;
    m3 = m3_diff + thrust_sat;
    m4 = m4_diff + thrust_sat;
  } else {
    float max_diff = fmax(fmax(m1_diff, m2_diff), fmax(m3_diff, m4_diff));
    float min_diff = fmin(fmin(m1_diff, m2_diff), fmin(m3_diff, m4_diff));
    float k1 = 1.0f;
    float k2 = 1.0f;
    if (max_diff > 0) {
      k1 = (1.0f - thrust) / max_diff;
    }
    if (min_diff < 0) {
      k2 = thrust / max_diff;
    }
    float k = fmin(fmin(k1, k2), 1.0);
    m1 = k * m1_diff + thrust;
    m2 = k * m2_diff + thrust;
    m3 = k * m3_diff + thrust;
    m4 = k * m4_diff + thrust;
  }

  m1 = clamp(m1, 0.0f, 1.0f);
  m2 = clamp(m2, 0.0f, 1.0f);
  m3 = clamp(m3, 0.0f, 1.0f);
  m4 = clamp(m4, 0.0f, 1.0f);

  static bool first_call = true;
  if (!first_call) {
    // Move up ONE line and to the beginning of that line
    printf("\033[1A\r");
  }
  first_call = false;

  // Row 1 with a newline
  printf("m3: %8.2f%%  m1: %8.2f%%\n", m3 * 100, m1 * 100);
  // Row 2 without newline (cursor stays at its end)
  printf("m2: %8.2f%%  m4: %8.2f%%", m2 * 100, m4 * 100);

  fflush(stdout); // critical for immediate display
}

static void update_pid(void) {
  pitch_pid.kp = params_config.pitch_rate_kp / 1000;
  pitch_pid.ki = params_config.pitch_rate_ki / 1000;
  pitch_pid.kd = params_config.pitch_rate_kd / 1000;

  roll_pid.kp = params_config.roll_rate_kp / 1000;
  roll_pid.ki = params_config.roll_rate_ki / 1000;
  roll_pid.kd = params_config.roll_rate_kd / 1000;

  yaw_pid.kp = params_config.yaw_rate_kp / 1000;
  yaw_pid.ki = params_config.yaw_rate_ki / 1000;
  yaw_pid.kd = params_config.yaw_rate_kd / 1000;
}

static void flight_control_task(void *pvParameters) {
  i2c_master_dev_handle_t imu_handle = pvParameters;
  uint8_t loop_count = 0;
  imu_data_t imu_data;
  telemetry_data_t telemetry_data;
  quat_t orientation;
  vec3_t orientation_euler;
  vec3_t angular_vel_cmd;
  vec3_t torque_cmd;

  while (1) {
    if (!arm_is_armed()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 0)
      continue;

    update_pid();

    loop_count++;

    esp_err_t err = read_imu_data(imu_handle, &imu_data);
    if (err != ESP_OK) {
      ESP_LOGW(CFG_LOG_TAG, "IMU read error: 0x%x", err);
      continue;
    }

    if ((loop_count % CFG_ATTITUDE_LOOP_DIVIDER) == 0) {
      update_mahony_imu(imu_data.ax, imu_data.ay, imu_data.az, imu_data.gx, imu_data.gy, imu_data.gz);
      get_orientation_quat(&orientation);
      get_correction_speed(&orientation, &angular_vel_cmd);
      fflush(stdout);
    }

    get_correction_torque(&imu_data, &angular_vel_cmd, &torque_cmd);
    motor_mixer(&torque_cmd, params_config.thrust / 100);

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
  pid_init(&pitch_pid,
           params_config.pitch_rate_kp / 1000,
           params_config.pitch_rate_ki / 1000,
           params_config.pitch_rate_kd / 1000);
  pid_init(&roll_pid,
           params_config.roll_rate_kp / 1000,
           params_config.roll_rate_ki / 1000,
           params_config.roll_rate_kd / 1000);
  pid_init(
      &yaw_pid, params_config.yaw_rate_kp / 1000, params_config.yaw_rate_ki / 1000, params_config.yaw_rate_kd / 1000);
  BaseType_t ret = xTaskCreatePinnedToCore(
      flight_control_task, "flight_control", 2048, (void *)imu_handle, 20, &s_flight_control_task_handle, 1);
  if (ret != pdPASS) {
    ESP_LOGE(CFG_LOG_TAG, "Failed to create flight control task");
    return nullptr;
  }
  if (flight_control_task_timer_init() != ESP_OK) {
    ESP_LOGE(CFG_LOG_TAG, "Failed to start flight control task timer");
    vTaskDelete(s_flight_control_task_handle);
    return nullptr;
  }
  return s_flight_control_task_handle;
}