#include "params.h"

#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"

static const char* TAG = "PARAMS";
static const char* NVS_NAMESPACE = "fc_params";

params_config_t params_config;

#define PARAM_ENTRY(m_name, field, def_val, min, max) \
  {.name = m_name,                                    \
   .value_p = &params_config.field,                   \
   .default_value = def_val,                          \
   .min_value = min,                                  \
   .max_value = max}

const params_entry_t params_list[] = {
    PARAM_ENTRY("ROLL_G_P", roll_gain_kp, 1.0f, 0.1f, 10.0f),
    PARAM_ENTRY("PITCH_G_P", pitch_gain_kp, 1.0f, 0.1f, 10.0f),
    PARAM_ENTRY("YAW_G_P", yaw_gain_kp, 1.0f, 0.1f, 10.0f),
    PARAM_ENTRY("ROLL_ANG_L", roll_angular_lim, 0.35f, 0.1f, 0.8f),
    PARAM_ENTRY("PITCH_ANG_L", pitch_angular_lim, 0.35f, 0.1f, 0.8f),
    PARAM_ENTRY("YAW_ANG_L", yaw_angular_lim, 0.35f, 0.1f, 0.8f),
};

const size_t params_count = sizeof(params_list) / sizeof(params_entry_t);

static esp_err_t nvs_init(void) {
  esp_err_t err = nvs_flash_init();
  if ((err == ESP_ERR_NVS_NO_FREE_PAGES) ||
      (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
    ESP_LOGW(TAG, "Erasing NVS partition...");
    nvs_flash_erase();
    err = nvs_flash_init();
  }

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "NVS partition initialization failed");
    return err;
  }

  return ESP_OK;
};

static esp_err_t load_from_nvs(nvs_handle_t handle) {
  uint32_t buf = 0;
  esp_err_t err;
  for (size_t i = 0; i < params_count; i++) {
    err = nvs_get_u32(handle, params_list[i].name, &buf);
    if (err == ESP_OK) {
      memcpy(params_list[i].value_p, &buf, sizeof(float));
    } else {
      return err;
    }
  }
  return ESP_OK;
}

static void load_and_save_defaults(nvs_handle_t handle) {
  uint32_t buf = 0;
  for (size_t i = 0; i < params_count; i++) {
    *params_list[i].value_p = params_list[i].default_value;
    memcpy(&buf, &params_list[i].default_value, sizeof(float));
    nvs_set_u32(handle, params_list[i].name, buf);
  }
};

esp_err_t params_init(void) {
  esp_err_t err = nvs_init();
  if (err != ESP_OK) {
    return err;
  }

  nvs_handle_t handle;
  err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS");
  }

  err = load_from_nvs(handle);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Config loaded from NVS");
  } else if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGI(TAG, "No config found, loading defaults");
    load_and_save_defaults(handle);
    nvs_commit(handle);
  } else {
    ESP_LOGE(TAG, "Failed to read NVS: %s", esp_err_to_name(err));
  }
  nvs_close(handle);
  return ESP_OK;
}

bool param_set(const char* name, float value, params_entry_t* p, uint16_t* id) {
  for (size_t i = 0; i < params_count; i++) {
    if (strncmp(params_list[i].name, name, PARAM_NAME_LEN) == 0) {
      if (value < params_list[i].min_value ||
          value > params_list[i].max_value) {
        return false;
      }
      nvs_handle_t handle;
      esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
      if (err != ESP_OK) {
        return false;
      }

      uint32_t buf = 0;
      memcpy(&buf, &value, sizeof(float));
      nvs_set_u32(handle, params_list[i].name, buf);
      nvs_commit(handle);
      nvs_close(handle);

      *params_list[i].value_p = value;
      memcpy(p, &params_list[i], sizeof(params_entry_t));
      *id = i;
      return true;
    }
  }
  return false;
}

bool param_get(const char* name, params_entry_t* p) {
  for (size_t i = 0; i < params_count; i++) {
    if (strncmp(params_list[i].name, name, PARAM_NAME_LEN) == 0) {
      memcpy(p, &params_list[i], sizeof(params_entry_t));
      return true;
    }
  }
  return false;
}