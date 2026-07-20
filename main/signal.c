#include "signal.h"

#include "driver/ledc.h"
#include "esp_log.h"

const char *TAG = "SIGNAL";
#define BUZZER_GPIO 32
#define LEDC_CHANNEL LEDC_CHANNEL_0

static TaskHandle_t signal_task_handle;

typedef struct {
  int freq;
  int duration_ms;
} signal_note_t;

static const signal_note_t *signal_melodies[SIGNAL_ENUM_END] = {
    (const signal_note_t[]){{523, 100}, {659, 100}, {784, 150}, {0, 0}},
    (const signal_note_t[]){{880, 80}, {0, 50}, {880, 80}, {0, 0}},
  (const signal_note_t[]){{440, 100}, {880, 100}, {0, 0}},
    (const signal_note_t[]){{440, 150}, {0, 100}, {440, 150}, {0, 0}},
    (const signal_note_t[]){{330, 100}, {660, 100}, {0, 0}},
    (const signal_note_t[]){{660, 100}, {330, 100}, {0, 0}},
    (const signal_note_t[]){{1000, 80}, {0, 80}, {1000, 80}, {0, 80}, {1000, 80}, {0, 0}},
    (const signal_note_t[]){{800, 100}, {0, 100}, {800, 100}, {0, 100}, {800, 100}, {0, 0}},
};

static const ledc_timer_config_t buzzer_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK,
};

static const ledc_channel_config_t buzzer_channel = {
    .gpio_num = BUZZER_GPIO,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
};

static const signal_note_t *current_melody = nullptr;
static int current_note_index = 0;
static uint64_t note_start_time_ms = 0;
static bool is_playing = false;

static void play_note(signal_note_t note) {
  if (note.freq > 0) {
    ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, note.freq);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 512);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
  } else {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
  }

  note_start_time_ms = pdTICKS_TO_MS(xTaskGetTickCount());
}

static void signal_update(void) {
  if (!is_playing)
    return;

  uint64_t now = pdTICKS_TO_MS(xTaskGetTickCount());
  uint64_t elapsed_ms = now - note_start_time_ms;
  int duration_ms = current_melody[current_note_index].duration_ms;

  if (elapsed_ms >= duration_ms) {
    current_note_index++;

    if (current_melody[current_note_index].freq == 0 && current_melody[current_note_index].duration_ms == 0) {
      is_playing = false;
      current_melody = nullptr;
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
      return;
    }

    play_note(current_melody[current_note_index]);
  }
}

static void signal_task(void *pvParameters) {
  while (1) {
    signal_update();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void signal_play(signal_codes code) {
  if (code >= SIGNAL_ENUM_END)
    return;
  is_playing = false;
  current_melody = signal_melodies[code];
  current_note_index = 0;
  is_playing = true;
  play_note(current_melody[0]);
}

TaskHandle_t signal_init(void) {
  ledc_timer_config(&buzzer_timer);
  ledc_channel_config(&buzzer_channel);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
  BaseType_t ret = xTaskCreatePinnedToCore(signal_task, "signal", 2048, nullptr, 15, &signal_task_handle, 0);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create TX task");
    return nullptr;
  }
  return signal_task_handle;
}