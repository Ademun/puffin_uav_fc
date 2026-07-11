#include "status.h"

static _Atomic uint32_t s_sensor_health = 0;
static _Atomic uint32_t s_custom_flags = 0;
static _Atomic arm_state_t s_arm_state = ARM_STATE_DISARMED;

void status_sensor_set_healthy(const uint32_t mav_sys_status_sensor_bit) {
  atomic_fetch_or_explicit(&s_sensor_health, mav_sys_status_sensor_bit, memory_order_release);
}

void status_sensor_set_unhealthy(const uint32_t mav_sys_status_sensor_bit) {
  atomic_fetch_or_explicit(&s_sensor_health, ~mav_sys_status_sensor_bit, memory_order_release);
}

uint32_t status_sensor_health(void) {
  return atomic_load_explicit(&s_sensor_health, memory_order_acquire);
}

uint32_t status_sensors_unhealthy(void) {
  return STATUS_SENSOR_REQUIRED_MASK & ~status_sensor_health();
}

void status_flag_set(const status_flag_t flag) {
  atomic_fetch_or_explicit(&s_custom_flags, (uint32_t)flag, memory_order_release);
}

void status_flag_clear(const status_flag_t flag) {
  atomic_fetch_or_explicit(&s_custom_flags, ~(uint32_t)flag, memory_order_release);
}

uint32_t status_missing_flags(void) {
  uint32_t flags = atomic_load_explicit(&s_custom_flags, memory_order_acquire);
  return STATUS_FLAG_REQUIRED_MASK & ~ flags;
}

bool arm_is_armed(void) {
  return atomic_load_explicit(&s_arm_state, memory_order_acquire) == ARM_STATE_ARMED;
}

bool arm_is_ready_to_arm(void) {
  return status_sensors_unhealthy() == 0 && status_missing_flags() == 0;
}

bool arm_toggle_armed(bool want_armed) {
  if (want_armed) {
    if (!arm_is_ready_to_arm()) {
      return false;
    }
    arm_state_t expected = ARM_STATE_DISARMED;
    return atomic_compare_exchange_strong_explicit(&s_arm_state, &expected, ARM_STATE_ARMED, memory_order_acq_rel, memory_order_acquire);
  }
  atomic_store_explicit(&s_arm_state, ARM_STATE_DISARMED, memory_order_release);
  return true;
}