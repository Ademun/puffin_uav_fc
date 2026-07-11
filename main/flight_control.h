#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"

TaskHandle_t flight_control_start(i2c_master_dev_handle_t imu_handle);