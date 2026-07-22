idf_component_register(
        SRCS "params.c" "main.c" "flight_control.c" "wifi.c" "comms.c" "telemetry.c" "status.c" "signal.c" "imu/imu.c"
        INCLUDE_DIRS "." "../lib" "imu"
        REQUIRES driver esp_driver_i2c mavlink nvs_flash
        PRIV_REQUIRES esp_event esp_timer esp_driver_ledc esp_netif esp_wifi
)
