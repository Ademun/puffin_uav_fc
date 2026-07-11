idf_component_register(
    SRCS "imu.c" "geom.c" "Mahony.c" "pid.c"
    INCLUDE_DIRS "."           
    REQUIRES driver esp_driver_i2c         
)