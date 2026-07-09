idf_component_register(
    SRCS "imu.c" "geom.c" "Mahony.c"            
    INCLUDE_DIRS "."           
    REQUIRES driver esp_driver_i2c         
)