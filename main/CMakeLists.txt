idf_component_register(
        SRCS "params.c" "main.c" "flight_control.c" "wifi.c" "comms.c" "telemetry.c" "status.c" "signal.c"
        INCLUDE_DIRS "." "../lib" "../components")
