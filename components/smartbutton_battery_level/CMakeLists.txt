# -----------------------------------------------------------------------------------
# Copyright 2025, Gilles Zunino
# -----------------------------------------------------------------------------------

set(srcs
    "smartbutton_battery_level.c"
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_adc esp_driver_gpio esp_system
)