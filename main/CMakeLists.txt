# -----------------------------------------------------------------------------------
# Copyright 2025, Gilles Zunino
# -----------------------------------------------------------------------------------

set(srcs
        "main.c"
)

idf_component_register(
        SRCS "${srcs}"
        INCLUDE_DIRS "."
        PRIV_REQUIRES esp_wifi nvs_flash
)
