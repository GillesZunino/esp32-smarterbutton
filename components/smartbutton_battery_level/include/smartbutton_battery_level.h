// -----------------------------------------------------------------------------------
// Copyright 2025, Gilles Zunino
// -----------------------------------------------------------------------------------

#include <esp_check.h>

esp_err_t initialize_battery_level_measurement();
esp_err_t deinitialize_battery_level_measurement();

esp_err_t estimate_battery_remaining_percentage(uint8_t* battery_percentage);