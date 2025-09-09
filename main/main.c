// -----------------------------------------------------------------------------------
// Copyright 2025, Gilles Zunino
// -----------------------------------------------------------------------------------

#include "sdkconfig.h"

#include <freertos/FreeRTOS.h>

#include <esp_log.h>
#include <nvs_flash.h>

#include "smartbutton_battery_level.h"


// Smarter Button Log Tag
static const char *SmarterButtonTag = "smrt_btn";



void app_main(void) {
    // Initialize NVS — It is used to store PHY calibration data and WiFi driver configuration
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Initialize battery level measurement
    ESP_ERROR_CHECK(initialize_battery_level_measurement());


    do {
        uint8_t battery_percentage = 0;
        err = estimate_battery_remaining_percentage(&battery_percentage);
        if (err != ESP_OK) {
            ESP_LOGE(SmarterButtonTag, "Failed to estimate battery percentage: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(SmarterButtonTag, "Battery percentage: %" PRIu8 "%%", battery_percentage);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    } while (1);

    
    ESP_ERROR_CHECK(deinitialize_battery_level_measurement());
}
