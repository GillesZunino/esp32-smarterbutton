// -----------------------------------------------------------------------------------
// Copyright 2025, Gilles Zunino
// -----------------------------------------------------------------------------------

#include <freertos/FreeRTOS.h>

#include <esp_log.h>
#include <nvs_flash.h>
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
    do {
        vTaskDelay(pdMS_TO_TICKS(1000));
    } while (1);
}
