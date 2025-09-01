// -----------------------------------------------------------------------------------
// Copyright 2025, Gilles Zunino
// -----------------------------------------------------------------------------------

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>

#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

#include "smartbutton_battery_level.h"


// Battery Level Log Tag
static const char *VBatMeasurementTag = "vbat_measure";



//
// Maximum and minimum battery voltages
//  * The highest voltage possible on the board is 3 * 1.5 = 4.5V
//  * The lowest voltage an ESP32 will operate at on our board is 2.6V. This assumes the battery can still deliver at least 30mA
// 
//  Values below are expressed in mV
//
static const int BATTERY_VOLTAGE_MAX_MV = 4500;
static const int BATTERY_VOLTAGE_MIN_MV = 2650;


// GPIO used to enable the battery measurement voltage divider - The divider is energized when this GPIO is high
static const gpio_num_t BATTERY_MEASUREMENT_ENABLE_GPIO = GPIO_NUM_32;

// The ADC unit the battery measurement voltage divider is connected to
static const adc_unit_t BATTERY_DIVIDER_ADC_UNIT = ADC_UNIT_1;
static const adc_channel_t BATTERY_DIVIDER_ADC_CHANNEL = ADC_CHANNEL_7;

//
// Attenuation applied to the signal pre ADC conversion
//  * 0dB   -> Can measure [100mV .. 950mV]  - Error [-23mV .. 23mV]
//  * 2.5dB -> Can measure [100mV .. 1250mV] - Error [-30mV .. 30mV]
//  * 6dB   -> Can measure [150mV .. 1750mV] - Error [-40mv .. 40mV]
//  * 12dB  -> Can measure [150mV .. 2450mV] - Error [-60mV .. 60mV]
// These ranges were documented in ESP-IDF v4.4 and have been removed since - See https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/api-reference/peripherals/adc.html#_CPPv426esp_adc_cal_raw_to_voltage8uint32_tPK29esp_adc_cal_characteristics_t
// See [Functional Description -> Analog Peripherials -> Analog to Digital Converter] in https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
// 
static const adc_atten_t BATTERY_DIVIDER_ADC_ATTEN = ADC_ATTEN_DB_12;

//
// ESP32 built in ADC offers a maximum of 12 bits bandwidth. This represents 4096 possible values or "steps"
// See https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc/index.html
//
static const adc_bitwidth_t BATTERY_DIVIDER_ADC_BITWIDTH = ADC_BITWIDTH_12;



// ADC handle for one shot reading and line fitting calibration
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_adc_linear_calibration_handle = NULL;



static esp_err_t initialize_oneshot_adc(adc_unit_t adc_unit, adc_channel_t adc_channel);
static esp_err_t deinitialize_oneshot_adc();

static esp_err_t enable_disable_battery_voltage_divider(bool enable);

static esp_err_t get_average_battery_voltage(uint32_t* mV);
static esp_err_t read_battery_voltage_divider(int* mV, TickType_t delay, uint16_t retryCount);
static int convert_measured_voltage_divider_to_battery_voltage(int adc_measured);
static uint8_t convert_battery_voltage_to_percentage(uint32_t battery_voltage);


esp_err_t initialize_battery_level_measurement() {
    ESP_RETURN_ON_FALSE((s_adc_handle == NULL) && (s_adc_linear_calibration_handle == NULL), ESP_ERR_INVALID_STATE, VBatMeasurementTag, "'initialize_battery_level_measurement()' has already been called");
    ESP_RETURN_ON_ERROR(initialize_oneshot_adc(BATTERY_DIVIDER_ADC_UNIT, BATTERY_DIVIDER_ADC_CHANNEL), VBatMeasurementTag, "initialize_oneshot_adc() failed");
    return ESP_OK;
}


esp_err_t deinitialize_battery_level_measurement() {
    ESP_RETURN_ON_FALSE((s_adc_handle != NULL) && (s_adc_linear_calibration_handle != NULL), ESP_ERR_INVALID_STATE, VBatMeasurementTag, "'initialize_battery_level_measurement()' has already been called");
    ESP_RETURN_ON_ERROR(deinitialize_oneshot_adc(), VBatMeasurementTag, "deinitialize_oneshot_adc() failed");
    return ESP_OK;
}

esp_err_t estimate_battery_remaining_percentage(uint8_t* battery_percentage) {
    ESP_RETURN_ON_FALSE((s_adc_handle != NULL) && (s_adc_linear_calibration_handle != NULL), ESP_ERR_INVALID_STATE, VBatMeasurementTag, "'initialize_battery_level_measurement()' has not been called");
    ESP_RETURN_ON_FALSE(battery_percentage != NULL, ESP_ERR_INVALID_ARG, VBatMeasurementTag, "'battery_percentage' must not be NULL");

    *battery_percentage = 0;

    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(enable_disable_battery_voltage_divider(true), error, VBatMeasurementTag, "enable_disable_battery_voltage_divider(true) failed");

        uint32_t measured_voltage;
        ESP_GOTO_ON_ERROR(get_average_battery_voltage(&measured_voltage), error, VBatMeasurementTag, "get_average_battery_voltage() failed");

    ESP_GOTO_ON_ERROR(enable_disable_battery_voltage_divider(false), error, VBatMeasurementTag, "enable_disable_battery_voltage_divider(false) failed");

    *battery_percentage = convert_battery_voltage_to_percentage(measured_voltage);
    return ret;

error:
    esp_err_t err = enable_disable_battery_voltage_divider(false);
    if (err != ESP_OK) {
        ESP_LOGE(VBatMeasurementTag, "enable_disable_battery_voltage_divider() failed with '%s' [%" PRId32 "]", esp_err_to_name(err), err);
    }

    return ret;
}

static esp_err_t initialize_oneshot_adc(adc_unit_t adc_unit, adc_channel_t adc_channel) {
    ESP_ERROR_CHECK(s_adc_handle != NULL ? ESP_ERR_INVALID_STATE : ESP_OK);
    ESP_ERROR_CHECK(s_adc_linear_calibration_handle != NULL ? ESP_ERR_INVALID_STATE : ESP_OK);

    s_adc_handle = NULL;
    s_adc_linear_calibration_handle = NULL;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = adc_unit,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    adc_oneshot_unit_handle_t staged_adc_handle;
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_config, &staged_adc_handle), VBatMeasurementTag, "adc_oneshot_new_unit() failed");

    s_adc_handle = staged_adc_handle;

    adc_oneshot_chan_cfg_t adc_config = {
        .atten = BATTERY_DIVIDER_ADC_ATTEN,
        .bitwidth = BATTERY_DIVIDER_ADC_BITWIDTH
    };

    esp_err_t ret;
    ESP_GOTO_ON_ERROR(adc_oneshot_config_channel(s_adc_handle, adc_channel, &adc_config), error, VBatMeasurementTag, "adc_oneshot_config_channel() failed");

    adc_cali_scheme_ver_t supportedCalibrationSchemes;
    ESP_GOTO_ON_ERROR(adc_cali_check_scheme(&supportedCalibrationSchemes), error, VBatMeasurementTag, "adc_cali_check_scheme() failed");

    adc_cali_line_fitting_efuse_val_t lineFittingEFuseValues;
    ESP_GOTO_ON_ERROR(adc_cali_scheme_line_fitting_check_efuse(&lineFittingEFuseValues), error, VBatMeasurementTag, "adc_cali_scheme_line_fitting_check_efuse() failed");
    ESP_GOTO_ON_FALSE(!(lineFittingEFuseValues == ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF), ESP_ERR_NOT_SUPPORTED, error, VBatMeasurementTag, "Line calibration requires static VREF (ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF)");

    adc_cali_line_fitting_config_t lineCalibrationConfiguration = {
        .unit_id = BATTERY_DIVIDER_ADC_UNIT,
        .atten = BATTERY_DIVIDER_ADC_ATTEN,
        .bitwidth = BATTERY_DIVIDER_ADC_BITWIDTH
    };

    adc_cali_handle_t staged_linear_calibration_handle;
    ESP_GOTO_ON_ERROR(adc_cali_create_scheme_line_fitting(&lineCalibrationConfiguration, &staged_linear_calibration_handle), error, VBatMeasurementTag, "adc_cali_create_scheme_line_fitting() failed");

    s_adc_linear_calibration_handle = staged_linear_calibration_handle;

    return ESP_OK;

error:
    esp_err_t err = deinitialize_oneshot_adc();
    if (err != ESP_OK) {
        ESP_LOGE(VBatMeasurementTag, "deinitialize_oneshot_adc() failed with '%s' [%" PRId32 "]", esp_err_to_name(err), err);
    }   

    return ret;
}

static esp_err_t deinitialize_oneshot_adc() {
    esp_err_t err = ESP_OK;

    if (s_adc_linear_calibration_handle != NULL) {
        adc_cali_delete_scheme_line_fitting(s_adc_linear_calibration_handle);
        s_adc_linear_calibration_handle = NULL;
    }

    if (s_adc_handle != NULL) {
        err = adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
    }

    return err;
}

static esp_err_t enable_disable_battery_voltage_divider(bool enable) {
    uint32_t level = enable ? 1 : 0;
    ESP_RETURN_ON_ERROR(gpio_set_direction(BATTERY_MEASUREMENT_ENABLE_GPIO, GPIO_MODE_OUTPUT), VBatMeasurementTag, "gpio_set_direction(BATTERY_MEASUREMENT_ENABLE_GPIO, GPIO_MODE_OUTPUT) failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(BATTERY_MEASUREMENT_ENABLE_GPIO, level), VBatMeasurementTag, "gpio_set_level(BATTERY_MEASUREMENT_ENABLE_GPIO, %" PRId32 ") failed", level);
    return ESP_OK;
}

static esp_err_t get_average_battery_voltage(uint32_t* mV) {
    const uint16_t SampleCount = CONFIG_SMARTBUTTON_BATTERY_LEVEL_SAMPLE_COUNT;
    const TickType_t SampleDelay = CONFIG_SMARTBUTTON_BATTERY_LEVEL_SAMPLE_DELAY;

    const TickType_t RetryDelay = CONFIG_SMARTBUTTON_BATTERY_LEVEL_SAMPLE_RETRY_DELAY;
    const uint16_t RetryCount = CONFIG_SMARTBUTTON_BATTERY_LEVEL_SAMPLE_RETRY_COUNT;

    *mV = 0;

    esp_err_t ret = ESP_OK;
    uint32_t total = 0;
    for (uint16_t sampleIndex = 0; sampleIndex < SampleCount; sampleIndex++) {
        int mV;
        ESP_GOTO_ON_ERROR(read_battery_voltage_divider(&mV, RetryDelay, RetryCount), error, VBatMeasurementTag, "read_battery_voltage_divider() failed");
        total += mV;

        vTaskDelay(SampleDelay);
    }

    *mV = total / SampleCount;

    ESP_LOGI(VBatMeasurementTag, "[VBAT] Average battery voltage -> %" PRId32 " mV", *mV);

    return ESP_OK;
    
error:
    return ret;
}

static esp_err_t read_battery_voltage_divider(int* mV, TickType_t delay, uint16_t retryCount) {
    esp_err_t err = ESP_OK;
    uint16_t retries = 0;

    *mV = 0;

    do {
        int adc_raw;
        err = adc_oneshot_get_calibrated_result(s_adc_handle, s_adc_linear_calibration_handle, BATTERY_DIVIDER_ADC_CHANNEL, &adc_raw);
        switch (err) {
            case ESP_ERR_TIMEOUT:
                if (retries < retryCount) {
                    retries++;
                    vTaskDelay(delay);
                } else {
                    return err;
                }
            break;
        case ESP_OK:
                *mV = convert_measured_voltage_divider_to_battery_voltage(adc_raw);
#if CONFIG_SMARTBUTTON_BATTERY_LEVEL_ENABLE_DEBUG_LOG
                ESP_LOGI(VBatMeasurementTag, "[VBAT] ADC%" PRId32 " Channel[%" PRId32 "] -> %" PRId32 " | %" PRId32 " mV", BATTERY_DIVIDER_ADC_UNIT + 1, BATTERY_DIVIDER_ADC_CHANNEL, adc_raw, *mV);
#endif
            return ESP_OK;

        default:
            return err;
        }
    } while (false);

    return ESP_ERR_TIMEOUT;
}

static int convert_measured_voltage_divider_to_battery_voltage(int adc_measured) {
    //
    // ESP 32 ADC can measures V in the [0V .. 3.3V] range with a (default) VRef = 1100mV
    // Our battery voltage is roughly in the [0V .. BATTERY_VOLTAGE_MAX_MV] range which is higher than 3.3V
    //
    // To measure more than the ESP32 maximum, we use a voltage divider and attenuation to bring the voltage between [0 .. 2.25V]
    // Bringing the voltage close to the middle of the ADC range ([0..3.3V] -> [1.65V]) improves measurement accuracy
    //
    /* Ignoring the BJT VCE(SAT), the measured voltage is approximately Vout = Vbat / 2

                                               Vbat
                                                │
                                                │
                                               ┌┴┐
                                               │ │ R1 = 10kΩ
                                               │ │
                                               └┬┘
                                                │
                                                C
                                                │
                                              |/
                                       -- B --|    NPN (2N2222)
                                              |\
                                                |
                                                E
                                                |
                                                │─────── Vout
                                               ┌┴┐
                                               │ │ R2 = 10kΩ
                                               │ │
                                               └┬┘
                                                │
                                               ─┴─
                                               GND
    */

    // Calculate effective battery voltage based on the voltage divider physical characteristics
    return 2 * adc_measured;
}

static uint8_t convert_battery_voltage_to_percentage(uint32_t battery_voltage) {
    if (battery_voltage <= BATTERY_VOLTAGE_MIN_MV) {
        return 0;
    } else {
        if (battery_voltage >= BATTERY_VOLTAGE_MAX_MV) {
            return 100;
        } else {
            uint8_t percentage = (100 * (battery_voltage - BATTERY_VOLTAGE_MIN_MV)) / (BATTERY_VOLTAGE_MAX_MV - BATTERY_VOLTAGE_MIN_MV);
            return percentage > 100 ? 100 : percentage;
        }
    }
}