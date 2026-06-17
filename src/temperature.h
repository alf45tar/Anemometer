#pragma once

#include <stdint.h>
#include "esp_log.h"
#include "driver/temperature_sensor.h"

/**
 * @brief Read the internal temperature sensor
 * 
 * @return Temperature in centidegrees Celsius (int16_t)
 *         Returns 0 if the sensor read fails
 */
static inline int16_t read_internal_temp_centi(const char *TAG )
{ 
    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    float temperature_c = 0.0f;

    if (temperature_sensor_install(&temp_sensor_config, &temp_sensor) != ESP_OK) {
        ESP_LOGW(TAG, "temperature_sensor_install failed");
        return 0;
    }

    if (temperature_sensor_enable(temp_sensor) != ESP_OK) {
        ESP_LOGW(TAG, "temperature_sensor_enable failed");
        temperature_sensor_uninstall(temp_sensor);
        return 0;
    }

    if (temperature_sensor_get_celsius(temp_sensor, &temperature_c) != ESP_OK) {
        ESP_LOGW(TAG, "temperature_sensor_get_celsius failed");
        temperature_c = 0.0f;
    }

    if (temperature_sensor_disable(temp_sensor) != ESP_OK) {
        ESP_LOGW(TAG, "temperature_sensor_disable failed");
    }
    if (temperature_sensor_uninstall(temp_sensor) != ESP_OK) {
        ESP_LOGW(TAG, "temperature_sensor_uninstall failed");
    }

    long temp_centi = lroundf(temperature_c * 100.0f);
    if (temp_centi < INT16_MIN) {
        temp_centi = INT16_MIN;
    } else if (temp_centi > INT16_MAX) {
        temp_centi = INT16_MAX;
    }

    return (int16_t)temp_centi;
}
