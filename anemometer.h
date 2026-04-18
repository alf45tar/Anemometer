#pragma once

#define SENSOR_PIN 6
#define SLEEP_DURATION 5
#define HEARTBEAT_INTERVAL 60

/* Anemometer conversion constants */
#define PULSES_PER_ROTATION 2.0f
#define ANEMOMETER_FACTOR_MPS_PER_RPS 1.0f

/* BLE/BTHome beacon behavior */
#define BLE_ADV_DURATION_MS 1500
#define BLE_ADV_INTERVAL_UNITS 1600 /* 1600 * 0.625 ms = 1 s */

/* Battery measurement via ADC */
#define BATTERY_ADC_GPIO 0
#define BATTERY_ADC_SAMPLES 8
#define BATTERY_VOLTAGE_DIVIDER_RATIO 2.0f
#define BATTERY_MIN_MV 3300
#define BATTERY_MAX_MV 4200
