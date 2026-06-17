/*
 * ======================================================================================
 * ESP32-C6 ULTRA-LOW POWER (ULP) BLE ANEMOMETER
 * ======================================================================================
 * * DESCRIPTION:
 *    This firmware transforms an ESP32-C6 into a high-efficiency wind speed sensor.
 *    It uses the ULP (Ultra-Low Power) co-processor to count pulses from a reed
 *    switch while the main core are in Deep Sleep.
 * * HOW IT WORKS:
 *    1. SLEEP: The ESP32 enters Deep Sleep (consuming ~10-15µA).
 *    2. ULP MONITORING: The ULP co-processor remains active, using interrupt-driven
 *        edge detection on SENSOR_PIN to detect magnet passes.
 *    3. WAKEUP: Every 5 seconds, the main core wakes up to process the ULP data.
 *    4. LOGIC ENGINE:
 *        - If Wind Speed > 0: Calculates speed and broadcasts immediately via BLE.
 *        - If Speed Changes: Broadcasts immediately to show the change.
 *        - If No Wind: Skips BLE transmission to save battery.
 *        - Heartbeat: Every 60 seconds, it forces a broadcast (even if no wind) so Home
 *          Assistant knows the sensor is still online.
 * * DATA PROTOCOL:
 *    Uses BTHome V2 (Bluetooth Low Energy). Compatible with Home Assistant and Shelly
 * * HARDWARE NOTES:
 *    - SENSOR_PIN must be an RTC-capable pin.
 *    - Pulses: 2 pulses per revolution (assuming 2 magnets).
 *    - Radius: 0.078m (center to cup middle).
 *    - Calibration: 2.5x factor to compensate for cup drag/aerodynamics.
 * * POWER CONSUMPTION:
 *    - Deep Sleep: ~15µA (it depends by the board, less feautures is better)
 *    - BLE Broadcast (1.5s): ~100mA
 *    - Estimated Battery Life (1000mAh Li-ion): >1 year with 1-minute heartbeats.
 * ======================================================================================
 */

#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/temperature_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "ble_hci.h"
#include "ulp_lp_core.h"
#include "ulp_main.h"
#include "lp_core_uart.h"
#include "anemometer.h"
#include "battery.h"
#include "temperature.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static const char *TAG = "anemometer";


static void init_ulp_program(void);
static void init_ble(float wind_speed_mps, uint8_t battery_percent, int16_t internal_temp_centi, uint8_t packet_id);
static void build_ble_adv(float wind_speed_mps, uint8_t battery_percent, int16_t internal_temp_centi, uint8_t packet_id);

static RTC_DATA_ATTR bool ulp_program_initialized;
static RTC_DATA_ATTR uint32_t last_pulse_count;
static RTC_DATA_ATTR uint32_t heartbeat_elapsed_seconds;
static RTC_DATA_ATTR uint32_t last_advertised_pulse_delta;
static RTC_DATA_ATTR uint8_t bthome_packet_id;

static uint8_t s_adv_data[31];
static uint8_t s_adv_len;

void app_main(void)
{
#if CONFIG_LOG_DEFAULT_LEVEL > 0
    /*
    *   Set default log level to WARN to reduce noise, but set INFO for our tag.
    */
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    /*
    *  If user is using USB-serial-jtag then serial monitor needs some time to
    *  re-connect to the USB port. We wait 3 sec here to allow for it to make the reconnection
    *  before we print anything. Otherwise the chip will go back to sleep again before the user
    *  has time to monitor any output.
    */
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "Main processor will wake every %d seconds", SLEEP_DURATION);

    uint32_t wake_causes = esp_sleep_get_wakeup_causes();

    if (wake_causes & BIT(ESP_SLEEP_WAKEUP_ULP)) {
        ESP_LOGI(TAG, "ULP woke up the main CPU!");
    }

    if (wake_causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) {
        ESP_LOGI(TAG, "Timer woke up the main CPU!");
    }
#endif /* CONFIG_LOG_DEFAULT_LEVEL > 0 */

    /* Load the LP program and initialize NVS flash only once on cold boot */
    if (!ulp_program_initialized) {
        ESP_LOGI(TAG, "Initializing ULP program and Flash...");
        init_ulp_program();
        ulp_program_initialized = true;
        
        /* Initialize NVS once since it doesn't need re-init for pure non-connectable BLE beacons */
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        /* Baseline on first boot to avoid using stale/uninitialized pulse deltas */
        last_pulse_count = ulp_pulse_count;
        heartbeat_elapsed_seconds = 0;
        last_advertised_pulse_delta = 0;
        bthome_packet_id = 0;
    }

    heartbeat_elapsed_seconds += SLEEP_DURATION;
    uint32_t pulse_delta = ulp_pulse_count - last_pulse_count;
    bool wind_changed = (pulse_delta != last_advertised_pulse_delta);
    bool heartbeat_due = (heartbeat_elapsed_seconds >= HEARTBEAT_INTERVAL);
    bool should_advertise = wind_changed || heartbeat_due;
    uint32_t heartbeat_remaining_s = heartbeat_due ? 0 : (HEARTBEAT_INTERVAL - heartbeat_elapsed_seconds);

    if (should_advertise) {
        /* Compute telemetry only when we are going to advertise */
        float rotations = (float)pulse_delta / PULSES_PER_ROTATION;
        float rps = rotations / (float)SLEEP_DURATION;
        float rpm = rps * 60.0f;
        float wind_speed_mps = rps * (2.0f * 3.14159265358979323846f * RADIUS) * CALIBRATION_FACTOR;
        float wind_speed_kmh = wind_speed_mps * 3.6f;
        uint16_t battery_mv = read_battery_mv(TAG);
        uint8_t battery_percent = battery_mv_to_percent(battery_mv);
        int16_t internal_temp_centi = read_internal_temp_centi(TAG);
        float internal_temp_c = (float)internal_temp_centi / 100.0f;

        /* Print speed if changed. */
        if (wind_changed) {
            ESP_LOGI(TAG,
                    "Pulse count: %"PRIu32" (+%"PRIu32") | RPM: %.1f | Wind: %.2f m/s (%.2f km/h) | Battery: %u mV (%u%%) | Temperature: %.2f °C",
                    ulp_pulse_count,
                    pulse_delta,
                    rpm,
                    wind_speed_mps,
                    wind_speed_kmh,
                    battery_mv,
                    battery_percent,
                    internal_temp_c);
        }

        /* Heartbeat every 60 seconds, if no pulse count changing */
        if (heartbeat_due) {
            ESP_LOGI(TAG,
                    "Heartbeat - Pulse count: %"PRIu32" | RPM: %.1f | Wind: %.2f m/s (%.2f km/h) | Battery: %u mV (%u%%) | Temperature: %.2f °C",
                    ulp_pulse_count,
                    rpm,
                    wind_speed_mps,
                    wind_speed_kmh,
                    battery_mv,
                    battery_percent,
                    internal_temp_c);
        }

        /* Broadcast a BTHome frame as a non-connectable BLE beacon only when needed */
        init_ble(wind_speed_mps, battery_percent, internal_temp_centi, bthome_packet_id);
        bthome_packet_id = (uint8_t)(((uint16_t)bthome_packet_id + 1U) & 0xFFU);

        /* Keep beaconing for a short window before deep sleep */
        vTaskDelay(pdMS_TO_TICKS(BLE_ADV_DURATION_MS));

        /* Stop advertising before sleeping */
        ESP_ERROR_CHECK(ble_hci_set_adv_enable(false));
        ESP_ERROR_CHECK(ble_hci_deinit());

        /* Baselines must be updated at the end of the math transmission cycle, 
         * regardless of whether it was triggered by a heartbeat or a true wind shift.
         */
        last_pulse_count = ulp_pulse_count;
        last_advertised_pulse_delta = pulse_delta;
        heartbeat_elapsed_seconds = 0;
    } else {
        ESP_LOGI(TAG,
                 "Pulse count: %"PRIu32" (no change) | Heartbeat in %"PRIu32"s | Wind unchanged | skipping BLE advertising",
                 ulp_pulse_count,
                 heartbeat_remaining_s);
    }
#if CONFIG_LOG_DEFAULT_LEVEL > 0
    /* Go back to sleep, only the ULP will run */
    ESP_LOGI(TAG, "Entering in deep sleep");

    /* Small delay to ensure the messages are printed */
    vTaskDelay(pdMS_TO_TICKS(500));
#endif /* CONFIG_LOG_DEFAULT_LEVEL > 0 */

    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(SLEEP_DURATION * 1000000ULL));

    esp_deep_sleep_start();
}

static void init_ble(float wind_speed_mps, uint8_t battery_percent, int16_t internal_temp_centi, uint8_t packet_id)
{
#if CONFIG_LOG_DEFAULT_LEVEL > 0
    uint8_t ble_addr[6] = {0};
    if (esp_read_mac(ble_addr, ESP_MAC_BT) == ESP_OK) {
        ESP_LOGI(TAG,
                 "BLE MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 ble_addr[0], ble_addr[1], ble_addr[2], ble_addr[3], ble_addr[4], ble_addr[5]);
    } else {
        ESP_LOGW(TAG, "Unable to read BLE MAC");
    }
#endif /* CONFIG_LOG_DEFAULT_LEVEL > 0 */

    ESP_ERROR_CHECK(ble_hci_init());

    build_ble_adv(wind_speed_mps, battery_percent, internal_temp_centi, packet_id);

    ble_hci_adv_param_t adv_param = {
        .adv_int_min = BLE_ADV_INTERVAL_UNITS,                  /* Sets the minimum and maximum time between advertisements */
        .adv_int_max = BLE_ADV_INTERVAL_UNITS,                  /* Setting them to the same value creates a fixed advertising interval */
        .adv_type = ADV_TYPE_NONCONN_IND,                       /* Non-connectable undirected advertising (beacon mode) */
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,                  /* Use public MAC address for better compatibility */
        .peer_addr = {0},                                       /* Not used in non-connectable mode, but set to zero just in case */
        .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,                 /* Not used in non-connectable mode, but set to public just in case */
        .channel_map = ADV_CHNL_ALL,                            /* Advertise on all channels (37, 38, 39) for best discoverability */
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, /* Allow all scan and connection requests (though we won't accept connections in non-connectable mode) */
    };

    ESP_ERROR_CHECK(ble_hci_set_adv_param(&adv_param));
    ESP_ERROR_CHECK(ble_hci_set_adv_data(s_adv_len, s_adv_data));
    ESP_ERROR_CHECK(ble_hci_set_adv_enable(true));
    ESP_LOGI(TAG, "BLE HCI advertising started");
}

static void build_ble_adv(float wind_speed_mps, uint8_t battery_percent, int16_t internal_temp_centi, uint8_t packet_id)
{
    uint8_t *cursor = s_adv_data;
    static const uint8_t adv_name[] = {
        'W', 'i', 'n', 'd', ' ', 'S', 'e', 'n', 's', 'o', 'r'
    };
    const uint8_t adv_name_len = (uint8_t)sizeof(adv_name);

    if (wind_speed_mps < 0.0f) {
        wind_speed_mps = 0.0f;
    }

    uint32_t speed_x100 = (uint32_t)lroundf(wind_speed_mps * 100.0f);
    if (speed_x100 > UINT16_MAX) {
        speed_x100 = UINT16_MAX;
    }

    uint8_t battery_payload = battery_percent;

    uint8_t temperature_payload[2] = {
        (uint8_t)(internal_temp_centi & 0xFF),
        (uint8_t)((internal_temp_centi >> 8) & 0xFF),
    };
    uint8_t speed_payload[2] = {
        (uint8_t)(speed_x100 & 0xFF),
        (uint8_t)((speed_x100 >> 8) & 0xFF),
    };

    /* GAP flags: general discoverable + BR/EDR not supported. */
    *cursor++ = 0x02;
    *cursor++ = 0x01;
    *cursor++ = 0x06;

    /* BTHome service data AD structure: len, type, UUID (0xFCD2), device info. */
    *cursor++ = 0x0E;
    *cursor++ = 0x16;
    *cursor++ = 0xD2;
    *cursor++ = 0xFC;
    *cursor++ = 0x40;

    /* Packet ID = object ID 0x00, 1 byte value. */
    *cursor++ = 0x00;
    *cursor++ = packet_id;

    /* Battery = object ID 0x01, 1 byte value. */
    *cursor++ = 0x01;
    *cursor++ = battery_payload;

    /* Temperature = object ID 0x02, 2 bytes little-endian in 0.01 degC units. */
    *cursor++ = 0x02;
    *cursor++ = temperature_payload[0];
    *cursor++ = temperature_payload[1];

    /* Speed = object ID 0x44, 2 bytes little-endian in 0.01 m/s units. */
    *cursor++ = 0x44;
    *cursor++ = speed_payload[0];
    *cursor++ = speed_payload[1];

    /* Complete Local Name AD structure at the end. */
    *cursor++ = (uint8_t)(1 + adv_name_len);
    *cursor++ = 0x09;
    for (uint8_t i = 0; i < adv_name_len; i++) {
        *cursor++ = adv_name[i];
    }

    s_adv_len = (uint8_t)(cursor - s_adv_data);

#if CONFIG_LOG_DEFAULT_LEVEL > 0
    if (esp_log_level_get(TAG) >= ESP_LOG_INFO) {
        char hex_line[(31 * 3) + 1] = {0};
        size_t pos = 0;
        for (uint8_t i = 0; i < s_adv_len; i++) {
            int written = snprintf(&hex_line[pos], sizeof(hex_line) - pos, "%02X%s",
                                s_adv_data[i],
                                (i + 1U < s_adv_len) ? " " : "");
            if (written <= 0) {
                break;
            }
            pos += (size_t)written;
            if (pos >= sizeof(hex_line)) {
                break;
            }
        }
        ESP_LOGI(TAG, "BTHome V2 payload (%d bytes, packet id %u): %s", s_adv_len, packet_id, hex_line);
    }
#endif /* CONFIG_LOG_DEFAULT_LEVEL > 0 */
}

static void init_ulp_program(void)
{
    /* Initialize selected GPIO as RTC IO, enable input, enable pullup, disable pulldown */
    rtc_gpio_init(SENSOR_PIN);
    rtc_gpio_set_direction(SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en(SENSOR_PIN);
    rtc_gpio_pulldown_dis(SENSOR_PIN);

    ESP_ERROR_CHECK(ulp_lp_core_load_binary(ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start)));

    /* Start the program */
    ulp_lp_core_cfg_t cfg = {
        .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU,
    };

    ESP_ERROR_CHECK(ulp_lp_core_run(&cfg));
}