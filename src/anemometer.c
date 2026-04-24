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
#include <math.h>
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "ble_hci.h"
#include "ulp_lp_core.h"
#include "ulp_main.h"
#include "lp_core_uart.h"
#include "anemometer.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static const char *TAG = "anemometer";


static void init_ulp_program(void);
static void init_ble(float wind_speed_mps, uint8_t battery_percent, uint8_t packet_id);
static void build_ble_adv(float wind_speed_mps, uint8_t battery_percent, uint8_t packet_id);
static uint16_t read_battery_mv(void);
static uint8_t battery_mv_to_percent(uint16_t battery_mv);

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

    /* Load the LP program only once so timer wakeups do not reset its state */
    if (!ulp_program_initialized) {
        ESP_LOGI(TAG, "Initializing ULP program!");
        init_ulp_program();
        ulp_program_initialized = true;

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
        uint16_t battery_mv = read_battery_mv();
        uint8_t battery_percent = battery_mv_to_percent(battery_mv);

        /* Print speed if changed. */
        if (wind_changed) {
            ESP_LOGI(TAG,
                    "Pulse count: %"PRIu32" (+%"PRIu32") | RPM: %.1f | Wind: %.2f m/s (%.2f km/h) | Battery: %u mV (%u%%)",
                    ulp_pulse_count,
                    pulse_delta,
                    rpm,
                    wind_speed_mps,
                    wind_speed_kmh,
                    battery_mv,
                    battery_percent);
            last_pulse_count = ulp_pulse_count;
            heartbeat_elapsed_seconds = 0;
        }

        /* Heartbeat every 60 seconds, if no pulse count changing */
        if (heartbeat_due) {
            ESP_LOGI(TAG,
                    "Heartbeat - Pulse count: %"PRIu32" | RPM: %.1f | Wind: %.2f m/s (%.2f km/h) | Battery: %u mV (%u%%)",
                    ulp_pulse_count,
                    rpm,
                    wind_speed_mps,
                    wind_speed_kmh,
                    battery_mv,
                    battery_percent);
            heartbeat_elapsed_seconds = 0;
        }

        /* Broadcast a BTHome frame as a non-connectable BLE beacon only when needed */
        init_ble(wind_speed_mps, battery_percent, bthome_packet_id);
        bthome_packet_id = (uint8_t)(((uint16_t)bthome_packet_id + 1U) & 0xFFU);
        last_advertised_pulse_delta = pulse_delta;

        /* Keep beaconing for a short window before deep sleep */
        vTaskDelay(pdMS_TO_TICKS(BLE_ADV_DURATION_MS));

        /* Stop advertising before sleeping */
        ESP_ERROR_CHECK(ble_hci_set_adv_enable(false));
        ESP_ERROR_CHECK(ble_hci_deinit());
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

static void init_ble(float wind_speed_mps, uint8_t battery_percent, uint8_t packet_id)
{
    /* NVS is required by the BT controller stack initialization path */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if CONFIG_LOG_DEFAULT_LEVEL > 0
    uint8_t ble_addr[6] = {0};
    if (esp_read_mac(ble_addr, ESP_MAC_BT) == ESP_OK) {
        ESP_LOGI(TAG,
                 "BLE MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 ble_addr[0],
                 ble_addr[1],
                 ble_addr[2],
                 ble_addr[3],
                 ble_addr[4],
                 ble_addr[5]);
    } else {
        ESP_LOGW(TAG, "Unable to read BLE MAC");
    }
#endif /* CONFIG_LOG_DEFAULT_LEVEL > 0 */

    ESP_ERROR_CHECK(ble_hci_init());

    build_ble_adv(wind_speed_mps, battery_percent, packet_id);

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

static void build_ble_adv(float wind_speed_mps, uint8_t battery_percent, uint8_t packet_id)
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
    uint8_t speed_payload[2] = {
        (uint8_t)(speed_x100 & 0xFF),
        (uint8_t)((speed_x100 >> 8) & 0xFF),
    };

    /* GAP flags: general discoverable + BR/EDR not supported. */
    *cursor++ = 0x02;
    *cursor++ = 0x01;
    *cursor++ = 0x06;

    /* BTHome service data AD structure: len, type, UUID (0xFCD2), device info. */
    *cursor++ = 0x0B;
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

static uint16_t read_battery_mv(void)
{
    /* To ensure accurate ADC readings with high-value resistors, a 0.1uF (100nF)
    *  ceramic capacitor MUST be placed between the ADC pin and GND.
    *  Without the 0.1uF "reservoir" cap, the 470k
    *  source impedance causes a significant voltage drop (sag) during
    *  the sampling window, resulting in artificially low readings.
    *  Resistor Divider Static Drain: assuming a 470k/470k divider,
    *  the continuous leakage current is calculated as:
    *         I ≈ 4.2V / 940,000Ω ≈ 4.5µA
    *  This 4.5µA drain is CONTINUOUS, even during deep sleep.
    *  This is such a low value that the internal chemical self-discharge
    *  of the battery itself probably consumes more.
    */
    adc_unit_t unit_id = ADC_UNIT_1;
    adc_channel_t channel = ADC_CHANNEL_0;
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_cali_handle_t cali_handle = NULL;
    bool use_cali = false;
    int total_raw = 0;
    int avg_raw = 0;
    int measured_mv = 0;

    if (adc_oneshot_io_to_channel(BATTERY_ADC_GPIO, &unit_id, &channel) != ESP_OK) {
        ESP_LOGW(TAG, "Battery ADC GPIO %d is not an ADC pin", BATTERY_ADC_GPIO);
        return 0;
    }

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = unit_id,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&init_config, &adc_handle) != ESP_OK) {
        ESP_LOGW(TAG, "adc_oneshot_new_unit failed");
        return 0;
    }

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    if (adc_oneshot_config_channel(adc_handle, channel, &chan_config) != ESP_OK) {
        ESP_LOGW(TAG, "adc_oneshot_config_channel failed");
        adc_oneshot_del_unit(adc_handle);
        return 0;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit_id,
        .chan = channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle) == ESP_OK) {
        use_cali = true;
    }
#endif

    /* Average several ADC readings to reduce noise on the battery input. */
    for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
        int raw = 0;
        if (adc_oneshot_read(adc_handle, channel, &raw) == ESP_OK) {
            total_raw += raw;
        }
    }

    avg_raw = total_raw / BATTERY_ADC_SAMPLES;
    if (use_cali) {
        if (adc_cali_raw_to_voltage(cali_handle, avg_raw, &measured_mv) != ESP_OK) {
            measured_mv = 0;
        }
    } else {
        measured_mv = (avg_raw * 3300) / 4095;
    }

    if (use_cali) {
        adc_cali_delete_scheme_curve_fitting(cali_handle);
    }
    adc_oneshot_del_unit(adc_handle);

    return (uint16_t)((float)measured_mv * BATTERY_VOLTAGE_DIVIDER_RATIO);
}

static uint8_t battery_mv_to_percent(uint16_t battery_mv)
{
    if (battery_mv <= BATTERY_MIN_MV) {
        return 0;
    }
    if (battery_mv >= BATTERY_MAX_MV) {
        return 100;
    }

    uint32_t num = (uint32_t)(battery_mv - BATTERY_MIN_MV) * 100U;
    uint32_t den = (uint32_t)(BATTERY_MAX_MV - BATTERY_MIN_MV);
    return (uint8_t)(num / den);
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