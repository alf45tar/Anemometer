#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_bt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "ulp_lp_core.h"
#include "ulp_main.h"
#include "lp_core_uart.h"
#include "anemometer.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static const char *TAG = "anemometer";


static void init_ulp_program(void);
static void init_ble(float wind_speed_mps, uint8_t battery_percent);
static void build_ble_adv(float wind_speed_mps, uint8_t battery_percent);
static void ble_host_task(void *param);
static void ble_on_sync(void);
static void start_ble_advertising(void);
static uint16_t read_battery_mv(void);
static uint8_t battery_mv_to_percent(uint16_t battery_mv);

static RTC_DATA_ATTR bool ulp_program_initialized;
static RTC_DATA_ATTR uint32_t last_pulse_count;
static RTC_DATA_ATTR uint32_t heartbeat_elapsed_seconds;
static RTC_DATA_ATTR uint32_t last_advertised_pulse_delta;

static uint8_t s_adv_data[31];
static uint8_t s_adv_len;
static uint8_t s_own_addr_type;
static volatile bool s_ble_synced;
static volatile bool s_adv_started;

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    if (esp_log_level_get(TAG) >= ESP_LOG_INFO) {
        /*
        *  If user is using USB-serial-jtag then serial monitor needs some time to
        *  re-connect to the USB port. We wait 3 sec here to allow for it to make the reconnection
        *  before we print anything. Otherwise the chip will go back to sleep again before the user
        *  has time to monitor any output.
        */
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    ESP_LOGI(TAG, "Main processor will wake every %d seconds", SLEEP_DURATION);

    uint32_t wake_causes = esp_sleep_get_wakeup_causes();

    if (wake_causes & BIT(ESP_SLEEP_WAKEUP_ULP)) {
        ESP_LOGI(TAG, "ULP woke up the main CPU!");
    }

    if (wake_causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) {
        ESP_LOGI(TAG, "Timer woke up the main CPU!");
    }

    /* Load the LP program only once so timer wakeups do not reset its state. */
    if (!ulp_program_initialized) {
        ESP_LOGI(TAG, "Initializing ULP program!");
        init_ulp_program();
        ulp_program_initialized = true;

        /* Baseline on first boot to avoid using stale/uninitialized pulse deltas. */
        last_pulse_count = ulp_pulse_count;
        heartbeat_elapsed_seconds = 0;
    }

    heartbeat_elapsed_seconds += SLEEP_DURATION;
    uint32_t pulse_delta = ulp_pulse_count - last_pulse_count;
    bool wind_changed = (pulse_delta != last_advertised_pulse_delta);
    bool heartbeat_due = (heartbeat_elapsed_seconds >= HEARTBEAT_INTERVAL);
    bool should_advertise = wind_changed || heartbeat_due;
    uint32_t heartbeat_remaining_s = heartbeat_due ? 0 : (HEARTBEAT_INTERVAL - heartbeat_elapsed_seconds);

    if (should_advertise) {
        /* Compute telemetry only when we are going to advertise. */
        float rotations = (float)pulse_delta / PULSES_PER_ROTATION;
        float rps = rotations / (float)SLEEP_DURATION;
        float rpm = rps * 60.0f;
        float wind_speed_mps = rps * ANEMOMETER_FACTOR_MPS_PER_RPS;
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
        init_ble(wind_speed_mps, battery_percent);
        last_advertised_pulse_delta = pulse_delta;

        /* Wait briefly so the host can sync and start advertising. */
        vTaskDelay(pdMS_TO_TICKS(250));
        if (!s_ble_synced) {
            ESP_LOGW(TAG, "BLE host not synced yet");
        }

        /* Keep beaconing for a short window before deep sleep. */
        vTaskDelay(pdMS_TO_TICKS(BLE_ADV_DURATION_MS));
    } else {
        ESP_LOGI(TAG,
                 "Pulse count: %"PRIu32" (no change) | Heartbeat in %"PRIu32"s | Wind unchanged | skipping BLE advertising",
                 ulp_pulse_count,
                 heartbeat_remaining_s);
    }

    /* Go back to sleep, only the ULP will run */
    ESP_LOGI(TAG, "Entering in deep sleep");

    /* Small delay to ensure the messages are printed */
    if (esp_log_level_get(TAG) >= ESP_LOG_INFO) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(SLEEP_DURATION * 1000000ULL));

    esp_deep_sleep_start();
}

static void init_ble(float wind_speed_mps, uint8_t battery_percent)
{
    /* NVS is required so NimBLE can store BLE state and controller data in flash */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize the BLE controller and NimBLE host stack */
    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb = ble_on_sync;

    build_ble_adv(wind_speed_mps, battery_percent);

    s_ble_synced = false;
    s_adv_started = false;

    /* Start the NimBLE host task */
    nimble_port_freertos_init(ble_host_task);
}

static void build_ble_adv(float wind_speed_mps, uint8_t battery_percent)
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
    *cursor++ = 0x09;
    *cursor++ = 0x16;
    *cursor++ = 0xD2;
    *cursor++ = 0xFC;
    *cursor++ = 0x40;

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
        ESP_LOGI(TAG, "BTHome V2 payload (%d bytes): %s", s_adv_len, hex_line);
    }
}

static uint16_t read_battery_mv(void)
{
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

static void ble_on_sync(void)
{
    uint8_t ble_addr[6] = {0};

    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: rc=%d", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(s_own_addr_type, ble_addr, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG,
                 "BLE MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 ble_addr[5],
                 ble_addr[4],
                 ble_addr[3],
                 ble_addr[2],
                 ble_addr[1],
                 ble_addr[0]);
    } else {
        ESP_LOGW(TAG, "Unable to read BLE MAC, rc=%d", rc);
    }

    s_ble_synced = true;
    start_ble_advertising();
}

static void start_ble_advertising(void)
{
    struct ble_gap_adv_params adv_params = {0};

    //adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    //adv_params.disc_mode = BLE_GAP_DISC_MODE_NON;
    //adv_params.itvl_min = BLE_ADV_INTERVAL_UNITS;
    //adv_params.itvl_max = BLE_ADV_INTERVAL_UNITS;

    int rc = ble_gap_adv_set_data(s_adv_data, s_adv_len);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_data failed: rc=%d", rc);
        return;
    }

    rc = ble_gap_adv_start(s_own_addr_type,
                           NULL,
                           BLE_HS_FOREVER,
                           &adv_params,
                           NULL,
                           NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: rc=%d", rc);
        return;
    }

    s_adv_started = true;
    ESP_LOGI(TAG, "BLE advertising started");
}

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void init_ulp_program(void)
{
    /* Initialize selected GPIO as RTC IO, enable input, enable pullup, disable pulldown */
    rtc_gpio_init(SENSOR_PIN);
    rtc_gpio_set_direction(SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en(SENSOR_PIN);
    rtc_gpio_pulldown_dis(SENSOR_PIN);

    lp_core_uart_cfg_t uart_cfg = LP_CORE_UART_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(lp_core_uart_init(&uart_cfg));

    esp_err_t err = ulp_lp_core_load_binary(ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start));
    ESP_ERROR_CHECK(err);

    /* Start the program */
    ulp_lp_core_cfg_t cfg = {
        .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU,
    };

    err = ulp_lp_core_run(&cfg);
    ESP_ERROR_CHECK(err);
}