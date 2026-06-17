#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "anemometer.h"

/* Battery filtered voltage state with RTC retention */
RTC_DATA_ATTR int32_t filtered_battery_mv = -1; /* -1 = uninitialized; stores mV in Q8 fixed-point */

#define BATTERY_FP_SHIFT 8
#define BATTERY_EMA_SHIFT 2  /* alpha = 1 / (1 << 2) = 1/4 */

static inline uint8_t interpolate_battery_percent(uint16_t battery_mv)
{
    /**
     * 18650 Battery Voltage Lookup Table (Standard Li-ion)
     * Maps discrete voltage points to percentage for more accurate SoC estimation
     */
    static const uint16_t volt_table_mv[] = {
        3200,   /* 0% - Cutoff voltage */
        3350,   /* 5% */
        3450,   /* 10% */
        3600,   /* 20% */
        3650,   /* 30% */
        3700,   /* 40% */
        3800,   /* 60% */
        3900,   /* 80% */
        4050,   /* 95% */
        4200    /* 100% - Nominal full charge */
    };

    static const uint8_t perc_table[] = {
        0, 5, 10, 20, 30, 40, 60, 80, 95, 100
    };

    static const size_t table_size = sizeof(volt_table_mv) / sizeof(volt_table_mv[0]);

    /* Boundary checks */
    if (battery_mv <= volt_table_mv[0]) {
        return 0;
    }
    if (battery_mv >= volt_table_mv[table_size - 1]) {
        return 100;
    }

    /* Find the bracketing segment */
    for (size_t i = 0; i < table_size - 1; i++) {
        if (battery_mv < volt_table_mv[i + 1]) {
            /* Linear interpolation: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1) */
            uint16_t v_start = volt_table_mv[i];
            uint16_t v_end = volt_table_mv[i + 1];
            uint8_t p_start = perc_table[i];
            uint8_t p_end = perc_table[i + 1];

            uint32_t numerator = (uint32_t)(battery_mv - v_start) * (p_end - p_start);
            uint32_t denominator = (uint32_t)(v_end - v_start);
            return p_start + (uint8_t)(numerator / denominator);
        }
    }

    return 100;
}

static inline uint8_t battery_mv_to_percent(uint16_t battery_mv)
{
    /* First reading: initialize filter */
    if (filtered_battery_mv < 0) {
        filtered_battery_mv = (int32_t)battery_mv << BATTERY_FP_SHIFT;  /* Store as mV in Q8 fixed-point */
    }

    /* EMA update in Q8 using shift math: y += (x - y) / 4 (alpha = 1/4). */
    int32_t raw_mv_q8 = (int32_t)battery_mv << BATTERY_FP_SHIFT;
    filtered_battery_mv += (raw_mv_q8 - filtered_battery_mv) >> BATTERY_EMA_SHIFT;

    /* Convert filtered mV back to regular units */
    uint16_t filtered_mv = (uint16_t)(filtered_battery_mv >> BATTERY_FP_SHIFT);

    /* Use interpolation for accurate percentage */
    return interpolate_battery_percent(filtered_mv);
}

static inline uint16_t read_battery_mv(const char *tag)
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
        ESP_LOGW(tag, "Battery ADC GPIO %d is not an ADC pin", BATTERY_ADC_GPIO);
        return 0;
    }

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = unit_id,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&init_config, &adc_handle) != ESP_OK) {
        ESP_LOGW(tag, "adc_oneshot_new_unit failed");
        return 0;
    }

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    if (adc_oneshot_config_channel(adc_handle, channel, &chan_config) != ESP_OK) {
        ESP_LOGW(tag, "adc_oneshot_config_channel failed");
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