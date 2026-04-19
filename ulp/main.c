/*
 * ======================================================================================
 * ESP32-C6 ULP PULSE COUNTER FOR ANEMOMETER
 * ======================================================================================
 * This program runs on the LP/ULP core while the main CPU is in deep sleep.
 *
 * Function overview:
 * 1. Configure a GPIO rising-edge interrupt on the anemometer sensor pin.
 * 2. On each interrupt, apply a short debounce filter to reject false edges.
 * 3. Increment pulse_count for valid pulses so the main firmware can compute
 *    rotational speed and wind speed when it wakes up.
 * 4. Stay in an interrupt wait state between pulses to minimize power use.
 * ======================================================================================
 */

#include <stdint.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "ulp_lp_core.h"
#include "ulp_lp_core_utils.h"
#include "ulp_lp_core_gpio.h"
#include "ulp_lp_core_interrupts.h"
#include "riscv/csr.h"
#include "../src/anemometer.h"


uint32_t pulse_count;
static uint32_t last_trigger_time_cycles;


void LP_CORE_ISR_ATTR ulp_lp_core_lp_io_intr_handler(void)
{
    ulp_lp_core_gpio_clear_intr_status();
    uint32_t trigger_time_cycles = RV_READ_CSR(mcycle);
    /* Do some simple debouncing, do not count spurious pulses */
    if (trigger_time_cycles - last_trigger_time_cycles > DEBOUNCE_INTERVAL_CYCLES) {
        pulse_count++;
        last_trigger_time_cycles = trigger_time_cycles;
    }
}



int main (void)
{
    ulp_lp_core_intr_enable();
    ulp_lp_core_gpio_intr_enable(SENSOR_PIN, GPIO_INTR_POSEDGE);

    while(1) {

        /* Put CPU into a wait state to reduce power consumption while waiting for pulses */
        ulp_lp_core_wait_for_intr();

    }

    return 0;
}
