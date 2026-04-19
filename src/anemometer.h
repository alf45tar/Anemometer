#pragma once

#define SENSOR_PIN 6                        // GPIO used for the anemometer pulse input
                                            // Must be RTC-capable pin: GPIO0-GPIO7 on ESP32-C6
                                            // Avoid GPIO4 and GPIO5 which are used for strapping
#define SLEEP_DURATION 5                    // Deep-sleep interval in seconds between wakeups
#define HEARTBEAT_INTERVAL 60               // Force a periodic telemetry heartbeat every N seconds
#define DEBOUNCE_INTERVAL_CYCLES 1000       // Minimum ULP clock cycles between valid edges to filter out noise (debouncing)

/* Anemometer conversion constants */
#define PULSES_PER_ROTATION 2.0f            // Sensor pulses generated for one full rotor rotation
#define RADIUS 0.078f                       // Rotor radius in meters (center to cup midpoint)
#define CALIBRATION_FACTOR 2.5f             // Empirical multiplier to match real wind speed

/* BLE/BTHome beacon behavior */
#define BLE_ADV_DURATION_MS 1500            // Advertising window duration after each wakeup
#define BLE_ADV_INTERVAL_UNITS 160          // BLE adv interval units (0.625 ms each): 160 = 100 ms

/* Battery measurement via ADC */
#define BATTERY_ADC_GPIO 0                  // GPIO connected to battery sense divider output
                                            // Must be an ADC-capable pin: GPIO0-GPIO6 on ESP32-C6
                                            // Avoid GPIO4 and GPIO5 which are used for strapping
#define BATTERY_ADC_SAMPLES 8               // Number of ADC readings averaged per measurement
#define BATTERY_VOLTAGE_DIVIDER_RATIO 2.0f  // Scale factor from ADC node voltage to battery voltage
#define BATTERY_MIN_MV 3300                 // Battery voltage mapped to 0%
#define BATTERY_MAX_MV 4150                 // Battery voltage mapped to 100%
