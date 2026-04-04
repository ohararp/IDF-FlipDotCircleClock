#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "stepper.h"
#include "as5600.h"
#include "calibration.h"
#include "gpio_config.h"
#include "nvm_storage.h"
#include "oled_display.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

static const char *TAG = "stepper";

// Current absolute step position (0 to STEPPER_STEPS_PER_REV-1)
static int s_step_now = 0;

// Step delay loaded from NVS at init (microseconds)
static uint16_t s_step_delay_us = STEPPER_DEFAULT_DELAY_US;

// ── GPIO helpers ─────────────────────────────────────────────────────────────

// Set direction pin: true=CW (clockwise), false=CCW (counterclockwise)
static void set_direction(bool clockwise)
{
    gpio_set_level(PIN_STEPPER_DIR, clockwise ? 1 : 0);
}

// Generate one step pulse: HIGH→LOW on STEP pin with configured delay
static void one_step(bool clockwise)
{
    set_direction(clockwise);
    gpio_set_level(PIN_STEPPER_STEP, 1); // rising edge triggers TMC2209 step
    gpio_set_level(PIN_STEPPER_STEP, 0); // return low immediately
    esp_rom_delay_us(s_step_delay_us);   // inter-step delay (blocks, µs-precise)

    // Update position counter with wrap-around at full revolution
    if (clockwise) {
        s_step_now = (s_step_now + 1) % STEPPER_STEPS_PER_REV;
    } else {
        s_step_now = (s_step_now - 1 + STEPPER_STEPS_PER_REV) % STEPPER_STEPS_PER_REV;
    }
}

// ── Log helper: prints to both serial and OLED terminal ──────────────────────

static void home_log(const char *fmt, ...)
{
    char buf[64];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    ESP_LOGI(TAG, "%s", buf);
    oled_terminal_print(buf);
}

// ── Public API ───────────────────────────────────────────────────────────────

// Configure stepper GPIOs (EN, STEP, DIR, MS), load step delay from NVS
esp_err_t stepper_init(void)
{
    // STEP pin: output, start LOW
    gpio_config_t step_cfg = {
        .pin_bit_mask = (1ULL << PIN_STEPPER_STEP),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&step_cfg);
    gpio_set_level(PIN_STEPPER_STEP, 0);

    // DIR pin: output, start LOW (CCW default)
    gpio_config_t dir_cfg = {
        .pin_bit_mask = (1ULL << PIN_STEPPER_DIR),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&dir_cfg);
    gpio_set_level(PIN_STEPPER_DIR, 0);

    // EN pin: output, start HIGH (disabled — motor freewheels until explicitly enabled)
    gpio_config_t en_cfg = {
        .pin_bit_mask = (1ULL << PIN_STEPPER_EN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&en_cfg);
    gpio_set_level(PIN_STEPPER_EN, 1); // HIGH = disabled on TMC2209

    // MS pin: output, HIGH = 32x microstepping on TMC2209
    gpio_config_t ms_cfg = {
        .pin_bit_mask = (1ULL << PIN_STEPPER_MS),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&ms_cfg);
    gpio_set_level(PIN_STEPPER_MS, 1); // 32x microstepping

    // Load step delay from NVS (default 450 µs if not set)
    nvm_get_step_delay(&s_step_delay_us);

    ESP_LOGI(TAG, "Stepper initialized: EN=%d STEP=%d DIR=%d MS=%d delay=%d us",
             PIN_STEPPER_EN, PIN_STEPPER_STEP, PIN_STEPPER_DIR,
             PIN_STEPPER_MS, s_step_delay_us);

    return ESP_OK;
}

// Pull EN pin LOW to enable TMC2209 motor driver (holds torque)
void stepper_enable(void)
{
    gpio_set_level(PIN_STEPPER_EN, 0); // LOW = enabled
    ESP_LOGI(TAG, "Motor enabled");
}

// Pull EN pin HIGH to disable TMC2209 motor driver (motor freewheels)
void stepper_disable(void)
{
    gpio_set_level(PIN_STEPPER_EN, 1); // HIGH = disabled
    ESP_LOGI(TAG, "Motor disabled");
}

// Step motor N steps in given direction using configured delay
void stepper_multi_step(int steps, bool clockwise)
{
    stepper_enable();
    for (int i = 0; i < steps; i++) {
        one_step(clockwise);
    }
}

// Get current step position (0 to STEPPER_STEPS_PER_REV-1)
int stepper_get_position(void)
{
    return s_step_now;
}

// Set step position counter directly (e.g., after homing sets 0)
void stepper_set_position(int position)
{
    s_step_now = position % STEPPER_STEPS_PER_REV;
}

// Home to 12 o'clock using AS5600 calibration offset — no Hall sensor needed
esp_err_t stepper_find_home(void)
{
    home_log("Home: starting...");
    stepper_enable();

    if (!as5600_is_connected()) {
        home_log("Home: no AS5600!");
        home_log("Home: calibrate first");
        return ESP_ERR_NOT_FOUND;
    }

    // Load calibrated 12 o'clock angle from NVS
    uint16_t home_angle = calibration_get_offset();
    if (home_angle == 0) {
        home_log("Home: not calibrated!");
        home_log("Home: hold C to cal");
        return ESP_ERR_NOT_FOUND;
    }

    // Read current AS5600 position
    uint16_t current;
    esp_err_t ret = as5600_read_raw_angle(&current);
    if (ret != ESP_OK) {
        home_log("Home: AS5600 read err");
        return ret;
    }

    home_log("Home: cur=%d tgt=%d", current, home_angle);

    // Move to calibrated 12 o'clock using closed-loop AS5600 feedback
    if (stepper_move_to_angle(home_angle, 15)) {
        s_step_now = 0; // at 12 o'clock = step 0
        home_log("Home: done!");
    } else {
        home_log("Home: move failed!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

// Closed-loop move using AS5600 feedback with batched stepping (port of moveToAngle)
bool stepper_move_to_angle(uint16_t target_raw, int tolerance)
{
    if (!as5600_is_connected()) {
        return false; // caller should fall back to open-loop
    }

    stepper_enable();
    int steps_taken = 0;
    const int max_steps = 1000; // safety limit to prevent infinite loops

    while (steps_taken < max_steps) {
        // Read current AS5600 angle
        uint16_t current;
        if (as5600_read_raw_angle(&current) != ESP_OK) {
            return false;
        }

        // Calculate shortest-path difference (positive = CW needed)
        int diff = as5600_angle_diff(current, target_raw);
        int abs_diff = abs(diff);

        // Check if within tolerance — target reached
        if (abs_diff <= tolerance) {
            s_step_now = as5600_to_steps(current); // sync step counter with encoder
            return true;
        }

        // Choose batch size based on distance to target
        bool cw = (diff > 0);
        int batch;
        if (abs_diff > 200) {
            batch = 50;       // far: 50 steps (~16 AS5600 units) between reads
        } else if (abs_diff > 50) {
            batch = 15;       // medium: 15 steps between reads
        } else if (abs_diff > tolerance * 2) {
            batch = 5;        // close: 5 steps between reads
        } else {
            batch = 1;        // very close: single-step fine-tuning
        }

        // Take batched steps without reading sensor between them
        for (int i = 0; i < batch && steps_taken < max_steps; i++) {
            one_step(cw);
            steps_taken++;
        }
    }

    ESP_LOGW(TAG, "move_to_angle: max_steps exceeded");
    return false;
}
