#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stepper.h"
#include "as5600.h"
#include "calibration.h"
#include "gpio_config.h"
#include "nvm_storage.h"
#include "oled_display.h"
#include "esp_log.h"
#include "pid_ctrl.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
    esp_rom_delay_us(2);                 // 2 µs minimum pulse width for TMC2209
    gpio_set_level(PIN_STEPPER_STEP, 0); // return low
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

// Home to 12 o'clock — open-loop CW to position 0, then PID fine-tune if AS5600 calibrated
esp_err_t stepper_find_home(void)
{
    home_log("Home: starting...");
    stepper_enable();

    // Open-loop: CW to step position 0 (12 o'clock)
    int steps_to_home = (STEPPER_STEPS_PER_REV - s_step_now) % STEPPER_STEPS_PER_REV;
    home_log("Home: pos=%d moving %d CW", s_step_now, steps_to_home);

    if (steps_to_home > 0) {
        stepper_multi_step(steps_to_home, true);
    }
    s_step_now = 0;

    // PID fine-tune: if AS5600 is connected and calibrated, move to exact calibrated angle
    if (as5600_is_connected()) {
        uint16_t home_angle = calibration_get_offset();
        if (home_angle != 0) {
            home_log("Home: PID tune to %d", home_angle);
            if (stepper_move_to_angle(home_angle, 5)) { // ±5 AS5600 units ≈ ±0.4°
                s_step_now = 0; // confirmed at 12 o'clock
                home_log("Home: PID done!");
            } else {
                home_log("Home: PID failed, using open-loop");
            }
        }
    }

    home_log("Home: done!");
    return ESP_OK;
}

// PID closed-loop move to target AS5600 angle using espressif/pid_ctrl
// Error = shortest-path angular diff → PID output = steps to take per iteration
bool stepper_move_to_angle(uint16_t target_raw, int tolerance)
{
    if (!as5600_is_connected()) {
        return false;
    }

    stepper_enable();

    stepper_enable();

    // Read initial position
    uint16_t current;
    as5600_read_raw_angle(&current);
    ESP_LOGI(TAG, "PID move: cur=%d tgt=%d tol=%d", current, target_raw, tolerance);

    // AS5600-to-step ratio: ~3.12 steps per AS5600 unit (12800 / 4096)
    const float STEPS_PER_UNIT = (float)STEPPER_STEPS_PER_REV / 4096.0f;

    // Save original step delay so we can restore it after
    uint16_t original_delay = s_step_delay_us;

    const int max_iterations = 100;
    bool converged = false;

    for (int iter = 0; iter < max_iterations; iter++) {
        if (as5600_read_raw_angle(&current) != ESP_OK) break;

        // CW-only distance to target (always positive, wraps around 4096)
        int cw_dist = (target_raw - current + 4096) % 4096;

        // Check if within tolerance — target reached
        if (cw_dist <= tolerance || cw_dist >= (4096 - tolerance)) {
            s_step_now = as5600_to_steps(current);
            ESP_LOGI(TAG, "PID converged: cur=%d err=%d iter=%d", current, cw_dist, iter);
            converged = true;
            break;
        }

        // Non-linear approach: step count, motor speed, and settle time scale with distance
        int steps;
        int settle_ms;

        if (cw_dist > 1000) {
            // Very far (>88°): 80% of distance, fast motor, short settle
            steps = (int)(cw_dist * STEPS_PER_UNIT * 0.8f);
            s_step_delay_us = 200;   // fast — 200µs per step
            settle_ms = 30;
        } else if (cw_dist > 200) {
            // Far (>18°): 60% of distance, medium speed
            steps = (int)(cw_dist * STEPS_PER_UNIT * 0.6f);
            s_step_delay_us = 350;   // medium — 350µs per step
            settle_ms = 40;
        } else if (cw_dist > 50) {
            // Close (>4.4°): 40% of distance, slower for precision
            steps = (int)(cw_dist * STEPS_PER_UNIT * 0.4f);
            s_step_delay_us = 500;   // slow — 500µs per step
            settle_ms = 50;
        } else {
            // Very close (<4.4°): proportional steps at slow speed for precision
            steps = (int)(cw_dist * STEPS_PER_UNIT * 0.5f);
            if (steps < 1) steps = 1;
            s_step_delay_us = 600;   // very slow — 600µs per step
            settle_ms = 40;
        }

        if (steps < 1) steps = 1;

        // Execute steps — CW only
        for (int s = 0; s < steps; s++) {
            one_step(true);
        }

        // Variable settle delay for AS5600 to stabilize
        vTaskDelay(pdMS_TO_TICKS(settle_ms));

        // Log every 5th iteration
        if (iter % 5 == 0) {
            ESP_LOGI(TAG, "PID[%d]: cur=%d err=%d steps=%d delay=%dus",
                     iter, current, cw_dist, steps, s_step_delay_us);
        }
    }

    // Restore original step delay
    s_step_delay_us = original_delay;

    if (!converged) {
        as5600_read_raw_angle(&current);
        ESP_LOGW(TAG, "PID failed: cur=%d tgt=%d after %d iters", current, target_raw, max_iterations);
    }

    return converged;
}
