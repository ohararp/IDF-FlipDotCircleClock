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

// Home to 12 o'clock using open-loop step counting (CW to step position 0)
esp_err_t stepper_find_home(void)
{
    home_log("Home: starting...");
    stepper_enable();

    // Calculate CW steps needed to reach position 0 (12 o'clock)
    int steps_to_home = (STEPPER_STEPS_PER_REV - s_step_now) % STEPPER_STEPS_PER_REV;
    home_log("Home: pos=%d moving %d CW", s_step_now, steps_to_home);

    if (steps_to_home > 0) {
        stepper_multi_step(steps_to_home, true); // CW to 12 o'clock
    }

    s_step_now = 0; // at 12 o'clock = step 0
    home_log("Home: done!");
    return ESP_OK;
}

// Closed-loop move using AS5600 feedback with batched stepping (port of moveToAngle)
bool stepper_move_to_angle(uint16_t target_raw, int tolerance)
{
    if (!as5600_is_connected()) {
        return false;
    }

    stepper_enable();
    int steps_taken = 0;
    const int max_steps = 13000; // safety limit — slightly more than one full revolution

    // Read initial position
    uint16_t current;
    if (as5600_read_raw_angle(&current) != ESP_OK) return false;
    ESP_LOGI(TAG, "move_to_angle: cur=%d tgt=%d tol=%d", current, target_raw, tolerance);

    int prev_abs_diff = 4096; // track if we're converging or diverging

    while (steps_taken < max_steps) {
        // Read AS5600 angle
        if (as5600_read_raw_angle(&current) != ESP_OK) return false;

        // Calculate shortest-path difference (positive = target is CW in AS5600 space)
        int diff = as5600_angle_diff(current, target_raw);
        int abs_diff = abs(diff);

        // Target reached
        if (abs_diff <= tolerance) {
            s_step_now = as5600_to_steps(current);
            ESP_LOGI(TAG, "move_to_angle: converged at %d (%d steps)", current, steps_taken);
            return true;
        }

        // Detect divergence: if error increased after a batch, we're going the wrong way
        if (steps_taken > 100 && abs_diff > prev_abs_diff + 50) {
            ESP_LOGW(TAG, "move_to_angle: diverging, cur=%d tgt=%d diff=%d", current, target_raw, diff);
            return false;
        }
        prev_abs_diff = abs_diff;

        // Direction: CW motor = decreasing AS5600 angle (inverted)
        bool cw = (diff < 0);

        // Batch size: take multiple steps before re-reading sensor (I2C read is slow)
        int batch;
        if (abs_diff > 500) {
            batch = 100;      // very far: 100 steps (~32 AS5600 units) between reads
        } else if (abs_diff > 200) {
            batch = 50;       // far: 50 steps between reads
        } else if (abs_diff > 50) {
            batch = 15;       // medium: 15 steps between reads
        } else if (abs_diff > tolerance * 2) {
            batch = 5;        // close: 5 steps between reads
        } else {
            batch = 1;        // very close: single-step fine-tuning
        }

        // Execute batch — all steps in same direction without reading sensor
        for (int i = 0; i < batch && steps_taken < max_steps; i++) {
            one_step(cw);
            steps_taken++;
        }

        // Brief settling delay after batch so AS5600 reading stabilizes
        esp_rom_delay_us(500);
    }

    ESP_LOGW(TAG, "move_to_angle: max_steps exceeded, cur=%d tgt=%d", current, target_raw);
    return false;
}
