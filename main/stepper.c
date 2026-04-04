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

    // TODO: PID fine-tune disabled until AS5600 reads are verified
    if (false && as5600_is_connected()) {
        uint16_t home_angle = calibration_get_offset();
        if (home_angle != 0) {
            home_log("Home: PID tune to %d", home_angle);
            if (stepper_move_to_angle(home_angle, 100)) {
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

    // Create PID controller — positional mode, tuned for stepper + AS5600
    // AS5600 ratio: ~3.12 motor steps per AS5600 unit (12800 steps / 4096 units)
    pid_ctrl_parameter_t pid_params = {
        .kp = 0.1f,              // very conservative: 0.1 steps per AS5600 unit of error
        .ki = 0.0f,              // no integral — stepper has no steady-state error
        .kd = 0.05f,             // light derivative dampening
        .max_output = 50.0f,     // max 50 steps per iteration (prevents overshoot)
        .min_output = -50.0f,    // allow both CW and CCW
        .max_integral = 0.0f,    // no integral accumulation
        .min_integral = 0.0f,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
    };
    pid_ctrl_config_t pid_cfg = { .init_param = pid_params };
    pid_ctrl_block_handle_t pid = NULL;

    esp_err_t ret = pid_new_control_block(&pid_cfg, &pid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PID init failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Read initial position for logging
    uint16_t current;
    as5600_read_raw_angle(&current);
    ESP_LOGI(TAG, "PID move: cur=%d tgt=%d tol=%d", current, target_raw, tolerance);

    const int max_iterations = 200; // 200 × 20ms = 4s max convergence time
    bool converged = false;

    for (int iter = 0; iter < max_iterations; iter++) {
        // Read current AS5600 angle
        if (as5600_read_raw_angle(&current) != ESP_OK) break;

        // Calculate shortest-path error (positive = CW in AS5600 space)
        float error = (float)as5600_angle_diff(current, target_raw);

        // Check if within tolerance — target reached
        if (fabsf(error) <= (float)tolerance) {
            s_step_now = as5600_to_steps(current);
            ESP_LOGI(TAG, "PID converged: cur=%d err=%.0f iter=%d", current, error, iter);
            converged = true;
            break;
        }

        // Compute PID output: positive output = CW in AS5600 space
        float output = 0;
        pid_compute(pid, error, &output);

        // Convert PID output to step direction and count
        // CW motor = INCREASING AS5600 angle (confirmed from logs)
        int steps = (int)fabsf(output);
        if (steps == 0) {
            // PID says don't move — close enough, declare converged
            s_step_now = as5600_to_steps(current);
            ESP_LOGI(TAG, "PID converged (deadband): cur=%d err=%.0f iter=%d", current, error, iter);
            converged = true;
            break;
        }
        bool cw = (output > 0);   // positive output = CW motor (increases AS5600 angle)

        // Execute steps
        for (int s = 0; s < steps; s++) {
            one_step(cw);
        }

        // Settle delay: let motor physically move and AS5600 output stabilize
        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms between PID iterations for reliable AS5600 reads

        // Log every 10th iteration for tuning visibility
        if (iter % 10 == 0) {
            ESP_LOGI(TAG, "PID[%d]: cur=%d err=%.0f out=%.1f steps=%d cw=%d",
                     iter, current, error, output, steps, cw);
        }
    }

    if (!converged) {
        as5600_read_raw_angle(&current);
        ESP_LOGW(TAG, "PID failed: cur=%d tgt=%d after %d iterations", current, target_raw, max_iterations);
    }

    // Clean up PID controller
    pid_del_control_block(pid);
    return converged;
}
