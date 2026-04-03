#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "stepper.h"
#include "as5600.h"
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

// ── Hall sensor ──────────────────────────────────────────────────────────────

// Read Hall sensor: A3144 is active-low (pulls LOW when magnet present)
bool stepper_hall_detected(void)
{
    return gpio_get_level(PIN_HALL_SENSOR) == 0; // LOW = magnet present
}

// Debounce Hall sensor: require N consecutive matching readings (port of hallStable)
static bool hall_stable(bool expect_detected, int samples, int delay_us)
{
    for (int i = 0; i < samples; i++) {
        if (stepper_hall_detected() != expect_detected) {
            return false; // reading didn't match — not stable
        }
        esp_rom_delay_us(delay_us);
    }
    return true; // all N samples matched expected state
}

// ── Public API ───────────────────────────────────────────────────────────────

// Configure all stepper/Hall GPIOs as outputs/inputs, set 32x microstepping, load NVS delay
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

    // Hall sensor: input with pull-up (A3144 is open-collector, needs pull-up)
    gpio_config_t hall_cfg = {
        .pin_bit_mask = (1ULL << PIN_HALL_SENSOR),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&hall_cfg);

    // Load step delay from NVS (default 450 µs if not set)
    nvm_get_step_delay(&s_step_delay_us);

    ESP_LOGI(TAG, "Stepper initialized: EN=%d STEP=%d DIR=%d MS=%d HALL=%d delay=%d us",
             PIN_STEPPER_EN, PIN_STEPPER_STEP, PIN_STEPPER_DIR,
             PIN_STEPPER_MS, PIN_HALL_SENSOR, s_step_delay_us);

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

// Helper: format and print to both serial log and OLED terminal
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

// Symmetric Hall sensor edge detection — port of CircuitPython findExactHome()
esp_err_t stepper_find_home(void)
{
    home_log("Home: starting...");
    stepper_enable();
    s_step_now = 0; // reset counter for relative edge measurements

    // Step 1: Move CW until Hall triggers (enter magnet zone)
    home_log("Home: seek magnet CW");
    while (!hall_stable(true, 5, 500)) {
        one_step(true); // CW
    }

    // Step 2: Reverse CCW until Hall releases (edge A = release point)
    home_log("Home: find edge A CCW");
    while (!hall_stable(false, 5, 500)) {
        one_step(false); // CCW
    }
    int edge_a = s_step_now;
    home_log("Home: edgeA @ %d", edge_a);

    // Step 3: Continue CCW through magnet until Hall triggers again
    home_log("Home: crossing magnet");
    while (!hall_stable(true, 5, 500)) {
        one_step(false); // CCW
    }

    // Step 4: Reverse CW until Hall releases (edge B = release point)
    home_log("Home: find edge B CW");
    while (!hall_stable(false, 5, 500)) {
        one_step(true); // CW
    }
    int edge_b = s_step_now;
    home_log("Home: edgeB @ %d", edge_b);

    // Step 5: Calculate magnet center with wrap-around handling
    int raw_diff = abs(edge_a - edge_b);
    int center, magnet_width;

    if (raw_diff > STEPPER_STEPS_PER_REV / 2) {
        // Edges wrap around the 0/STEPS boundary
        if (edge_a < edge_b) {
            center = ((edge_a + STEPPER_STEPS_PER_REV) + edge_b) / 2 % STEPPER_STEPS_PER_REV;
        } else {
            center = (edge_a + (edge_b + STEPPER_STEPS_PER_REV)) / 2 % STEPPER_STEPS_PER_REV;
        }
        magnet_width = STEPPER_STEPS_PER_REV - raw_diff;
    } else {
        center = (edge_a + edge_b) / 2;
        magnet_width = raw_diff;
    }

    // Calculate shortest path to center with wrap-around
    int steps_to_center = center - s_step_now;
    if (steps_to_center > STEPPER_STEPS_PER_REV / 2) {
        steps_to_center -= STEPPER_STEPS_PER_REV;
    } else if (steps_to_center < -STEPPER_STEPS_PER_REV / 2) {
        steps_to_center += STEPPER_STEPS_PER_REV;
    }

    home_log("Home: w=%d ctr=%d", magnet_width, center);

    // Move to calculated magnet center
    if (steps_to_center > 0) {
        for (int i = 0; i < steps_to_center; i++) {
            one_step(true); // CW
        }
    } else if (steps_to_center < 0) {
        for (int i = 0; i < -steps_to_center; i++) {
            one_step(false); // CCW
        }
    }

    // Set home position (12 o'clock)
    s_step_now = 0;

    // Apply stored home offset from NVS (corrects mechanical misalignment)
    int16_t offset = 0;
    nvm_get_home_offset(&offset);
    if (offset != 0) {
        home_log("Home: offset %d", offset);
        stepper_multi_step(abs(offset), offset > 0);
        s_step_now = 0; // reset after offset applied
    }

    home_log("Home: done!");
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
