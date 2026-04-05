// Animation sequences for FlipDotCircleClock.
// All animations are blocking — they run on the caller's task with vTaskDelay.
// Caller must suspend display task and acquire hardware mutex before calling.

#include "animations.h"
#include "action_log.h"
#include "stepper.h"
#include "as5600.h"
#include "calibration.h"
#include "flipdot.h"
#include "timekeeping.h"
#include "oled_display.h"
#include "gpio_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

static const char *TAG = "anim";

// Steps per hour position on the clock face (12800 / 12 ≈ 1067)
#define STEPS_PER_HOUR (STEPPER_STEPS_PER_REV / 12)

// Delay between animation frames
#define FRAME_DELAY_MS 1000

// Helper: restore clock to current time after animation completes
static void restore_time(void)
{
    struct tm local;
    if (timekeeping_get_local_time(&local) == ESP_OK) {
        // Re-home motor to 12 o'clock
        stepper_find_home();
        // Update flipdot hour display
        int hour12 = local.tm_hour % 12;
        if (hour12 == 0) hour12 = 12;
        flipdot_power_on();
        flipdot_show_hour(hour12);
        vTaskDelay(pdMS_TO_TICKS(500));
        flipdot_power_off();
        // Move minute hand to current position
        clock_update_minute(&local);
    }
}

// Sync: home → synchronized hand sweep to each hour + matching flipdot → restore
void anim_sync(void)
{
    ESP_LOGI(TAG, "Sync animation starting");
    action_log_add("Sync animation started");
    oled_terminal_print("Anim: Sync");

    // Blank display and home minute hand
    flipdot_power_on();
    flipdot_blank();
    vTaskDelay(pdMS_TO_TICKS(500));

    stepper_find_home();

    // Get calibration offset for AS5600 hour angle calculation
    uint16_t home_offset = calibration_get_offset();

    vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS));
    // Sweep hand to each hour position with matching flipdot display
    for (int h = 1; h <= 12; h++) {
        // Move hand to hour position using PID closed-loop (each hour = 5 minutes)
        uint16_t target_raw = as5600_minute_to_raw(h * 5, home_offset);
        stepper_move_to_angle(target_raw, 1);
        stepper_set_position(h * STEPS_PER_HOUR);

        // Display matching hour on flipdots
        flipdot_show_hour(h);

        char buf[20];
        snprintf(buf, sizeof(buf), "Sync %d/12", h);
        oled_terminal_print(buf);
        ESP_LOGI(TAG, "Sync: %d/12", h);
        vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS));
    }
    flipdot_power_off();

    // Restore current time
    oled_terminal_print("Sync: restoring...");
    restore_time();
    ESP_LOGI(TAG, "Sync animation complete");
    action_log_add("Sync animation complete");
}
