#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "neopixel.h"
#include "ds3231.h"
#include "oled_display.h"
#include "nvm_storage.h"
#include "stepper.h"
#include "as5600.h"
#include "calibration.h"
#include "buttons.h"
#include "flipdot.h"
#include "timekeeping.h"

static const char *TAG = "main";

// Track whether OLED was successfully initialized
static bool s_oled_ok = false;

// Handle for the display task so it can be suspended/resumed during homing/animations
static TaskHandle_t s_display_task = NULL;

// ── Background tasks ─────────────────────────────────────────────────────────

// Task: toggle NeoPixel purple/off at 500 ms — visual heartbeat during startup
static void neopixel_blink_task(void *arg)
{
    bool led_on = false;
    while (1) {
        if (led_on) {
            neopixel_set_color(32, 0, 32); // purple = WiFi connecting status color
        } else {
            neopixel_off();
        }
        led_on = !led_on;
        vTaskDelay(pdMS_TO_TICKS(500)); // 1 Hz blink rate
    }
}

// Task: read local time every second, update OLED with time display
static void display_task(void *arg)
{
    struct tm local;
    while (1) {
        if (timekeeping_get_local_time(&local) == ESP_OK) {
            // Log local time to serial for debugging
            ESP_LOGI(TAG, "Local: %04d-%02d-%02d %02d:%02d:%02d",
                     local.tm_year + 1900, local.tm_mon + 1, local.tm_mday,
                     local.tm_hour, local.tm_min, local.tm_sec);
            // Update OLED with local time (task is suspended during homing/animations)
            if (s_oled_ok) {
                oled_update_time(&local);
            }
        } else {
            ESP_LOGW(TAG, "Failed to read local time");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 s refresh interval
    }
}

// ── Hardware setup — called once at boot before main loop ────────────────────

static void setup(void)
{
    const esp_app_desc_t *app_desc = esp_app_get_description();
    ESP_LOGI(TAG, "FlipDotCircleClock v%s starting...", app_desc->version);

    // 1. Persistent config
    ESP_ERROR_CHECK(nvm_init());
    ESP_ERROR_CHECK(timekeeping_init());

    // 2. NeoPixel status LED — blink purple during setup
    ESP_ERROR_CHECK(neopixel_init());
    xTaskCreate(neopixel_blink_task, "neopixel", 2048, NULL, 2, NULL);

    // 3. I2C bus + peripherals (RTC, OLED, AS5600 all share one bus)
    i2c_master_bus_handle_t i2c_bus = NULL;
    if (ds3231_init(&i2c_bus) == ESP_OK) {
        // Seed RTC with compile timestamp (__DATE__/__TIME__ expanded here for freshness)
        const char *months[] = {"Jan","Feb","Mar","Apr","May","Jun",
                                "Jul","Aug","Sep","Oct","Nov","Dec"};
        struct tm ct = {0};
        sscanf(__TIME__, "%d:%d:%d", &ct.tm_hour, &ct.tm_min, &ct.tm_sec);
        char mon[4]; int day, year;
        sscanf(__DATE__, "%3s %d %d", mon, &day, &year);
        ct.tm_mday = day;
        ct.tm_year = year - 1900;
        for (int i = 0; i < 12; i++) {
            if (strncmp(mon, months[i], 3) == 0) { ct.tm_mon = i; break; }
        }
        ds3231_set_time(&ct);
        ESP_LOGI(TAG, "RTC seeded: %04d-%02d-%02d %02d:%02d:%02d",
                 ct.tm_year + 1900, ct.tm_mon + 1, ct.tm_mday,
                 ct.tm_hour, ct.tm_min, ct.tm_sec);

        // OLED display at 0x3C
        if (oled_init(i2c_bus) == ESP_OK) {
            s_oled_ok = true;
        } else {
            ESP_LOGW(TAG, "OLED init failed — serial only");
        }

        // AS5600 magnetic encoder at 0x36 (optional — closed-loop fallback)
        if (as5600_init(i2c_bus) == ESP_OK) {
            uint16_t angle;
            as5600_read_raw_angle(&angle);
            ESP_LOGI(TAG, "AS5600: raw=%d (%.1f deg)", angle, as5600_to_degrees(angle));
        } else {
            ESP_LOGW(TAG, "AS5600 not found — open-loop mode");
        }
    } else {
        ESP_LOGW(TAG, "I2C bus failed — no RTC, OLED, or AS5600");
    }

    // 4. Flip-dot display (SPI bit-bang + 24V relay)
    ESP_ERROR_CHECK(flipdot_init());

    // 5. Stepper motor + homing
    ESP_ERROR_CHECK(stepper_init());
    stepper_find_home();

    // 6. Set initial clock state from local time
    struct tm local;
    if (timekeeping_get_local_time(&local) == ESP_OK) {
        clock_update_hour(&local);
        clock_update_minute(&local);
        ESP_LOGI(TAG, "Clock set to %02d:%02d", local.tm_hour, local.tm_min);
    }

    // 7. Buttons (B + C with ISR + debounce)
    ESP_ERROR_CHECK(buttons_init());

    // 8. Background display task (1 s OLED + serial refresh)
    xTaskCreate(display_task, "display", 4096, NULL, 3, &s_display_task);
}

// ── Main entry point ─────────────────────────────────────────────────────────

void app_main(void)
{
    setup();

    // ── Main clock loop: RTC comparison-based scheduling ─────────────────────

    // Initialize trackers to impossible values so first loop triggers all updates
    int min_old = -1, hr_old = -1;
    struct tm local;
    button_event_t event;

    while (1) {
        // Service flipdot relay hold window — turns off relay when hold period expires
        flipdot_service_power_window();

        // Read local time for scheduling checks
        if (timekeeping_get_local_time(&local) == ESP_OK) {
            // Minute boundary crossed → move minute hand
            if (local.tm_min != min_old) {
                ESP_LOGI(TAG, "Minute changed: %02d → %02d", min_old, local.tm_min);
                clock_update_minute(&local);
                min_old = local.tm_min;
            }

            // Hour boundary crossed → update flipdot display + re-home + position minute
            if (local.tm_hour != hr_old) {
                ESP_LOGI(TAG, "Hour changed: %02d → %02d", hr_old, local.tm_hour);
                if (s_display_task) vTaskSuspend(s_display_task);
                clock_update_hour(&local);
                stepper_find_home();
                clock_update_minute(&local);
                vTaskDelay(pdMS_TO_TICKS(2000)); // hold OLED terminal for 2 s
                if (s_display_task) vTaskResume(s_display_task);
                hr_old = local.tm_hour;
                min_old = local.tm_min; // prevent double minute update
            }
        }

        // Check for button events (non-blocking, 100 ms timeout)
        if (buttons_get_event(&event, 100)) {
            switch (event) {
            case BUTTON_EVENT_B_PRESS:
                // Flipdot test: blank → pause → show hour 12 → pause → relay off
                ESP_LOGI(TAG, "Button B — flipdot test");
                flipdot_power_on();
                flipdot_blank();
                vTaskDelay(pdMS_TO_TICKS(1000));
                flipdot_show_hour(12);
                vTaskDelay(pdMS_TO_TICKS(1000));
                flipdot_power_off();
                break;
            case BUTTON_EVENT_C_PRESS:
                // Re-home motor — suspend display task for OLED terminal
                ESP_LOGI(TAG, "Button C — re-homing");
                if (s_display_task) vTaskSuspend(s_display_task);
                stepper_find_home();
                clock_update_minute(&local); // reposition after homing
                vTaskDelay(pdMS_TO_TICKS(3000)); // hold terminal for 3 s
                if (s_display_task) vTaskResume(s_display_task);
                min_old = local.tm_min;  // sync trackers
                hr_old = local.tm_hour;
                break;
            }
        }
    }
}
