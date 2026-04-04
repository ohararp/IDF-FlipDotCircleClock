#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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
#include "animations.h"
#include "gpio_config.h"
#include "timekeeping.h"

static const char *TAG = "main";

// Track whether OLED was successfully initialized
static bool s_oled_ok = false;

// Handle for the display task so it can be suspended/resumed during homing/animations
static TaskHandle_t s_display_task = NULL;

// Mutex protecting stepper motor and flipdot hardware from concurrent access
static SemaphoreHandle_t s_hw_mutex = NULL;

// Track current displayed hour so +1 hour can wrap correctly
static int s_current_hour_12 = 0;

// Startup trackers — set by setup() so main loop doesn't duplicate the first update
static int s_startup_hr = -1;
static int s_startup_min = -1;

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

// Task: read local time every second, update OLED — shows AS5600 angle during calibration
static void display_task(void *arg)
{
    struct tm local;
    while (1) {
        if (calibration_is_active() && as5600_is_connected()) {
            // During calibration: show live AS5600 angle so user can see hand position
            uint16_t angle;
            if (as5600_read_raw_angle(&angle) == ESP_OK) {
                char buf[26];
                snprintf(buf, sizeof(buf), "CAL: AS5600=%d", angle);
                oled_terminal_print(buf);
                ESP_LOGI(TAG, "CAL: AS5600 raw=%d (%.1f deg)", angle, as5600_to_degrees(angle));
            }
        } else if (timekeeping_get_local_time(&local) == ESP_OK) {
            // Normal mode: log local time to serial and update OLED
            ESP_LOGI(TAG, "Local: %04d-%02d-%02d %02d:%02d:%02d",
                     local.tm_year + 1900, local.tm_mon + 1, local.tm_mday,
                     local.tm_hour, local.tm_min, local.tm_sec);
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

    // 1. Persistent config + timekeeping
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
        if (as5600_setup(i2c_bus) == ESP_OK) {
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

    // 5. Stepper motor init
    ESP_ERROR_CHECK(stepper_init());

    // 6. Startup sequence: blank dots → home → set hour → set minute (one pass, no repeats)
    struct tm local;
    if (timekeeping_get_local_time(&local) == ESP_OK) {
        // Blank flipdots first
        flipdot_power_on();
        flipdot_blank();
        vTaskDelay(pdMS_TO_TICKS(500));
        flipdot_power_off();

        // Home minute hand to 12 o'clock
        stepper_find_home();

        // Set flipdot hour display
        s_current_hour_12 = local.tm_hour % 12;
        if (s_current_hour_12 == 0) s_current_hour_12 = 12;
        flipdot_power_on();
        flipdot_show_hour(s_current_hour_12);
        vTaskDelay(pdMS_TO_TICKS(500));
        flipdot_power_off();

        // Move minute hand to current minute
        clock_update_minute(&local);

        ESP_LOGI(TAG, "Clock set to %02d:%02d (h12=%d)", local.tm_hour, local.tm_min, s_current_hour_12);

        // Pre-set trackers so main loop doesn't re-trigger on first iteration
        s_startup_hr = local.tm_hour;
        s_startup_min = local.tm_min;
    }

    // 7. Hardware mutex — protects stepper + flipdot from concurrent access
    s_hw_mutex = xSemaphoreCreateMutex();

    // 8. Buttons A (GPIO 1), B (GPIO 38), C (GPIO 33) with short/long press detection
    ESP_ERROR_CHECK(buttons_init());

    // 9. Background display task (1 s OLED + serial refresh)
    xTaskCreate(display_task, "display", 4096, NULL, 3, &s_display_task);
}

// ── Main entry point ─────────────────────────────────────────────────────────

void app_main(void)
{
    setup();

    // ── Main clock loop: RTC comparison-based scheduling + button handling ────

    // Initialize trackers from startup so main loop doesn't repeat the first update
    int min_old = s_startup_min, hr_old = s_startup_hr;
    struct tm local;
    app_button_event_t event;

    while (1) {
        // Service flipdot relay hold window — turns off relay when hold period expires
        flipdot_service_power_window();

        // Read local time for scheduling checks — skip all clock updates during calibration
        if (!calibration_is_active() && timekeeping_get_local_time(&local) == ESP_OK) {
            // Minute boundary crossed → move minute hand (with mutex)
            if (local.tm_min != min_old) {
                if (xSemaphoreTake(s_hw_mutex, pdMS_TO_TICKS(500))) {
                    ESP_LOGI(TAG, "Minute changed: %02d → %02d", min_old, local.tm_min);
                    clock_update_minute(&local);
                    min_old = local.tm_min;
                    xSemaphoreGive(s_hw_mutex);
                }
            }

            // Hour boundary crossed → update flipdot + re-home + position minute (with mutex)
            if (local.tm_hour != hr_old) {
                if (xSemaphoreTake(s_hw_mutex, pdMS_TO_TICKS(500))) {
                    ESP_LOGI(TAG, "Hour changed: %02d → %02d", hr_old, local.tm_hour);
                    if (s_display_task) vTaskSuspend(s_display_task);
                    clock_update_hour(&local);
                    stepper_find_home();
                    clock_update_minute(&local);
                    s_current_hour_12 = local.tm_hour % 12;
                    if (s_current_hour_12 == 0) s_current_hour_12 = 12;
                    vTaskDelay(pdMS_TO_TICKS(2000)); // hold OLED terminal for 2 s
                    if (s_display_task) vTaskResume(s_display_task);
                    hr_old = local.tm_hour;
                    min_old = local.tm_min; // prevent double minute update
                    xSemaphoreGive(s_hw_mutex);
                }
            }
        }

        // Check for button events (non-blocking, 100 ms timeout)
        if (buttons_get_event(&event, 100)) {
            // All button actions: acquire hardware mutex + suspend display task
            if (xSemaphoreTake(s_hw_mutex, pdMS_TO_TICKS(2000))) {
                if (s_display_task) vTaskSuspend(s_display_task);

                switch (event) {

                case BUTTON_EVENT_A_SHORT:
                    // Sync animation: synchronized hand + flipdot sweep through all 12 hours
                    ESP_LOGI(TAG, "Btn A short — sync animation");
                    anim_sync();
                    timekeeping_get_local_time(&local);
                    min_old = local.tm_min;
                    hr_old = local.tm_hour;
                    break;

                case BUTTON_EVENT_A_LONG:
                    // Placeholder — reserved for future use
                    ESP_LOGI(TAG, "Btn A long — unassigned");
                    oled_terminal_print("A long: unassigned");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    break;

                case BUTTON_EVENT_B_SHORT:
                    // +1 hour: read RTC, add 1 hour, write back, update flipdot display
                    timekeeping_get_local_time(&local);
                    local.tm_hour = (local.tm_hour + 1) % 24;  // +1 hour, wrap 23→0
                    ds3231_set_time(&local);                    // write adjusted time back to RTC
                    s_current_hour_12 = local.tm_hour % 12;
                    if (s_current_hour_12 == 0) s_current_hour_12 = 12;
                    ESP_LOGI(TAG, "Btn B short — +1 hour → %02d:%02d (h12=%d)",
                             local.tm_hour, local.tm_min, s_current_hour_12);
                    flipdot_power_on();
                    flipdot_show_hour(s_current_hour_12);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    flipdot_power_off();
                    hr_old = local.tm_hour;  // sync tracker to prevent re-trigger
                    min_old = local.tm_min;
                    break;

                case BUTTON_EVENT_B_LONG:
                    // Placeholder — reserved for future use
                    ESP_LOGI(TAG, "Btn B long — unassigned");
                    oled_terminal_print("B long: unassigned");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    break;

                case BUTTON_EVENT_C_SHORT:
                    // +1 minute: read RTC, add 1 min, write back, move minute hand
                    if (!calibration_is_active()) {
                        timekeeping_get_local_time(&local);
                        local.tm_min = (local.tm_min + 1) % 60;    // +1 min, wrap 59→0
                        if (local.tm_min == 0) local.tm_hour = (local.tm_hour + 1) % 24; // carry
                        local.tm_sec = 0;                           // reset seconds
                        ds3231_set_time(&local);                    // write adjusted time back to RTC
                        ESP_LOGI(TAG, "Btn C short — +1 min → %02d:%02d", local.tm_hour, local.tm_min);
                        // Move minute hand to correct position using PID closed-loop
                        clock_update_minute(&local);
                        min_old = local.tm_min;  // sync tracker to prevent re-trigger
                        hr_old = local.tm_hour;
                    }
                    break;

                case BUTTON_EVENT_C_LONG:
                    if (!calibration_is_active()) {
                        // Enter calibration — motor stays enabled, user repositions hand
                        ESP_LOGI(TAG, "Btn C long — enter calibration");
                        calibration_enter();
                        // Drain any pending button events from this same long press
                        { app_button_event_t discard;
                          while (buttons_get_event(&discard, 200)) {} }
                    } else if (calibration_ready_to_save()) {
                        // Save calibration — only if >=3s have elapsed since entering
                        ESP_LOGI(TAG, "Btn C long — save calibration");
                        calibration_save();
                        vTaskDelay(pdMS_TO_TICKS(2000)); // hold result on screen
                        // Re-sync minute hand to current time after calibration
                        timekeeping_get_local_time(&local);
                        clock_update_minute(&local);
                        min_old = local.tm_min;
                        hr_old = local.tm_hour;
                    } else {
                        // Too soon — ignore (still repositioning hand)
                        ESP_LOGI(TAG, "Btn C long — too soon, wait 3s after entering cal");
                    }
                    break;
                }

                // Resume display task after any button action completes
                if (s_display_task) vTaskResume(s_display_task);
                xSemaphoreGive(s_hw_mutex);
            }
        }
    }
}
