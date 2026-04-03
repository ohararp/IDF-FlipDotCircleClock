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

static const char *TAG = "main";

// Track whether OLED was successfully initialized (shared between init and display task)
static bool s_oled_ok = false;

// Handle for the display task so it can be suspended/resumed during homing
static TaskHandle_t s_display_task = NULL;

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

// Task: read DS3231 RTC every second, log to serial, and update OLED if available
static void display_task(void *arg)
{
    struct tm now;
    while (1) {
        if (ds3231_get_time(&now) == ESP_OK) {
            // Log time to serial for debugging
            ESP_LOGI(TAG, "RTC: %04d-%02d-%02d %02d:%02d:%02d",
                     now.tm_year + 1900, now.tm_mon + 1, now.tm_mday,
                     now.tm_hour, now.tm_min, now.tm_sec);
            // Update OLED with current time (task is suspended during homing so no conflict)
            if (s_oled_ok) {
                oled_update_time(&now);
            }
        } else {
            ESP_LOGW(TAG, "Failed to read RTC");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 s refresh interval
    }
}

void app_main(void)
{
    const esp_app_desc_t *app_desc = esp_app_get_description();
    ESP_LOGI(TAG, "FlipDotCircleClock v%s starting...", app_desc->version);

    // Init NVS flash and load persistent config (timezone, step delay, calibration, etc.)
    ESP_ERROR_CHECK(nvm_init());

    // Log current config values for verification
    uint8_t tz = 0; uint16_t step_delay = 0;
    nvm_get_timezone_index(&tz);
    nvm_get_step_delay(&step_delay);
    ESP_LOGI(TAG, "NVS config: timezone=%d, step_delay=%d us", tz, step_delay);

    // Init WS2812 NeoPixel on PIN_NEOPIXEL via RMT peripheral
    ESP_ERROR_CHECK(neopixel_init());
    // Background task: blink purple at 1 Hz to indicate boot/WiFi-connecting state
    xTaskCreate(neopixel_blink_task, "neopixel", 2048, NULL, 2, NULL);

    // Init I2C master bus (400 kHz) and attach DS3231 RTC at 0x68; returns bus handle for OLED/AS5600
    i2c_master_bus_handle_t i2c_bus = NULL;
    if (ds3231_init(&i2c_bus) == ESP_OK) {
        // Seed RTC with firmware compile timestamp — parsed here in main.c so it recompiles every build
        {
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
            ESP_LOGI(TAG, "RTC set to compile time: %04d-%02d-%02d %02d:%02d:%02d",
                     ct.tm_year + 1900, ct.tm_mon + 1, ct.tm_mday,
                     ct.tm_hour, ct.tm_min, ct.tm_sec);
        }

        // Init SH1107 128x64 OLED at 0x3C on the shared I2C bus via U8G2
        if (oled_init(i2c_bus) == ESP_OK) {
            s_oled_ok = true;
        } else {
            ESP_LOGW(TAG, "OLED init failed — display task will log to serial only");
        }
        // Init AS5600 magnetic encoder at 0x36 on the shared I2C bus (optional — closed-loop fallback)
        if (as5600_init(i2c_bus) == ESP_OK) {
            uint16_t angle;
            as5600_read_raw_angle(&angle);
            ESP_LOGI(TAG, "AS5600 connected: raw=%d (%.1f deg)", angle, as5600_to_degrees(angle));
        } else {
            ESP_LOGW(TAG, "AS5600 not found — using open-loop motor control");
        }
    } else {
        ESP_LOGW(TAG, "DS3231 not found — skipping RTC, OLED, and AS5600");
    }

    // Init flip-dot SPI pins (SCK=36, MOSI=35, LATCH=37), OE (18), and 24V relay (11)
    ESP_ERROR_CHECK(flipdot_init());

    // Init stepper GPIOs (EN, STEP, DIR, MS) and Hall sensor input; loads step delay from NVS
    ESP_ERROR_CHECK(stepper_init());
    // Home motor to 12 o'clock using symmetric Hall sensor edge detection (logs to OLED terminal)
    stepper_find_home();

    // Init Button C (GPIO 33) with ISR + debounce for re-triggering homing
    ESP_ERROR_CHECK(buttons_init());

    // Background task: refresh RTC time to serial + OLED every 1 s (save handle for suspend/resume)
    xTaskCreate(display_task, "display", 4096, NULL, 3, &s_display_task);

    // Main loop: listen for button events
    button_event_t event;
    while (1) {
        // Service flipdot relay hold window — turns off relay when hold period expires
        flipdot_service_power_window();

        // Block up to 100 ms waiting for a button press
        if (buttons_get_event(&event, 100)) {
            switch (event) {
            case BUTTON_EVENT_B_PRESS:
                // Flipdot test: blank → pause → show hour 12 (all dots) → pause → relay off
                ESP_LOGI(TAG, "Button B pressed — flipdot test");
                flipdot_power_on();
                flipdot_blank();
                vTaskDelay(pdMS_TO_TICKS(1000)); // 1 s pause so blank is visible
                flipdot_show_hour(12);
                vTaskDelay(pdMS_TO_TICKS(1000)); // 1 s hold so dots settle
                flipdot_power_off();
                break;
            case BUTTON_EVENT_C_PRESS:
                // Suspend display task so OLED terminal isn't overwritten during homing
                ESP_LOGI(TAG, "Button C pressed — re-homing");
                if (s_display_task) vTaskSuspend(s_display_task);
                stepper_find_home();
                vTaskDelay(pdMS_TO_TICKS(3000)); // hold terminal on screen for 3 s so results are readable
                if (s_display_task) vTaskResume(s_display_task);
                break;
            }
        }
    }
}
