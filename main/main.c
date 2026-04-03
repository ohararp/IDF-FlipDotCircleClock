#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "neopixel.h"
#include "ds3231.h"
#include "oled_display.h"

static const char *TAG = "main";

// Track whether OLED was successfully initialized (shared between init and display task)
static bool s_oled_ok = false;

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
            // Update OLED with current time if display is available
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

    // Init WS2812 NeoPixel on PIN_NEOPIXEL via RMT peripheral
    ESP_ERROR_CHECK(neopixel_init());
    // Background task: blink purple at 1 Hz to indicate boot/WiFi-connecting state
    xTaskCreate(neopixel_blink_task, "neopixel", 2048, NULL, 2, NULL);

    // Init I2C master bus (400 kHz) and attach DS3231 RTC at 0x68; returns bus handle for OLED/AS5600
    i2c_master_bus_handle_t i2c_bus = NULL;
    if (ds3231_init(&i2c_bus) == ESP_OK) {
        // Seed RTC with firmware compile timestamp (__DATE__/__TIME__) so clock starts reasonably
        ds3231_set_time_from_compile();

        // Init SH1107 128x64 OLED at 0x3C on the shared I2C bus via U8G2
        if (oled_init(i2c_bus) == ESP_OK) {
            s_oled_ok = true;
        } else {
            ESP_LOGW(TAG, "OLED init failed — display task will log to serial only");
        }

        // Background task: refresh RTC time to serial + OLED every 1 s
        xTaskCreate(display_task, "display", 4096, NULL, 3, NULL);
    } else {
        ESP_LOGW(TAG, "DS3231 not found — skipping RTC and display tasks");
    }
}
