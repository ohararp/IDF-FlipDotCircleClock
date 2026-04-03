#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "led_strip.h"
#include "gpio_config.h"

static const char *TAG = "main";

static led_strip_handle_t s_led_strip;

static void neopixel_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = PIN_NEOPIXEL,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip));
    led_strip_clear(s_led_strip);
}

void app_main(void)
{
    const esp_app_desc_t *app_desc = esp_app_get_description();
    ESP_LOGI(TAG, "FlipDotCircleClock v%s starting...", app_desc->version);
    ESP_LOGI(TAG, "Board: Unexpected Maker FeatherS3 (ESP32-S3)");

    neopixel_init();
    ESP_LOGI(TAG, "NeoPixel initialized on GPIO %d", PIN_NEOPIXEL);

    bool led_on = false;
    while (1) {
        if (led_on) {
            // Purple — matches CircuitPython "WiFi connecting" status color
            led_strip_set_pixel(s_led_strip, 0, 32, 0, 32);
        } else {
            led_strip_clear(s_led_strip);
        }
        led_strip_refresh(s_led_strip);
        led_on = !led_on;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
