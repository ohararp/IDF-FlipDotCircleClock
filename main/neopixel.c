#include "neopixel.h"
#include "gpio_config.h"
#include "led_strip.h"

// Handle for the single WS2812 NeoPixel LED
static led_strip_handle_t s_led_strip;

// Configure WS2812 on PIN_NEOPIXEL using RMT peripheral at 10 MHz resolution
esp_err_t neopixel_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = PIN_NEOPIXEL,                     // GPIO 40
        .max_leds = 1,                                       // single LED on FeatherS3
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // WS2812 expects G-R-B order
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,                  // 10 MHz = 100 ns resolution per tick
        .flags.with_dma = false,                             // single LED doesn't need DMA
    };

    // Create RMT-backed LED strip driver
    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
    if (ret != ESP_OK) {
        return ret;
    }
    // Start with LED off
    return led_strip_clear(s_led_strip);
}

// Set NeoPixel to specified RGB color (0–255 per channel) and push to hardware
esp_err_t neopixel_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    esp_err_t ret = led_strip_set_pixel(s_led_strip, 0, r, g, b); // pixel index 0
    if (ret != ESP_OK) {
        return ret;
    }
    return led_strip_refresh(s_led_strip); // push pixel data over RMT
}

// Turn off the NeoPixel by clearing pixel buffer and refreshing
esp_err_t neopixel_off(void)
{
    esp_err_t ret = led_strip_clear(s_led_strip); // zero out all pixel data
    if (ret != ESP_OK) {
        return ret;
    }
    return led_strip_refresh(s_led_strip); // push cleared data over RMT
}
