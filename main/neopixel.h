#pragma once

#include "esp_err.h"

// Initialize WS2812 NeoPixel on PIN_NEOPIXEL via RMT peripheral, LED starts off
esp_err_t neopixel_init(void);

// Set NeoPixel to RGB color (0–255 per channel), immediately updates hardware
esp_err_t neopixel_set_color(uint8_t r, uint8_t g, uint8_t b);

// Turn off the NeoPixel (all channels to 0)
esp_err_t neopixel_off(void);
