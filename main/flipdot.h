#pragma once

#include "esp_err.h"
#include <stdint.h>

// Configure SPI pins (SCK=36, MOSI=35, LATCH=37), OE pin (18), and relay pin (11)
esp_err_t flipdot_init(void);

// Turn on 24V relay with 200 ms precharge delay for capacitor charging
void flipdot_power_on(void);

// Turn off 24V relay immediately
void flipdot_power_off(void);

// Extend relay hold window by 80 ms — prevents relay chatter during rapid updates
void flipdot_extend_power_window(void);

// Check and turn off relay if hold window has expired (call periodically)
void flipdot_service_power_window(void);

// Send 3-column pattern to shift registers with column staggering (100 ms between columns)
void flipdot_set_pattern(const uint8_t cols[3]);

// Display hour (1–12) on the 3×4 flip-dot matrix using predefined bit patterns
void flipdot_show_hour(int hour);

// Blank all dots (all off)
void flipdot_blank(void);

// Light all 12 dots (test pattern)
void flipdot_all_on(void);
