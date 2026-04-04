#pragma once

// Button handling using espressif/button component (v4.1.6)
// https://components.espressif.com/components/espressif/button
// Provides hardware-debounced GPIO buttons with short/long press detection.

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Button event types: single click (short) and long press start (fires while held) for each button
typedef enum {
    BUTTON_EVENT_A_SHORT,   // Button A (GPIO 1) single click — re-home sequence
    BUTTON_EVENT_A_LONG,    // Button A (GPIO 1) long press (2s held) — reserved for WiFi
    BUTTON_EVENT_B_SHORT,   // Button B (GPIO 38) single click — +1 hour
    BUTTON_EVENT_B_LONG,    // Button B (GPIO 38) long press (2s held) — sync animation
    BUTTON_EVENT_C_SHORT,   // Button C (GPIO 33) single click — +1 minute
    BUTTON_EVENT_C_LONG,    // Button C (GPIO 33) long press (2s held) — AS5600 calibration
} app_button_event_t;

// Configure all 3 buttons (A/B/C) using espressif/button component
esp_err_t buttons_init(void);

// Check for a button event; blocks up to timeout_ms (0 = non-blocking)
bool buttons_get_event(app_button_event_t *event, uint32_t timeout_ms);
