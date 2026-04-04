#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Button event types: short press (<2s) and long press (>=2s) for each button
typedef enum {
    BUTTON_EVENT_A_SHORT,   // Button A (GPIO 1) short press — re-home sequence
    BUTTON_EVENT_A_LONG,    // Button A (GPIO 1) long press — reserved for WiFi reconnect
    BUTTON_EVENT_B_SHORT,   // Button B (GPIO 38) short press — +1 hour
    BUTTON_EVENT_B_LONG,    // Button B (GPIO 38) long press — sync animation
    BUTTON_EVENT_C_SHORT,   // Button C (GPIO 33) short press — +1 minute
    BUTTON_EVENT_C_LONG,    // Button C (GPIO 33) long press — AS5600 calibration
} button_event_t;

// Configure all 3 buttons (A/B/C) with pull-up, ISR on both edges, event queue
esp_err_t buttons_init(void);

// Check for a button event; blocks up to timeout_ms (0 = non-blocking)
bool buttons_get_event(button_event_t *event, uint32_t timeout_ms);
