#pragma once

#include "esp_err.h"
#include <stdbool.h>

// Button event types (expandable for buttons A and B later)
typedef enum {
    BUTTON_EVENT_C_PRESS,   // Button C (GPIO 33) short press
} button_event_t;

// Configure Button C (GPIO 33) with pull-up and ISR, create event queue
esp_err_t buttons_init(void);

// Check for a button event (non-blocking); returns true if event available
bool buttons_get_event(button_event_t *event, uint32_t timeout_ms);
