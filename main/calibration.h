#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Enter calibration mode: disable motor so user can manually position hand to 12 o'clock
esp_err_t calibration_enter(void);

// Save current AS5600 angle as the 12 o'clock reference to NVS, re-enable motor
esp_err_t calibration_save(void);

// Cancel calibration without saving, re-enable motor
esp_err_t calibration_cancel(void);

// Load saved AS5600 calibration offset from NVS (returns 0 if uncalibrated)
uint16_t calibration_get_offset(void);

// Returns true if enough time (3s) has elapsed since entering calibration to accept a save
bool calibration_ready_to_save(void);

// Check if currently in calibration mode
bool calibration_is_active(void);
