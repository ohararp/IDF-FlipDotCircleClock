#pragma once

#include "esp_err.h"
#include <stdint.h>

// Default configuration values matching CircuitPython defaults
#define NVM_DEFAULT_TIMEZONE_INDEX  0     // US/Eastern
#define NVM_DEFAULT_HOME_OFFSET     0     // no offset from Hall sensor home
#define NVM_DEFAULT_STEP_DELAY_US   450   // 450 µs step pulse delay
#define NVM_DEFAULT_AS5600_CAL      0     // uncalibrated

// Initialize NVS flash and open the "flipclock" namespace; sets defaults on first boot
esp_err_t nvm_init(void);

// Get/set timezone index (0–17, maps to timezone definitions array)
esp_err_t nvm_get_timezone_index(uint8_t *index);
esp_err_t nvm_set_timezone_index(uint8_t index);

// Get/set Hall sensor home offset in steps (signed, corrects mechanical misalignment)
esp_err_t nvm_get_home_offset(int16_t *offset);
esp_err_t nvm_set_home_offset(int16_t offset);

// Get/set stepper step delay in microseconds (controls motor speed)
esp_err_t nvm_get_step_delay(uint16_t *delay_us);
esp_err_t nvm_set_step_delay(uint16_t delay_us);

// Get/set AS5600 calibration angle (12-bit raw value, 0–4095)
esp_err_t nvm_get_as5600_cal_angle(uint16_t *angle);
esp_err_t nvm_set_as5600_cal_angle(uint16_t angle);

// Erase all stored config and restore defaults
esp_err_t nvm_factory_reset(void);
