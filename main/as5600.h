#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#define AS5600_I2C_ADDR 0x36 // 7-bit I2C address of the AS5600 magnetic encoder

// Attach AS5600 to an existing I2C bus, probe for presence
esp_err_t as5600_init(i2c_master_bus_handle_t bus_handle);

// Read raw 12-bit angle (0–4095, maps to 0°–360°)
esp_err_t as5600_read_raw_angle(uint16_t *angle);

// Check if AS5600 was detected during init
bool as5600_is_connected(void);

// Convert raw 12-bit angle to degrees (0.0–360.0)
float as5600_to_degrees(uint16_t raw);

// Calculate shortest-path angular difference (positive = CW needed)
int as5600_angle_diff(uint16_t current, uint16_t target);

// Convert raw AS5600 value (0–4095) to motor steps (0–STEPPER_STEPS_PER_REV)
int as5600_to_steps(uint16_t raw);

// Convert minute (0–59) to expected AS5600 raw value using home offset
uint16_t as5600_minute_to_raw(int minute, uint16_t home_offset);
