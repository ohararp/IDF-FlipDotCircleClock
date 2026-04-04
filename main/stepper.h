#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Configure stepper GPIOs (EN, STEP, DIR, MS), load step delay from NVS
esp_err_t stepper_init(void);

// Enable motor holding torque (EN pin LOW = TMC2209 enabled)
void stepper_enable(void);

// Disable motor holding torque (EN pin HIGH = TMC2209 disabled, motor freewheels)
void stepper_disable(void);

// Step motor N steps in given direction; uses current step delay from NVS
void stepper_multi_step(int steps, bool clockwise);

// Home to 12 o'clock using AS5600 calibration offset (no Hall sensor)
esp_err_t stepper_find_home(void);

// Get current step position (0–12799, wraps at STEPPER_STEPS_PER_REV)
int stepper_get_position(void);

// Manually set step position counter (e.g., after homing or calibration)
void stepper_set_position(int position);

// Closed-loop move to target AS5600 raw angle using batched stepping + feedback
// Returns true if target reached, false if AS5600 unavailable or max_steps exceeded.
bool stepper_move_to_angle(uint16_t target_raw, int tolerance);
