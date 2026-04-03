#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Configure stepper GPIOs (EN, STEP, DIR, MS), Hall sensor input, load step delay from NVS
esp_err_t stepper_init(void);

// Enable motor holding torque (EN pin LOW = TMC2209 enabled)
void stepper_enable(void);

// Disable motor holding torque (EN pin HIGH = TMC2209 disabled, motor freewheels)
void stepper_disable(void);

// Step motor N steps in given direction; uses current step delay from NVS
void stepper_multi_step(int steps, bool clockwise);

// Symmetric Hall sensor edge detection: find magnet center and set as home (stepNow=0)
esp_err_t stepper_find_home(void);

// Get current step position (0–12799, wraps at STEPPER_STEPS_PER_REV)
int stepper_get_position(void);

// Manually set step position counter (e.g., after homing or calibration)
void stepper_set_position(int position);

// Read Hall sensor: true = magnet detected (sensor active LOW)
bool stepper_hall_detected(void);

// Closed-loop move to target AS5600 raw angle using batched stepping + feedback
// Falls back to open-loop if AS5600 unavailable. Returns true if target reached.
bool stepper_move_to_angle(uint16_t target_raw, int tolerance);
