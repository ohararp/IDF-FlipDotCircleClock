#pragma once

#include <time.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#define DS3231_I2C_ADDR 0x68 // 7-bit I2C address of the DS3231 RTC

// Create I2C master bus (SDA/SCL from gpio_config.h, 400 kHz), attach DS3231, return bus handle for sharing
esp_err_t ds3231_init(i2c_master_bus_handle_t *ret_bus_handle);

// Read current time from DS3231 registers 0x00–0x06 into struct tm
esp_err_t ds3231_get_time(struct tm *time);

// Write struct tm to DS3231 registers 0x00–0x06 (BCD-encoded, 24-hour format)
esp_err_t ds3231_set_time(const struct tm *time);

// Seed DS3231 with firmware compile timestamp (__DATE__/__TIME__), called on every boot
esp_err_t ds3231_set_time_from_compile(void);
