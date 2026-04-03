#pragma once

#include <time.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#define OLED_I2C_ADDR 0x3C // 7-bit I2C address of the SH1107 OLED

// Initialize SH1107 128x64 OLED on an existing I2C bus via U8G2 library
esp_err_t oled_init(i2c_master_bus_handle_t bus_handle);

// Display time as HH:MM:SS in large font, centered on screen
void oled_update_time(const struct tm *time);

// Display status info: IP address, RSSI, uptime, motor position
void oled_update_status(const char *ip, int rssi, uint32_t uptime, int motor_pos);

// Clear the entire OLED display
void oled_clear(void);
