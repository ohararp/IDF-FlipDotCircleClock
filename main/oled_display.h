#pragma once

#include <time.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#define OLED_I2C_ADDR 0x3C // 7-bit I2C address of the SH1107 OLED

// Initialize SH1107 128x64 OLED on an existing I2C bus via U8G2 library
esp_err_t oled_init(i2c_master_bus_handle_t bus_handle);

// Main display: time, date, status, network info, WiFi dot (matches CP layout)
// Pulls network state from network.h directly. status_text can be NULL for default.
void oled_update_main(const struct tm *time, const char *status_text);

// Set a global status message on OLED Line 2 (pass NULL or "" to clear)
void oled_set_status(const char *msg);

// Clear the entire OLED display
void oled_clear(void);

// Print a line to the OLED scrolling terminal (8 lines × ~25 chars, 5x7 font)
void oled_terminal_print(const char *line);

// Display a QR code on the OLED (centered, scaled to fit 64x64 area)
void oled_show_qr(const char *text);
