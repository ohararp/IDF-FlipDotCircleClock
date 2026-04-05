#pragma once

#include <time.h>
#include "esp_err.h"

// Start HTTP web server on port 80 (call after WiFi connects)
esp_err_t web_server_start(void);

// Stop HTTP web server
void web_server_stop(void);

// Update cached time (called from display task every second — avoids I2C in web handler)
void web_server_update_time(const struct tm *t);

// Update cached AS5600 angle (called from display task — avoids I2C in web handler)
void web_server_update_as5600(uint16_t angle);
