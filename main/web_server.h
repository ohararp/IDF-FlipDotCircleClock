#pragma once

#include "esp_err.h"

// Start HTTP web server on port 80 (call after WiFi connects)
esp_err_t web_server_start(void);

// Stop HTTP web server
void web_server_stop(void);
