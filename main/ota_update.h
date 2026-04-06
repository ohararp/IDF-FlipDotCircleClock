#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

// Register OTA HTTP handlers on the given server (/ota/check, /ota/upload)
esp_err_t ota_register_handlers(httpd_handle_t server);
