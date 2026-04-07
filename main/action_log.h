#pragma once

#include "esp_err.h"

// Max entries in RAM ring buffer (512 entries × ~100 bytes ≈ 50KB in PSRAM)
#define ACTION_LOG_RAM_SIZE 512

// Initialize action log (RAM ring buffer + LittleFS persistent log)
esp_err_t action_log_init(void);

// Add a timestamped entry to both RAM buffer and persistent log
void action_log_add(const char *msg);

// Get recent entries as JSON string (caller must free). Returns "{"entries":[...]}"
char *action_log_get_recent_json(void);

// Get persistent log entries as JSON string (caller must free)
char *action_log_get_persistent_json(void);
