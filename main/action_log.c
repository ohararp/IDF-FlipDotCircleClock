// Action log: RAM ring buffer (128 entries)
// LittleFS persistent logging planned but disabled until partition/component verified.

#include "action_log.h"
#include "timekeeping.h"
#include "esp_log.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

static const char *TAG = "log";

// ── RAM ring buffer ──────────────────────────────────────────────────────────

#define MSG_MAX_LEN 80
#define TS_MAX_LEN  20

typedef struct {
    char ts[TS_MAX_LEN];   // "HH:MM:SS"
    char msg[MSG_MAX_LEN];
} log_entry_t;

static log_entry_t s_entries[ACTION_LOG_RAM_SIZE];
static int s_head = 0;      // next write position
static int s_count = 0;     // total entries (capped at ACTION_LOG_RAM_SIZE)
static SemaphoreHandle_t s_log_mutex = NULL;

// ── Public API ───────────────────────────────────────────────────────────────

esp_err_t action_log_init(void)
{
    s_log_mutex = xSemaphoreCreateMutex();
    memset(s_entries, 0, sizeof(s_entries));
    s_head = 0;
    s_count = 0;
    action_log_add("System boot");
    return ESP_OK;
}

void action_log_add(const char *msg)
{
    char ts[TS_MAX_LEN];
    struct tm local;
    if (timekeeping_get_local_time(&local) == ESP_OK) {
        snprintf(ts, sizeof(ts), "%02d:%02d:%02d",
                 local.tm_hour, local.tm_min, local.tm_sec);
    } else {
        snprintf(ts, sizeof(ts), "--:--:--");
    }

    if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100))) {
        strncpy(s_entries[s_head].ts, ts, TS_MAX_LEN - 1);
        strncpy(s_entries[s_head].msg, msg, MSG_MAX_LEN - 1);
        s_head = (s_head + 1) % ACTION_LOG_RAM_SIZE;
        if (s_count < ACTION_LOG_RAM_SIZE) s_count++;
        xSemaphoreGive(s_log_mutex);
    }

    ESP_LOGI(TAG, "[%s] %s", ts, msg);
}

char *action_log_get_recent_json(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *arr = cJSON_AddArrayToObject(root, "entries");

    if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100))) {
        // Newest first
        for (int i = s_count - 1; i >= 0; i--) {
            int start = (s_count < ACTION_LOG_RAM_SIZE) ? 0 : s_head;
            int idx = (start + i) % ACTION_LOG_RAM_SIZE;
            cJSON *entry = cJSON_CreateObject();
            cJSON_AddStringToObject(entry, "ts", s_entries[idx].ts);
            cJSON_AddStringToObject(entry, "msg", s_entries[idx].msg);
            cJSON_AddItemToArray(arr, entry);
        }
        xSemaphoreGive(s_log_mutex);
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

char *action_log_get_persistent_json(void)
{
    // Persistent logging disabled — return empty
    cJSON *root = cJSON_CreateObject();
    cJSON_AddArrayToObject(root, "entries");
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}
