// Web server: serves index.html and JSON API endpoints
// Uses esp_http_server (built-in) + cJSON (built-in)

#include "web_server.h"
#include "web_commands.h"
#include "action_log.h"
#include "network.h"
#include "timekeeping.h"
#include "stepper.h"
#include "as5600.h"
#include "nvm_storage.h"
#include "ota_update.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_partition.h"
#include "cJSON.h"
#include <string.h>

static const char *TAG = "web";

// HTTP server handle
static httpd_handle_t s_server = NULL;

// Cached time — updated by display task every second, read by status handler (no I2C contention)
static struct tm s_cached_time = {0};
static bool s_time_valid = false;

// Cached AS5600 angle — updated alongside time
static uint16_t s_cached_as5600 = 0;

// Called from display task to update cached time and AS5600 (avoids I2C reads in web handler)
void web_server_update_time(const struct tm *t)
{
    s_cached_time = *t;
    s_time_valid = true;
}

// Called from display task to update cached AS5600 angle
void web_server_update_as5600(uint16_t angle)
{
    s_cached_as5600 = angle;
}

// Embedded index.html (linked via EMBED_FILES in CMakeLists.txt)
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

// ── URI Handlers ─────────────────────────────────────────────────────────────

// GET / — serve the embedded index.html
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

// GET /status.json — clock status, network info, motor position
static esp_err_t status_handler(httpd_req_t *req)
{
    // Use cached time from display task — no I2C reads in web handler
    struct tm local = s_cached_time;

    char time_str[9];
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", local.tm_hour, local.tm_min, local.tm_sec);

    char date_str[36];
    snprintf(date_str, sizeof(date_str), "%04d-%02d-%02d", local.tm_year + 1900, local.tm_mon + 1, local.tm_mday);

    int hour12 = local.tm_hour % 12;
    if (hour12 == 0) hour12 = 12;

    const timezone_def_t *tz_list = timekeeping_get_timezone_list();
    uint8_t tz_idx = timekeeping_get_timezone_index();

    uint16_t step_delay = 0;
    nvm_get_step_delay(&step_delay);

    // Use cached AS5600 angle from display task (avoids I2C contention)
    uint16_t as5600_angle = s_cached_as5600;

    const esp_app_desc_t *app = esp_app_get_description();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "time", time_str);
    cJSON_AddStringToObject(root, "date", date_str);
    cJSON_AddNumberToObject(root, "hour_12", hour12);
    cJSON_AddStringToObject(root, "timezone_name", tz_list[tz_idx].display_name);
    cJSON_AddNumberToObject(root, "timezone_index", tz_idx);
    cJSON_AddStringToObject(root, "ip", network_get_ip_str());
    cJSON_AddNumberToObject(root, "rssi", network_get_rssi());
    cJSON_AddBoolToObject(root, "wifi_connected", network_get_state() == NETWORK_CONNECTED);
    cJSON_AddBoolToObject(root, "ntp_synced", network_ntp_synced());
    cJSON_AddNumberToObject(root, "uptime_s", (int)(esp_timer_get_time() / 1000000));
    int free_heap = (int)esp_get_free_heap_size();
    int total_heap = (int)heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    cJSON_AddNumberToObject(root, "free_heap", free_heap);
    cJSON_AddNumberToObject(root, "total_heap", total_heap);
    int free_psram = (int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    int total_psram = (int)heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    cJSON_AddNumberToObject(root, "free_psram", free_psram);
    cJSON_AddNumberToObject(root, "total_psram", total_psram);
    // Flash: sum all partition sizes for "used", total from flash chip size
    int flash_used = 0;
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    while (it) {
        const esp_partition_t *p = esp_partition_get(it);
        flash_used += p->size;
        it = esp_partition_next(it);
    }
    esp_partition_iterator_release(it);
    int flash_total = 16 * 1024 * 1024; // 16MB flash on FeatherS3
    cJSON_AddNumberToObject(root, "flash_used", flash_used);
    cJSON_AddNumberToObject(root, "flash_total", flash_total);
    cJSON_AddStringToObject(root, "version", app->version);
    cJSON_AddNumberToObject(root, "motor_pos", stepper_get_position());
    cJSON_AddNumberToObject(root, "as5600_angle", as5600_angle);
    cJSON_AddNumberToObject(root, "step_delay_us", step_delay);

    char *json = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    free(json);
    cJSON_Delete(root);
    return ESP_OK;
}

// GET /log.json — action log from RAM buffer (or persistent if ?persistent=true)
static esp_err_t log_handler(httpd_req_t *req)
{
    // Check for ?persistent=true query param
    char query[32] = {0};
    char *json = NULL;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        strstr(query, "persistent=true")) {
        json = action_log_get_persistent_json();
    } else {
        json = action_log_get_recent_json();
    }

    httpd_resp_set_type(req, "application/json");
    if (json) {
        httpd_resp_send(req, json, strlen(json));
        free(json);
    } else {
        httpd_resp_sendstr(req, "{\"entries\":[]}");
    }
    return ESP_OK;
}

// GET /get_timezone — list of 18 timezones with current selection
static esp_err_t timezone_handler(httpd_req_t *req)
{
    const timezone_def_t *tz_list = timekeeping_get_timezone_list();
    uint8_t current_idx = timekeeping_get_timezone_index();

    cJSON *root = cJSON_CreateObject();
    cJSON *arr = cJSON_AddArrayToObject(root, "timezones");

    for (int i = 0; i < TIMEZONE_COUNT; i++) {
        cJSON *tz = cJSON_CreateObject();
        cJSON_AddStringToObject(tz, "key", tz_list[i].key);
        cJSON_AddStringToObject(tz, "name", tz_list[i].display_name);
        cJSON_AddNumberToObject(tz, "offset", tz_list[i].utc_offset_min);
        cJSON_AddBoolToObject(tz, "dst", tz_list[i].dst_rule != DST_NONE);
        cJSON_AddItemToArray(arr, tz);
    }

    cJSON_AddStringToObject(root, "current", tz_list[current_idx].key);

    char *json = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    free(json);
    cJSON_Delete(root);
    return ESP_OK;
}

// GET /get_speed — current step delay
static esp_err_t speed_handler(httpd_req_t *req)
{
    uint16_t delay = 0;
    nvm_get_step_delay(&delay);
    char buf[32];
    snprintf(buf, sizeof(buf), "{\"delay_us\":%d}", delay);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

// ── POST handlers — send commands to main loop via queue ─────────────────────

// Helper: send a simple command (no param) and respond with {"ok":true}
static esp_err_t simple_cmd_handler(httpd_req_t *req, web_cmd_type_t type)
{
    web_cmd_t cmd = { .type = type, .param = 0 };
    if (xQueueSend(g_web_cmd_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":true}");
    } else {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"queue full\"}");
    }
    return ESP_OK;
}

// Helper: read POST body as JSON, extract int param
static int32_t read_json_param(httpd_req_t *req, const char *key)
{
    char buf[128];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) return -1;
    buf[len] = '\0';
    cJSON *root = cJSON_Parse(buf);
    if (!root) return -1;
    cJSON *val = cJSON_GetObjectItem(root, key);
    int32_t result = val ? val->valueint : -1;
    cJSON_Delete(root);
    return result;
}

static esp_err_t set_hour_handler(httpd_req_t *req) { return simple_cmd_handler(req, WEB_CMD_SET_HOUR); }
static esp_err_t set_min_handler(httpd_req_t *req) { return simple_cmd_handler(req, WEB_CMD_SET_MIN); }
static esp_err_t home_handler(httpd_req_t *req) { return simple_cmd_handler(req, WEB_CMD_HOME); }
static esp_err_t wipe_handler(httpd_req_t *req) { return simple_cmd_handler(req, WEB_CMD_WIPE); }
static esp_err_t refresh_handler(httpd_req_t *req) { return simple_cmd_handler(req, WEB_CMD_REFRESH); }
static esp_err_t anim_sync_handler(httpd_req_t *req) { return simple_cmd_handler(req, WEB_CMD_ANIM_SYNC); }
static esp_err_t sync_ntp_handler(httpd_req_t *req) { return simple_cmd_handler(req, WEB_CMD_SYNC_NTP); }
static esp_err_t cal_start_handler(httpd_req_t *req) { return simple_cmd_handler(req, WEB_CMD_CAL_START); }
static esp_err_t cal_save_handler(httpd_req_t *req)
{
    web_cmd_t cmd = { .type = WEB_CMD_CAL_SAVE, .param = 0 };
    if (xQueueSend(g_web_cmd_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Return the cached AS5600 angle so frontend can display it
        char resp[64];
        snprintf(resp, sizeof(resp), "{\"ok\":true,\"angle\":%d}", s_cached_as5600);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, resp);
    } else {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"queue full\"}");
    }
    return ESP_OK;
}
static esp_err_t cal_cancel_handler(httpd_req_t *req) { return simple_cmd_handler(req, WEB_CMD_CAL_CANCEL); }

// POST /set_timezone — body: {"timezone":"US/Eastern"} or {"index":6}
static esp_err_t set_timezone_handler(httpd_req_t *req)
{
    // Try reading timezone key first, fall back to index
    int32_t idx = -1;

    char buf[128];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len > 0) {
        buf[len] = '\0';
        cJSON *root = cJSON_Parse(buf);
        if (root) {
            cJSON *key_val = cJSON_GetObjectItem(root, "timezone");
            cJSON *idx_val = cJSON_GetObjectItem(root, "index");
            if (key_val && cJSON_IsString(key_val)) {
                // Find index by key
                const timezone_def_t *tz_list = timekeeping_get_timezone_list();
                for (int i = 0; i < TIMEZONE_COUNT; i++) {
                    if (strcmp(tz_list[i].key, key_val->valuestring) == 0) {
                        idx = i;
                        break;
                    }
                }
            } else if (idx_val) {
                idx = idx_val->valueint;
            }
            cJSON_Delete(root);
        }
    }

    if (idx >= 0 && idx < TIMEZONE_COUNT) {
        web_cmd_t cmd = { .type = WEB_CMD_SET_TIMEZONE, .param = idx };
        xQueueSend(g_web_cmd_queue, &cmd, pdMS_TO_TICKS(100));
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":true}");
    } else {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid timezone\"}");
    }
    return ESP_OK;
}

// POST /set_speed — body: {"delay_us":450}
static esp_err_t set_speed_handler(httpd_req_t *req)
{
    int32_t delay = read_json_param(req, "delay_us");
    if (delay >= 100 && delay <= 1000) {
        web_cmd_t cmd = { .type = WEB_CMD_SET_SPEED, .param = delay };
        xQueueSend(g_web_cmd_queue, &cmd, pdMS_TO_TICKS(100));

        char resp[64];
        snprintf(resp, sizeof(resp), "{\"ok\":true,\"delay_us\":%d}", (int)delay);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, resp);
    } else {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"delay_us must be 100-1000\"}");
    }
    return ESP_OK;
}

// ── Public API ───────────────────────────────────────────────────────────────

esp_err_t web_server_start(void)
{
    if (s_server) {
        ESP_LOGW(TAG, "Web server already running");
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 24; // room for all endpoints + OTA
    config.stack_size = 8192;     // larger stack for JSON building

    esp_err_t ret = httpd_start(&s_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web server: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register URI handlers — GET endpoints
    httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_handler };
    httpd_uri_t uri_status = { .uri = "/status.json", .method = HTTP_GET, .handler = status_handler };
    httpd_uri_t uri_log = { .uri = "/log.json", .method = HTTP_GET, .handler = log_handler };
    httpd_uri_t uri_tz = { .uri = "/get_timezone", .method = HTTP_GET, .handler = timezone_handler };
    httpd_uri_t uri_speed = { .uri = "/get_speed", .method = HTTP_GET, .handler = speed_handler };

    httpd_register_uri_handler(s_server, &uri_root);
    httpd_register_uri_handler(s_server, &uri_status);
    httpd_register_uri_handler(s_server, &uri_log);
    httpd_register_uri_handler(s_server, &uri_tz);
    httpd_register_uri_handler(s_server, &uri_speed);

    // POST endpoints — all send commands to main loop via queue
    httpd_uri_t post_handlers[] = {
        { .uri = "/set_hour",    .method = HTTP_POST, .handler = set_hour_handler },
        { .uri = "/set_min",     .method = HTTP_POST, .handler = set_min_handler },
        { .uri = "/home",        .method = HTTP_POST, .handler = home_handler },
        { .uri = "/wipe",        .method = HTTP_POST, .handler = wipe_handler },
        { .uri = "/refresh",     .method = HTTP_POST, .handler = refresh_handler },
        { .uri = "/anim/sync",   .method = HTTP_POST, .handler = anim_sync_handler },
        { .uri = "/sync_ntp",    .method = HTTP_POST, .handler = sync_ntp_handler },
        { .uri = "/set_timezone",.method = HTTP_POST, .handler = set_timezone_handler },
        { .uri = "/set_speed",   .method = HTTP_POST, .handler = set_speed_handler },
        { .uri = "/cal_start",   .method = HTTP_POST, .handler = cal_start_handler },
        { .uri = "/cal_save",    .method = HTTP_POST, .handler = cal_save_handler },
        { .uri = "/cal_cancel",  .method = HTTP_POST, .handler = cal_cancel_handler },
    };
    for (int i = 0; i < sizeof(post_handlers) / sizeof(post_handlers[0]); i++) {
        httpd_register_uri_handler(s_server, &post_handlers[i]);
    }

    // Register OTA firmware update endpoints (/ota/check, /ota/upload)
    ota_register_handlers(s_server);

    ESP_LOGI(TAG, "Web server started on port 80");
    return ESP_OK;
}

void web_server_stop(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
        ESP_LOGI(TAG, "Web server stopped");
    }
}
