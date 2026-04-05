// Web server: serves index.html and JSON API endpoints
// Uses esp_http_server (built-in) + cJSON (built-in)

#include "web_server.h"
#include "network.h"
#include "timekeeping.h"
#include "stepper.h"
#include "as5600.h"
#include "nvm_storage.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "cJSON.h"
#include <string.h>

static const char *TAG = "web";

// HTTP server handle
static httpd_handle_t s_server = NULL;

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
    struct tm local;
    timekeeping_get_local_time(&local);

    char time_str[9];
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", local.tm_hour, local.tm_min, local.tm_sec);

    char date_str[11];
    snprintf(date_str, sizeof(date_str), "%04d-%02d-%02d", local.tm_year + 1900, local.tm_mon + 1, local.tm_mday);

    int hour12 = local.tm_hour % 12;
    if (hour12 == 0) hour12 = 12;

    const timezone_def_t *tz_list = timekeeping_get_timezone_list();
    uint8_t tz_idx = timekeeping_get_timezone_index();

    uint16_t step_delay = 0;
    nvm_get_step_delay(&step_delay);

    uint16_t as5600_angle = 0;
    if (as5600_is_connected()) {
        as5600_read_raw_angle(&as5600_angle);
    }

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
    cJSON_AddNumberToObject(root, "free_heap", (int)esp_get_free_heap_size());
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

// ── Public API ───────────────────────────────────────────────────────────────

esp_err_t web_server_start(void)
{
    if (s_server) {
        ESP_LOGW(TAG, "Web server already running");
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 20; // room for all endpoints
    config.stack_size = 8192;     // larger stack for JSON building

    esp_err_t ret = httpd_start(&s_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web server: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register URI handlers
    httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_handler };
    httpd_uri_t uri_status = { .uri = "/status.json", .method = HTTP_GET, .handler = status_handler };

    httpd_register_uri_handler(s_server, &uri_root);
    httpd_register_uri_handler(s_server, &uri_status);

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
