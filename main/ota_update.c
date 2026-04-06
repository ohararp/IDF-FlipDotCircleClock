// OTA firmware update via web upload
// Receives .bin file from browser, writes to next OTA partition, reboots

#include "ota_update.h"
#include "action_log.h"
#include "oled_display.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "cJSON.h"
#include <string.h>

static const char *TAG = "ota";

// Scratch buffer for receiving firmware chunks (4KB aligned for flash writes)
#define OTA_BUF_SIZE 4096

// ── POST /ota/check — return current firmware version ────────────────────────

static esp_err_t ota_check_handler(httpd_req_t *req)
{
    const esp_app_desc_t *app = esp_app_get_description();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "current_version", app->version);
    // GitHub release check not yet implemented — always report up to date
    cJSON_AddStringToObject(root, "latest_version", app->version);
    cJSON_AddBoolToObject(root, "update_available", false);

    char *json = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    free(json);
    cJSON_Delete(root);
    return ESP_OK;
}

// ── POST /ota/upload — receive .bin and flash to next OTA slot ───────────────

static esp_err_t ota_upload_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "OTA upload started, content length: %d", req->content_len);
    action_log_add("OTA upload started");
    oled_set_status("OTA: uploading...");

    // Reject suspiciously small or missing uploads
    if (req->content_len < 1024) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"File too small\"}");
        return ESP_OK;
    }

    // Find the next OTA partition to write to
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        ESP_LOGE(TAG, "No OTA partition available");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"No OTA partition\"}");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Writing to partition: %s (offset=0x%lx, size=%ldKB)",
             update_partition->label, update_partition->address,
             update_partition->size / 1024);

    // Begin OTA write session
    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"OTA begin failed\"}");
        return ESP_OK;
    }

    // Receive firmware in chunks and write to flash
    char *buf = malloc(OTA_BUF_SIZE);
    if (!buf) {
        esp_ota_abort(ota_handle);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Out of memory\"}");
        return ESP_OK;
    }

    int total_received = 0;
    bool failed = false;

    while (total_received < req->content_len) {
        // Read a chunk from HTTP body
        int received = httpd_req_recv(req, buf, OTA_BUF_SIZE);
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue; // Retry on timeout
            }
            ESP_LOGE(TAG, "Receive failed at %d bytes", total_received);
            failed = true;
            break;
        }

        // Write chunk to OTA partition
        err = esp_ota_write(ota_handle, buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed at %d bytes: %s",
                     total_received, esp_err_to_name(err));
            failed = true;
            break;
        }

        total_received += received;

        // Log progress every 10% to action log + OLED
        int pct = (total_received * 100) / req->content_len;
        int prev_pct = ((total_received - received) * 100) / req->content_len;
        if ((pct / 10) > (prev_pct / 10)) {
            char msg[48];
            snprintf(msg, sizeof(msg), "OTA: %d%%", pct);
            oled_set_status(msg);
            snprintf(msg, sizeof(msg), "OTA progress: %d%% (%dKB/%dKB)",
                     pct, total_received / 1024, req->content_len / 1024);
            action_log_add(msg);
        }
    }

    free(buf);

    if (failed) {
        esp_ota_abort(ota_handle);
        action_log_add("OTA upload failed");
        oled_set_status("OTA: FAILED");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Upload failed\"}");
        return ESP_OK;
    }

    // Finalize OTA — validates image header and hash
    oled_set_status("OTA: validating...");
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        action_log_add("OTA validation failed");
        httpd_resp_set_type(req, "application/json");

        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Invalid firmware image\"}");
        } else {
            httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"OTA finalize failed\"}");
        }
        return ESP_OK;
    }

    // Set the new partition as boot target
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        action_log_add("OTA set boot partition failed");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Set boot partition failed\"}");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "OTA complete: %dKB written to %s, rebooting...",
             total_received / 1024, update_partition->label);
    action_log_add("OTA complete, rebooting");
    oled_set_status("OTA: rebooting...");

    // Send success response before rebooting
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");

    // Delay briefly so the HTTP response gets sent, then reboot
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK; // Never reached
}

// ── Public API ───────────────────────────────────────────────────────────────

// Register OTA HTTP endpoints on the given server
esp_err_t ota_register_handlers(httpd_handle_t server)
{
    // POST /ota/check — check current version
    httpd_uri_t uri_check = {
        .uri = "/ota/check",
        .method = HTTP_POST,
        .handler = ota_check_handler,
    };
    httpd_register_uri_handler(server, &uri_check);

    // POST /ota/upload — receive .bin and flash
    httpd_uri_t uri_upload = {
        .uri = "/ota/upload",
        .method = HTTP_POST,
        .handler = ota_upload_handler,
    };
    httpd_register_uri_handler(server, &uri_upload);

    ESP_LOGI(TAG, "OTA endpoints registered");
    return ESP_OK;
}
