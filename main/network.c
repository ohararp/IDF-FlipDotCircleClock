// Network module: WiFi + BLE provisioning + NTP time sync
// Uses espressif/network_provisioning for BLE-based credential setup
// and esp_sntp for NTP time synchronization.
// Ref: https://components.espressif.com/components/espressif/network_provisioning

#include "network.h"
#include "timekeeping.h"
#include "oled_display.h"
#include "action_log.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "mdns.h"  // espressif/mdns component
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "network_provisioning/manager.h"
#include "network_provisioning/scheme_ble.h"
#include <string.h>

static const char *TAG = "network";

// Current network state
static network_state_t s_state = NETWORK_PROVISIONING;

// IP address string buffer
static char s_ip_str[16] = "N/A";

// NTP sync flag
static bool s_ntp_synced = false;

// Event group bits for WiFi connection signaling
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
static EventGroupHandle_t s_wifi_event_group;

// Retry counter and exponential backoff for WiFi reconnection
static int s_retry_count = 0;
#define MAX_RETRIES 5
// Backoff delays in ms: 2s, 4s, 8s, 16s, 30s
static const int s_backoff_ms[] = {2000, 4000, 8000, 16000, 30000};

// NTP sync counter (how many times NTP has synced since boot)
static int s_ntp_sync_count = 0;

// Periodic reconnect timer (fires every 60s after going OFFLINE)
static esp_timer_handle_t s_reconnect_timer = NULL;

// Track if NTP/SNTP has been initialized (so we restart instead of re-init)
static bool s_sntp_initialized = false;

// Forward declarations
static void start_ntp(void);
static void start_reconnect_timer(void);
static void stop_reconnect_timer(void);

// ── Event handlers ───────────────────────────────────────────────────────────

// WiFi + provisioning event handler
static void event_handler(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data)
{
    // WiFi STA events
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            s_state = NETWORK_DISCONNECTED;
            ESP_LOGW(TAG, "WiFi disconnected");
            action_log_add("WiFi disconnected");
            if (s_retry_count < MAX_RETRIES) {
                // Exponential backoff: wait before retrying
                int delay = s_backoff_ms[s_retry_count < 5 ? s_retry_count : 4];
                s_retry_count++;
                ESP_LOGI(TAG, "Retrying WiFi in %dms (%d/%d)...", delay, s_retry_count, MAX_RETRIES);
                vTaskDelay(pdMS_TO_TICKS(delay));
                esp_wifi_connect();
            } else {
                s_state = NETWORK_OFFLINE;
                ESP_LOGW(TAG, "WiFi offline after %d retries — periodic reconnect every 5 min", MAX_RETRIES);
                action_log_add("WiFi offline (retries exhausted)");
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                // Start periodic reconnect timer (every 60s)
                start_reconnect_timer();
            }
        }
    }

    // IP events — fires on every WiFi connect (including reconnects)
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "WiFi connected: %s", s_ip_str);
        action_log_add("WiFi connected");
        s_state = NETWORK_CONNECTED;
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        // Stop periodic reconnect timer if running
        stop_reconnect_timer();

        // Start or restart NTP on every connect/reconnect
        start_ntp();

        // Start mDNS so clock is discoverable as flipclock.local
        mdns_init();
        mdns_hostname_set("flipclock");
        ESP_LOGI(TAG, "mDNS: flipclock.local");
    }

    // Provisioning events
    if (event_base == NETWORK_PROV_EVENT) {
        switch (event_id) {
        case NETWORK_PROV_START:
            ESP_LOGI(TAG, "Provisioning started — use ESP BLE Prov app");
            action_log_add("BLE provisioning started");
            oled_terminal_print("BLE Prov started");
            oled_terminal_print("Use ESP BLE Prov app");
            s_state = NETWORK_PROVISIONING;
            break;
        case NETWORK_PROV_WIFI_CRED_RECV: {
            network_prov_wifi_sta_conn_info_t *info = (network_prov_wifi_sta_conn_info_t *)event_data;
            ESP_LOGI(TAG, "Prov: received SSID=%s", (const char *)info->ssid);
            oled_terminal_print("Prov: creds received");
            break;
        }
        case NETWORK_PROV_WIFI_CRED_SUCCESS:
            ESP_LOGI(TAG, "Prov: credentials applied successfully");
            action_log_add("WiFi credentials received");
            oled_terminal_print("Prov: success!");
            // Only change state if still in provisioning — don't overwrite CONNECTED
            if (s_state == NETWORK_PROVISIONING) {
                s_state = NETWORK_CONNECTING;
            }
            break;
        case NETWORK_PROV_WIFI_CRED_FAIL:
            ESP_LOGW(TAG, "Prov: credentials failed");
            action_log_add("WiFi credentials failed");
            oled_terminal_print("Prov: failed!");
            break;
        case NETWORK_PROV_END:
            ESP_LOGI(TAG, "Provisioning ended");
            network_prov_mgr_deinit();
            break;
        default:
            break;
        }
    }
}

// ── NTP ──────────────────────────────────────────────────────────────────────

// NTP sync callback — called when SNTP gets a valid time
static void ntp_sync_callback(struct timeval *tv)
{
    s_ntp_sync_count++;
    time_t now = tv->tv_sec;
    ESP_LOGI(TAG, "NTP synced (#%d): UTC epoch=%ld", s_ntp_sync_count, (long)now);
    oled_terminal_print("NTP synced!");
    action_log_add("NTP time synced");

    // Write UTC to RTC and enable timezone conversion
    timekeeping_sync_from_ntp(now);
    timekeeping_mark_ntp_synced();
    s_ntp_synced = true;
}

// Start or restart SNTP client (safe to call on every WiFi reconnect)
static void start_ntp(void)
{
    if (s_sntp_initialized) {
        ESP_LOGI(TAG, "Restarting NTP sync...");
        esp_sntp_restart();
    } else {
        ESP_LOGI(TAG, "Starting NTP sync...");
        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, "pool.ntp.org");
        esp_sntp_set_time_sync_notification_cb(ntp_sync_callback);
        esp_sntp_init();
        s_sntp_initialized = true;
    }
}

// Request NTP re-sync (called from main loop at each hour boundary)
void network_request_ntp_resync(void)
{
    if (s_state == NETWORK_CONNECTED && s_sntp_initialized) {
        ESP_LOGI(TAG, "Hourly NTP re-sync (top of hour)");
        action_log_add("NTP re-sync (hourly)");
        esp_sntp_restart();
    } else {
        ESP_LOGW(TAG, "NTP re-sync skipped (state=%d, sntp_init=%d)", s_state, s_sntp_initialized);
    }
}

// ── Periodic reconnect (after going OFFLINE) ─────────────────────────────────

// Timer callback: attempt WiFi reconnect every 60s while OFFLINE
static void reconnect_timer_cb(void *arg)
{
    if (s_state == NETWORK_OFFLINE) {
        ESP_LOGI(TAG, "Periodic reconnect attempt...");
        action_log_add("WiFi auto-reconnect attempt");
        s_retry_count = 0; // reset retry counter for fresh attempt
        s_state = NETWORK_CONNECTING;
        esp_wifi_connect();
    }
}

// Start the 5-minute periodic reconnect timer
static void start_reconnect_timer(void)
{
    if (s_reconnect_timer == NULL) {
        esp_timer_create_args_t args = {
            .callback = reconnect_timer_cb,
            .name = "wifi_reconnect",
        };
        esp_timer_create(&args, &s_reconnect_timer);
    }
    esp_timer_start_periodic(s_reconnect_timer, 300ULL * 1000000ULL); // 5 minutes
    ESP_LOGI(TAG, "Periodic reconnect timer started (5 min)");
}

// Stop the periodic reconnect timer (called when WiFi connects)
static void stop_reconnect_timer(void)
{
    if (s_reconnect_timer != NULL) {
        esp_timer_stop(s_reconnect_timer);
        ESP_LOGI(TAG, "Periodic reconnect timer stopped");
    }
}

// ── Provisioning ─────────────────────────────────────────────────────────────

// NVS key for "provision on next boot" flag
#define NVS_KEY_PROV_REQUEST "prov_req"

// Check if provisioning was requested (via Button A long → reboot)
static bool check_prov_requested(void)
{
    nvs_handle_t nvs;
    uint8_t val = 0;
    if (nvs_open("flipclock", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_get_u8(nvs, NVS_KEY_PROV_REQUEST, &val);
        if (val == 1) {
            // Clear the flag so we don't provision again on next reboot
            nvs_set_u8(nvs, NVS_KEY_PROV_REQUEST, 0);
            nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
    return val == 1;
}

// Set the "provision on next boot" flag in NVS
static void request_provisioning(void)
{
    nvs_handle_t nvs;
    if (nvs_open("flipclock", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, NVS_KEY_PROV_REQUEST, 1);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

// (has_wifi_credentials and decide_boot_path removed — replaced by prov manager check in network_init)

// Start BLE provisioning on-demand (called from Button A long press)
void network_start_provisioning(void)
{
    ESP_LOGI(TAG, "Starting BLE provisioning on-demand");

    // Initialize provisioning manager
    network_prov_mgr_config_t config = {
        .scheme = network_prov_scheme_ble,
        .scheme_event_handler = NETWORK_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM,
    };
    ESP_ERROR_CHECK(network_prov_mgr_init(config));

    // Generate device name with last 3 bytes of MAC for uniqueness
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    char service_name[20];
    snprintf(service_name, sizeof(service_name), "FlipClk_%02X%02X", mac[4], mac[5]);
    ESP_LOGI(TAG, "BLE device name: %s", service_name);

    // Start provisioning with Security 1 (proof-of-possession = "flipdot")
    const char *pop = "flipdot";
    s_state = NETWORK_PROVISIONING;
    ESP_ERROR_CHECK(network_prov_mgr_start_provisioning(
        NETWORK_PROV_SECURITY_1, pop, service_name, NULL));

    // Show provisioning info on OLED terminal + serial
    char buf[26];
    snprintf(buf, sizeof(buf), "BLE: %s", service_name);
    oled_terminal_print(buf);
    oled_terminal_print("POP: flipdot");
    oled_terminal_print("Use ESP BLE Prov app");
    oled_terminal_print("Hold A to reset WiFi");

    // Also print QR payload to serial for reference
    char qr_payload[128];
    snprintf(qr_payload, sizeof(qr_payload),
             "{\"ver\":\"v1\",\"name\":\"%s\",\"pop\":\"%s\",\"transport\":\"ble\"}",
             service_name, pop);
    ESP_LOGI(TAG, "QR payload: %s", qr_payload);
}

// ── Public API ───────────────────────────────────────────────────────────────

// Initialize networking — non-blocking. Three paths: provision, connect, or skip.
esp_err_t network_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    // Initialize TCP/IP stack and event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(NETWORK_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    // Initialize WiFi driver
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Init prov manager to check credential state (kept alive — not deinit'd)
    network_prov_mgr_config_t prov_config = {
        .scheme = network_prov_scheme_ble,
        .scheme_event_handler = NETWORK_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM,
    };
    ESP_ERROR_CHECK(network_prov_mgr_init(prov_config));

    // Check if Button A requested provisioning on this boot
    bool prov_requested = check_prov_requested();
    if (prov_requested) {
        ESP_LOGI(TAG, "Provisioning requested — erasing old credentials");
        network_prov_mgr_reset_wifi_provisioning();
    }

    // Check if credentials exist
    bool provisioned = false;
    ESP_ERROR_CHECK(network_prov_mgr_is_wifi_provisioned(&provisioned));

    if (prov_requested || !provisioned) {
        if (!prov_requested) {
            // No credentials and not requested — skip WiFi entirely
            ESP_LOGI(TAG, "No WiFi — hold A to setup");
            oled_terminal_print("No WiFi config");
            oled_terminal_print("Hold A to setup");
            network_prov_mgr_deinit(); // release BLE memory since we won't provision
            s_state = NETWORK_OFFLINE;
        } else {
            // Provisioning requested — start BLE prov (prov manager already init'd)
            ESP_LOGI(TAG, "Starting BLE provisioning");
            uint8_t mac[6];
            esp_wifi_get_mac(WIFI_IF_STA, mac);
            char service_name[20];
            snprintf(service_name, sizeof(service_name), "FlipClk_%02X%02X", mac[4], mac[5]);

            const char *pop = "flipdot";
            s_state = NETWORK_PROVISIONING;
            ESP_ERROR_CHECK(network_prov_mgr_start_provisioning(
                NETWORK_PROV_SECURITY_1, pop, service_name, NULL));

            // Show QR code on OLED for scanning
            char qr_payload[128];
            snprintf(qr_payload, sizeof(qr_payload),
                     "{\"ver\":\"v1\",\"name\":\"%s\",\"pop\":\"%s\",\"transport\":\"ble\"}",
                     service_name, pop);
            ESP_LOGI(TAG, "QR payload: %s", qr_payload);
            oled_show_qr(qr_payload);

            ESP_LOGI(TAG, "BLE device: %s, POP: %s", service_name, pop);
        }
    } else {
        // Credentials exist — connect directly
        ESP_LOGI(TAG, "WiFi credentials found — connecting");
        oled_terminal_print("WiFi: connecting...");
        s_state = NETWORK_CONNECTING;
        network_prov_mgr_deinit(); // don't need prov manager for direct connect
        esp_wifi_start();
        // NTP will start automatically in IP_EVENT_STA_GOT_IP handler
    }

    return ESP_OK;
}

// NTP and reconnect are now handled in event handlers — no background wait task needed

// Manual WiFi reconnect (Button B long)
void network_recover(void)
{
    if (s_state == NETWORK_CONNECTED) {
        ESP_LOGI(TAG, "WiFi already connected");
        oled_terminal_print("WiFi: already OK");
        return;
    }
    if (s_state == NETWORK_PROVISIONING) {
        ESP_LOGI(TAG, "Cannot reconnect during provisioning");
        return;
    }
    ESP_LOGI(TAG, "Manual WiFi reconnect");
    action_log_add("WiFi manual reconnect");
    oled_terminal_print("WiFi: reconnecting...");
    stop_reconnect_timer(); // stop periodic timer, we're trying manually
    s_retry_count = 0;
    s_state = NETWORK_CONNECTING;
    esp_wifi_connect();
}

// Stop active BLE provisioning and free resources
void network_stop_provisioning(void)
{
    ESP_LOGI(TAG, "Stopping BLE provisioning");
    network_prov_mgr_stop_provisioning();
    network_prov_mgr_deinit();
    if (s_state == NETWORK_PROVISIONING) {
        s_state = NETWORK_OFFLINE;
    }
}

// Request provisioning on next boot (sets NVS flag) and reboot
void network_reset_provisioning(void)
{
    ESP_LOGW(TAG, "Requesting WiFi provisioning on next boot");
    request_provisioning();
    esp_restart();
}

// Get current network state
network_state_t network_get_state(void)
{
    return s_state;
}

// Get IP address string
const char *network_get_ip_str(void)
{
    return s_ip_str;
}

// Get WiFi RSSI
int network_get_rssi(void)
{
    if (s_state != NETWORK_CONNECTED) return 0;
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return ap_info.rssi;
    }
    return 0;
}

// Check if NTP has synced
bool network_ntp_synced(void)
{
    return s_ntp_synced;
}

// Check if provisioning is active
bool network_is_provisioning(void)
{
    return s_state == NETWORK_PROVISIONING;
}
