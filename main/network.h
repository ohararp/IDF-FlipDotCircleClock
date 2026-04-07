#pragma once

// Network module: WiFi + BLE provisioning + NTP time sync
// Uses espressif/network_provisioning component for BLE-based credential setup
// and esp_sntp for NTP time synchronization.

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Network connection states
typedef enum {
    NETWORK_PROVISIONING, // BLE/SoftAP provisioning active, waiting for credentials
    NETWORK_CONNECTING,   // WiFi connection in progress
    NETWORK_CONNECTED,    // WiFi connected, NTP may or may not be synced
    NETWORK_DISCONNECTED, // WiFi lost, recovery pending
    NETWORK_OFFLINE,      // WiFi failed after retries, not attempting
} network_state_t;

// Initialize networking: check for stored credentials, start provisioning or connect
esp_err_t network_init(void);

// Manual reconnect attempt (triggered by Button A long)
void network_recover(void);

// Start BLE provisioning on-demand (called from Button A long). Shows QR on OLED.
// Blocks until credentials received, cancelled, or timeout. Reboots on success.
void network_start_provisioning(void);

// Stop active BLE provisioning and free resources (called on cancel/timeout)
void network_stop_provisioning(void);

// Erase stored WiFi credentials and restart BLE provisioning
void network_reset_provisioning(void);

// Get current network state
network_state_t network_get_state(void);

// Get IP address as string (e.g., "192.168.1.42"), or "N/A" if not connected
const char *network_get_ip_str(void);

// Get WiFi RSSI (signal strength in dBm), or 0 if not connected
int network_get_rssi(void);

// Check if NTP has synced at least once since boot
bool network_ntp_synced(void);

// Request NTP re-sync (called from main loop at each hour boundary)
void network_request_ntp_resync(void);

// Check if provisioning is currently active (blocks clock startup until complete)
bool network_is_provisioning(void);
