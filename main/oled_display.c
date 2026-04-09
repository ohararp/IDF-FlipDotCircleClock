#include <stdio.h>
#include <string.h>
#include "oled_display.h"
#include "network.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "qrcode.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "u8g2.h"

static const char *TAG = "oled";

// U8G2 display context — holds framebuffer and display state
static u8g2_t s_u8g2;

// Track if OLED was successfully initialized (prevents crash if I2C bus fails)
static bool s_oled_initialized = false;

// Mutex serializing all u8g2 framebuffer access — display task (Core 1),
// network event handler (Core 0), and SNTP callback (LWIP task) all touch
// the shared u8g2 context. Without this, interleaved u8g2 calls corrupt
// internal state and the screen freezes on the last drawn frame.
static SemaphoreHandle_t s_oled_mutex = NULL;
// Wait up to 100 ms for the mutex — long enough to ride out a ~50 ms I2C
// transmit on the other task, short enough to never block clock_task.
#define OLED_MUTEX_WAIT_MS 100

// Global status override — any module can set this to show a message on Line 2
static char s_status_override[32] = {0};

// I2C bus handle passed in from ds3231_init(), shared across all I2C devices
static i2c_master_bus_handle_t s_bus_handle;

// I2C device handle for the OLED, created during U8G2 byte init callback
static i2c_master_dev_handle_t s_oled_dev;

// U8G2 I2C byte-level callback — handles data transfer to the SH1107 over I2C
static uint8_t u8x8_byte_i2c_cb(u8x8_t *u8x8, uint8_t msg,
                                 uint8_t arg_int, void *arg_ptr)
{
    // Static buffer: 1 control byte + up to 128 data bytes + margin
    static uint8_t buffer[132];
    static uint8_t buf_idx;

    switch (msg) {
    case U8X8_MSG_BYTE_INIT:
        // Register OLED as an I2C device on the shared bus at 400 kHz
        {
            i2c_device_config_t dev_config = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = OLED_I2C_ADDR,        // 0x3C
                .scl_speed_hz = 400000,                  // 400 kHz fast-mode I2C
            };
            esp_err_t ret = i2c_master_bus_add_device(s_bus_handle, &dev_config, &s_oled_dev);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add OLED device: %s", esp_err_to_name(ret));
                return 0;
            }
        }
        break;

    case U8X8_MSG_BYTE_START_TRANSFER:
        buf_idx = 0; // reset buffer at start of each I2C transaction
        break;

    case U8X8_MSG_BYTE_SEND:
        // Accumulate data bytes into buffer for batch transmit
        for (size_t i = 0; i < arg_int; i++) {
            buffer[buf_idx++] = *((uint8_t *)arg_ptr + i);
        }
        break;

    case U8X8_MSG_BYTE_END_TRANSFER:
        // Transmit OLED data — short timeout, no retry (fail fast if bus is busy with AS5600)
        if (buf_idx > 0 && s_oled_dev != NULL) {
            esp_err_t ret = i2c_master_transmit(s_oled_dev, buffer, buf_idx, 50);
            if (ret != ESP_OK) {
                // Silently skip — display task will retry on next 1s cycle
                return 0;
            }
        }
        break;

    default:
        return 0;
    }
    return 1;
}

// U8G2 GPIO/delay callback — handles timing delays required by U8G2 init sequence
static uint8_t u8x8_gpio_delay_cb(u8x8_t *u8x8, uint8_t msg,
                                   uint8_t arg_int, void *arg_ptr)
{
    switch (msg) {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        break; // no GPIO init needed — I2C only
    case U8X8_MSG_DELAY_MILLI:
        vTaskDelay(pdMS_TO_TICKS(arg_int)); // millisecond delay via FreeRTOS
        break;
    case U8X8_MSG_DELAY_10MICRO:
        esp_rom_delay_us(arg_int * 10); // 10 µs increments via ROM delay
        break;
    case U8X8_MSG_DELAY_100NANO:
        __asm__ __volatile__("nop"); // minimal delay for 100 ns timing
        break;
    case U8X8_MSG_DELAY_I2C:
        esp_rom_delay_us(5); // I2C bus timing delay
        break;
    default:
        return 0;
    }
    return 1;
}

// Set up U8G2 for SH1107 64x128 (rotated to 128x64 landscape), init display hardware
esp_err_t oled_init(i2c_master_bus_handle_t bus_handle)
{
    s_bus_handle = bus_handle;

    // Create the U8G2 access mutex before any drawing happens
    if (s_oled_mutex == NULL) {
        s_oled_mutex = xSemaphoreCreateMutex();
    }

    // SH1107 native resolution is 64x128; U8G2_R1 rotates to 128x64 landscape
    u8g2_Setup_sh1107_i2c_64x128_f(
        &s_u8g2, U8G2_R1,           // full framebuffer mode, 90° rotation
        u8x8_byte_i2c_cb,           // I2C byte transfer callback
        u8x8_gpio_delay_cb          // GPIO/delay callback
    );

    // Initialize display controller registers and clear screen
    u8g2_InitDisplay(&s_u8g2);
    u8g2_SetPowerSave(&s_u8g2, 0); // wake display from power-save mode

    // Show brief splash screen to confirm OLED is working
    u8g2_ClearBuffer(&s_u8g2);
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr); // 8px Century bold font
    u8g2_DrawStr(&s_u8g2, 20, 35, "FlipDotClock");
    u8g2_SendBuffer(&s_u8g2);

    s_oled_initialized = true;
    ESP_LOGI(TAG, "SH1107 OLED initialized at 0x%02X (128x64, U8G2)", OLED_I2C_ADDR);
    return ESP_OK;
}

// Burn-in mitigation: per-frame pixel-shift offsets applied by all draw helpers.
// Set fresh by oled_update_main() based on the current minute, then read here.
static int s_shift_x = 0;
static int s_shift_y = 0;

// Helper: draw string centered horizontally at given y position (with burn-in shift applied)
static void draw_centered(const char *str, int y)
{
    int w = u8g2_GetStrWidth(&s_u8g2, str);
    u8g2_DrawStr(&s_u8g2, (128 - w) / 2 + s_shift_x, y + s_shift_y, str);
}

// Main display: time, status, network info, IP, date — matches CircuitPython layout
void oled_update_main(const struct tm *time, const char *status_text)
{
    if (!s_oled_initialized) return;
    // Serialize u8g2 access — display task vs network/SNTP terminal_print
    if (s_oled_mutex && xSemaphoreTake(s_oled_mutex, pdMS_TO_TICKS(OLED_MUTEX_WAIT_MS)) != pdTRUE) {
        return; // bus busy with another writer; try again next 1s tick
    }

    // ── Burn-in mitigation #3: hardware invert for 2s at the top of every hour ──
    // SH1107 cmd 0xA6 = normal, 0xA7 = inverse. Toggle the chip directly so we
    // don't have to redraw the framebuffer; tracked with a static edge detector.
    static bool s_inverted = false;
    bool want_inverted = (time->tm_min == 0 && time->tm_sec < 2);
    if (want_inverted != s_inverted) {
        u8x8_cad_StartTransfer(&s_u8g2.u8x8);
        u8x8_cad_SendCmd(&s_u8g2.u8x8, want_inverted ? 0xA7 : 0xA6);
        u8x8_cad_EndTransfer(&s_u8g2.u8x8);
        s_inverted = want_inverted;
    }

    // ── Burn-in mitigation #1: blank for 1s every 10 min (xx:00:30, xx:10:30, …) ──
    // Picked the :30 second so it doesn't collide with hourly NTP re-sync at :00.
    if (time->tm_min % 10 == 0 && time->tm_sec == 30) {
        u8g2_ClearBuffer(&s_u8g2);
        u8g2_SendBuffer(&s_u8g2);
        if (s_oled_mutex) xSemaphoreGive(s_oled_mutex);
        return;
    }

    // ── Burn-in mitigation #2: pixel shift — drift entire layout 0..2 px in x and y ──
    // 9-position cycle (3x3), advances every 5 minutes → full cycle every 45 minutes.
    int phase = (time->tm_hour * 60 + time->tm_min) / 5;
    s_shift_x = phase % 3;            // 0..2
    s_shift_y = (phase / 3) % 3;      // 0..2

    char time_str[9];   // "HH:MM:SS\0"
    char date_str[36];  // "YYYY-MM-DD\0"
    char wifi_col[12];  // "WiFi:OK" etc
    char ntp_col[12];   // "NTP:Sync" etc

    // Format time and date
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
             time->tm_hour, time->tm_min, time->tm_sec);
    snprintf(date_str, sizeof(date_str), "%04d-%02d-%02d",
             time->tm_year + 1900, time->tm_mon + 1, time->tm_mday);

    // Build WiFi status column
    network_state_t net = network_get_state();
    switch (net) {
    case NETWORK_CONNECTED:    snprintf(wifi_col, sizeof(wifi_col), "WiFi:OK");  break;
    case NETWORK_CONNECTING:   snprintf(wifi_col, sizeof(wifi_col), "WiFi:..."); break;
    case NETWORK_PROVISIONING: snprintf(wifi_col, sizeof(wifi_col), "WiFi:Off"); break;
    default:                   snprintf(wifi_col, sizeof(wifi_col), "WiFi:Off"); break;
    }

    // Build NTP/BLE status column
    if (net == NETWORK_PROVISIONING) {
        snprintf(ntp_col, sizeof(ntp_col), "BLE:Prov");
    } else if (network_ntp_synced()) {
        snprintf(ntp_col, sizeof(ntp_col), "NTP:Sync");
    } else {
        snprintf(ntp_col, sizeof(ntp_col), "NTP:Pend");
    }

    // Get IP address string
    const char *ip = network_get_ip_str();

    u8g2_ClearBuffer(&s_u8g2);

    // Rounded border — shrunk to 126x62 so 0-2px shift never clips off-screen
    u8g2_DrawRFrame(&s_u8g2, s_shift_x, s_shift_y, 126, 62, 5);

    // WiFi dot — top right corner (filled=connected, hollow=offline)
    if (net == NETWORK_CONNECTED) {
        u8g2_DrawDisc(&s_u8g2, 118 + s_shift_x, 8 + s_shift_y, 4, U8G2_DRAW_ALL); // filled
    } else {
        u8g2_DrawCircle(&s_u8g2, 118 + s_shift_x, 8 + s_shift_y, 4, U8G2_DRAW_ALL); // hollow
    }

    // Line 1: Time (6x12 monospace, centered) — y=18 (baseline)
    u8g2_SetFont(&s_u8g2, u8g2_font_6x12_tr);
    draw_centered(time_str, 18);

    // Line 2: Status override > status_text > firmware version (6x10, centered) — y=29
    u8g2_SetFont(&s_u8g2, u8g2_font_6x10_tr);
    if (s_status_override[0] != '\0') {
        // Global status override (e.g. OTA progress)
        draw_centered(s_status_override, 29);
    } else if (status_text && status_text[0] != '\0') {
        draw_centered(status_text, 29);
    } else {
        // Show firmware version when no status message active
        char ver_str[36];
        const esp_app_desc_t *app = esp_app_get_description();
        snprintf(ver_str, sizeof(ver_str), "v%s", app->version);
        draw_centered(ver_str, 29);
    }

    // Line 3: Two-column network status (6x10) — y=40, with burn-in shift applied
    u8g2_DrawStr(&s_u8g2, 6 + s_shift_x, 40 + s_shift_y, wifi_col);   // left column
    u8g2_DrawStr(&s_u8g2, 72 + s_shift_x, 40 + s_shift_y, ntp_col);   // right column

    // Line 4: IP address (6x10, centered) — y=51
    draw_centered(ip, 51);

    // Line 5: Date (6x10, centered) — y=61
    draw_centered(date_str, 61);

    u8g2_SendBuffer(&s_u8g2);
    if (s_oled_mutex) xSemaphoreGive(s_oled_mutex);
}

// Clear the OLED framebuffer and push blank screen to display
// Set a global status message on OLED Line 2 (pass NULL or "" to clear)
void oled_set_status(const char *msg)
{
    if (msg) {
        strncpy(s_status_override, msg, sizeof(s_status_override) - 1);
        s_status_override[sizeof(s_status_override) - 1] = '\0';
    } else {
        s_status_override[0] = '\0';
    }
}

void oled_clear(void)
{
    if (!s_oled_initialized) return;
    if (s_oled_mutex && xSemaphoreTake(s_oled_mutex, pdMS_TO_TICKS(OLED_MUTEX_WAIT_MS)) != pdTRUE) return;
    u8g2_ClearBuffer(&s_u8g2);
    u8g2_SendBuffer(&s_u8g2);
    if (s_oled_mutex) xSemaphoreGive(s_oled_mutex);
}

// ── Scrolling terminal mode ──────────────────────────────────────────────────

#define TERM_MAX_LINES  8   // 64px tall / 8px per line = 8 visible lines
#define TERM_LINE_LEN   26  // 128px wide / 5px per char ≈ 25 chars + null

// Ring buffer of terminal lines — newest at index (s_term_head - 1)
static char s_term_lines[TERM_MAX_LINES][TERM_LINE_LEN];
static int s_term_count = 0; // total lines added (for indexing into ring buffer)

// Append a line to the scrolling terminal and immediately render to OLED
void oled_terminal_print(const char *line)
{
    if (!s_oled_initialized) return; // skip if OLED not available
    if (s_oled_mutex && xSemaphoreTake(s_oled_mutex, pdMS_TO_TICKS(OLED_MUTEX_WAIT_MS)) != pdTRUE) {
        return; // bus busy — drop this terminal line rather than corrupt u8g2 state
    }
    // Write into ring buffer at next slot
    int idx = s_term_count % TERM_MAX_LINES;
    strncpy(s_term_lines[idx], line, TERM_LINE_LEN - 1);
    s_term_lines[idx][TERM_LINE_LEN - 1] = '\0'; // ensure null termination
    s_term_count++;

    // Render all visible lines to OLED
    u8g2_ClearBuffer(&s_u8g2);
    u8g2_SetFont(&s_u8g2, u8g2_font_5x7_tr); // 5x7 monospace font

    int visible = (s_term_count < TERM_MAX_LINES) ? s_term_count : TERM_MAX_LINES;
    int start = (s_term_count < TERM_MAX_LINES) ? 0 : s_term_count - TERM_MAX_LINES;

    for (int i = 0; i < visible; i++) {
        int buf_idx = (start + i) % TERM_MAX_LINES;
        u8g2_DrawStr(&s_u8g2, 0, 7 + (i * 8), s_term_lines[buf_idx]); // 8px line height
    }
    u8g2_SendBuffer(&s_u8g2); // push to display
    if (s_oled_mutex) xSemaphoreGive(s_oled_mutex);
}

// ── QR Code display ──────────────────────────────────────────────────────────

// Pointer to U8G2 context passed to QR callback (espressif/qrcode uses callback pattern)
static u8g2_t *s_qr_u8g2_ptr = NULL;

// Callback: render QR code pixels to OLED framebuffer via U8G2
static void qr_display_callback(esp_qrcode_handle_t qrcode)
{
    int size = esp_qrcode_get_size(qrcode); // QR module count (e.g., 21 for version 1)

    // Calculate scale factor to fit QR in 64x64 area (OLED is 128x64)
    int scale = 64 / size;
    if (scale < 1) scale = 1;
    int qr_px = size * scale;             // actual rendered size in pixels

    // Center QR on the left 64px of display, leave right side for text
    int x_offset = (64 - qr_px) / 2;
    int y_offset = (64 - qr_px) / 2;

    u8g2_ClearBuffer(s_qr_u8g2_ptr);

    // Draw QR modules as filled squares
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            if (esp_qrcode_get_module(qrcode, x, y)) {
                // Draw a scale×scale filled box for each black module
                u8g2_DrawBox(s_qr_u8g2_ptr, x_offset + x * scale, y_offset + y * scale, scale, scale);
            }
        }
    }

    // Add label text on the right side
    u8g2_SetFont(s_qr_u8g2_ptr, u8g2_font_5x7_tr);
    u8g2_DrawStr(s_qr_u8g2_ptr, 68, 12, "Scan with");
    u8g2_DrawStr(s_qr_u8g2_ptr, 68, 22, "ESP BLE");
    u8g2_DrawStr(s_qr_u8g2_ptr, 68, 32, "Prov app");
    u8g2_DrawStr(s_qr_u8g2_ptr, 68, 44, "POP:flipdot");
    u8g2_DrawStr(s_qr_u8g2_ptr, 68, 58, "C to cancel");

    u8g2_SendBuffer(s_qr_u8g2_ptr);
}

// Display a QR code on the OLED with label text
void oled_show_qr(const char *text)
{
    if (!s_oled_initialized) return;
    if (s_oled_mutex && xSemaphoreTake(s_oled_mutex, pdMS_TO_TICKS(OLED_MUTEX_WAIT_MS)) != pdTRUE) return;
    s_qr_u8g2_ptr = &s_u8g2; // pass U8G2 context to callback

    esp_qrcode_config_t cfg = {
        .display_func = qr_display_callback,
        .max_qrcode_version = 10,
        .qrcode_ecc_level = ESP_QRCODE_ECC_LOW, // low ECC = fewer modules, larger pixels
    };

    esp_err_t ret = esp_qrcode_generate(&cfg, text);
    if (s_oled_mutex) xSemaphoreGive(s_oled_mutex);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "QR generation failed: %s", esp_err_to_name(ret));
        oled_terminal_print("QR: gen failed"); // re-acquires the mutex internally
    }
}
