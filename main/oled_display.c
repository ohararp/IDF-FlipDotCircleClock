#include <stdio.h>
#include <string.h>
#include "oled_display.h"
#include "esp_log.h"
#include "qrcode.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "u8g2.h"

static const char *TAG = "oled";

// U8G2 display context — holds framebuffer and display state
static u8g2_t s_u8g2;

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
        // Transmit accumulated buffer to OLED — retry on timeout (BLE stack can cause contention)
        if (buf_idx > 0 && s_oled_dev != NULL) {
            esp_err_t ret;
            for (int attempt = 0; attempt < 3; attempt++) {
                ret = i2c_master_transmit(s_oled_dev, buffer, buf_idx, 500);
                if (ret == ESP_OK) break;
                vTaskDelay(pdMS_TO_TICKS(10)); // brief pause before retry
            }
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "I2C transmit failed: %s", esp_err_to_name(ret));
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

    ESP_LOGI(TAG, "SH1107 OLED initialized at 0x%02X (128x64, U8G2)", OLED_I2C_ADDR);
    return ESP_OK;
}

// Render HH:MM:SS in large centered font, with date in smaller font below
void oled_update_time(const struct tm *time)
{
    char time_str[9];  // "HH:MM:SS\0"
    char date_str[36]; // "YYYY-MM-DD\0" — oversized to satisfy -Wformat-truncation

    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
             time->tm_hour, time->tm_min, time->tm_sec);
    snprintf(date_str, sizeof(date_str), "%04d-%02d-%02d",
             time->tm_year + 1900, time->tm_mon + 1, time->tm_mday);

    u8g2_ClearBuffer(&s_u8g2);
    // Large time display centered horizontally
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB14_tr); // 14px Century bold font
    u8g2_DrawStr(&s_u8g2, 16, 30, time_str);
    // Smaller date below the time
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr); // 8px Century bold font
    u8g2_DrawStr(&s_u8g2, 24, 50, date_str);
    u8g2_SendBuffer(&s_u8g2); // push framebuffer to display over I2C
}

// Render status fields: IP, WiFi signal, uptime, and motor step position
void oled_update_status(const char *ip, int rssi, uint32_t uptime, int motor_pos)
{
    char line1[32]; // IP address line
    char line2[32]; // RSSI + uptime line
    char line3[32]; // motor position line

    snprintf(line1, sizeof(line1), "IP: %s", ip ? ip : "N/A");
    snprintf(line2, sizeof(line2), "RSSI:%d Up:%lus", rssi, (unsigned long)uptime);
    snprintf(line3, sizeof(line3), "Motor: %d", motor_pos);

    u8g2_ClearBuffer(&s_u8g2);
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr); // 8px font for status lines
    u8g2_DrawStr(&s_u8g2, 0, 15, line1);
    u8g2_DrawStr(&s_u8g2, 0, 35, line2);
    u8g2_DrawStr(&s_u8g2, 0, 55, line3);
    u8g2_SendBuffer(&s_u8g2); // push framebuffer to display over I2C
}

// Clear the OLED framebuffer and push blank screen to display
void oled_clear(void)
{
    u8g2_ClearBuffer(&s_u8g2);
    u8g2_SendBuffer(&s_u8g2);
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
    s_qr_u8g2_ptr = &s_u8g2; // pass U8G2 context to callback

    esp_qrcode_config_t cfg = {
        .display_func = qr_display_callback,
        .max_qrcode_version = 10,
        .qrcode_ecc_level = ESP_QRCODE_ECC_LOW, // low ECC = fewer modules, larger pixels
    };

    esp_err_t ret = esp_qrcode_generate(&cfg, text);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "QR generation failed: %s", esp_err_to_name(ret));
        oled_terminal_print("QR: gen failed");
    }
}
