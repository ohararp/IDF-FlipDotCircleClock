// AS5600 magnetic encoder wrapper
// Uses dddGR/esp_as5600 library (https://github.com/dddGR/esp_as5600)
// for reliable I2C register reads. This file provides our project's API on top.

#include "as5600.h"
#include "gpio_config.h"
#include "esp_log.h"
#include "esp_as5600.h" // dddGR library (https://github.com/dddGR/esp_as5600)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "as5600";

// I2C device handle shared with the dddGR library
static i2c_master_dev_handle_t s_as5600_dev;

// Track whether AS5600 was successfully detected during init
static bool s_connected = false;

// Attach AS5600 at 0x36 to the shared I2C bus and verify communication
esp_err_t as5600_setup(i2c_master_bus_handle_t bus_handle)
{
    // Register AS5600 as a device on the shared I2C bus at 400 kHz
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_DEFAULT_ADDRESS, // 0x36 from library header
        .scl_speed_hz = 400000,                   // 400 kHz fast-mode I2C
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_config, &s_as5600_dev);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add AS5600 device: %s", esp_err_to_name(ret));
        s_connected = false;
        return ret;
    }

    // Probe: try reading raw angle using library function
    uint16_t angle;
    as5600_get_raw_angle(s_as5600_dev, &angle);
    if (angle == 0) {
        // Try again — first read after power-on can be zero
        vTaskDelay(pdMS_TO_TICKS(10));
        as5600_get_raw_angle(s_as5600_dev, &angle);
    }

    s_connected = true;

    // Read diagnostic registers using library functions
    uint8_t agc;
    uint16_t magnitude;
    bool magnet_detected, too_weak, too_strong;
    as5600_get_agc(s_as5600_dev, &agc);
    as5600_get_magnitude(s_as5600_dev, &magnitude);
    as5600_get_magnet_detect(s_as5600_dev, &magnet_detected);
    as5600_get_magnet_low(s_as5600_dev, &too_weak);
    as5600_get_magnet_high(s_as5600_dev, &too_strong);

    ESP_LOGI(TAG, "AS5600 at 0x%02X: angle=%d (%.1f deg), AGC=%d, mag=%d",
             AS5600_DEFAULT_ADDRESS, angle, as5600_to_degrees(angle), agc, magnitude);
    ESP_LOGI(TAG, "AS5600 status: magnet=%s %s%s",
             magnet_detected ? "YES" : "NO",
             too_weak ? "TOO_WEAK " : "",
             too_strong ? "TOO_STRONG" : "");

    if (!magnet_detected) {
        ESP_LOGW(TAG, "AS5600: no magnet detected — closed-loop will not work!");
    }

    return ESP_OK;
}

// Read 12-bit angle from ANGLE register (0x0E:0x0F) — gives full 0–4095 range
// NOTE: RAW_ANGLE (0x0C:0x0D) only gives partial range due to start/stop position programming
esp_err_t as5600_read_raw_angle(uint16_t *angle)
{
    uint8_t high_byte, low_byte;
    uint8_t reg_h = 0x0E; // ANGLE high byte (not RAW_ANGLE 0x0C which has limited range)
    uint8_t reg_l = 0x0F; // ANGLE low byte

    esp_err_t ret = i2c_master_transmit_receive(s_as5600_dev, &reg_h, 1, &high_byte, 1, 100);
    if (ret != ESP_OK) return ret;

    ret = i2c_master_transmit_receive(s_as5600_dev, &reg_l, 1, &low_byte, 1, 100);
    if (ret != ESP_OK) return ret;

    // Log raw bytes for debugging
    ESP_LOGD(TAG, "0x0C=0x%02X 0x0D=0x%02X", high_byte, low_byte);

    *angle = (((uint16_t)high_byte << 8) | low_byte) & 0x0FFF;
    return ESP_OK;
}

// Temporary: read and log ALL angle-related registers for debugging
void as5600_debug_dump(void)
{
    uint8_t raw_h, raw_l, ang_h, ang_l, status, agc;
    uint8_t r;

    r = 0x0C; i2c_master_transmit_receive(s_as5600_dev, &r, 1, &raw_h, 1, 100);
    r = 0x0D; i2c_master_transmit_receive(s_as5600_dev, &r, 1, &raw_l, 1, 100);
    r = 0x0E; i2c_master_transmit_receive(s_as5600_dev, &r, 1, &ang_h, 1, 100);
    r = 0x0F; i2c_master_transmit_receive(s_as5600_dev, &r, 1, &ang_l, 1, 100);
    r = 0x0B; i2c_master_transmit_receive(s_as5600_dev, &r, 1, &status, 1, 100);
    r = 0x1A; i2c_master_transmit_receive(s_as5600_dev, &r, 1, &agc, 1, 100);

    uint16_t raw = (((uint16_t)raw_h << 8) | raw_l) & 0x0FFF;
    uint16_t ang = (((uint16_t)ang_h << 8) | ang_l) & 0x0FFF;

    ESP_LOGI(TAG, "DUMP: RAW[0x0C]=0x%02X [0x0D]=0x%02X → %d | ANG[0x0E]=0x%02X [0x0F]=0x%02X → %d | ST=0x%02X AGC=%d",
             raw_h, raw_l, raw, ang_h, ang_l, ang, status, agc);
}

// Return whether AS5600 was detected during init
bool as5600_is_connected(void)
{
    return s_connected;
}

// Convert 12-bit raw value to degrees (0.0–360.0)
float as5600_to_degrees(uint16_t raw)
{
    return (float)raw * 360.0f / 4096.0f;
}

// Shortest-path angular difference: positive = need to increase angle, negative = decrease
int as5600_angle_diff(uint16_t current, uint16_t target)
{
    int diff = ((int)target - (int)current) % 4096;
    if (diff < 0) diff += 4096;    // normalize to 0–4095
    if (diff > 2048) diff -= 4096; // wrap to shortest path (-2048 to +2047)
    return diff;
}

// Convert raw AS5600 value (0–4095) to motor steps (0–STEPPER_STEPS_PER_REV-1)
int as5600_to_steps(uint16_t raw)
{
    return (int)((uint32_t)raw * STEPPER_STEPS_PER_REV / 4096) % STEPPER_STEPS_PER_REV;
}

// Convert minute (0–59) to expected AS5600 raw value using home_offset as 12 o'clock reference
uint16_t as5600_minute_to_raw(int minute, uint16_t home_offset)
{
    int minute_angle = minute * 4096 / 60; // fraction of full circle in AS5600 units
    return (home_offset + minute_angle) % 4096;
}
