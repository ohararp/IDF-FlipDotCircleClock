#include "as5600.h"
#include "gpio_config.h"
#include "esp_log.h"

static const char *TAG = "as5600";

// I2C device handle for the AS5600 encoder
static i2c_master_dev_handle_t s_as5600_dev;

// Track whether AS5600 was successfully detected during init
static bool s_connected = false;

// AS5600 register addresses for raw angle (12-bit, read-only)
#define REG_RAW_ANGLE_H 0x0C // bits 11:8 in lower nibble
#define REG_RAW_ANGLE_L 0x0D // bits 7:0

// Attach AS5600 at 0x36 to the shared I2C bus and verify communication
esp_err_t as5600_init(i2c_master_bus_handle_t bus_handle)
{
    // Register AS5600 as a device on the shared I2C bus at 400 kHz
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDR,     // 0x36
        .scl_speed_hz = 400000,                // 400 kHz fast-mode I2C
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_config, &s_as5600_dev);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add AS5600 device: %s", esp_err_to_name(ret));
        s_connected = false;
        return ret;
    }

    // Probe: try reading the raw angle register to confirm AS5600 is present
    uint16_t angle;
    ret = as5600_read_raw_angle(&angle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AS5600 not responding at 0x%02X — closed-loop disabled", AS5600_I2C_ADDR);
        s_connected = false;
        return ret;
    }

    s_connected = true;
    ESP_LOGI(TAG, "AS5600 initialized at 0x%02X (raw angle: %d, %.1f deg)",
             AS5600_I2C_ADDR, angle, as5600_to_degrees(angle));
    return ESP_OK;
}

// Read 12-bit raw angle from registers 0x0C:0x0D
esp_err_t as5600_read_raw_angle(uint16_t *angle)
{
    uint8_t reg = REG_RAW_ANGLE_H;
    uint8_t data[2]; // [0]=high nibble, [1]=low byte

    esp_err_t ret = i2c_master_transmit_receive(s_as5600_dev, &reg, 1, data, 2, 100);
    if (ret != ESP_OK) {
        return ret;
    }

    // Combine high nibble (bits 11:8) and low byte (bits 7:0) into 12-bit value
    *angle = ((uint16_t)(data[0] & 0x0F) << 8) | data[1];
    return ESP_OK;
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

// Shortest-path angular difference: positive = CW needed, negative = CCW needed
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
