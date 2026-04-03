#include <string.h>
#include "ds3231.h"
#include "gpio_config.h"
#include "esp_log.h"

static const char *TAG = "ds3231";

// Device handle for all I2C transactions with the DS3231
static i2c_master_dev_handle_t s_ds3231_dev;

// DS3231 time register addresses (0x00–0x06 = seconds through year)
#define REG_SECONDS  0x00
#define REG_MINUTES  0x01
#define REG_HOURS    0x02
#define REG_DAY      0x03  // day of week (1–7)
#define REG_DATE     0x04  // day of month (1–31)
#define REG_MONTH    0x05  // month (1–12, bit 7 = century)
#define REG_YEAR     0x06  // year (0–99)

// Convert BCD-encoded byte to decimal (e.g., 0x25 → 25)
static uint8_t bcd_to_dec(uint8_t bcd)
{
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

// Convert decimal to BCD-encoded byte (e.g., 25 → 0x25)
static uint8_t dec_to_bcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

// Create shared I2C bus, attach DS3231 at 0x68, verify communication
esp_err_t ds3231_init(i2c_master_bus_handle_t *ret_bus_handle)
{
    // Configure I2C master bus on SDA/SCL pins from gpio_config.h
    i2c_master_bus_config_t bus_config = {
        .i2c_port = -1,                        // auto-select available I2C port
        .sda_io_num = PIN_I2C_SDA,             // GPIO 8
        .scl_io_num = PIN_I2C_SCL,             // GPIO 9
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,                // filter glitches < 7 clock cycles
        .flags.enable_internal_pullup = true,   // use internal pull-ups (external recommended for production)
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register DS3231 as a device on the bus at 400 kHz
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DS3231_I2C_ADDR,     // 0x68
        .scl_speed_hz = 400000,                // 400 kHz fast-mode I2C
    };

    ret = i2c_master_bus_add_device(bus_handle, &dev_config, &s_ds3231_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add DS3231 device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Probe: read seconds register to confirm DS3231 is present and responding
    uint8_t reg = REG_SECONDS;
    uint8_t val;
    ret = i2c_master_transmit_receive(s_ds3231_dev, &reg, 1, &val, 1, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DS3231 not responding at 0x%02X: %s", DS3231_I2C_ADDR, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "DS3231 initialized on I2C (SDA=%d, SCL=%d)", PIN_I2C_SDA, PIN_I2C_SCL);

    // Return bus handle so other I2C devices (OLED, AS5600) can share the bus
    if (ret_bus_handle) {
        *ret_bus_handle = bus_handle;
    }
    return ESP_OK;
}

// Read 7 consecutive BCD registers (0x00–0x06) and convert to struct tm
esp_err_t ds3231_get_time(struct tm *time)
{
    uint8_t reg = REG_SECONDS;
    uint8_t data[7]; // seconds, minutes, hours, day, date, month, year

    // Burst-read all 7 time registers in one I2C transaction
    esp_err_t ret = i2c_master_transmit_receive(s_ds3231_dev, &reg, 1, data, 7, 100);
    if (ret != ESP_OK) {
        return ret;
    }

    time->tm_sec  = bcd_to_dec(data[0] & 0x7F);       // bit 7 = oscillator halt flag
    time->tm_min  = bcd_to_dec(data[1] & 0x7F);       // mask upper unused bit
    time->tm_hour = bcd_to_dec(data[2] & 0x3F);       // bits 5:4 = tens, assumes 24h mode
    time->tm_wday = bcd_to_dec(data[3] & 0x07) - 1;   // DS3231: 1–7 → struct tm: 0–6
    time->tm_mday = bcd_to_dec(data[4] & 0x3F);       // bits 5:4 = tens of date
    time->tm_mon  = bcd_to_dec(data[5] & 0x1F) - 1;   // DS3231: 1–12 → struct tm: 0–11
    time->tm_year = bcd_to_dec(data[6]) + 100;         // DS3231: 0–99 → struct tm: years since 1900

    return ESP_OK;
}

// Write struct tm to DS3231 registers 0x00–0x06 as BCD
esp_err_t ds3231_set_time(const struct tm *time)
{
    uint8_t data[8];
    data[0] = REG_SECONDS;                             // start at register 0x00
    data[1] = dec_to_bcd(time->tm_sec);
    data[2] = dec_to_bcd(time->tm_min);
    data[3] = dec_to_bcd(time->tm_hour);               // 24-hour format
    data[4] = dec_to_bcd(time->tm_wday + 1);           // struct tm: 0–6 → DS3231: 1–7
    data[5] = dec_to_bcd(time->tm_mday);
    data[6] = dec_to_bcd(time->tm_mon + 1);            // struct tm: 0–11 → DS3231: 1–12
    data[7] = dec_to_bcd(time->tm_year - 100);         // struct tm: years since 1900 → DS3231: 0–99

    // Burst-write all 7 time registers in one I2C transaction
    return i2c_master_transmit(s_ds3231_dev, data, 8, 100);
}

// Parse __DATE__ ("Apr  3 2026") and __TIME__ ("14:30:00") into struct tm, write to DS3231
esp_err_t ds3231_set_time_from_compile(void)
{
    // Month abbreviation lookup for __DATE__ parsing
    const char *months[] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };

    struct tm compile_time = {0};

    // Extract HH:MM:SS from __TIME__ (e.g., "14:30:00")
    sscanf(__TIME__, "%d:%d:%d",
           &compile_time.tm_hour, &compile_time.tm_min, &compile_time.tm_sec);

    // Extract month abbreviation, day, year from __DATE__ (e.g., "Apr  3 2026")
    char mon_str[4];
    int day, year;
    sscanf(__DATE__, "%3s %d %d", mon_str, &day, &year);
    compile_time.tm_mday = day;
    compile_time.tm_year = year - 1900;                // struct tm: years since 1900

    // Map 3-char month abbreviation to 0-based month index
    for (int i = 0; i < 12; i++) {
        if (strncmp(mon_str, months[i], 3) == 0) {
            compile_time.tm_mon = i;
            break;
        }
    }

    // Write parsed compile time to DS3231 RTC
    esp_err_t ret = ds3231_set_time(&compile_time);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RTC set to compile time: %04d-%02d-%02d %02d:%02d:%02d",
                 compile_time.tm_year + 1900, compile_time.tm_mon + 1, compile_time.tm_mday,
                 compile_time.tm_hour, compile_time.tm_min, compile_time.tm_sec);
    }
    return ret;
}
