#include "nvm_storage.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "nvm";

// NVS namespace for all flip-clock persistent config
#define NVM_NAMESPACE "flipclock"

// NVS key names (max 15 chars per ESP-IDF NVS limit)
#define KEY_MAGIC       "magic"      // magic byte to detect first-boot
#define KEY_TZ_INDEX    "tz_index"   // timezone index (0–17)
#define KEY_HOME_OFFSET "home_off"   // Hall sensor home offset in steps
#define KEY_STEP_DELAY  "step_delay" // stepper pulse delay in µs
#define KEY_AS5600_CAL  "as5600_cal" // AS5600 calibration raw angle

// Magic value written on first init to detect uninitialized flash
#define NVM_MAGIC_VALUE 0xFC

// Persistent NVS handle, opened once during nvm_init()
static nvs_handle_t s_nvs_handle;

// Write default values to all config keys
static esp_err_t nvm_write_defaults(void)
{
    ESP_LOGI(TAG, "Writing factory defaults to NVS");

    // Mark flash as initialized with magic byte
    esp_err_t ret = nvs_set_u8(s_nvs_handle, KEY_MAGIC, NVM_MAGIC_VALUE);
    if (ret != ESP_OK) return ret;

    // Set all config keys to their default values
    ret = nvs_set_u8(s_nvs_handle, KEY_TZ_INDEX, NVM_DEFAULT_TIMEZONE_INDEX);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_i16(s_nvs_handle, KEY_HOME_OFFSET, NVM_DEFAULT_HOME_OFFSET);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_u16(s_nvs_handle, KEY_STEP_DELAY, NVM_DEFAULT_STEP_DELAY_US);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_u16(s_nvs_handle, KEY_AS5600_CAL, NVM_DEFAULT_AS5600_CAL);
    if (ret != ESP_OK) return ret;

    // Flush all pending writes to flash
    return nvs_commit(s_nvs_handle);
}

// Initialize NVS flash partition, open namespace, write defaults if first boot
esp_err_t nvm_init(void)
{
    // Initialize the default NVS partition; erase and retry if corrupted
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition corrupt — erasing and reinitializing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS flash init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Open the flipclock namespace in read-write mode
    ret = nvs_open(NVM_NAMESPACE, NVS_READWRITE, &s_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Check magic byte — if missing or wrong, this is first boot: write defaults
    uint8_t magic = 0;
    ret = nvs_get_u8(s_nvs_handle, KEY_MAGIC, &magic);
    if (ret == ESP_ERR_NVS_NOT_FOUND || magic != NVM_MAGIC_VALUE) {
        ret = nvm_write_defaults();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write defaults: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    ESP_LOGI(TAG, "NVS initialized (namespace: %s)", NVM_NAMESPACE);
    return ESP_OK;
}

// Read timezone index from NVS (0–17)
esp_err_t nvm_get_timezone_index(uint8_t *index)
{
    return nvs_get_u8(s_nvs_handle, KEY_TZ_INDEX, index);
}

// Write timezone index to NVS and commit to flash
esp_err_t nvm_set_timezone_index(uint8_t index)
{
    esp_err_t ret = nvs_set_u8(s_nvs_handle, KEY_TZ_INDEX, index);
    if (ret != ESP_OK) return ret;
    return nvs_commit(s_nvs_handle);
}

// Read home offset from NVS (signed step count)
esp_err_t nvm_get_home_offset(int16_t *offset)
{
    return nvs_get_i16(s_nvs_handle, KEY_HOME_OFFSET, offset);
}

// Write home offset to NVS and commit to flash
esp_err_t nvm_set_home_offset(int16_t offset)
{
    esp_err_t ret = nvs_set_i16(s_nvs_handle, KEY_HOME_OFFSET, offset);
    if (ret != ESP_OK) return ret;
    return nvs_commit(s_nvs_handle);
}

// Read step delay from NVS (microseconds)
esp_err_t nvm_get_step_delay(uint16_t *delay_us)
{
    return nvs_get_u16(s_nvs_handle, KEY_STEP_DELAY, delay_us);
}

// Write step delay to NVS and commit to flash
esp_err_t nvm_set_step_delay(uint16_t delay_us)
{
    esp_err_t ret = nvs_set_u16(s_nvs_handle, KEY_STEP_DELAY, delay_us);
    if (ret != ESP_OK) return ret;
    return nvs_commit(s_nvs_handle);
}

// Read AS5600 calibration angle from NVS (12-bit raw, 0–4095)
esp_err_t nvm_get_as5600_cal_angle(uint16_t *angle)
{
    return nvs_get_u16(s_nvs_handle, KEY_AS5600_CAL, angle);
}

// Write AS5600 calibration angle to NVS and commit to flash
esp_err_t nvm_set_as5600_cal_angle(uint16_t angle)
{
    esp_err_t ret = nvs_set_u16(s_nvs_handle, KEY_AS5600_CAL, angle);
    if (ret != ESP_OK) return ret;
    return nvs_commit(s_nvs_handle);
}

// Erase all keys in flipclock namespace, rewrite factory defaults
esp_err_t nvm_factory_reset(void)
{
    ESP_LOGW(TAG, "Factory reset — erasing all config");
    esp_err_t ret = nvs_erase_all(s_nvs_handle);
    if (ret != ESP_OK) return ret;
    return nvm_write_defaults();
}
