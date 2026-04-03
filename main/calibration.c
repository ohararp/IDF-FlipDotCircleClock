#include "calibration.h"
#include "as5600.h"
#include "stepper.h"
#include "nvm_storage.h"
#include "oled_display.h"
#include "esp_log.h"

static const char *TAG = "calibration";

// Track whether we're currently in calibration mode
static bool s_cal_active = false;

// Disable motor and enter calibration mode so user can manually position the hand
esp_err_t calibration_enter(void)
{
    if (!as5600_is_connected()) {
        ESP_LOGW(TAG, "Cannot calibrate — AS5600 not connected");
        oled_terminal_print("CAL: no AS5600!");
        return ESP_ERR_NOT_FOUND;
    }

    s_cal_active = true;
    stepper_disable(); // release motor so hand can be moved by hand
    oled_terminal_print("CAL: move to 12:00");
    oled_terminal_print("CAL: press C to save");
    ESP_LOGI(TAG, "Calibration mode — move hand to 12 o'clock, press C to save");
    return ESP_OK;
}

// Read current AS5600 angle, save as 12 o'clock reference to NVS, re-enable motor
esp_err_t calibration_save(void)
{
    if (!s_cal_active) {
        return ESP_ERR_INVALID_STATE;
    }

    // Read current AS5600 angle — this is where the user positioned 12 o'clock
    uint16_t angle;
    esp_err_t ret = as5600_read_raw_angle(&angle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read AS5600 during calibration");
        oled_terminal_print("CAL: read failed!");
        s_cal_active = false;
        stepper_enable();
        return ret;
    }

    // Save to NVS
    ret = nvm_set_as5600_cal_angle(angle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save calibration to NVS");
        oled_terminal_print("CAL: save failed!");
        s_cal_active = false;
        stepper_enable();
        return ret;
    }

    ESP_LOGI(TAG, "Calibration saved: AS5600 angle %d (%.1f deg) = 12 o'clock",
             angle, as5600_to_degrees(angle));
    oled_terminal_print("CAL: saved!");

    s_cal_active = false;
    stepper_enable();
    stepper_set_position(0); // we're at 12 o'clock now
    return ESP_OK;
}

// Cancel calibration without saving, re-enable motor
esp_err_t calibration_cancel(void)
{
    if (!s_cal_active) {
        return ESP_ERR_INVALID_STATE;
    }

    s_cal_active = false;
    stepper_enable();
    oled_terminal_print("CAL: cancelled");
    ESP_LOGI(TAG, "Calibration cancelled");
    return ESP_OK;
}

// Load saved AS5600 calibration offset from NVS (0 if uncalibrated)
uint16_t calibration_get_offset(void)
{
    uint16_t angle = 0;
    nvm_get_as5600_cal_angle(&angle);
    return angle;
}

// Check if currently in calibration mode
bool calibration_is_active(void)
{
    return s_cal_active;
}
