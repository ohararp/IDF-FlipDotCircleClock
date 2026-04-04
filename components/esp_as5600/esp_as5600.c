#include "esp_as5600.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: export
#include "freertos/projdefs.h"
#include "sdkconfig.h"          // IWYU pragma: export

static const char* TAG = "[AS5600]";

esp_err_t as5600_init(i2c_master_bus_handle_t* p_bus_handle,
                      i2c_master_bus_config_t* p_bus_config,
                      i2c_master_dev_handle_t* p_dev_handle,
                      i2c_device_config_t* p_dev_config) {
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(p_bus_config, p_bus_handle),
                        TAG,
                        "failed to configure I2C bus");
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(*p_bus_handle, p_dev_config, p_dev_handle),
        TAG,
        "failed to add I2C device");

    return ESP_OK;
}

esp_err_t as5600_read_byte(i2c_master_dev_handle_t handle,
                           uint8_t reg_addr,
                           uint8_t* data) {
    return i2c_master_transmit_receive(handle,
                                       &reg_addr,
                                       1,
                                       data,
                                       1,
                                       pdMS_TO_TICKS(AS5600_MASTER_TIMEOUT_MS));
}

esp_err_t as5600_write_byte(i2c_master_dev_handle_t handle,
                            uint8_t reg_addr,
                            uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(handle,
                               write_buf,
                               sizeof(write_buf),
                               pdMS_TO_TICKS(AS5600_MASTER_TIMEOUT_MS));
}

void as5600_get_raw_angle(i2c_master_dev_handle_t handle, uint16_t* data) {
    uint8_t buf[2];
    as5600_read_byte(handle, AS5600_RAW_ANGLE_L, &buf[0]);
    as5600_read_byte(handle, AS5600_RAW_ANGLE_H, &buf[1]);

    *data = (buf[0] + (buf[1] << 8));
}

void as5600_get_angle(i2c_master_dev_handle_t handle, uint16_t* data) {
    uint8_t buf[2];
    as5600_read_byte(handle, AS5600_ANGLE_L, &buf[0]);
    as5600_read_byte(handle, AS5600_ANGLE_H, &buf[1]);

    *data = (buf[0] + (buf[1] << 8));
}

void as5600_get_magnet_high(i2c_master_dev_handle_t handle, bool* data) {
    uint8_t buf;
    as5600_read_byte(handle, AS5600_STATUS, &buf);
    *data = (buf & AS5600_STATUS_MH) > 1;
}

void as5600_get_magnet_low(i2c_master_dev_handle_t handle, bool* data) {
    uint8_t buf;
    as5600_read_byte(handle, AS5600_STATUS, &buf);
    *data = (buf & AS5600_STATUS_ML) > 1;
}

void as5600_get_magnet_detect(i2c_master_dev_handle_t handle, bool* data) {
    uint8_t buf;
    as5600_read_byte(handle, AS5600_STATUS, &buf);
    *data = (buf & AS5600_STATUS_MD) > 1;
}

void as5600_get_agc(i2c_master_dev_handle_t handle, uint8_t* data) {
    as5600_read_byte(handle, AS5600_AGC, data);
}

void as5600_get_magnitude(i2c_master_dev_handle_t handle, uint16_t* data) {
    uint8_t buf[2];
    as5600_read_byte(handle, AS5600_MAGNITUDE_L, &buf[0]);
    as5600_read_byte(handle, AS5600_MAGNITUDE_H, &buf[1]);

    *data = (buf[0] + (buf[1] << 8));
}

void as5600_get_power_mode(i2c_master_dev_handle_t handle,
                           as5600_power_mode_type_e* power_mode) {
    uint8_t buf;
    as5600_read_byte(handle, AS5600_CONF_L, &buf);
    *power_mode = (as5600_power_mode_type_e)((buf & AS5600_CONF_POWER_MODE)
                                             >> TRAILING_ZERO(
                                                 AS5600_CONF_POWER_MODE));
    assert(*power_mode < POWER_MODE_MAX_NUM);
}

void as5600_get_hysteresis(i2c_master_dev_handle_t handle,
                           as5600_hysteresis_type_e* hysteresis) {
    uint8_t buf;
    as5600_read_byte(handle, AS5600_CONF_L, &buf);
    *hysteresis = (as5600_hysteresis_type_e)((buf & AS5600_CONF_HYSTERESIS)
                                             >> TRAILING_ZERO(
                                                 AS5600_CONF_HYSTERESIS));
    assert(*hysteresis < HYSTERESIS_MAX_NUM);
}

void as5600_get_output_stage(i2c_master_dev_handle_t handle,
                             as5600_output_stage_type_e* output_stage) {
    uint8_t buf;
    as5600_read_byte(handle, AS5600_CONF_L, &buf);

    *output_stage = (as5600_output_stage_type_e)((buf & AS5600_CONF_OUT_STAGE)
                                                 >> TRAILING_ZERO(
                                                     AS5600_CONF_OUT_STAGE));
    assert(*output_stage < OUTPUT_STAGE_MAX_NUM);
}

void as5600_get_pwm_freq(i2c_master_dev_handle_t handle,
                         as5600_pwm_freq_type_e* pwm_freq) {
    uint8_t buf;
    as5600_read_byte(handle, AS5600_CONF_L, &buf);
    *pwm_freq = (as5600_pwm_freq_type_e)((buf & AS5600_CONF_PWM_FREQ)
                                         >> TRAILING_ZERO(
                                             AS5600_CONF_PWM_FREQ));
    assert(*pwm_freq < PWM_FREQ_MAX_NUM);
}

void as5600_get_slow_filter(i2c_master_dev_handle_t handle,
                            as5600_slow_filter_type_e* slow_filter) {
    uint8_t buf;
    as5600_read_byte(handle, AS5600_CONF_H, &buf);
    *slow_filter = (as5600_slow_filter_type_e)((buf & AS5600_CONF_SLOW_FILTER)
                                               >> TRAILING_ZERO(
                                                   AS5600_CONF_SLOW_FILTER));
    assert(*slow_filter < SLOW_FILTER_MAX_NUM);
}

void as5600_get_fast_filter(i2c_master_dev_handle_t handle,
                            as5600_fast_filter_type_e* fast_filter) {
    uint8_t buf;
    as5600_read_byte(handle, AS5600_CONF_H, &buf);
    *fast_filter = (as5600_fast_filter_type_e)((buf & AS5600_CONF_FAST_FILTER)
                                               >> TRAILING_ZERO(
                                                   AS5600_CONF_FAST_FILTER));
    assert(*fast_filter < FAST_FILTER_MAX_NUM);
}

void as5600_get_watchdog(i2c_master_dev_handle_t handle,
                         as5600_watchdog_type_e* watchdog) {
    uint8_t buf;
    as5600_read_byte(handle, AS5600_CONF_H, &buf);
    *watchdog = (as5600_power_mode_type_e)((buf & AS5600_CONF_WATCHDOG)
                                           >> TRAILING_ZERO(
                                               AS5600_CONF_WATCHDOG));
    assert(*watchdog < WATCHDOG_MAX_NUM);
}

void as5600_get_mang(i2c_master_dev_handle_t handle, uint16_t* max_angle) {
    uint8_t buf[2];
    as5600_read_byte(handle, AS5600_MANG_L, &buf[0]);
    as5600_read_byte(handle, AS5600_MANG_H, &buf[1]);

    *max_angle = (buf[0] + (buf[1] << 8));
}

void as5600_get_mpos(i2c_master_dev_handle_t handle, uint16_t* stop_position) {
    uint8_t buf[2];
    as5600_read_byte(handle, AS5600_MPOS_L, &buf[0]);
    as5600_read_byte(handle, AS5600_MPOS_H, &buf[1]);

    *stop_position = (buf[0] + (buf[1] << 8));
}

void as5600_get_zpos(i2c_master_dev_handle_t handle, uint16_t* start_position) {
    uint8_t buf[2];
    as5600_read_byte(handle, AS5600_ZPOS_L, &buf[0]);
    as5600_read_byte(handle, AS5600_ZPOS_H, &buf[1]);

    *start_position = (buf[0] + (buf[1] << 8));
}

void as5600_get_zmco(i2c_master_dev_handle_t handle, uint8_t* zmco) {
    as5600_read_byte(handle, AS5600_ZMCO, zmco);
}

esp_err_t as5600_set_power_mode(i2c_master_dev_handle_t handle,
                                as5600_power_mode_type_e power_mode) {
    if ( power_mode >= POWER_MODE_MAX_NUM ) {
        return false;
    }

    uint8_t buf;
    uint8_t write_buf = ((uint8_t)power_mode)
        << TRAILING_ZERO(AS5600_CONF_POWER_MODE);

    as5600_read_byte(handle, AS5600_CONF_L, &buf);

    buf ^= (buf ^ write_buf) & AS5600_CONF_POWER_MODE;

    return as5600_write_byte(handle, AS5600_CONF_L, buf) == ESP_OK;
}

esp_err_t as5600_set_hysteresis(i2c_master_dev_handle_t handle,
                                as5600_hysteresis_type_e hysteresis) {
    if ( hysteresis >= HYSTERESIS_MAX_NUM ) {
        return false;
    }

    uint8_t buf;
    uint8_t write_buf = ((uint8_t)hysteresis)
        << TRAILING_ZERO(AS5600_CONF_HYSTERESIS);

    as5600_read_byte(handle, AS5600_CONF_L, &buf);

    buf ^= (buf ^ write_buf) & AS5600_CONF_HYSTERESIS;

    return as5600_write_byte(handle, AS5600_CONF_L, buf) == ESP_OK;
}

esp_err_t as5600_set_output_stage(i2c_master_dev_handle_t handle,
                                  as5600_output_stage_type_e output_stage) {
    if ( output_stage >= OUTPUT_STAGE_MAX_NUM ) {
        return false;
    }

    uint8_t buf;
    uint8_t write_buf = ((uint8_t)output_stage)
        << TRAILING_ZERO(AS5600_CONF_OUT_STAGE);

    as5600_read_byte(handle, AS5600_CONF_L, &buf);

    buf ^= (buf ^ write_buf) & AS5600_CONF_OUT_STAGE;

    return as5600_write_byte(handle, AS5600_CONF_L, buf) == ESP_OK;
}

esp_err_t as5600_set_pwm_freq(i2c_master_dev_handle_t handle,
                              as5600_pwm_freq_type_e pwm_freq) {
    if ( pwm_freq >= PWM_FREQ_MAX_NUM ) {
        return false;
    }

    uint8_t buf;
    uint8_t write_buf = ((uint8_t)pwm_freq)
        << TRAILING_ZERO(AS5600_CONF_PWM_FREQ);

    as5600_read_byte(handle, AS5600_CONF_L, &buf);

    buf ^= (buf ^ write_buf) & AS5600_CONF_PWM_FREQ;

    return as5600_write_byte(handle, AS5600_CONF_L, buf) == ESP_OK;
}

esp_err_t as5600_set_slow_filter(i2c_master_dev_handle_t handle,
                                 as5600_slow_filter_type_e slow_filter) {
    if ( slow_filter >= SLOW_FILTER_MAX_NUM ) {
        return false;
    }

    uint8_t buf;
    uint8_t write_buf = ((uint8_t)slow_filter)
        << TRAILING_ZERO(AS5600_CONF_SLOW_FILTER);

    as5600_read_byte(handle, AS5600_CONF_H, &buf);

    buf ^= (buf ^ write_buf) & AS5600_CONF_SLOW_FILTER;

    return as5600_write_byte(handle, AS5600_CONF_H, buf) == ESP_OK;
}

esp_err_t as5600_set_fast_filter(i2c_master_dev_handle_t handle,
                                 as5600_fast_filter_type_e fast_filter) {
    if ( fast_filter >= FAST_FILTER_MAX_NUM ) {
        return false;
    }

    uint8_t buf;
    uint8_t write_buf = ((uint8_t)fast_filter)
        << TRAILING_ZERO(AS5600_CONF_FAST_FILTER);

    as5600_read_byte(handle, AS5600_CONF_H, &buf);

    buf ^= (buf ^ write_buf) & AS5600_CONF_FAST_FILTER;
    as5600_write_byte(handle, AS5600_CONF_H, buf);

    return as5600_write_byte(handle, AS5600_CONF_H, buf) == ESP_OK;
}

esp_err_t as5600_set_watchdog(i2c_master_dev_handle_t handle,
                              as5600_watchdog_type_e watchdog) {
    if ( watchdog >= WATCHDOG_MAX_NUM ) {
        return false;
    }

    uint8_t buf;
    uint8_t write_buf = ((uint8_t)watchdog)
        << TRAILING_ZERO(AS5600_CONF_WATCHDOG);

    as5600_read_byte(handle, AS5600_CONF_H, &buf);

    buf ^= (buf ^ write_buf) & AS5600_CONF_WATCHDOG;
    as5600_write_byte(handle, AS5600_CONF_H, buf);

    return as5600_write_byte(handle, AS5600_CONF_H, buf) == ESP_OK;
}

esp_err_t as5600_set_mang(i2c_master_dev_handle_t handle,
                          uint16_t max_angle,
                          as5600_burn_settings_context_t* context) {
    esp_err_t err;
    err = as5600_write_byte(handle, AS5600_MANG_L, max_angle & 0xFF);

    if ( err != ESP_OK ) {
        ESP_LOGE(TAG, "as5600_set_mang: failed to write mang.");
        return err;
    }

    err = as5600_write_byte(handle, AS5600_MANG_H, max_angle >> 8);

    if ( err != ESP_OK ) {
        ESP_LOGE(TAG, "as5600_set_mang: failed to write mang.");
        return err;
    }

    context->max_angle = max_angle;

    return true;
}

esp_err_t as5600_set_mpos(i2c_master_dev_handle_t handle,
                          uint16_t stop_position,
                          as5600_burn_angle_context_t* context) {
    esp_err_t err;
    err = as5600_write_byte(handle, AS5600_MPOS_L, stop_position & 0xFF);

    if ( err != ESP_OK ) {
        ESP_LOGE(TAG, "as5600_set_mpos: failed to write mang.");
        return false;
    }

    err = as5600_write_byte(handle, AS5600_MPOS_H, stop_position >> 8);

    if ( err != ESP_OK ) {
        ESP_LOGE(TAG, "as5600_set_mpos: failed to write mang.");
        return false;
    }

    if ( context ) {
        context->stop_position = stop_position;
    }

    return true;
}

esp_err_t as5600_set_zpos(i2c_master_dev_handle_t handle,
                          uint16_t start_position,
                          as5600_burn_angle_context_t* context) {
    esp_err_t err;
    err = as5600_write_byte(handle, AS5600_ZPOS_L, start_position & 0xFF);

    if ( err != ESP_OK ) {
        ESP_LOGE(TAG, "as5600_set_zpos: failed to write mang.");
        return false;
    }

    err = as5600_write_byte(handle, AS5600_ZPOS_H, start_position >> 8);

    if ( err != ESP_OK ) {
        ESP_LOGE(TAG, "as5600_set_zpos: failed to write mang.");
        return false;
    }

    if ( context ) {
        context->start_position = start_position;
    }

    return ESP_OK;
}

esp_err_t as5600_burn_angle(i2c_master_dev_handle_t handle,
                            as5600_burn_angle_context_t* context) {
#ifndef CONFIG_AS5600_ENABLE_BURN_ANGLE
    ESP_LOGE(TAG,
             "as5600_burn_angle: function is disabled. Please configure "
             "AS5600_ENABLE_BURN_ANGLE to enable it. Use at your own risk!");
    return ESP_FAIL;
#endif

    uint8_t zmco;

    as5600_get_zmco(handle, &zmco);

    float angular_angle = AS5600_TO_ANGULAR(context->stop_position
                                            - context->start_position);
    if ( (uint16_t)angular_angle < AS5600_MIN_ANGULAR ) {
        ESP_LOGE(
            TAG,
            "as5600_burn_angle: failed to burn angle - angle too small (given "
            "- %f, min - %f)",
            angular_angle,
            (float)AS5600_MIN_ANGULAR);
        return ESP_FAIL;
    }

    if ( zmco >= AS5600_BURN_ANGLE_MAX_COUNT ) {
        ESP_LOGE(TAG,
                 "as5600_burn_angle: failed to burn angle - angle burn count "
                 "exceeded (maximum - %d)",
                 AS5600_BURN_ANGLE_MAX_COUNT);
        return ESP_FAIL;
    }

    return as5600_write_byte(handle, AS5600_BURN, AS5600_BURN_ANGLE);
}

esp_err_t as5600_burn_settings(i2c_master_dev_handle_t handle,
                               as5600_burn_settings_context_t* context) {
#ifndef CONFIG_AS5600_ENABLE_BURN_SETTINGS
    ESP_LOGE(TAG,
             "as5600_burn_settings: function is disabled. Please configure "
             "AS5600_ENABLE_BURN_SETTINGS to enable it. Use at your own risk!");
    return ESP_FAIL;
#endif

    uint8_t zmco;

    as5600_get_zmco(handle, &zmco);

    float angular_angle = AS5600_TO_ANGULAR(context->max_angle);
    if ( (uint16_t)angular_angle < AS5600_MIN_ANGULAR ) {
        ESP_LOGE(
            TAG,
            "as5600_burn_settings: failed to burn settings - angle too small "
            "(given - %f, min - %f)",
            angular_angle,
            (float)AS5600_MIN_ANGULAR);
        return ESP_FAIL;
    }

    if ( zmco ) {
        ESP_LOGE(TAG,
                 "as5600_burn_settings: failed to burn settings - settings of "
                 "angle already burnt");
        return ESP_FAIL;
    }

    return as5600_write_byte(handle, AS5600_BURN, AS5600_BURN_SETTINGS);
}

bool as5600_burn_verify(i2c_master_dev_handle_t handle,
                        uint8_t command,
                        void* context) {
    if ( as5600_write_byte(handle, AS5600_BURN, AS5600_OTP_01) != ESP_OK
         || as5600_write_byte(handle, AS5600_BURN, AS5600_OTP_02) != ESP_OK
         || as5600_write_byte(handle, AS5600_BURN, AS5600_OTP_03) != ESP_OK ) {
        ESP_LOGE(TAG, "as5600_burn_verify: failed to load OTP content");
        return false;
    }

    switch ( command ) {
        case AS5600_BURN_ANGLE: {
            as5600_burn_angle_context_t* burn_angle_context =
                (as5600_burn_angle_context_t*)context;
            uint16_t start_position;
            uint16_t stop_position;

            as5600_get_zpos(handle, &start_position);
            as5600_get_mpos(handle, &stop_position);

            if ( burn_angle_context->start_position == start_position
                 && burn_angle_context->stop_position == stop_position ) {
                return true;
            }

            return false;
        }
        case AS5600_BURN_SETTINGS: {
            as5600_burn_settings_context_t* burn_settings_context =
                (as5600_burn_settings_context_t*)context;
            uint16_t max_angle;

            as5600_get_mang(handle, &max_angle);

            if ( burn_settings_context->max_angle == max_angle ) {
                return true;
            }

            return false;
        }
        default: {
            ESP_LOGE(
                TAG,
                "as5600_burn_verify: unknown burn command (0x%X) to be verified!",
                command);
            return false;
        }
    }
}
