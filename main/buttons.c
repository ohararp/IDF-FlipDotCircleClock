// Button handling using espressif/button component (v4.1.6)
// https://components.espressif.com/components/espressif/button
// https://github.com/espressif/esp-iot-solution/tree/master/components/button
// Provides hardware-debounced GPIO buttons with configurable short/long press.

#include "buttons.h"
#include "gpio_config.h"
#include "esp_log.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "buttons";

// FreeRTOS queue for button events (callbacks → main task)
static QueueHandle_t s_button_queue;

// Long press threshold in milliseconds (fires while button is still held)
#define LONG_PRESS_MS 1000

// ── Callback helpers: each posts an event to the queue ───────────────────────

// Generic callback that posts the event stored in usr_data to the queue
static void button_cb(void *btn_handle, void *usr_data)
{
    app_button_event_t event = (app_button_event_t)(intptr_t)usr_data;
    xQueueSend(s_button_queue, &event, 0); // non-blocking post
}

// ── Helper: create one GPIO button with short + long press callbacks ─────────

static esp_err_t create_button(int gpio, app_button_event_t short_event, app_button_event_t long_event)
{
    // Button config: set long press threshold to 2.0s
    button_config_t btn_cfg = {
        .long_press_time = LONG_PRESS_MS,   // 2000 ms hold triggers long press
        .short_press_time = 0,              // use default short press time
    };

    // GPIO config: active LOW (pull-up buttons, pressed = GND)
    button_gpio_config_t gpio_cfg = {
        .gpio_num = gpio,
        .active_level = 0,                  // LOW when pressed
    };

    button_handle_t handle = NULL;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create button on GPIO %d: %s", gpio, esp_err_to_name(ret));
        return ret;
    }

    // Register single click callback (fires on release after short press)
    ret = iot_button_register_cb(handle, BUTTON_SINGLE_CLICK, NULL, button_cb, (void *)(intptr_t)short_event);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register short press on GPIO %d", gpio);
        return ret;
    }

    // Register long press start callback (fires while button is still held at 2s threshold)
    button_event_args_t long_args = { .long_press.press_time = LONG_PRESS_MS };
    ret = iot_button_register_cb(handle, BUTTON_LONG_PRESS_START, &long_args, button_cb, (void *)(intptr_t)long_event);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register long press on GPIO %d", gpio);
        return ret;
    }

    return ESP_OK;
}

// ── Public API ───────────────────────────────────────────────────────────────

// Configure all 3 buttons using espressif/button component with debounce + short/long detection
esp_err_t buttons_init(void)
{
    // Create queue for button events (depth 8)
    s_button_queue = xQueueCreate(8, sizeof(app_button_event_t));
    if (!s_button_queue) {
        ESP_LOGE(TAG, "Failed to create button event queue");
        return ESP_FAIL;
    }

    // Create Button A (GPIO 1): short = re-home, long = WiFi reconnect
    ESP_ERROR_CHECK(create_button(PIN_BUTTON_A, BUTTON_EVENT_A_SHORT, BUTTON_EVENT_A_LONG));

    // Create Button B (GPIO 38): short = +1 hour, long = sync animation
    ESP_ERROR_CHECK(create_button(PIN_BUTTON_B, BUTTON_EVENT_B_SHORT, BUTTON_EVENT_B_LONG));

    // Create Button C (GPIO 33): short = +1 minute, long = AS5600 calibration
    ESP_ERROR_CHECK(create_button(PIN_BUTTON_C, BUTTON_EVENT_C_SHORT, BUTTON_EVENT_C_LONG));

    ESP_LOGI(TAG, "Buttons A=%d B=%d C=%d (espressif/button, short+long, %d ms threshold)",
             PIN_BUTTON_A, PIN_BUTTON_B, PIN_BUTTON_C, LONG_PRESS_MS);
    return ESP_OK;
}

// Check for a button event; blocks up to timeout_ms (0 = non-blocking)
bool buttons_get_event(app_button_event_t *event, uint32_t timeout_ms)
{
    return xQueueReceive(s_button_queue, event, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}
