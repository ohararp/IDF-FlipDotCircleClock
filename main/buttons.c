#include "buttons.h"
#include "gpio_config.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_timer.h"

static const char *TAG = "buttons";

// FreeRTOS queue for button events (ISR → main task)
static QueueHandle_t s_button_queue;

// Debounce: minimum 200 ms between accepted presses
#define DEBOUNCE_US 200000
static int64_t s_last_press_us = 0;

// ISR handler for Button C — posts event to queue with debounce
static void IRAM_ATTR button_c_isr(void *arg)
{
    int64_t now = esp_timer_get_time(); // µs since boot
    if (now - s_last_press_us < DEBOUNCE_US) {
        return; // ignore bounce
    }
    s_last_press_us = now;

    button_event_t event = BUTTON_EVENT_C_PRESS;
    xQueueSendFromISR(s_button_queue, &event, NULL);
}

// Configure Button C (GPIO 33) as input with pull-up, attach falling-edge ISR
esp_err_t buttons_init(void)
{
    // Create queue for button events (depth 4 — handles rapid presses)
    s_button_queue = xQueueCreate(4, sizeof(button_event_t));
    if (!s_button_queue) {
        ESP_LOGE(TAG, "Failed to create button event queue");
        return ESP_FAIL;
    }

    // Button C: input with internal pull-up, interrupt on falling edge (press = LOW)
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << PIN_BUTTON_C),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE, // trigger on press (HIGH → LOW)
    };
    gpio_config(&btn_cfg);

    // Install GPIO ISR service and attach handler for Button C
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_BUTTON_C, button_c_isr, NULL);

    ESP_LOGI(TAG, "Button C initialized on GPIO %d (ISR + debounce)", PIN_BUTTON_C);
    return ESP_OK;
}

// Check for a button event; blocks up to timeout_ms (0 = non-blocking)
bool buttons_get_event(button_event_t *event, uint32_t timeout_ms)
{
    return xQueueReceive(s_button_queue, event, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}
