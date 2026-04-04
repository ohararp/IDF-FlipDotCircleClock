#include "buttons.h"
#include "gpio_config.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_timer.h"

static const char *TAG = "buttons";

// FreeRTOS queue for button events (ISR/timer → main task)
static QueueHandle_t s_button_queue;

// Long press threshold: 2.0 seconds — fires while still held (no release needed)
#define LONG_PRESS_MS 2000

// Debounce: minimum 200 ms between accepted presses per button
#define DEBOUNCE_US 200000

// Per-button state for press tracking and long-press timer
typedef struct {
    int gpio;                   // GPIO pin number for this button
    int64_t press_time_us;      // timestamp when button was pressed (falling edge)
    int64_t last_event_us;      // last accepted event time for debounce
    bool long_fired;            // true if long-press already fired for this press cycle
    TimerHandle_t long_timer;   // FreeRTOS timer that fires after 2s hold
    button_event_t short_event; // event type to emit on short press (release < 2s)
    button_event_t long_event;  // event type to emit on long press (held >= 2s)
} button_state_t;

static button_state_t s_btn_a, s_btn_b, s_btn_c;

// Timer callback: fires after 2s hold — emit long-press event while button is still held
static void long_press_timer_cb(TimerHandle_t timer)
{
    button_state_t *btn = (button_state_t *)pvTimerGetTimerID(timer);

    // Verify button is still held (pin LOW = pressed with pull-up)
    if (gpio_get_level(btn->gpio) == 0) {
        btn->long_fired = true; // mark so release doesn't also fire short press
        xQueueSend(s_button_queue, &btn->long_event, 0);
    }
}

// ISR handler: falling edge = start timer, rising edge = short press or cancel
static void IRAM_ATTR button_isr(void *arg)
{
    button_state_t *btn = (button_state_t *)arg;
    int64_t now = esp_timer_get_time();

    // Debounce: ignore events too close together
    if (now - btn->last_event_us < DEBOUNCE_US) return;
    btn->last_event_us = now;

    int level = gpio_get_level(btn->gpio);

    if (level == 0) {
        // Falling edge: button pressed — start long-press timer
        btn->press_time_us = now;
        btn->long_fired = false;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTimerStartFromISR(btn->long_timer, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
        // Rising edge: button released — stop timer, emit short press if long didn't fire
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTimerStopFromISR(btn->long_timer, &xHigherPriorityTaskWoken);

        if (!btn->long_fired) {
            // Long press didn't fire, so this is a short press
            xQueueSendFromISR(s_button_queue, &btn->short_event, &xHigherPriorityTaskWoken);
        }
        btn->press_time_us = 0;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Initialize a single button: configure state, create long-press timer
static void init_button(button_state_t *btn, int gpio, button_event_t short_ev, button_event_t long_ev, const char *name)
{
    btn->gpio = gpio;
    btn->press_time_us = 0;
    btn->last_event_us = 0;
    btn->long_fired = false;
    btn->short_event = short_ev;
    btn->long_event = long_ev;

    // Create one-shot timer for long-press detection (fires once after 2s hold)
    btn->long_timer = xTimerCreate(name, pdMS_TO_TICKS(LONG_PRESS_MS), pdFALSE, btn, long_press_timer_cb);
}

// Configure all 3 buttons (A=GPIO1, B=GPIO38, C=GPIO33) with pull-up and both-edge ISR
esp_err_t buttons_init(void)
{
    // Create queue for button events (depth 8 — handles burst presses)
    s_button_queue = xQueueCreate(8, sizeof(button_event_t));
    if (!s_button_queue) {
        ESP_LOGE(TAG, "Failed to create button event queue");
        return ESP_FAIL;
    }

    // Initialize per-button state and long-press timers
    init_button(&s_btn_a, PIN_BUTTON_A, BUTTON_EVENT_A_SHORT, BUTTON_EVENT_A_LONG, "btn_a");
    init_button(&s_btn_b, PIN_BUTTON_B, BUTTON_EVENT_B_SHORT, BUTTON_EVENT_B_LONG, "btn_b");
    init_button(&s_btn_c, PIN_BUTTON_C, BUTTON_EVENT_C_SHORT, BUTTON_EVENT_C_LONG, "btn_c");

    // All 3 buttons: input with internal pull-up, interrupt on both edges
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << PIN_BUTTON_A) | (1ULL << PIN_BUTTON_B) | (1ULL << PIN_BUTTON_C),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE, // both press and release
    };
    gpio_config(&btn_cfg);

    // Install GPIO ISR service and attach handlers
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_BUTTON_A, button_isr, &s_btn_a);
    gpio_isr_handler_add(PIN_BUTTON_B, button_isr, &s_btn_b);
    gpio_isr_handler_add(PIN_BUTTON_C, button_isr, &s_btn_c);

    ESP_LOGI(TAG, "Buttons A=%d B=%d C=%d (short/long, 2.0s hold-to-trigger)",
             PIN_BUTTON_A, PIN_BUTTON_B, PIN_BUTTON_C);
    return ESP_OK;
}

// Check for a button event; blocks up to timeout_ms (0 = non-blocking)
bool buttons_get_event(button_event_t *event, uint32_t timeout_ms)
{
    return xQueueReceive(s_button_queue, event, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}
