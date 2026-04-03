#include "flipdot.h"
#include "gpio_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "flipdot";

// ── Timing constants (matching CircuitPython values) ─────────────────────────

#define RELAY_PRECHARGE_MS  200  // capacitor charge time after relay closes
#define RELAY_HOLD_MS       80   // minimum relay hold after last flip
#define COLUMN_STAGGER_MS   100  // delay between column actuations to prevent inrush brownout
#define SETTLE_US           5000 // 5 ms coil actuation time after latching data

// ── Shift register bit layout per column ─────────────────────────────────────
// Each column has 4 dots, each dot uses 3 bits in the 16-bit shift register word:
//   Dot 0: bit 0=SUP, bit 1=SET, bit 2=RES
//   Dot 1: bit 3=SUP, bit 4=SET, bit 5=RES
//   Dot 2: bit 6=SUP, bit 7=SET, bit 8=RES
//   Dot 3: bit 9=SUP, bit 10=SET, bit 11=RES
// SUP = supply/enable (must be set to actuate dot)
// SET = flip dot to "on" (white) state
// RES = flip dot to "off" (black/reset) state

static const int SUP_BITS[4] = {0, 3, 6, 9};   // supply enable bit for each dot
static const int SET_BITS[4] = {1, 4, 7, 10};   // set (flip on) bit for each dot
static const int RES_BITS[4] = {2, 5, 8, 11};   // reset (flip off) bit for each dot

// ── State tracking ───────────────────────────────────────────────────────────

static bool s_relay_on = false;
static int64_t s_relay_off_at_us = 0;           // µs timestamp when relay should turn off
static uint8_t s_old_cols[3] = {255, 255, 255}; // previous column state (255 = unknown/force update)

// ── Bit helpers ──────────────────────────────────────────────────────────────

static inline uint16_t bit_set(uint16_t val, int bit) { return val | (1U << bit); }
static inline uint16_t bit_clr(uint16_t val, int bit) { return val & ~(1U << bit); }
static inline int bit_get(uint8_t val, int bit) { return (val >> bit) & 1; }

// ── Bit-banged SPI ───────────────────────────────────────────────────────────

// Shift out 8 bits MSB-first on MOSI/SCK pins (matches simpleio.shift_out)
static void shift_out_byte(uint8_t data)
{
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(PIN_FLIPDOT_MOSI, (data >> i) & 1); // set data bit
        gpio_set_level(PIN_FLIPDOT_SCK, 1);                 // clock rising edge
        gpio_set_level(PIN_FLIPDOT_SCK, 0);                 // clock falling edge
    }
}

// Send 3-column register data — exact mirror of CircuitPython shiftData()
static void shift_data(const uint16_t reg_data[3])
{
    // Enable outputs
    gpio_set_level(PIN_FLIPDOT_OE, 1);  // OE_ENABLE = HIGH

    // Phase 1: shift SET/RESET data for all 3 columns
    for (int i = 0; i < 3; i++) {
        gpio_set_level(PIN_FLIPDOT_LATCH, 0);           // latch LOW before each column
        shift_out_byte((reg_data[i] >> 8) & 0xFF);      // high byte MSB-first
        shift_out_byte(reg_data[i] & 0xFF);              // low byte MSB-first
    }
    gpio_set_level(PIN_FLIPDOT_LATCH, 1);               // latch HIGH — transfer to outputs
    gpio_set_level(PIN_FLIPDOT_LATCH, 0);               // latch LOW — complete pulse
    esp_rom_delay_us(SETTLE_US);                         // 5 ms coil actuation time

    // Phase 2: shift zeros to clear/discharge all coils
    for (int i = 0; i < 3; i++) {
        gpio_set_level(PIN_FLIPDOT_LATCH, 0);
        shift_out_byte(0x00);
        shift_out_byte(0x00);
    }
    gpio_set_level(PIN_FLIPDOT_LATCH, 1);               // latch HIGH — apply clear
    gpio_set_level(PIN_FLIPDOT_LATCH, 0);               // latch LOW — complete pulse

    // Disable outputs
    gpio_set_level(PIN_FLIPDOT_OE, 0);                  // OE_DISABLE = LOW
}

// ── Register encoding (port of setFlipsCore) ─────────────────────────────────

// Convert simple dot patterns (1 bit per dot) to 12-bit shift register words
// with SUP/SET/RES encoding and XOR optimization, then shift to hardware
static void set_flips_core(const uint8_t cols[3], bool force_all)
{
    // Reverse column order to match CircuitPython: colData = [dataIn[2], dataIn[1], dataIn[0]]
    uint8_t col_data[3] = {cols[2], cols[1], cols[0]};
    uint16_t reg_data[3] = {0, 0, 0};

    for (int i = 0; i < 3; i++) {
        uint8_t xor_data = col_data[i] ^ s_old_cols[i]; // which dots changed?
        for (int j = 0; j < 4; j++) {
            int changed = bit_get(xor_data, j);
            int dot_on = bit_get(col_data[i], j);

            if (changed || force_all) {
                // Dot needs updating — enable supply
                reg_data[i] = bit_set(reg_data[i], SUP_BITS[j]);
                if (dot_on) {
                    reg_data[i] = bit_set(reg_data[i], SET_BITS[j]); // flip to ON
                    reg_data[i] = bit_clr(reg_data[i], RES_BITS[j]);
                } else {
                    reg_data[i] = bit_clr(reg_data[i], SET_BITS[j]);
                    reg_data[i] = bit_set(reg_data[i], RES_BITS[j]); // flip to OFF
                }
            } else {
                // Dot unchanged — disable all control bits
                reg_data[i] = bit_clr(reg_data[i], SUP_BITS[j]);
                reg_data[i] = bit_clr(reg_data[i], SET_BITS[j]);
                reg_data[i] = bit_clr(reg_data[i], RES_BITS[j]);
            }
        }
    }

    // Stagger columns: shift one column at a time with delay between to prevent inrush brownout
    for (int i = 0; i < 3; i++) {
        if (reg_data[i] != 0) {
            uint16_t staggered[3] = {0, 0, 0};
            staggered[i] = reg_data[i];
            shift_data(staggered);
            vTaskDelay(pdMS_TO_TICKS(COLUMN_STAGGER_MS)); // 100 ms recharge
        }
    }

    // Update cache with new column state (using reversed order)
    s_old_cols[0] = col_data[0];
    s_old_cols[1] = col_data[1];
    s_old_cols[2] = col_data[2];
}

// ── Public API ───────────────────────────────────────────────────────────────

// Configure all flipdot GPIO pins as outputs, start with relay off and OE disabled
esp_err_t flipdot_init(void)
{
    // SPI clock pin (bit-banged)
    gpio_config_t sck_cfg = {
        .pin_bit_mask = (1ULL << PIN_FLIPDOT_SCK),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&sck_cfg);
    gpio_set_level(PIN_FLIPDOT_SCK, 0);

    // SPI data pin (bit-banged MOSI)
    gpio_config_t mosi_cfg = {
        .pin_bit_mask = (1ULL << PIN_FLIPDOT_MOSI),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&mosi_cfg);
    gpio_set_level(PIN_FLIPDOT_MOSI, 0);

    // Latch/strobe pin — rising edge transfers shift register to outputs
    gpio_config_t latch_cfg = {
        .pin_bit_mask = (1ULL << PIN_FLIPDOT_LATCH),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&latch_cfg);
    gpio_set_level(PIN_FLIPDOT_LATCH, 0);

    // Output Enable pin — HIGH = enabled, LOW = disabled
    gpio_config_t oe_cfg = {
        .pin_bit_mask = (1ULL << PIN_FLIPDOT_OE),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&oe_cfg);
    gpio_set_level(PIN_FLIPDOT_OE, 0); // start disabled

    // 24V relay pin — HIGH = relay on, LOW = relay off
    gpio_config_t relay_cfg = {
        .pin_bit_mask = (1ULL << PIN_RELAY),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&relay_cfg);
    gpio_set_level(PIN_RELAY, 0); // start with relay off

    ESP_LOGI(TAG, "Flipdot initialized: SCK=%d MOSI=%d LATCH=%d OE=%d RELAY=%d",
             PIN_FLIPDOT_SCK, PIN_FLIPDOT_MOSI, PIN_FLIPDOT_LATCH,
             PIN_FLIPDOT_OE, PIN_RELAY);
    return ESP_OK;
}

// Close relay and wait for capacitors to charge (200 ms precharge)
void flipdot_power_on(void)
{
    if (!s_relay_on) {
        gpio_set_level(PIN_RELAY, 1);                      // close relay
        s_relay_on = true;
        vTaskDelay(pdMS_TO_TICKS(RELAY_PRECHARGE_MS));     // 200 ms precharge
        ESP_LOGD(TAG, "Relay ON (precharge %d ms)", RELAY_PRECHARGE_MS);
    }
}

// Open relay immediately
void flipdot_power_off(void)
{
    if (s_relay_on) {
        gpio_set_level(PIN_RELAY, 0);
        s_relay_on = false;
        ESP_LOGD(TAG, "Relay OFF");
    }
}

// Extend relay hold window — relay stays on for at least RELAY_HOLD_MS after last call
void flipdot_extend_power_window(void)
{
    int64_t new_off = esp_timer_get_time() + (RELAY_HOLD_MS * 1000);
    if (new_off > s_relay_off_at_us) {
        s_relay_off_at_us = new_off;
    }
}

// Turn off relay if hold window has expired (call from main loop or timer)
void flipdot_service_power_window(void)
{
    if (s_relay_on && esp_timer_get_time() >= s_relay_off_at_us) {
        flipdot_power_off();
    }
}

// Send 3-column dot pattern with proper SUP/SET/RES encoding and column staggering
void flipdot_set_pattern(const uint8_t cols[3])
{
    set_flips_core(cols, true); // force_all=true to actuate every dot
    flipdot_extend_power_window();
}

// Hour-to-dot-pattern lookup table (3×4 matrix)
// Each value is a 4-bit mask: bit 0=dot 0, bit 1=dot 1, bit 2=dot 2, bit 3=dot 3
// Column order: [col0, col1, col2] — col0 fills first, then col1, then col2
static const uint8_t s_hour_patterns[13][3] = {
    {0,  0,  0},   // hour 0 (blank)
    {2,  0,  0},   // hour 1:  1 dot  in col 0
    {6,  0,  0},   // hour 2:  2 dots in col 0
    {14, 0,  0},   // hour 3:  3 dots in col 0
    {30, 1,  0},   // hour 4:  4 dots in col 0 + 1 in col 1
    {30, 3,  0},   // hour 5:  4+2
    {30, 7,  0},   // hour 6:  4+3
    {30, 15, 0},   // hour 7:  4+4
    {30, 15, 1},   // hour 8:  4+4+1
    {30, 15, 3},   // hour 9:  4+4+2
    {30, 15, 7},   // hour 10: 4+4+3
    {30, 15, 15},  // hour 11: 4+4+4
    {15, 15, 15},  // hour 12: all 12 dots on (special pattern)
};

// Display hour (1–12) on the flip-dot matrix
void flipdot_show_hour(int hour)
{
    if (hour < 0 || hour > 12) hour = 0;
    ESP_LOGI(TAG, "Show hour %d: [%d, %d, %d]",
             hour, s_hour_patterns[hour][0], s_hour_patterns[hour][1], s_hour_patterns[hour][2]);
    flipdot_set_pattern(s_hour_patterns[hour]);
}

// Blank all dots (flip all to OFF state)
void flipdot_blank(void)
{
    const uint8_t blank[3] = {0, 0, 0};
    flipdot_set_pattern(blank);
    ESP_LOGI(TAG, "Display blanked");
}

// Light all 12 dots (test pattern)
void flipdot_all_on(void)
{
    flipdot_show_hour(12);
    ESP_LOGI(TAG, "All dots on");
}
