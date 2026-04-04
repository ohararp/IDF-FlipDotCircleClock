#include "timekeeping.h"
#include "ds3231.h"
#include "nvm_storage.h"
#include "stepper.h"
#include "as5600.h"
#include "flipdot.h"
#include "calibration.h"
#include "oled_display.h"
#include "gpio_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "timekeep";

// ── 18 timezone definitions (ported from CircuitPython TIMEZONES tuple) ──────

static const timezone_def_t s_timezones[TIMEZONE_COUNT] = {
    {"US/Hawaii",    "Hawaii",             -600, DST_NONE},
    {"US/Alaska",    "Alaska",             -540, DST_US},
    {"US/Pacific",   "Pacific (LA)",       -480, DST_US},
    {"US/Mountain",  "Mountain (Denver)",  -420, DST_US},
    {"US/Arizona",   "Arizona",            -420, DST_NONE},
    {"US/Central",   "Central (Chicago)",  -360, DST_US},
    {"US/Eastern",   "Eastern (NY)",       -300, DST_US},
    {"EU/London",    "London",                0, DST_EU},
    {"EU/Paris",     "Paris",               60, DST_EU},
    {"EU/Berlin",    "Berlin",              60, DST_EU},
    {"EU/Moscow",    "Moscow",             180, DST_NONE},
    {"AS/Dubai",     "Dubai",              240, DST_NONE},
    {"AS/Mumbai",    "Mumbai",             330, DST_NONE},
    {"AS/Singapore", "Singapore",          480, DST_NONE},
    {"AS/Tokyo",     "Tokyo",              540, DST_NONE},
    {"OC/Sydney",    "Sydney",             600, DST_AU},
    {"OC/Auckland",  "Auckland",           720, DST_NZ},
    {"UTC",          "UTC",                  0, DST_NONE},
};

// Currently active timezone index (0–17)
static uint8_t s_tz_index = 0;

// ── DST calculation helpers ──────────────────────────────────────────────────

// Find the nth occurrence of a weekday in a month (weekday: 0=Sun..6=Sat, n: 1=first, -1=last)
static int nth_weekday(int year, int month, int weekday, int n)
{
    if (n == -1) {
        // Last occurrence: start from last day of month and walk back
        struct tm last = {0};
        last.tm_year = year - 1900;
        last.tm_mon = month; // next month (0-based, so month=current 1-based)
        last.tm_mday = 0;   // day 0 of next month = last day of this month
        mktime(&last);       // normalize
        int day = last.tm_mday;
        int wday = last.tm_wday; // 0=Sun
        int diff = (wday - weekday + 7) % 7;
        return day - diff;
    } else {
        // Nth occurrence: find first weekday in month, add (n-1)*7
        struct tm first = {0};
        first.tm_year = year - 1900;
        first.tm_mon = month - 1; // 0-based
        first.tm_mday = 1;
        mktime(&first);
        int wday = first.tm_wday; // 0=Sun
        int diff = (weekday - wday + 7) % 7;
        return 1 + diff + (n - 1) * 7;
    }
}

// US DST: 2nd Sunday March 2am → 1st Sunday November 2am
static bool is_dst_us(int year, int month, int day, int hour)
{
    if (month < 3 || month > 11) return false;
    if (month > 3 && month < 11) return true;
    int start = nth_weekday(year, 3, 0, 2);  // 2nd Sunday of March (0=Sun)
    int end   = nth_weekday(year, 11, 0, 1); // 1st Sunday of November
    if (month == 3)  return day > start || (day == start && hour >= 2);
    if (month == 11) return day < end   || (day == end   && hour < 2);
    return false;
}

// EU DST: last Sunday March 1am UTC → last Sunday October 1am UTC
static bool is_dst_eu(int year, int month, int day, int hour)
{
    if (month < 3 || month > 10) return false;
    if (month > 3 && month < 10) return true;
    int start = nth_weekday(year, 3, 0, -1);  // last Sunday of March
    int end   = nth_weekday(year, 10, 0, -1); // last Sunday of October
    if (month == 3)  return day > start || (day == start && hour >= 1);
    if (month == 10) return day < end   || (day == end   && hour < 1);
    return false;
}

// AU DST: 1st Sunday October 2am → 1st Sunday April 3am (southern hemisphere Oct–Apr)
static bool is_dst_au(int year, int month, int day, int hour)
{
    if (month > 10 || month < 4) return true;  // Nov–Mar always DST
    if (month > 4 && month < 10) return false;  // May–Sep never DST
    int start = nth_weekday(year, 10, 0, 1); // 1st Sunday of October
    int end   = nth_weekday(year, 4, 0, 1);  // 1st Sunday of April
    if (month == 10) return day > start || (day == start && hour >= 2);
    if (month == 4)  return day < end   || (day == end   && hour < 3);
    return false;
}

// NZ DST: last Sunday September 2am → 1st Sunday April 3am (southern hemisphere Sep–Apr)
static bool is_dst_nz(int year, int month, int day, int hour)
{
    if (month > 9 || month < 4) return true;   // Oct–Mar always DST
    if (month > 4 && month < 9) return false;   // May–Aug never DST
    int start = nth_weekday(year, 9, 0, -1); // last Sunday of September
    int end   = nth_weekday(year, 4, 0, 1);  // 1st Sunday of April
    if (month == 9) return day > start || (day == start && hour >= 2);
    if (month == 4) return day < end   || (day == end   && hour < 3);
    return false;
}

// Calculate DST offset in minutes (60 if active, 0 otherwise) for a given UTC time
static int calculate_dst_offset(const timezone_def_t *tz, const struct tm *utc)
{
    int year  = utc->tm_year + 1900;
    int month = utc->tm_mon + 1;     // struct tm: 0–11 → 1–12
    int day   = utc->tm_mday;
    int hour  = utc->tm_hour;

    switch (tz->dst_rule) {
    case DST_US: return is_dst_us(year, month, day, hour) ? 60 : 0;
    case DST_EU: return is_dst_eu(year, month, day, hour) ? 60 : 0;
    case DST_AU: return is_dst_au(year, month, day, hour) ? 60 : 0;
    case DST_NZ: return is_dst_nz(year, month, day, hour) ? 60 : 0;
    default:     return 0;
    }
}

// ── Public API ───────────────────────────────────────────────────────────────

// Return pointer to the 18-element timezone definitions array
const timezone_def_t *timekeeping_get_timezone_list(void)
{
    return s_timezones;
}

// Load timezone index from NVS
esp_err_t timekeeping_init(void)
{
    nvm_get_timezone_index(&s_tz_index);
    if (s_tz_index >= TIMEZONE_COUNT) s_tz_index = 0; // clamp to valid range
    ESP_LOGI(TAG, "Timezone: [%d] %s (%s, UTC%+d min, DST=%d)",
             s_tz_index, s_timezones[s_tz_index].key,
             s_timezones[s_tz_index].display_name,
             s_timezones[s_tz_index].utc_offset_min,
             s_timezones[s_tz_index].dst_rule);
    return ESP_OK;
}

// Read local time from RTC.
// Pre-NTP: RTC stores local time directly (seeded from compile timestamp).
// Post-NTP (Step 12): RTC will store UTC and this function will apply timezone + DST.
esp_err_t timekeeping_get_local_time(struct tm *local)
{
    // For now, RTC contains local time — read it directly
    return ds3231_get_time(local);

    // TODO (Step 12): When NTP is added, RTC will store UTC. Uncomment this:
    // const timezone_def_t *tz = &s_timezones[s_tz_index];
    // int dst_min = calculate_dst_offset(tz, local);
    // int total_offset_sec = (tz->utc_offset_min + dst_min) * 60;
    // time_t epoch = mktime(local);
    // epoch += total_offset_sec;
    // struct tm *result = localtime(&epoch);
    // *local = *result;
    // return ESP_OK;
}

// Set active timezone by index, save to NVS
esp_err_t timekeeping_set_timezone(uint8_t tz_index)
{
    if (tz_index >= TIMEZONE_COUNT) return ESP_ERR_INVALID_ARG;
    s_tz_index = tz_index;
    nvm_set_timezone_index(tz_index);
    ESP_LOGI(TAG, "Timezone changed to [%d] %s", tz_index, s_timezones[tz_index].display_name);
    return ESP_OK;
}

// Get current timezone index
uint8_t timekeeping_get_timezone_index(void)
{
    return s_tz_index;
}

// Stub: sync RTC from NTP UTC epoch (will be used in Step 12)
esp_err_t timekeeping_sync_from_ntp(time_t utc_epoch)
{
    struct tm *utc = gmtime(&utc_epoch);
    return ds3231_set_time(utc);
}

// ── Clock update functions ───────────────────────────────────────────────────

// Move stepper to the current minute position (0–59 → 0–12799 steps)
// Open-loop bulk CW move, then PID fine-tune with AS5600 if available
void clock_update_minute(const struct tm *local)
{
    int minute = local->tm_min;
    if (as5600_is_connected()) {
        uint16_t home_offset = calibration_get_offset();
        if (home_offset != 0) {
            // PID closed-loop only — no open-loop bulk move (prevents full-revolution overshoot)
            uint16_t target_raw = as5600_minute_to_raw(minute, home_offset);
            stepper_move_to_angle(target_raw, 5); // ±5 AS5600 units ≈ ±0.4°
            stepper_set_position((minute * STEPPER_STEPS_PER_REV) / 60);
            ESP_LOGI(TAG, "Minute update (PID): %02d → AS5600 tgt %d", minute, target_raw);
            return;
        }
    }

    // Open-loop fallback: CW step counting (only if AS5600 unavailable/uncalibrated)
    int target_steps = (minute * STEPPER_STEPS_PER_REV) / 60;
    int current_pos = stepper_get_position();
    int steps_needed = (target_steps - current_pos + STEPPER_STEPS_PER_REV) % STEPPER_STEPS_PER_REV;
    if (steps_needed > 0) {
        stepper_multi_step(steps_needed, true);
    }
    stepper_set_position(target_steps);
    ESP_LOGI(TAG, "Minute update (open-loop): %02d → step %d (moved %d)", minute, target_steps, steps_needed);
}

// Update flip-dot display with current hour (12-hour format)
void clock_update_hour(const struct tm *local)
{
    int hour = local->tm_hour % 12; // convert 24h → 12h
    if (hour == 0) hour = 12;       // 0 (midnight/noon) → 12

    flipdot_power_on();
    flipdot_blank();
    vTaskDelay(pdMS_TO_TICKS(500)); // 500 ms cap recharge after blank
    flipdot_show_hour(hour);
    vTaskDelay(pdMS_TO_TICKS(500)); // 500 ms settle
    flipdot_power_off();            // CRITICAL: always shut off 24V relay

    ESP_LOGI(TAG, "Hour update: %02d → flipdot %d", local->tm_hour, hour);
}
