#pragma once

#include <time.h>
#include <stdint.h>
#include "esp_err.h"

// DST rule identifiers
typedef enum {
    DST_NONE = 0, // no daylight saving
    DST_US,       // US: 2nd Sun Mar 2am → 1st Sun Nov 2am
    DST_EU,       // EU: last Sun Mar 1am UTC → last Sun Oct 1am UTC
    DST_AU,       // AU: 1st Sun Oct 2am → 1st Sun Apr 3am (southern hemisphere)
    DST_NZ,       // NZ: last Sun Sep 2am → 1st Sun Apr 3am (southern hemisphere)
} dst_rule_t;

// Timezone definition: name, display label, UTC offset in minutes, DST rule
typedef struct {
    const char *key;          // e.g. "US/Eastern"
    const char *display_name; // e.g. "Eastern (New York)"
    int16_t utc_offset_min;   // base UTC offset in minutes (negative = west)
    dst_rule_t dst_rule;      // which DST rule applies
} timezone_def_t;

// Number of defined timezones
#define TIMEZONE_COUNT 18

// Access the timezone definitions array
const timezone_def_t *timekeeping_get_timezone_list(void);

// Initialize timekeeping: load timezone index from NVS
esp_err_t timekeeping_init(void);

// Get local time (RTC + timezone offset + DST) — this is the primary time source
esp_err_t timekeeping_get_local_time(struct tm *local);

// Set the active timezone by index (0–17), saves to NVS
esp_err_t timekeeping_set_timezone(uint8_t tz_index);

// Get current timezone index
uint8_t timekeeping_get_timezone_index(void);

// Stub for NTP sync — updates RTC with UTC time, applies timezone/DST (used in Step 12)
esp_err_t timekeeping_sync_from_ntp(time_t utc_epoch);

// Move stepper to current minute position (AS5600 closed-loop or open-loop fallback)
void clock_update_minute(const struct tm *local);

// Update flip-dot display for current hour (relay on → blank → show → relay off)
void clock_update_hour(const struct tm *local);
