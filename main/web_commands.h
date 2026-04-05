#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Commands from web API (Core 0) to clock task (main loop)
typedef enum {
    WEB_CMD_SET_HOUR,       // +1 hour
    WEB_CMD_SET_MIN,        // +1 minute
    WEB_CMD_HOME,           // home motor
    WEB_CMD_WIPE,           // blank flipdots + home
    WEB_CMD_REFRESH,        // force hour display update
    WEB_CMD_ANIM_SYNC,      // sync animation
    WEB_CMD_SET_TIMEZONE,   // change timezone (param = index)
    WEB_CMD_SET_SPEED,      // change step delay (param = delay_us)
    WEB_CMD_SYNC_NTP,       // manual NTP sync
    WEB_CMD_CAL_START,      // enter calibration
    WEB_CMD_CAL_SAVE,       // save calibration
    WEB_CMD_CAL_CANCEL,     // cancel calibration
} web_cmd_type_t;

// Command with optional parameter
typedef struct {
    web_cmd_type_t type;
    int32_t param;          // used by SET_TIMEZONE (index) and SET_SPEED (delay_us)
} web_cmd_t;

// Global command queue — created by main.c, used by web_server.c
extern QueueHandle_t g_web_cmd_queue;
