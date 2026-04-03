# FlipDotCircleClock: CircuitPython → ESP-IDF 6.0 Porting Plan

## Context
The FlipDotCircleClock is a flip-dot circle clock currently running as a single ~103KB CircuitPython `code.py` on an **Unexpected Maker FeatherS3** (ESP32-S3 240MHz dual-core, 2.4GHz WiFi + BLE 5.0). The goal is to port it to ESP-IDF 6.0 C for better performance, real-time control, and FreeRTOS task management.

**Hardware controlled:**
- DS3231 RTC (I2C 0x68), SH1107 OLED 128x64 (I2C 0x3C), AS5600 magnetic encoder (I2C 0x36, optional)
- 3-column × 4-row flip-dot matrix (12 dots, 24V via relay on IO11, SPI control)
- TMC2209 stepper driver: MT-1701HSM140AE 0.9°/step motor, 400 base steps × 32 microsteps = **12,800 steps/rev**
- Hall effect sensor (A3144) for home position, NeoPixel LED (IO48), 3 buttons
- WiFi with NTP, HTTP web server on port 80

## Commit Rule: Always Update README.md

Every commit must update `README.md` to reflect the current state — mark completed steps in the porting checklist, and note any new features or modules added. Never commit without checking the README is current.

## Code Style: Modular main.c

Keep `main.c` as lean as possible — it should only handle init sequencing and task creation. All functional logic (drivers, timekeeping, animations, etc.) belongs in dedicated modules (`ds3231.c`, `stepper.c`, `network.c`, etc.) with clean header interfaces. This makes each module independently reusable and portable to derivative projects.

## Code Commenting Style (all files)

All code across every file must be highly readable with clear intent. Follow these rules:
- Every function definition gets a one-line comment above it describing what it does and why.
- Every non-trivial line or block gets an inline or preceding one-line comment explaining intent.
- Comments must be **concise but descriptive** — never longer than one line.
- Include concrete details: peripheral names, I2C addresses, pin numbers, timing values, register names.
- Magic numbers always get an inline comment (e.g., `0x7F // mask off oscillator-halt bit`).
- This applies to `.c` files, `.h` files, and `CMakeLists.txt` alike.

## Architecture: FreeRTOS Task Design (Dual-Core Pinning)

The ESP32-S3 has two cores. Tasks are explicitly pinned to separate network I/O from timing-sensitive hardware control.

### Core 0 — "Network Core"
ESP-IDF's WiFi/LWIP stack runs on Core 0 by default. All network-bound tasks stay here to avoid cross-core contention with the WiFi driver.

| Task | Priority | Purpose |
|------|----------|---------|
| `network_task` | 4 | WiFi connect/recover/health checks, NTP sync |
| `web_server_task` | 4 | HTTP server + JSON API handlers |

### Core 1 — "Real-Time Core"
All hardware with µs-level timing requirements runs here, isolated from WiFi ISR jitter.

| Task | Priority | Purpose |
|------|----------|---------|
| `clock_task` | 5 | Motor control (minute/hour updates), flip-dot SPI, relay sequencing |
| `display_task` | 3 | OLED I2C refresh every 1s |

### Cross-Core Communication
- Web API handlers (Core 0) **never** directly call motor/flipdot functions
- Instead, they post commands to a FreeRTOS queue consumed by `clock_task` (Core 1)
- Shared state (time, position, config) protected by mutexes

### Implementation Pattern
```c
// Core 0 — network-bound
xTaskCreatePinnedToCore(network_task,    "network", 4096, NULL, 4, NULL, 0);
// web server uses esp_http_server which creates its own task on Core 0

// Core 1 — real-time hardware
xTaskCreatePinnedToCore(clock_task,      "clock",   4096, NULL, 5, NULL, 1);
xTaskCreatePinnedToCore(display_task,    "display", 4096, NULL, 3, NULL, 1);
```

Use `esp_timer` for periodic callbacks (1s display, 30s health check).
Use FreeRTOS queues for cross-core command passing (web API → clock_task).

**No BLE** — not needed for current features. Can be added later if a use case emerges.

## File Structure (final target)

```
FlipDotCircleClock/
├── CMakeLists.txt
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   ├── idf_component.yml
│   ├── main.c                  # App entry, init sequence, clock task
│   ├── gpio_config.h           # All pin definitions
│   ├── ds3231.c / .h           # RTC driver (I2C)
│   ├── as5600.c / .h           # Magnetic encoder driver (I2C)
│   ├── oled_display.c / .h     # SH1107 OLED via esp_lcd
│   ├── flipdot.c / .h          # Flip-dot SPI control + relay power
│   ├── stepper.c / .h          # TMC2209 motor control + homing
│   ├── timekeeping.c / .h      # Timezone, DST, NTP sync, RTC sync
│   ├── network.c / .h          # WiFi connect/recover/health
│   ├── web_server.c / .h       # HTTP server + JSON API
│   ├── nvm_storage.c / .h      # NVS-based persistent config
│   ├── animations.c / .h       # Demo, chaos, sync sequences
│   ├── calibration.c / .h      # AS5600 calibration routines
│   ├── ota_update.c / .h       # GitHub release OTA updates
│   └── action_log.c / .h      # Ring buffer + persistent LittleFS logging
├── partitions.csv              # Custom partition table (factory + ota_0 + ota_1 + littlefs)
└── frontend/
    └── index.html              # Web UI (embed via SPIFFS or embed binary)
```

---

## Step 1: Project Skeleton + GPIO Definitions + LED Blink

**Goal:** Bootable ESP-IDF project that proves the toolchain works.

**Implement:**
- Root `CMakeLists.txt`, `main/CMakeLists.txt`, `sdkconfig.defaults`
- `gpio_config.h` — all pin definitions from CircuitPython code:
  - Stepper: EN(6), STEP(12), DIR(5), MS(17)
  - SPI flip-dot: SCK(36), MOSI(35), CS/LATCH(37), OE(18)
  - Relay: GPIO 11
  - Hall sensor: GPIO 14
  - Buttons: GPIO 1, 38, 33
  - I2C: SDA(8), SCL(9)
  - NeoPixel: GPIO 48
- `main.c` — minimal app_main() that blinks NeoPixel LED via RMT/LED strip driver
- `idf_component.yml` — declare `espressif/led_strip` dependency

**ESP-IDF APIs:** `gpio`, `led_strip` component

**Verify:** Build, flash, see NeoPixel toggling. Confirm serial monitor output.

---

## Step 2: I2C Bus + DS3231 RTC Driver

**Goal:** Read time from the DS3231 RTC over I2C.

**Implement:**
- `ds3231.c/.h` — I2C driver for DS3231 (address 0x68)
  - `ds3231_init(i2c_master_bus_handle_t *ret_bus_handle)` — creates shared I2C bus, returns handle for OLED/AS5600
  - `ds3231_get_time(struct tm *time)` — read BCD registers, convert
  - `ds3231_set_time(const struct tm *time)` — write BCD registers
  - `ds3231_set_time_from_compile()` — seed RTC with `__DATE__`/`__TIME__` on every boot
- Print current RTC time to serial every second

**ESP-IDF APIs:** `i2c_master` (new driver in v5.x+), `esp_log`

**Verify:** Serial output shows correct date/time from RTC. Compile-time seeding sets RTC on each flash.

---

## Step 3: OLED Display (SH1107)

**Goal:** Status display showing time, network info, motor position.

**Implement:**
- `oled_display.c/.h`:
  - `oled_init(i2c_master_bus_handle_t bus)` — initialize SH1107 via `esp_lcd` panel driver
  - `oled_update_time(const struct tm *time)` — HH:MM:SS display
  - `oled_update_status(const char *ip, int rssi, uint32_t uptime, int motor_pos)`
  - `oled_clear()`
  - Use LVGL or direct framebuffer writes for text rendering
- `idf_component.yml` — add `espressif/ssd1306` component (supports SH1107)

**ESP-IDF APIs:** `esp_lcd`, `esp_lcd_panel_io_i2c`, `espressif/ssd1306` component

**Verify:** OLED shows current time from RTC, refreshing every second. Status fields display placeholder data.

---

## Step 4: NVS Storage Module

**Goal:** Persistent configuration storage matching CircuitPython NVM layout.

**Implement:**
- `nvm_storage.c/.h` — NVS wrapper:
  - `nvm_init()` — open NVS namespace, check magic byte
  - `nvm_get/set_timezone_index()`
  - `nvm_get/set_home_offset()`
  - `nvm_get/set_step_delay()`
  - `nvm_get/set_as5600_cal_angle()`
  - `nvm_factory_reset()` — restore defaults
- Defaults: timezone=0 (US/Eastern), step_delay=450µs, home_offset=0
- NVM layout mirrors CircuitPython: bytes 0-5 timezone, bytes 6-7 AS5600 cal angle, bytes 8+ motor speed

**ESP-IDF APIs:** `nvs_flash`, `nvs`

**Verify:** Write values, reboot, read back — values persist. Factory reset clears to defaults.

---

## Step 5: Stepper Motor Control + Hall Sensor Homing

**Goal:** Drive the TMC2209 stepper and home to 12 o'clock position.

**Implement:**
- `stepper.c/.h`:
  - `stepper_init()` — configure GPIOs (EN, STEP, DIR, MS), enable 32-microstep mode
  - `stepper_enable() / stepper_disable()` — motor holding torque (EN pin low/high). Default: enabled. Togglable via web API.
  - `stepper_multi_step(int steps, bool clockwise)` — pulse generation with configurable delay
  - `stepper_find_home()` — symmetric Hall sensor edge detection (port `findExactHome()` logic)
  - `stepper_get_position() / stepper_set_position()` — track step count
- Hall sensor: GPIO 14 input with pull-up, read during homing
- Constants: 12800 steps/rev, configurable step delay from NVS

**ESP-IDF APIs:** `gpio`, `esp_timer` (for µs-precision step pulses), `esp_rom_delay_us`

**Verify:** Motor homes reliably to 12 o'clock. Serial logs show Hall transitions. `stepper_multi_step(12800, true)` does exactly one full revolution.

---

## Step 6: AS5600 Magnetic Encoder + Closed-Loop Motor Control

**Goal:** Read absolute angle and implement closed-loop positioning.

**Implement:**
- `as5600.c/.h`:
  - `as5600_init(i2c_master_bus_handle_t bus)` — probe address 0x36, detect presence
  - `as5600_read_raw_angle()` — 12-bit value (0-4095)
  - `as5600_is_connected()` — availability check for fallback logic
- `calibration.c/.h`:
  - `calibration_enter()` — disable motor, prompt user
  - `calibration_save()` — read AS5600, store to NVS
  - `calibration_cancel()`
  - `calibration_get_offset()` — load from NVS
- Extend `stepper.c`:
  - `stepper_move_to_angle(uint16_t target_angle)` — closed-loop move using AS5600 feedback
  - Fallback to open-loop `multi_step()` if AS5600 unavailable

**ESP-IDF APIs:** `i2c_master`

**Verify:** Serial prints raw angle as motor rotates manually. `stepper_move_to_angle()` converges to target within ±2 steps. Calibration persists across reboots.

---

## Step 7: Flip-Dot Display (SPI + Relay Power Management)

**Goal:** Control the 3x4 flip-dot matrix with proper 24V relay sequencing.

**Implement:**
- `flipdot.c/.h`:
  - `flipdot_init()` — configure SPI bus (SCK=36, MOSI=35, CS=37), OE pin (18), relay pin (11)
  - `flipdot_set_pattern(uint8_t pattern[4])` — send 4-byte pattern via SPI, toggle latch/OE
  - `flipdot_power_on()` / `flipdot_power_off()` — relay with 200ms precharge delay, 80ms hold after last flip
  - `flipdot_extend_power_window()` — prevent relay chatter during rapid updates
  - Column staggering: 500ms between columns to allow capacitor recharge and prevent inrush brownout
  - `flipdot_show_hour(int hour)` — map 1-12 hour to bit pattern
  - `flipdot_blank()` / `flipdot_all_on()` — test patterns

**ESP-IDF APIs:** `spi_master`, `gpio`, `esp_timer`

**Verify:** Call `flipdot_show_hour(3)` — correct 3 dots flip. `flipdot_blank()` then `flipdot_all_on()` cycles all dots. Relay precharge timing verified with scope/multimeter.

---

## Step 8: Timekeeping — Timezone, DST, Clock Update Logic

**Goal:** Clock runs autonomously using RTC time with proper timezone/DST. Motor and flipdots update on schedule.

**Implement:**
- `timekeeping.c/.h`:
  - `timekeeping_init()` — load timezone from NVS, read RTC
  - `timekeeping_get_local_time(struct tm *time)` — RTC + timezone offset + DST
  - `timekeeping_set_timezone(int tz_index)` — update NVS, recalculate
  - `timekeeping_apply_dst(struct tm *utc, int tz_index)` — port DST rules:
    - US: 2nd Sun Mar → 1st Sun Nov
    - EU: Last Sun Mar → Last Sun Oct
    - AU: 1st Sun Oct → 1st Sun Apr
    - NZ: Last Sun Sep → 1st Sun Apr
  - `timekeeping_sync_from_ntp(time_t ntp_time)` — update RTC (stub for now, used in Step 12)
- 18 timezone definitions as const struct array
- `clock_update_minute()` — move stepper to current minute (AS5600 closed-loop or open-loop fallback)
- `clock_update_hour()` — update flip-dot display for current hour (relay on → blank → show hour → relay off)
- Scheduling: RTC comparison-based (port of CircuitPython main loop):
  - Track `sec_old`, `min_old`, `hr_old` — compare against RTC each loop iteration
  - Every second: update OLED time display
  - Every minute: call `clock_update_minute()`
  - Every hour: call `clock_update_hour()`, re-home motor to 12 o'clock, then position to current minute

**ESP-IDF APIs:** `time.h` (POSIX), `sys/time.h`

**Verify:** Correct local time with timezone offset. Minute hand advances each minute. Flip-dots update each hour. Motor re-homes hourly.

---

## Step 9: Full Button Input + Calibration

**Goal:** All 3 buttons functional with short/long press detection (2.0s threshold).

**Implement:**
- Extend `buttons.c/.h`:
  - Add Button A (GPIO 1) with ISR
  - Add long-press detection: track press duration, emit separate short/long events
  - `BUTTON_EVENT_A_SHORT` / `BUTTON_EVENT_A_LONG`
  - `BUTTON_EVENT_B_SHORT` / `BUTTON_EVENT_B_LONG`
  - `BUTTON_EVENT_C_SHORT` / `BUTTON_EVENT_C_LONG`
- Button A short: re-home sequence (blank display → home motor → update hour + minute)
- Button A long: reserved for WiFi reconnect (no-op until Step 12)
- Button B short: +1 hour (advance flipdot display, wrap 12→1)
- Button B long: trigger sync animation (Step 10)
- Button C short: +1 minute (advance minute hand one step position)
- Button C long: enter/confirm AS5600 calibration (wire into existing `calibration.c`)
- Stepper/flipdot mutex: `xSemaphoreCreateMutex()` to protect motor and flipdot access from concurrent button presses and clock updates

**ESP-IDF APIs:** `gpio` ISR, `freertos/semphr.h`, `esp_timer` (for press duration)

**Verify:** Each button produces correct short/long events. +1 hour/minute works. Calibration enter/save works via long-press C. No motor conflicts between button presses and clock updates.

---

## Step 10: Animations

**Goal:** Port the three animation sequences, triggered by buttons.

**Implement:**
- `animations.c/.h`:
  - `anim_demo()` — go home, sweep hours 1–12 on flipdots (1.5s each), restore current time
  - `anim_chaos()` — random hour positions with motor + flipdot (12 iterations, 1.5s each), restore
  - `anim_sync()` — synchronized hand sweep to each hour (STEPS/12 per hour, 1.5s each), restore
  - All animations are blocking (run on caller's task with `vTaskDelay()`)
  - Suspend display task during animation to prevent OLED conflicts
  - Acquire motor/flipdot mutex before running
- Button triggers:
  - Button A short: `anim_demo()` (or re-home — TBD based on testing)
  - Button B long: `anim_sync()`

**ESP-IDF APIs:** `esp_random()` for chaos mode, `vTaskDelay()`

**Verify:** Trigger each animation from button press. Motor and flip-dots operate in sync. Animations complete without watchdog timeouts. Clock restores correct time after animation.

---

## Step 11: Integration + Polish (Standalone Clock)

**Goal:** Fully functional standalone clock — runs indefinitely on RTC alone without WiFi.

**Implement:**
- Complete power-on sequence:
  1. NVS init
  2. NeoPixel init
  3. I2C bus + RTC + OLED + AS5600
  4. Stepper init + homing
  5. Flipdot init + show current hour
  6. Position minute hand to current minute
  7. Start display task + main button/clock loop
- NeoPixel status colors (standalone, no WiFi yet):
  - Green: clock running normally
  - Yellow: RTC read error or hardware fault
  - Purple: startup/homing in progress
- Watchdog timer: configure `esp_task_wdt` for main loop and display task
- Brown-out detection: log reset reason on boot via `esp_reset_reason()`, show on OLED terminal
- Motor mutex enforced across all motor/flipdot access points
- Soak test: run for 24+ hours verifying:
  - No memory leaks (`esp_get_free_heap_size()` stable)
  - Minute hand position stays accurate (no cumulative drift)
  - Hour display updates correctly at each hour boundary
  - No watchdog resets or crashes
  - Relay always turns off after flipdot updates

**ESP-IDF APIs:** `esp_task_wdt`, `esp_reset_reason()`, `esp_get_free_heap_size()`

**Verify:** Power on → homes → shows hour → positions minute → runs autonomously. Buttons all work. Leave running overnight — still correct in the morning.

---

## Step 12: WiFi + NTP Synchronization

**Goal:** WiFi connection management and NTP time sync.

**Implement:**
- `network.c/.h`:
  - `network_init()` — init WiFi STA mode, register event handlers
  - `network_connect(const char *ssid, const char *password)` — connect with retry
  - `network_get_state()` — CONNECTED / CONNECTING / DISCONNECTED / OFFLINE
  - `network_recover()` — exponential backoff (1s→2s→4s→8s), 3 failures → OFFLINE
  - `network_health_check()` — periodic WiFi status validation
  - `network_get_rssi()` / `network_get_ip_str()`
  - mDNS: register hostname (e.g. `flipclock.local`) via `mdns` component
  - NTP sync via `esp_sntp`:
    - `network_sync_ntp()` — trigger SNTP, update RTC on callback
    - Hourly re-sync via esp_timer
- WiFi credentials: hardcoded via Kconfig (`menuconfig`) for development
  - `CONFIG_WIFI_SSID` and `CONFIG_WIFI_PASSWORD` in `Kconfig.projbuild`
  - **TODO (future):** SoftAP captive portal provisioning + optional `settings.toml` on SPIFFS
- Wire `timekeeping_sync_from_ntp()` to SNTP callback
- Button A long press: trigger `network_recover()` for manual WiFi reconnect
- NeoPixel colors updated:
  - Purple: WiFi connecting
  - Green: WiFi + NTP synced
  - Yellow: WiFi connection failed
  - Cyan: WiFi OK, NTP failed
- Pin WiFi/NTP tasks to Core 0, keep motor/flipdot on Core 1

**ESP-IDF APIs:** `esp_wifi`, `esp_event`, `esp_netif`, `esp_sntp`, `mdns`, `nvs`

**Verify:** Connects to WiFi, gets IP. `flipclock.local` resolves. NTP syncs RTC. Recovery works after disconnect. OFFLINE mode after 3 failures. Clock continues running on RTC when WiFi is down.

---

## Step 13: HTTP Web Server + JSON API + OTA

**Goal:** Full web control interface and over-the-air firmware updates.

**Implement:**
- `web_server.c/.h`:
  - `web_server_start()` — start `httpd` on port 80
  - `web_server_stop()`
  - URI handlers:
    - `GET /` — serve index.html (embedded in flash)
    - `GET /status.json` — time, timezone, IP, RSSI, uptime, free heap, motor pos
    - `GET /get_timezone` — list of 18 timezones
    - `GET /get_speed` — current step delay
    - `GET /log.json` — recent action log
    - `POST /set_hour` / `POST /set_min` — manual adjustments
    - `POST /home` — trigger homing
    - `POST /refresh` — force hour display update
    - `POST /sync_wifi` — manual NTP sync
    - `POST /set_timezone` — change timezone
    - `POST /set_speed` — change step delay
    - `POST /anim/demo` / `POST /anim/chaos` / `POST /anim/sync`
    - `POST /cal_start` / `POST /cal_save` / `POST /cal_cancel`
    - `POST /wipe` — blank flip-dots and home minute hand
    - `POST /stepper_enable` / `POST /stepper_disable` — toggle motor EN pin
    - `POST /ota/check` / `POST /ota/update` — OTA update endpoints
  - Web API handlers post commands to FreeRTOS queue consumed by clock task (Core 0 → Core 1)
- `ota_update.c/.h`:
  - `ota_check_for_update()` — query GitHub Releases API for newest tag
  - `ota_perform_update()` — download `.bin` via `esp_https_ota`
  - `ota_rollback()` — reboot to previous partition
  - Auto-rollback on 3 consecutive boot failures
- Custom `partitions.csv` with OTA layout (factory + ota_0 + ota_1 + littlefs)
- Action log: in-memory ring buffer (~32 entries) + persistent LittleFS log (64KB)
- Embed `index.html` via `EMBED_FILES` in CMakeLists.txt
- `cJSON` for JSON response building

**ESP-IDF APIs:** `esp_http_server`, `cJSON`, `esp_https_ota`, `esp_ota_ops`, `esp_crt_bundle`

**Verify:** Browser loads web UI. All endpoints functional. OTA updates from GitHub releases. Rollback works. 24+ hour soak test with WiFi + web server active.
