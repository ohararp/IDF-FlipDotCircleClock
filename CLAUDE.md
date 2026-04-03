# FlipDotCircleClock: CircuitPython → ESP-IDF 6.0 Porting Plan

## Context
The FlipDotCircleClock is a flip-dot circle clock currently running as a single ~103KB CircuitPython `code.py` on an **Unexpected Maker FeatherS3** (ESP32-S3 240MHz dual-core, 2.4GHz WiFi + BLE 5.0). The goal is to port it to ESP-IDF 6.0 C for better performance, real-time control, and FreeRTOS task management.

**Hardware controlled:**
- DS3231 RTC (I2C 0x68), SH1107 OLED 128x64 (I2C 0x3C), AS5600 magnetic encoder (I2C 0x36, optional)
- 3-column × 4-row flip-dot matrix (12 dots, 24V via relay on IO11, SPI control)
- TMC2209 stepper driver: MT-1701HSM140AE 0.9°/step motor, 400 base steps × 32 microsteps = **12,800 steps/rev**
- Hall effect sensor (A3144) for home position, NeoPixel LED (IO48), 3 buttons
- WiFi with NTP, HTTP web server on port 80

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
  - `ds3231_init(i2c_master_bus_handle_t bus)`
  - `ds3231_get_time(struct tm *time)` — read BCD registers, convert
  - `ds3231_set_time(const struct tm *time)` — write BCD registers
- Initialize I2C master bus in `main.c` (SDA=8, SCL=9, 400kHz)
- Print current RTC time to serial every second

**ESP-IDF APIs:** `i2c_master` (new driver in v5.x+), `esp_log`

**Verify:** Serial output shows correct date/time from RTC. If RTC battery is good, time persists across resets.

---

## Step 3: NVS Storage Module

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

## Step 4: Stepper Motor Control + Hall Sensor Homing

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

## Step 5: AS5600 Magnetic Encoder + Closed-Loop Motor Control

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

## Step 6: Flip-Dot Display (SPI + Relay Power Management)

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

## Step 7: OLED Display (SH1107)

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

## Step 8: Timekeeping — Timezone, DST, Clock Update Logic

**Goal:** Full time management with minute/hour update scheduling.

**Implement:**
- `timekeeping.c/.h`:
  - `timekeeping_init()` — load timezone from NVS, read RTC
  - `timekeeping_get_local_time(struct tm *time)` — RTC + timezone + DST
  - `timekeeping_set_timezone(int tz_index)` — update NVS, recalculate
  - `timekeeping_apply_dst(struct tm *utc, int tz_index)` — port DST rules:
    - US: 2nd Sun Mar → 1st Sun Nov
    - EU: Last Sun Mar → Last Sun Oct
    - AU: 1st Sun Oct → 1st Sun Apr
    - NZ: Last Sun Sep → 1st Sun Apr
  - `timekeeping_sync_from_ntp(time_t ntp_time)` — update RTC
- 18 timezone definitions as const struct array
- Integrate with stepper (`minUpdate` equivalent) and flipdot (`hrUpdate` equivalent):
  - `clock_update_minute()` — move motor to current minute position
  - `clock_update_hour()` — update flip-dot display for current hour
- Use `esp_timer` for 1-minute and 1-hour periodic callbacks

**ESP-IDF APIs:** `esp_timer`, `time.h` (POSIX), `sys/time.h`

**Verify:** Correct local time across timezone changes. DST transitions produce correct offsets. Motor moves to correct minute position. Flip-dots show correct hour.

---

## Step 9: WiFi + NTP Synchronization

**Goal:** WiFi connection management and NTP time sync.

**Implement:**
- `network.c/.h`:
  - `network_init()` — init WiFi STA mode, register event handlers
  - `network_connect(const char *ssid, const char *password)` — connect with retry
  - `network_get_state()` — CONNECTED / CONNECTING / DISCONNECTED / OFFLINE
  - `network_recover()` — exponential backoff (1s→2s→4s→8s), 3 failures → OFFLINE
  - `network_health_check()` — periodic WiFi status validation
  - `network_get_rssi()` / `network_get_ip_str()`
  - mDNS: register hostname (e.g. `flipclock.local`) via `mdns` component so the clock is discoverable without knowing its IP
  - NTP sync via `esp_sntp`:
    - `network_sync_ntp()` — trigger SNTP, update RTC on callback
    - Hourly re-sync via esp_timer
- WiFi credentials: hardcoded via Kconfig (`menuconfig`) for development
  - `CONFIG_WIFI_SSID` and `CONFIG_WIFI_PASSWORD` in `Kconfig.projbuild`
  - **TODO (future):** Replace with SoftAP captive portal provisioning + optional `settings.toml` on SPIFFS, with Button A long-press to force AP mode. Consider USB MSC (TinyUSB) to expose flash as a drive for direct file editing over USB-C.

**ESP-IDF APIs:** `esp_wifi`, `esp_event`, `esp_netif`, `esp_sntp`, `mdns`, `nvs`

**Verify:** Connects to WiFi, gets IP. `flipclock.local` resolves from another device on the network. NTP syncs RTC (compare before/after). Recovery works after WiFi disconnect. OFFLINE mode after 3 failures. Health check triggers recovery.

---

## Step 10: HTTP Web Server + JSON API

**Goal:** Full web control interface matching CircuitPython endpoints.

**Implement:**
- `web_server.c/.h`:
  - `web_server_start()` — start `httpd` on port 80
  - `web_server_stop()`
  - Register URI handlers:
    - `GET /` — serve index.html (embedded in flash)
    - `GET /status.json` — time, timezone, IP, RSSI, uptime, free heap (`esp_get_free_heap_size()`), motor pos
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
    - `POST /wipe` — physical reset: blank all flip-dots and home the minute hand to 12 o'clock
    - `POST /stepper_enable` / `POST /stepper_disable` — toggle stepper motor EN pin
- Action log — two tiers:
  - **In-memory ring buffer** (~32 entries): fast access, served by `GET /log.json`
  - **Persistent log on LittleFS** (64KB partition): timestamped entries written to flash, survives reboots. Rotate file at size limit. Served by `GET /log.json?persistent=true`. Invaluable for debugging issues that occur between reboots.
- Embed `index.html` via `EMBED_FILES` in CMakeLists.txt or SPIFFS partition
- `cJSON` for JSON response building

**ESP-IDF APIs:** `esp_http_server`, `cJSON` (bundled with ESP-IDF), `esp_partition` or `EMBED_FILES`

**Verify:** Browser loads web UI. Status endpoint returns valid JSON with free heap and uptime. `/log.json` returns recent actions. `/wipe` blanks dots and homes motor. Stepper enable/disable toggles motor holding torque. Test all 18 timezone selections.

---

## Step 11: OTA Update via GitHub Release

**Goal:** Over-the-air firmware updates by downloading the latest release binary from GitHub.

**Implement:**
- `ota_update.c/.h`:
  - `ota_check_for_update()` — query GitHub Releases API (`https://api.github.com/repos/ohararp/FlipDotCircleClock/releases/latest`) for newest release tag
  - `ota_get_current_version()` — read version from app description (compiled into firmware)
  - `ota_perform_update(const char *firmware_url)` — download `.bin` from GitHub release asset, write to OTA partition via `esp_https_ota`
  - `ota_rollback()` — mark current partition invalid, reboot to previous
  - Version comparison: semantic versioning string compare (e.g. "v1.2.3")
- Partition table: dual OTA partitions (`ota_0`, `ota_1`) + factory partition
  - Custom `partitions.csv` with OTA layout
- HTTPS support: bundle GitHub's CA certificate for TLS validation
- Trigger OTA from:
  - Web UI button (`POST /ota/check`, `POST /ota/update`)
  - Automatic check on boot (optional, configurable via NVS)
- Progress reporting: OLED shows download percentage, web API returns status
- Safety: validate firmware before marking as valid; auto-rollback on 3 consecutive boot failures

**ESP-IDF APIs:** `esp_https_ota`, `esp_ota_ops`, `esp_app_desc`, `esp_http_client`, `esp_crt_bundle` (for GitHub TLS)

**Files:**
- `ota_update.c/.h` — OTA logic
- `partitions.csv` — custom partition table with OTA slots
- `version.h` or use `PROJECT_VER` in `CMakeLists.txt`
- Add OTA endpoints to `web_server.c`

**Verify:**
- Build v1.0.0, flash. Create GitHub release v1.0.1 with `.bin` asset.
- Trigger update from web UI → firmware downloads, reboots to new version.
- Corrupt a release → rollback to previous version works.
- Version check correctly identifies "up to date" vs "update available".

---

## Step 12: Animations

**Goal:** Port the three animation sequences.

**Implement:**
- `animations.c/.h`:
  - `anim_demo()` — full rotation with hour countdown
  - `anim_chaos()` — random hour patterns
  - `anim_sync()` — synchronized hand + display sweep
  - All animations are blocking (run on caller's task with yields)
  - Use `vTaskDelay()` for timing within animations
- Wire into web server POST handlers from Step 10

**ESP-IDF APIs:** `esp_random()` for chaos mode, `vTaskDelay()`

**Verify:** Trigger each animation from web UI. Motor and flip-dots operate in sync. Animations complete without watchdog timeouts.

---

## Step 13: Integration, Button Input, and Polish

**Goal:** Full feature parity with CircuitPython version (plus OTA).

**Implement:**
- Button handling: GPIO ISR + debounce for buttons A(1), B(38), C(33)
  - Button A (IO1): Reset/animation trigger
  - Button B (IO38): +1 hour / sync animation
  - Button C (IO33): +1 minute / AS5600 calibration (long-press)
- NeoPixel status LED (IO48) colors:
  - Purple: WiFi connecting
  - Green: WiFi + NTP synced
  - Yellow: WiFi connection failed
  - Cyan: WiFi OK, NTP failed
- FreeRTOS task orchestration:
  - `clock_task` — minute/hour updates via esp_timer callbacks
  - `display_task` — 1s OLED refresh loop
  - `network_task` — WiFi management + NTP + health checks
  - Web server runs on its own internal task
- Shared state protection: mutex for motor access (web API vs clock task)
- Watchdog timer configuration
- Brown-out detection: log reset reason on boot via `esp_reset_reason()`, display on OLED and in `/status.json` if last reset was brown-out (`ESP_RST_BROWNOUT`). Useful for diagnosing 24V/12V/5V mixed-power issues.
- Power-on sequence matching CircuitPython:
  1. GPIO/peripheral init
  2. RTC read
  3. Motor init + homing
  4. Relay precharge
  5. OLED startup
  6. WiFi + NTP
  7. Web server start
  8. Main loop

**ESP-IDF APIs:** `gpio` ISR, `freertos/semphr.h`, `esp_task_wdt`

**Verify:** Full end-to-end test:
- Power on → homes → syncs time → displays correct time
- Minute hand advances each minute
- Hour display updates each hour
- Web UI fully functional
- WiFi recovery works after disconnect
- Buttons respond correctly
- NVS settings persist across reboots
- Run for 24+ hours without drift or crashes
