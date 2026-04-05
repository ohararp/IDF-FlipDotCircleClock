# FlipDotCircleClock: CircuitPython → ESP-IDF 6.0 Porting Plan

## Context
The FlipDotCircleClock is a flip-dot circle clock currently running as a single ~103KB CircuitPython `code.py` on an **Unexpected Maker FeatherS3** (ESP32-S3 240MHz dual-core, 2.4GHz WiFi + BLE 5.0). The goal is to port it to ESP-IDF 6.0 C for better performance, real-time control, and FreeRTOS task management.

**Hardware controlled:**
- DS3231 RTC (I2C 0x68), SH1107 OLED 128x64 (I2C 0x3C), AS5600 magnetic encoder (I2C 0x36)
- 3-column × 4-row flip-dot matrix (12 dots, 24V via relay on IO11, bit-banged SPI control)
- TMC2209 stepper driver: MT-1701HSM140AE 0.9°/step motor, 400 base steps × 32 microsteps = **12,800 steps/rev**
- AS5600 magnetic encoder for PID closed-loop positioning (±0.4° accuracy), NeoPixel LED (IO40), 3 buttons
- WiFi with NTP, HTTP web server on port 80 (planned Steps 12-13)

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

## File Structure (current + planned)

```
FlipDotCircleClock/
├── CMakeLists.txt
├── sdkconfig.defaults
├── components/
│   └── esp_as5600/             # dddGR/esp_as5600 library (https://github.com/dddGR/esp_as5600)
│       ├── CMakeLists.txt
│       ├── esp_as5600.c
│       └── include/esp_as5600.h
├── main/
│   ├── CMakeLists.txt
│   ├── idf_component.yml      # espressif/led_strip, espressif/button, espressif/pid_ctrl, u8g2
│   ├── main.c                  # setup() + main clock loop, button handlers
│   ├── gpio_config.h           # All pin definitions + stepper constants
│   ├── neopixel.c / .h         # WS2812 NeoPixel via RMT (GPIO 40)
│   ├── ds3231.c / .h           # DS3231 RTC I2C driver (0x68)
│   ├── as5600.c / .h           # AS5600 encoder wrapper (reads ANGLE register 0x0E, not RAW_ANGLE)
│   ├── oled_display.c / .h     # SH1107 OLED via U8G2 library + scrolling terminal mode
│   ├── flipdot.c / .h          # Flip-dot bit-banged SPI + SUP/SET/RES encoding + relay power
│   ├── stepper.c / .h          # TMC2209 motor control + PID closed-loop via AS5600
│   ├── timekeeping.c / .h      # Timezone, DST definitions, clock_update_minute/hour
│   ├── nvm_storage.c / .h      # NVS-based persistent config (timezone, step delay, calibration)
│   ├── buttons.c / .h          # espressif/button component wrapper (A/B/C short+long press)
│   ├── calibration.c / .h      # AS5600 calibration enter/save/cancel with 3s cooldown
│   ├── animations.c / .h       # (Step 10) Demo, chaos, sync sequences
│   ├── network.c / .h          # WiFi STA + BLE provisioning + NTP sync + mDNS
│   ├── web_server.c / .h       # (Step 13) HTTP server + JSON API
│   └── ota_update.c / .h       # (Step 13) GitHub release OTA updates
├── partitions.csv              # (Step 13) Custom partition table
└── frontend/
    └── index.html              # (Step 13) Web UI
```

---

## Step 1: Project Skeleton + GPIO Definitions + LED Blink ✅

**Implemented:**
- Root `CMakeLists.txt`, `main/CMakeLists.txt`, `sdkconfig.defaults` (ESP32-S3, 16MB flash, 240MHz, USB CDC console)
- `gpio_config.h` — all pin definitions + stepper constants (STEPPER_STEPS_PER_REV=12800, STEPPER_DEFAULT_DELAY_US=450)
- `neopixel.c/.h` — modular WS2812 driver on GPIO 40 via `espressif/led_strip` + RMT peripheral
  - `neopixel_init()`, `neopixel_set_color(r,g,b)`, `neopixel_off()`
- `main.c` — blinks NeoPixel purple at 1 Hz

**Dependencies:** `espressif/led_strip: "^3.0.0"`

---

## Step 2: I2C Bus + DS3231 RTC Driver ✅

**Implemented:**
- `ds3231.c/.h` — I2C driver for DS3231 (address 0x68)
  - `ds3231_init(i2c_master_bus_handle_t *ret_bus_handle)` — creates shared I2C master bus (SDA=8, SCL=9, 400kHz), returns handle for OLED/AS5600
  - `ds3231_get_time(struct tm *time)` — burst-read 7 BCD registers, convert to struct tm
  - `ds3231_set_time(const struct tm *time)` — write BCD registers
- Compile-time RTC seeding: `__DATE__`/`__TIME__` parsed in `main.c` (not ds3231.c) so it recompiles every build
- **RTC convention:** stores local time directly (not UTC). Timezone offset deferred to Step 12 when NTP is added.

**ESP-IDF APIs:** `i2c_master` (new driver), `esp_log`

---

## Step 3: OLED Display (SH1107) ✅

**Implemented:**
- `oled_display.c/.h` — SH1107 128x64 OLED via **U8G2 library** (not esp_lcd/LVGL)
  - `oled_init(i2c_master_bus_handle_t bus)` — U8G2 setup with `u8g2_Setup_sh1107_i2c_64x128_f()`, 90° rotation to 128x64 landscape
  - `oled_update_time(const struct tm *time)` — large HH:MM:SS (14px font) + date (8px font)
  - `oled_update_status(const char *ip, int rssi, uint32_t uptime, int motor_pos)` — 3-line status
  - `oled_terminal_print(const char *line)` — scrolling 8-line terminal (5x7 monospace font) for homing/debug output
  - `oled_clear()`
- I2C byte callback + GPIO/delay callback for U8G2 ↔ ESP-IDF I2C bridge
- Display task shows live AS5600 angle during calibration mode

**Dependencies:** `u8g2` (git: olikraus/u8g2)

---

## Step 4: NVS Storage Module ✅

**Implemented:**
- `nvm_storage.c/.h` — NVS wrapper (namespace: "flipclock")
  - `nvm_init()` — init NVS flash, open namespace, write defaults on first boot (magic byte 0xFC)
  - `nvm_get/set_timezone_index()` — uint8_t (0–17)
  - `nvm_get/set_home_offset()` — int16_t (signed step offset)
  - `nvm_get/set_step_delay()` — uint16_t (microseconds)
  - `nvm_get/set_as5600_cal_angle()` — uint16_t (12-bit raw angle)
  - `nvm_factory_reset()` — erase all keys, rewrite defaults
- Defaults: timezone=0 (US/Eastern), step_delay=450µs, home_offset=0, as5600_cal=0
- Each setter commits to flash immediately via `nvs_commit()`

**ESP-IDF APIs:** `nvs_flash`, `nvs`

---

## Step 5: Stepper Motor Control ✅

**Implemented:**
- `stepper.c/.h` — TMC2209 driver:
  - `stepper_init()` — configure GPIOs (EN=6, STEP=12, DIR=5, MS=17), 32x microstepping, load step delay from NVS
  - `stepper_enable() / stepper_disable()` — EN pin LOW/HIGH for motor torque control
  - `stepper_multi_step(int steps, bool clockwise)` — pulse generation with 2µs high-time + configurable delay
  - `stepper_find_home()` — open-loop CW to step position 0, then PID fine-tune to calibrated AS5600 angle
  - `stepper_get_position() / stepper_set_position()` — software step counter (0–12799, wraps)
  - `stepper_move_to_angle(uint16_t target_raw, int tolerance)` — PID closed-loop positioning via AS5600
- **Hall sensor removed** from code (PIN_HALL_SENSOR still defined in gpio_config.h). Homing uses AS5600 calibration.
- PID controller: `espressif/pid_ctrl` v0.2.0, positional mode, Kp=0.5, Kd=0.1, CW-only, ±5 AS5600 unit tolerance (±0.4°)
- `home_log()` helper prints to both serial and OLED terminal during homing

**ESP-IDF APIs:** `gpio`, `esp_rom_delay_us`, `pid_ctrl`
**Dependencies:** `espressif/pid_ctrl: "^0.2.0"`

---

## Step 6: AS5600 Magnetic Encoder + Calibration ✅

**Implemented:**
- `as5600.c/.h` — wrapper around `dddGR/esp_as5600` local component:
  - `as5600_setup(i2c_master_bus_handle_t bus)` — attach to shared I2C bus at 0x36, log magnet status/AGC/magnitude
  - `as5600_read_raw_angle()` — reads **ANGLE register (0x0E:0x0F)**, not RAW_ANGLE (0x0C:0x0D which only gives 0–2048 due to start/stop programming)
  - `as5600_is_connected()`, `as5600_to_degrees()`, `as5600_angle_diff()`, `as5600_to_steps()`, `as5600_minute_to_raw()`
  - `as5600_debug_dump()` — reads all angle registers + status for diagnostics
- `calibration.c/.h`:
  - `calibration_enter()` — motor stays **enabled** (user positions hand against holding torque), 3s minimum before save accepted
  - `calibration_save()` — read AS5600 angle, save to NVS as 12 o'clock reference
  - `calibration_cancel()`, `calibration_get_offset()`, `calibration_ready_to_save()`, `calibration_is_active()`
  - Display task shows live AS5600 angle on OLED during calibration
- **Critical lesson:** AS5600 RAW_ANGLE register gives half-range (0–2048) on this hardware. ANGLE register gives full 0–4095.

**Local component:** `components/esp_as5600/` (https://github.com/dddGR/esp_as5600)

---

## Step 7: Flip-Dot Display (Bit-Banged SPI + Relay) ✅

**Implemented:**
- `flipdot.c/.h` — bit-banged SPI (not `spi_master` driver) with SUP/SET/RES encoding:
  - `flipdot_init()` — configure GPIO outputs: SCK(36), MOSI(35), LATCH(37), OE(18), RELAY(11)
  - `flipdot_set_pattern(const uint8_t cols[3])` — encode 4-bit dot patterns into 12-bit shift register words (3 bits per dot: SUP=enable, SET=on, RES=off), column staggering (100ms between columns)
  - `flipdot_power_on()` / `flipdot_power_off()` — relay with 200ms precharge delay
  - `flipdot_extend_power_window()` / `flipdot_service_power_window()` — 80ms relay hold timer
  - `flipdot_show_hour(int hour)` — lookup table mapping hours 1–12 to 3-column bit patterns
  - `flipdot_blank()` / `flipdot_all_on()`
  - `shift_data()` mirrors CircuitPython `shiftData()` exactly: OE enable → shift 3×16-bit data → latch → 5ms settle → shift zeros → latch → OE disable
- Column order reversed to match CircuitPython: `colData = [dataIn[2], dataIn[1], dataIn[0]]`
- XOR optimization: only actuate dots that changed state (tracked via `s_old_cols[]` cache)

**ESP-IDF APIs:** `gpio`, `esp_timer`

---

## Step 8: Timekeeping — Timezone, DST, Clock Update Logic ✅

**Implemented:**
- `timekeeping.c/.h`:
  - `timekeeping_init()` — load timezone index from NVS
  - `timekeeping_get_local_time(struct tm *time)` — reads RTC directly (pre-NTP: RTC stores local time). Timezone/DST offset code exists but is bypassed until NTP is added in Step 12.
  - `timekeeping_set_timezone(int tz_index)` — update NVS
  - DST rules implemented: `is_dst_us()`, `is_dst_eu()`, `is_dst_au()`, `is_dst_nz()` with `nth_weekday()` helper
  - `timekeeping_sync_from_ntp(time_t utc_epoch)` — stub, writes UTC to RTC
  - 18 timezone definitions as `const timezone_def_t[]` with key, display name, UTC offset, DST rule
- `clock_update_minute()` — PID closed-loop only (no open-loop bulk move): calls `stepper_move_to_angle()` with AS5600 target. Falls back to open-loop if uncalibrated.
- `clock_update_hour()` — relay on → blank → show hour (12h format) → relay off
- `main.c` clock loop: RTC comparison-based (`min_old`, `hr_old` trackers)
  - Every second: display task updates OLED (separate FreeRTOS task)
  - Every minute: `clock_update_minute()` via PID
  - Every hour: `clock_update_hour()` + `stepper_find_home()` + `clock_update_minute()`
  - Clock updates skipped during calibration mode
- `setup()` function: single clean startup sequence (blank → home → show hour → set minute), pre-sets trackers to avoid duplicate first update

**ESP-IDF APIs:** `time.h` (POSIX)

---

## Step 9: Full Button Input + Calibration ✅

**Implemented:**
- `buttons.c/.h` — uses **`espressif/button` v4.1.6** component (not custom ISR):
  - `buttons_init()` — creates 3 GPIO buttons with `iot_button_new_gpio_device()`, registers `BUTTON_SINGLE_CLICK` and `BUTTON_LONG_PRESS_START` callbacks
  - 2.0s long-press threshold — fires while button is still held (no release needed)
  - Events posted to FreeRTOS queue: `BUTTON_EVENT_A_SHORT/LONG`, `BUTTON_EVENT_B_SHORT/LONG`, `BUTTON_EVENT_C_SHORT/LONG`
- Button A short: home minute hand to 12 o'clock and stay
- Button A long: AS5600 live debug mode (motor off, continuous angle readout, press C to exit)
- Button B short: +1 hour (write RTC, update flipdots)
- Button B long: animation placeholder (Step 10)
- Button C short: +1 minute (write RTC, PID move to correct position)
- Button C long: enter/confirm AS5600 calibration (3s cooldown between enter and save)
- Hardware mutex: `xSemaphoreCreateMutex()` protects stepper + flipdot. All button handlers suspend display task + acquire mutex before acting.
- Display task suspended during all button actions to prevent OLED conflicts

**Dependencies:** `espressif/button: "^4.1.6"`

---

## Step 10: Animations ✅

**Implemented:**
- `animations.c/.h` — sync animation only (demo and chaos removed):
  - `anim_sync()` — home → blank flipdots → sweep hand to each hour (1–12) using PID `stepper_move_to_angle()` with matching flipdot display (1s per hour) → restore current time
  - Blocking — runs on caller's task with `vTaskDelay()`
  - Display task suspended + hardware mutex acquired by caller in main.c
  - `restore_time()` helper: homes motor, sets flipdot hour, moves minute hand to current time
- Button A short triggers `anim_sync()`
- Button A long and B long: placeholder (unassigned, print message)
- **Motor control improvements (also in this step):**
  - `multi_step_ramped()`: trapezoidal speed profile (accel 20% → cruise 60% → decel 20%) with linear delay interpolation
  - `stepper_move_to_angle()`: non-linear proportional control with ramped stepping. Far=600→150→400µs, medium=500→250→400µs, close=500→350→500µs
  - PID tolerance tightened to ±1 AS5600 unit (±0.09°)
  - `stepper_find_home()`: PID-only (no open-loop pre-move that caused overshoot)

**ESP-IDF APIs:** `vTaskDelay()`

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

## Step 12: WiFi + BLE Provisioning + NTP Sync ✅

**Implemented:**
- `network.c/.h` — WiFi STA + BLE provisioning + NTP:
  - `network_init()` — init WiFi stack, check NVS for provisioning request flag or stored credentials:
    - Provisioning requested (Button A long → reboot): erase old creds, start BLE provisioning with QR on OLED
    - Credentials stored: connect WiFi in background (non-blocking), start NTP wait task
    - No credentials, no request: skip WiFi, show "No WiFi — hold A to setup"
  - `network_start_provisioning()` — start BLE provisioning via `espressif/network_provisioning` component
    - Device name: "FlipClk_XXXX" (MAC-based), proof-of-possession: "flipdot"
    - QR code displayed on OLED via `espressif/qrcode` component for ESP BLE Prov app scanning
    - Security 1 (Curve25519 + AES256-CTR)
  - `network_get_state()` — PROVISIONING / CONNECTING / CONNECTED / DISCONNECTED / OFFLINE
  - `network_get_ip_str()`, `network_get_rssi()`, `network_ntp_synced()`
  - `network_reset_provisioning()` — sets NVS prov request flag, reboots
  - `network_stop_provisioning()` — cleanly stop BLE and free resources
  - NTP via `esp_sntp`: sync on WiFi connect, hourly re-sync timer, writes UTC to RTC
  - mDNS: `flipclock.local` registered after WiFi connects
- `oled_display.c` — `oled_show_qr()`: QR code rendered on left 64px with label text on right (POP, cancel instructions)
- `timekeeping.c` — dual-mode RTC:
  - Before NTP sync: RTC stores local time (compile timestamp), read directly
  - After NTP sync: RTC stores UTC, `timekeeping_get_local_time()` applies timezone + DST offset
  - `timekeeping_mark_ntp_synced()` flag switches between modes
- **Boot UX:** clock always starts immediately. WiFi is optional and on-demand:
  - Button A long: sets NVS flag → reboots → BLE provisioning starts with QR on OLED
  - Button C (raw GPIO poll): cancel provisioning during BLE mode, resume clock
  - After provisioning: reboots to connect WiFi + sync NTP
- NeoPixel 1Hz blink: green=WiFi+NTP, cyan=WiFi OK NTP pending, yellow=offline, purple=provisioning
- Custom `partitions.csv`: 3MB app partition (BLE+WiFi firmware >1MB)
- `sdkconfig.defaults`: BLE NimBLE enabled, Security 1 protocomm
- Default timezone: US/Eastern (index 6)
- I2C retry logic (3 attempts, 500ms timeout) for BLE/I2C coexistence
- Long press threshold: 1 second (was 2s)

**Dependencies:** `espressif/network_provisioning: "^1.2.2"`, `espressif/mdns: "^1.4.0"`, `espressif/qrcode: "^0.2.0"`

**Known limitation:** BLE provisioning can only start on a clean boot (Button A long triggers reboot). Cannot start BLE after WiFi stack is initialized — BLE memory is released after prov manager deinit.

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
