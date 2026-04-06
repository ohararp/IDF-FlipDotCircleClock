# FlipDotCircleClock

**A flip-dot circle clock powered by ESP-IDF 6.0 — fast boot, real-time motor control, and wireless everything.**

A stepper-driven minute hand sweeps around a magnetic-encoder-tracked dial while a 3x4 flip-dot matrix displays the hour. Originally written in CircuitPython, now ported to ESP-IDF 6.0 C with dual-core FreeRTOS for the performance and reliability a clock deserves.

---

## Why ESP-IDF?

The CircuitPython version worked, but it had limits. The ESP-IDF port is a different beast:

| | CircuitPython | ESP-IDF 6.0 |
|---|---|---|
| **Boot to running clock** | ~8-10 seconds | Under 2 seconds |
| **WiFi connect** | Slow, blocking | Background, non-blocking |
| **Motor control** | Open-loop, no feedback | PID closed-loop, ±0.09° accuracy |
| **Concurrency** | Single-threaded | Dual-core FreeRTOS |
| **Web interface** | None | Full dashboard + OTA updates |
| **WiFi setup** | Hardcoded credentials | BLE provisioning via phone app |
| **Firmware updates** | USB cable required | Upload .bin from any browser |

---

## Features

### Clock Hardware
- **Minute hand:** TMC2209 stepper driver, 0.9°/step motor, 32x microstepping (12,800 steps/rev)
- **Hour display:** 3-column x 4-row flip-dot matrix (12 dots, 24V relay-switched, bit-banged SPI)
- **Positioning:** AS5600 magnetic encoder with PID closed-loop control — ±0.09° accuracy (±1 encoder unit)
- **Timekeeping:** DS3231 RTC with battery backup, automatic NTP sync when WiFi is available
- **Display:** SH1107 128x64 OLED showing time, date, network status, firmware version
- **Status LED:** NeoPixel with 1Hz blink — green (online), cyan (WiFi OK, NTP pending), yellow (offline), purple (provisioning)

### Connectivity
- **WiFi:** Background STA connection with exponential backoff and automatic reconnect
- **BLE Provisioning:** Set up WiFi from your phone using the ESP BLE Prov app — scan a QR code on the OLED, done
- **NTP:** Automatic time sync on WiFi connect, hourly re-sync, 18 timezone definitions with DST rules (US/EU/AU/NZ)
- **mDNS:** Access the clock at `http://flipclock.local`

### Web Dashboard
A dark-themed, responsive single-page web app served directly from the ESP32:

- **Live status:** Time, date, WiFi/NTP indicators, RSSI, uptime, memory usage, firmware version
- **Clock controls:** +1 Hour, +1 Minute, Home, Sync Animation, NTP Sync
- **Motor tuning:** Live AS5600 angle readout, adjustable step speed (100-1000 us)
- **AS5600 calibration:** Start/Save/Cancel with live angle feedback
- **Timezone selector:** 18 timezones grouped by region, auto-applied on change
- **OTA firmware update:** Upload a .bin file from any browser — no USB cable needed
- **Action log:** Scrollable event log with timestamps, auto-refreshing every 5 seconds
- **Responsive layout:** Two-column on desktop, single-column on mobile

### Precision Motor Control
- **Trapezoidal speed ramping:** Smooth acceleration/cruise/deceleration profiles
- **Non-linear proportional control:** Variable speed based on distance to target
- **PID homing:** Finds 12 o'clock position using calibrated AS5600 reference angle
- **XOR optimization:** Flip-dots only actuate dots that actually changed state

---

## Hardware

| Component | Details |
|---|---|
| **MCU** | Unexpected Maker FeatherS3 (ESP32-S3, 240 MHz dual-core, 16 MB flash, 8 MB PSRAM) |
| **Stepper** | TMC2209 driver + MT-1701HSM140AE motor (0.9°/step, 32x microstepping) |
| **Encoder** | AS5600 magnetic encoder (I2C 0x36, 12-bit, ANGLE register 0x0E) |
| **RTC** | DS3231 (I2C 0x68, battery-backed) |
| **OLED** | SH1107 128x64 (I2C 0x3C, U8G2 library) |
| **Flip-dots** | 3x4 matrix, 24V relay-switched, bit-banged SPI (SUP/SET/RES encoding) |
| **LED** | WS2812 NeoPixel on GPIO 40 (via RMT peripheral) |
| **Buttons** | 3x GPIO with espressif/button component (short + long press) |

---

## Architecture

The ESP32-S3's dual cores are used strategically:

**Core 0 — Network:** WiFi/LWIP stack, HTTP web server, BLE provisioning

**Core 1 — Real-Time:** OLED display updates, NeoPixel status, motor control (via main loop + hardware mutex)

Cross-core communication uses a FreeRTOS command queue — web API handlers on Core 0 post commands that the main loop on Core 1 executes with proper hardware mutex protection.

---

## Getting Started

Requires [ESP-IDF v6.0](https://docs.espressif.com/projects/esp-idf/en/v6.0/).

```bash
# Clone
git clone https://github.com/ohararp/IDF-FlipDotCircleClock.git
cd IDF-FlipDotCircleClock

# Build
source $IDF_PATH/export.sh
idf.py build

# Flash (first time — erases flash for new OTA partition table)
idf.py -p /dev/cu.usbmodem1101 erase-flash
idf.py -p /dev/cu.usbmodem1101 flash monitor
```

After the first flash, all subsequent updates can be done wirelessly via the web UI.

---

## OTA Firmware Updates

No more plugging in USB cables to update firmware:

1. Build your new firmware: `idf.py build`
2. Open `http://flipclock.local` in any browser
3. Scroll to **Firmware Update**, select `build/FlipDotCircleClock.bin`
4. Click **Upload** — the OLED shows progress (`OTA: 10%`... `OTA: 90%`... `OTA: rebooting...`)
5. Clock reboots on the new firmware in seconds

The partition table has two 3MB OTA slots that alternate — if an update fails, the previous firmware is still there.

---

## BLE WiFi Provisioning

No hardcoded WiFi credentials. Set up WiFi from your phone:

1. Long-press **Button A** (1 second) — clock reboots into provisioning mode
2. OLED displays a QR code
3. Open the **ESP BLE Prov** app ([Android](https://play.google.com/store/apps/details?id=com.espressif.provble) / [iOS](https://apps.apple.com/app/esp-ble-provisioning/id1473590141))
4. Scan the QR code, enter your WiFi password
5. Clock reboots, connects to WiFi, syncs time via NTP

Proof-of-possession: `flipdot` | Security: Curve25519 + AES256-CTR

---

## Pin Assignments

| Function | GPIO | Notes |
|---|---|---|
| Stepper EN | 6 | Active low |
| Stepper STEP | 12 | Pulse output |
| Stepper DIR | 5 | Direction |
| Stepper MS | 17 | HIGH = 32x microstepping |
| Flip-dot SCK | 36 | Bit-banged SPI clock |
| Flip-dot MOSI | 35 | Bit-banged SPI data |
| Flip-dot LATCH | 37 | Chip select |
| Flip-dot OE | 18 | Output enable |
| Relay (24V) | 11 | Flip-dot power |
| I2C SDA | 8 | Shared: RTC, OLED, AS5600 |
| I2C SCL | 9 | 400 kHz |
| Button A | 1 | Home / Animations / BLE Prov |
| Button B | 38 | +1 Hour |
| Button C | 33 | +1 Minute / Calibration |
| NeoPixel | 40 | WS2812 via RMT |

---

## Porting Progress

- [x] **Step 1:** Project skeleton, GPIO definitions, NeoPixel blink
- [x] **Step 2:** I2C bus + DS3231 RTC driver
- [x] **Step 3:** OLED display (SH1107) via U8G2
- [x] **Step 4:** NVS persistent storage
- [x] **Step 5:** Stepper motor control (TMC2209, 32x microstepping)
- [x] **Step 6:** AS5600 magnetic encoder + PID closed-loop calibration
- [x] **Step 7:** Flip-dot display (bit-banged SPI + relay)
- [x] **Step 8:** Timekeeping (18 timezones, DST rules, minute/hour scheduling)
- [x] **Step 9:** Full button input + calibration flow
- [x] **Step 10:** Animations + trapezoidal speed ramping
- [x] **Step 12:** WiFi + BLE provisioning + NTP sync
- [x] **Step 13:** Web server + JSON API + action log + OTA firmware updates

---

## License

MIT
