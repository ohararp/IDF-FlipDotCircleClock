# IDF-FlipDotCircleClock

ESP-IDF 6.0 (C/FreeRTOS) port of the [FlipDotCircleClock](https://github.com/ohararp/FlipDotCircleClock) — a flip-dot circle clock with a stepper-driven minute hand, originally written in CircuitPython.

## Hardware

- **MCU:** Unexpected Maker FeatherS3 (ESP32-S3, 240 MHz dual-core, 16 MB flash)
- **Hour display:** 3-column x 4-row flip-dot matrix (12 dots, 24V relay-switched, SPI-controlled)
- **Minute hand:** TMC2209 stepper driver, 0.9deg/step motor, 32x microstepping (12,800 steps/rev)
- **Homing:** A3144 Hall effect sensor
- **Peripherals:** DS3231 RTC (I2C), SH1107 128x64 OLED (I2C), AS5600 magnetic encoder (I2C, optional)
- **Interface:** WiFi + NTP, HTTP web server, 3 buttons, NeoPixel status LED

## Building

Requires [ESP-IDF v6.0](https://docs.espressif.com/projects/esp-idf/en/v6.0/).

```bash
# Source ESP-IDF environment
source $IDF_PATH/export.sh

# Build
idf.py build

# Flash and monitor (adjust port as needed)
idf.py -p /dev/cu.usbmodem1101 flash monitor
```

## Porting Progress

- [x] **Step 1:** Project skeleton, GPIO definitions, NeoPixel blink
- [x] **Step 2:** I2C bus + DS3231 RTC driver (compile-time seeding on each flash)
- [x] **Step 3:** OLED display (SH1107) + scrolling terminal mode
- [x] **Step 4:** NVS persistent storage
- [x] **Step 5:** Stepper motor control + Hall sensor homing + Button C re-trigger
- [x] **Step 6:** AS5600 magnetic encoder + closed-loop control + calibration
- [x] **Step 7:** Flip-dot display (SPI + relay) + Button B test trigger
- [x] **Step 8:** Timekeeping (timezone, DST, minute/hour update scheduling)
- [x] **Step 9:** Full button input (A/B/C short+long press, calibration, mutex, AS5600-only homing)
- [ ] **Step 10:** Animations (demo, chaos, sync)
- [ ] **Step 11:** Integration + polish (standalone soak test, watchdog, brown-out)
- [ ] **Step 12:** WiFi + NTP synchronization
- [ ] **Step 13:** HTTP web server + JSON API + OTA updates

## License

MIT
