#pragma once

// =============================================================================
// FlipDotCircleClock GPIO Pin Definitions
// Board: Unexpected Maker FeatherS3 (ESP32-S3)
// =============================================================================

// --- Stepper Motor (TMC2209) ---
#define PIN_STEPPER_EN      6    // Enable (active low)
#define PIN_STEPPER_STEP    12   // Step pulse
#define PIN_STEPPER_DIR     5    // Direction
#define PIN_STEPPER_MS      17   // Microstepping (HIGH = 32x on TMC2209)

// --- Hall Effect Sensor (A3144) ---
#define PIN_HALL_SENSOR     14   // Home position detection

// --- Flip-Dot Display (SPI) ---
#define PIN_FLIPDOT_SCK     36   // SPI Clock
#define PIN_FLIPDOT_MOSI    35   // SPI Data
#define PIN_FLIPDOT_LATCH   37   // Latch / CS
#define PIN_FLIPDOT_OE      18   // Output Enable

// --- 24V Relay ---
#define PIN_RELAY           11   // Flip-dot power relay

// --- I2C Bus ---
#define PIN_I2C_SDA         8
#define PIN_I2C_SCL         9

// --- Buttons ---
#define PIN_BUTTON_A        1    // Reset / animation
#define PIN_BUTTON_B        38   // +1 hour / sync animation
#define PIN_BUTTON_C        33   // +1 minute / AS5600 calibration

// --- NeoPixel Status LED ---
#define PIN_NEOPIXEL        40

// --- Stepper Motor Constants ---
#define STEPPER_BASE_STEPS      400    // 0.9° per step
#define STEPPER_MICROSTEPS      32     // TMC2209 32x microstepping
#define STEPPER_STEPS_PER_REV   (STEPPER_BASE_STEPS * STEPPER_MICROSTEPS)  // 12800
#define STEPPER_DEFAULT_DELAY_US 450   // Default step pulse delay in microseconds
