#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ─── I2C Pins (ESP32 WROOM-32E defaults) ────────────────────────────────────
// GPIO 18/19: shared with BMP280 on Wire1 (different I2C addresses: 0x40 vs 0x76)
#define SERVO_SDA_PIN     18
#define SERVO_SCL_PIN     19
#define PCA9685_I2C_ADDR  0x40   // default; change if A0-A5 pins are bridged

// ─── PWM / Servo Timing ──────────────────────────────────────────────────────
#define SERVO_FREQ_HZ     50     // Standard servo frequency
#define SERVO_MIN_US      500    // Pulse width for 0°   (µs)
#define SERVO_MAX_US      2500   // Pulse width for 180° (µs)

// ─── Channel Count ───────────────────────────────────────────────────────────
#define SERVO_CHANNEL_COUNT 16

// ─── Public API ──────────────────────────────────────────────────────────────

/**
 * Initialise I2C and the PCA9685.
 * Call once from setup().
 *
 * @param sda  SDA GPIO  (default SERVO_SDA_PIN)
 * @param scl  SCL GPIO  (default SERVO_SCL_PIN)
 * @return true on success, false if the PCA9685 is not found on the bus.
 */
bool servo_init(uint8_t sda = SERVO_SDA_PIN, uint8_t scl = SERVO_SCL_PIN);

/**
 * Move a servo to an absolute angle.
 *
 * @param channel  PCA9685 channel (0–15)
 * @param degrees  Target angle    (0–180)
 */
void servo_set_angle(uint8_t channel, float degrees);

/**
 * Set a servo via a raw pulse width.
 *
 * @param channel   PCA9685 channel (0–15)
 * @param pulse_us  Pulse width in microseconds (clamped to SERVO_MIN_US … SERVO_MAX_US)
 */
void servo_set_pulse_us(uint8_t channel, uint16_t pulse_us);

/**
 * Release a channel (set PWM to 0 – servo goes limp / holds last position
 * depending on the servo model).
 *
 * @param channel  PCA9685 channel (0–15)
 */
void servo_release(uint8_t channel);

/**
 * Release all 16 channels at once.
 */
void servo_release_all();

/**
 * Smoothly move a servo from its current angle to a target angle.
 *
 * @param channel     PCA9685 channel (0–15)
 * @param from_deg    Starting angle  (0–180)
 * @param to_deg      Target angle    (0–180)
 * @param duration_ms Total travel time in milliseconds
 * @param steps       Number of intermediate steps (default 50)
 */
void servo_sweep(uint8_t channel,
                 float   from_deg,
                 float   to_deg,
                 uint32_t duration_ms,
                 uint16_t steps = 50);