/**
 * imu.cpp — BNO08x UART-RVC driver for ESP32 WROOM-32E
 *
 * Mode : UART-RVC (Robot Vacuum Cleaner) — simplest BNO08x output mode
 * Bus  : HardwareSerial1 on GPIO 25 (RX only — one wire from BNO08x TX)
 * Rate : 100 Hz fixed, 115200 baud fixed
 *
 * No library required. Parses the raw 19-byte UART-RVC packet directly.
 *
 * Hardware setup:
 *   GY-BNO08X PS0 jumper → SOLDERED CLOSED  (sets UART-RVC mode)
 *   GY-BNO08X PS1 jumper → OPEN             (must be unsoldered)
 *   BNO08x TX pin        → ESP32 GPIO 25
 *
 * UART-RVC packet (19 bytes, 100 Hz):
 *   [0]    0xAA  — header byte 1
 *   [1]    0xAA  — header byte 2
 *   [2-3]  Yaw       int16 LE  1/100 deg
 *   [4-5]  Pitch     int16 LE  1/100 deg
 *   [6-7]  Roll      int16 LE  1/100 deg
 *   [8-9]  Accel X   int16 LE  1/1000 g
 *   [10-11]Accel Y   int16 LE  1/1000 g
 *   [12-13]Accel Z   int16 LE  1/1000 g
 *   [14-18]reserved / status
 *
 * Acceleration units from chip: 1/1000 g (millig)
 * We convert to m/s² and subtract 1g from accel_y to remove gravity.
 */

#include "imu.h"
#include <Arduino.h>
#include <math.h>

// ─── Constants ────────────────────────────────────────────────────────────────

// Gravity constant for unit conversion (m/s²)
static constexpr float kGravity = 9.80665f;

// UART-RVC packet size and header
static constexpr uint8_t  kPacketLen    = 19;
static constexpr uint8_t  kHeader1      = 0xAA;
static constexpr uint8_t  kHeader2      = 0xAA;

// ─── Internal state ───────────────────────────────────────────────────────────

static bool _ready = false;

// Raw byte buffer — we accumulate bytes until we have a full packet
static uint8_t  _buf[kPacketLen];
static uint8_t  _bufIdx = 0;       // how many bytes collected so far
static bool     _synced = false;   // true once we've found a 0xAA 0xAA header

// ─── Helpers ─────────────────────────────────────────────────────────────────

/**
 * Read a signed 16-bit integer from two bytes, little-endian.
 * buf[offset] = low byte, buf[offset+1] = high byte.
 */
static int16_t _read_i16(const uint8_t *buf, uint8_t offset)
{
    return (int16_t)((uint16_t)buf[offset] | ((uint16_t)buf[offset + 1] << 8));
}

/**
 * Compute tilt angle from vertical using pitch and roll.
 *
 * When the rocket is perfectly vertical, pitch = 0 and roll = 0.
 * We use the proper dot-product formula so it stays accurate at
 * large angles (not just a simple addition of pitch + roll).
 *
 *   cos(tilt) = cos(pitch) * cos(roll)
 *   tilt = acos( cos(pitch) * cos(roll) )
 */
static float _compute_tilt(float pitch_deg, float roll_deg)
{
    float pitch_rad = pitch_deg * DEG_TO_RAD;
    float roll_rad  = roll_deg  * DEG_TO_RAD;
    float cos_tilt  = cosf(pitch_rad) * cosf(roll_rad);
    cos_tilt = constrain(cos_tilt, -1.0f, 1.0f);   // clamp for acos safety
    return acosf(cos_tilt) * RAD_TO_DEG;
}

// ─── Public implementation ────────────────────────────────────────────────────

bool imuInit()
{
    // UART-RVC uses HardwareSerial1.
    // GPIO 25 = RX (receives BNO08x TX stream)
    // GPIO 26 = TX (unused — BNO08x doesn't need commands in RVC mode)
    Serial1.begin(IMU_BAUD_RATE, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);

    // Flush any garbage bytes that arrived during power-up
    delay(200);
    while (Serial1.available()) Serial1.read();

    _bufIdx = 0;
    _synced = false;
    _ready  = true;

    Serial.printf("[IMU] BNO08x UART-RVC ready (RX=GPIO%d, baud=%d)\n",
                  IMU_RX_PIN, IMU_BAUD_RATE);
    Serial.println("[IMU] Verify: PS0 soldered, PS1 open on GY-BNO08X board");
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────

bool imuRead(ImuSample &sample)
{
    if (!_ready) return false;

    // Process all bytes currently in the UART receive buffer.
    // We don't block — if a full packet isn't here yet, return false
    // and the caller will try again next loop iteration.
    while (Serial1.available()) {
        uint8_t byte = (uint8_t)Serial1.read();

        if (!_synced) {
            // ── Header sync ──────────────────────────────────────────────────
            // Scan incoming bytes until we see 0xAA followed by 0xAA.
            // This aligns us to packet boundaries after power-on or glitches.
            if (_bufIdx == 0 && byte == kHeader1) {
                _buf[_bufIdx++] = byte;   // got first header byte
            } else if (_bufIdx == 1 && byte == kHeader2) {
                _buf[_bufIdx++] = byte;   // got second header byte — synced!
                _synced = true;
            } else {
                // Not a valid header sequence — reset and keep scanning
                _bufIdx = 0;
            }
        } else {
            // ── Collect packet bytes ─────────────────────────────────────────
            // We already have the two header bytes; collect the remaining
            // (kPacketLen - 2) data bytes.
            _buf[_bufIdx++] = byte;

            if (_bufIdx == kPacketLen) {
                // ── Full packet received — parse it ───────────────────────────
                // Reset for next packet
                _bufIdx = 0;
                _synced = false;

                // ── Parse Euler angles ────────────────────────────────────────
                // Raw values are in 1/100 degree units — divide by 100.0
                // Bytes 2-3: Yaw, 4-5: Pitch, 6-7: Roll
                sample.yaw   = _read_i16(_buf, 2) / 100.0f;   // degrees
                sample.pitch = _read_i16(_buf, 4) / 100.0f;   // degrees
                sample.roll  = _read_i16(_buf, 6) / 100.0f;   // degrees

                // ── Parse acceleration ────────────────────────────────────────
                // Raw values are in 1/1000 g units — convert to m/s²
                // Bytes 8-9: X, 10-11: Y, 12-13: Z
                float raw_ax = _read_i16(_buf,  8) / 1000.0f * kGravity;
                float raw_ay = _read_i16(_buf, 10) / 1000.0f * kGravity;
                float raw_az = _read_i16(_buf, 12) / 1000.0f * kGravity;

                // ── Remove gravity from vertical axis ─────────────────────────
                // UART-RVC accel INCLUDES gravity. When the rocket is vertical
                // and stationary, accel_y reads ~+9.81 m/s² (1g, nose up).
                // We subtract 1g so that at rest accel_y ≈ 0 m/s².
                // This matches the Kalman filter and liftoff detection expectations.
                //
                // If your board is mounted with Y pointing TAIL-ward (downward),
                // change this to: raw_ay + kGravity
                sample.accel_x = raw_ax;
                sample.accel_y = raw_ay - kGravity;   // gravity removed --- IF IT READS 9.81 THEN DO + GRAVITY
                sample.accel_z = raw_az;

                // ── Tilt from vertical ────────────────────────────────────────
                sample.tilt_deg = _compute_tilt(sample.pitch, sample.roll);

                return true;   // valid packet parsed
            }
        }
    }

    // No complete packet available yet — caller should try next loop
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────

bool imuReady()
{
    return _ready;
}