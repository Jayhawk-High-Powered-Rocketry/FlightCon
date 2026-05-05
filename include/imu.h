#pragma once

#include <stdint.h>
#include <stdbool.h>

// ─── UART Pins ───────────────────────────────────────────────────────────────
// BNO08x UART-RVC mode — receive only (ESP32 only needs to listen)
//
// Wiring:
//   BNO08x TX  →  ESP32 GPIO 25 (RX)
//   BNO08x GND →  ESP32 GND
//   BNO08x VCC →  ESP32 3.3V
//   GPIO 26 is unused — UART-RVC is one-wire receive only
//
// PS0 jumper must be SOLDERED CLOSED on the GY-BNO08X board.
// PS1 jumper must be OPEN (unsoldered).
// This puts the chip into UART-RVC mode at boot.

#define IMU_RX_PIN    25      // ESP32 receives from BNO08x TX
#define IMU_TX_PIN    26      // unused — required by HardwareSerial API only
#define IMU_BAUD_RATE 115200  // UART-RVC fixed baud rate — do not change

// ─── UART-RVC Packet Format ──────────────────────────────────────────────────
// The BNO08x streams 19-byte packets at 100 Hz in UART-RVC mode:
//
//   Byte  0:    0xAA  (header byte 1)
//   Byte  1:    0xAA  (header byte 2)
//   Byte  2-3:  Yaw       (int16 little-endian, units = 1/100 degree)
//   Byte  4-5:  Pitch     (int16 little-endian, units = 1/100 degree)
//   Byte  6-7:  Roll      (int16 little-endian, units = 1/100 degree)
//   Byte  8-9:  Accel X   (int16 little-endian, units = 1/1000 g)
//   Byte 10-11: Accel Y   (int16 little-endian, units = 1/1000 g)
//   Byte 12-13: Accel Z   (int16 little-endian, units = 1/1000 g)
//   Byte 14-18: reserved / status
//
// Yaw: 0–360°   Pitch: ±180°   Roll: ±180°
// Acceleration includes gravity — subtract 1g from vertical axis at rest.

// ─── IMU Mounting ─────────────────────────────────────────────────────────────
// Mount the GY-BNO08X so that:
//   Y axis arrow on the PCB silkscreen points toward the NOSE of the rocket
//   X axis arrow points along the airbrake deployment axis
//   Z axis points perpendicular to both (out the side of the rocket)
//
// When the rocket is vertical on the pad:
//   accel_y ≈ +1g  (gravity along Y, pointing down into the tail)
//   accel_x ≈  0g
//   accel_z ≈  0g
//
// The raw UART-RVC accel INCLUDES gravity, so we subtract 1g from accel_y
// in imuRead() to get net thrust/motion acceleration. This is what the
// Kalman filter and liftoff detection use.

// ─── Data Structure ───────────────────────────────────────────────────────────

struct ImuSample {
    // Euler angles (degrees)
    float yaw;    // 0–360°  — heading (rotation around vertical axis)
    float pitch;  // ±180°   — nose up/down
    float roll;   // ±180°   — rotation around rocket long axis

    // Acceleration (m/s²) — gravity REMOVED (net motion acceleration only)
    // accel_y is along the rocket body axis, positive toward nose
    // At rest on pad all three should read ~0 m/s²
    float accel_x;
    float accel_y;   // vertical axis — liftoff + burnout detection + Kalman input
    float accel_z;

    // Tilt from vertical (degrees) — IREC 7.3.1 safety check
    // 0° = perfectly vertical, 90° = horizontal
    float tilt_deg;
};

// ─── Public API ───────────────────────────────────────────────────────────────

/**
 * @brief  Initialise UART1 for BNO08x UART-RVC reception on GPIO 25.
 *         No external library required — raw UART packet parsing only.
 *         Call once from setup() after Serial.begin().
 * @return true on success.
 */
bool imuInit();

/**
 * @brief  Non-blocking read of the latest BNO08x UART-RVC packet.
 *         Syncs to the 0xAA 0xAA header, parses 19 bytes, validates checksum.
 *         Call every loop iteration — returns false if no complete packet yet.
 * @param[out] sample  Populated with latest data on success.
 * @return true if a valid new packet was parsed.
 */
bool imuRead(ImuSample &sample);

/**
 * @brief  Returns true if imuInit() has been called.
 */
bool imuReady();