#pragma once
#include <Arduino.h>
#include <stdint.h>

// Binary telemetry packet — 27 bytes, hex-encoded to 54 chars for LoRa.
//
// Byte  0:     flags
//                bits[7:5] state    (0=PAD 1=BOOST 2=COAST 3=DESCENDING 4=DESCENDED)
//                bit[4]    airbrake
//                bit[3]    baro_ok
//                bit[2]    imu_ok
//                bits[1:0] reserved
// Bytes  1- 4: alt_kf   float32 LE  metres
// Bytes  5- 8: vel_kf   float32 LE  m/s
// Bytes  9-12: baro_alt float32 LE  metres
// Bytes 13-16: accel_y  float32 LE  m/s² (full float32 — near-Mach accuracy)
// Bytes 17-18: tilt     int16   LE  degrees × 100
// Bytes 19-20: roll     int16   LE  degrees × 100
// Bytes 21-22: pitch    int16   LE  degrees × 100
// Bytes 23-24: yaw      uint16  LE  degrees × 100  (uint to hold 0–360°)
// Bytes 25-26: temp     int16   LE  °C      × 100

static constexpr size_t TELEM_PACKET_BYTES = 27;

// Pack one telemetry frame into buf (must be TELEM_PACKET_BYTES).
// Call this first; pass buf to telem_buf_to_hex for LoRa and logger_record for storage.
void telem_pack_buf(uint8_t* buf,
                    uint8_t state,
                    bool    airbrake,
                    bool    baro_ok,
                    bool    imu_ok,
                    float   alt_kf,
                    float   vel_kf,
                    float   baro_alt,
                    float   accel_y,
                    float   tilt_deg,
                    float   roll_deg,
                    float   pitch_deg,
                    float   yaw_deg,
                    float   temp_c);

// Hex-encode a raw byte buffer into a String for LoRa transmission.
String telem_buf_to_hex(const uint8_t* buf, size_t len);

// Convenience wrapper: pack + hex-encode in one call.
String telem_pack_hex(uint8_t state,
                      bool    airbrake,
                      bool    baro_ok,
                      bool    imu_ok,
                      float   alt_kf,
                      float   vel_kf,
                      float   baro_alt,
                      float   accel_y,
                      float   tilt_deg,
                      float   roll_deg,
                      float   pitch_deg,
                      float   yaw_deg,
                      float   temp_c);
