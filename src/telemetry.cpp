#include "telemetry.h"
#include <string.h>

// ─── Byte-level pack helpers ──────────────────────────────────────────────────

static void pack_f32(uint8_t* p, float v)
{
    // memcpy is the UB-free way to reinterpret float bits.
    // ESP32 and Raspberry Pi are both little-endian so the bytes are identical
    // on both sides with no swap needed.
    memcpy(p, &v, 4);
}

static void pack_i16(uint8_t* p, float v, float scale, float lo, float hi)
{
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    int16_t iv = (int16_t)(v * scale);
    p[0] = (uint8_t)(iv & 0xFF);
    p[1] = (uint8_t)((iv >> 8) & 0xFF);
}

static void pack_u16(uint8_t* p, float v, float scale, float lo, float hi)
{
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    uint16_t uv = (uint16_t)(v * scale);
    p[0] = (uint8_t)(uv & 0xFF);
    p[1] = (uint8_t)((uv >> 8) & 0xFF);
}

// ─── Public implementation ────────────────────────────────────────────────────

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
                    float   temp_c)
{
    // Byte 0 — flags
    buf[0] = (uint8_t)(
        ((state & 0x07u) << 5) |
        (airbrake ? 0x10u : 0u) |
        (baro_ok  ? 0x08u : 0u) |
        (imu_ok   ? 0x04u : 0u)
    );

    // Float fields — full IEEE 754 precision, no quantisation loss
    pack_f32(buf + 1,  alt_kf);
    pack_f32(buf + 5,  vel_kf);
    pack_f32(buf + 9,  baro_alt);
    pack_f32(buf + 13, accel_y);

    // Angle fields — 0.01° resolution is more than sufficient
    pack_i16(buf + 17, tilt_deg,   100.0f,  -180.0f,  180.0f);
    pack_i16(buf + 19, roll_deg,   100.0f,  -180.0f,  180.0f);
    pack_i16(buf + 21, pitch_deg,  100.0f,   -90.0f,   90.0f);
    pack_u16(buf + 23, yaw_deg,    100.0f,    0.0f,   360.0f);
    pack_i16(buf + 25, temp_c,     100.0f,  -100.0f,  100.0f);
}

String telem_buf_to_hex(const uint8_t* buf, size_t len)
{
    static const char kHex[16] = {
        '0','1','2','3','4','5','6','7',
        '8','9','A','B','C','D','E','F'
    };

    String hex;
    hex.reserve(len * 2);
    for (size_t i = 0; i < len; i++) {
        hex += kHex[buf[i] >> 4];
        hex += kHex[buf[i] & 0x0F];
    }
    return hex;
}

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
                      float   temp_c)
{
    uint8_t buf[TELEM_PACKET_BYTES];
    telem_pack_buf(buf, state, airbrake, baro_ok, imu_ok,
                   alt_kf, vel_kf, baro_alt, accel_y,
                   tilt_deg, roll_deg, pitch_deg, yaw_deg, temp_c);
    return telem_buf_to_hex(buf, TELEM_PACKET_BYTES);
}
