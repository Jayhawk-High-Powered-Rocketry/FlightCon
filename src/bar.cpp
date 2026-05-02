/**
 * bar.cpp — BMP280 Barometer driver for ESP32 WROOM-32E
 *
 * Bus   : I2C1 (separate from IMU I2C0 on GPIO 21 / 22)
 * Addr  : 0x76  (SDO → GND)  |  0x77  (SDO → 3.3V)
 * Lib   : Adafruit BMP280  (install via Library Manager or platformio.ini)
 *
 * platformio.ini dependency:
 *   lib_deps =
 *       adafruit/Adafruit BMP280 Library @ ^2.6.8
 *       adafruit/Adafruit Unified Sensor @ ^1.1.14
 *
 * Arduino IDE: Sketch → Include Library → Manage Libraries → "Adafruit BMP280"
 */

#include "bar.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <math.h>

// ─── Internal state ──────────────────────────────────────────────────────────

// Use Wire1 (second I2C port) to avoid conflicts with IMU on Wire (pins 25/26)
static Adafruit_BMP280 _bmp(&Wire1);
static bool            _initialised = false;

// ─── Helpers ─────────────────────────────────────────────────────────────────

/** Return -999.0f and print a warning when the sensor is not ready. */
static float _not_ready(const char *fn)
{
    Serial.printf("[BAR] %s called before successful bar_init()\n", fn);
    return -999.0f;
}

// ─── Public implementation ───────────────────────────────────────────────────

bool bar_init(void)
{
    // Initialize Wire1 (second I2C port) on pins 21/22.
    // This is separate from the IMU's I2C (Wire on pins 25/26).
    Wire1.begin(BMP280_SDA_PIN, BMP280_SCL_PIN);

    if (!_bmp.begin(BMP280_I2C_ADDR)) {
        Serial.printf("[BAR] BMP280 not found at 0x%02X — check wiring / SDO pin\n",
                      BMP280_I2C_ADDR);
        _initialised = false;
        return false;
    }

    // ── Sensor configuration ──────────────────────────────────────────────
    // Oversampling & filter tuned for UAV / fast-moving platform use-case.
    // Adjust to taste; see BMP280 datasheet section 3.4 for recommended modes.
    _bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,      // Continuous measurement
        Adafruit_BMP280::SAMPLING_X2,      // Temperature oversampling ×2
        Adafruit_BMP280::SAMPLING_X16,     // Pressure oversampling ×16
        Adafruit_BMP280::FILTER_X16,       // IIR filter coefficient 16
        Adafruit_BMP280::STANDBY_MS_500    // ODR ≈ 26 Hz in normal mode
    );

    _initialised = true;
    Serial.printf("[BAR] BMP280 initialised at 0x%02X (SDA=%d SCL=%d)\n",
                  BMP280_I2C_ADDR, BMP280_SDA_PIN, BMP280_SCL_PIN);
    return true;
}

float bar_get_temperature(void)
{
    if (!_initialised) return _not_ready(__func__);
    return _bmp.readTemperature();   // °C
}

float bar_get_pressure(void)
{
    if (!_initialised) return _not_ready(__func__);
    return _bmp.readPressure() / 100.0f;  // Pa → hPa
}

float bar_get_altitude(float sea_level_hpa)
{
    if (!_initialised) return _not_ready(__func__);

    float pressure_hpa = _bmp.readPressure() / 100.0f;

    // International Standard Atmosphere (ISA) barometric formula
    // h = 44330 * (1 - (P / P0)^(1/5.255))
    if (pressure_hpa <= 0.0f) return -999.0f;
    return 44330.0f * (1.0f - powf(pressure_hpa / sea_level_hpa, 0.1902949f));
}

bool bar_read_all(float &temp_c,
                  float &pressure_hpa,
                  float &altitude_m,
                  float  sea_level_hpa)
{
    if (!_initialised) {
        _not_ready(__func__);
        return false;
    }

    // Read temperature first — BMP280 uses it internally for pressure
    // compensation, so order matters.
    temp_c       = _bmp.readTemperature();
    pressure_hpa = _bmp.readPressure() / 100.0f;

    if (pressure_hpa <= 0.0f) {
        altitude_m = -999.0f;
        return false;
    }

    altitude_m = 44330.0f * (1.0f - powf(pressure_hpa / sea_level_hpa, 0.1902949f));
    return true;
}