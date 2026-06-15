/*
All variables in this file were declared in main.cpp and moved here
to centralize the global variables for easy editing.

*/
#include <Arduino.h>
// ─── Transmit interval ────────────────────────────────────────────────────────
static constexpr uint32_t kTransmitIntervalMs = 1000; // 1 Hz downlink
static constexpr uint32_t kCommandRxWindowMs  = 12;   // brief RX dwell for uplink command checks
static uint32_t lastTxMs   = 0;


// ─── Airbrake config ──────────────────────────────────────────────────────────
static constexpr int   kServoChannel  = 0;
static constexpr float kAirbrakeAngle = 45.0f;   // degrees — deployed
static constexpr float kNeutralAngle  = 8.0f;    // degrees — retracted / safe

// ─── IREC compliance constants ────────────────────────────────────────────────

// AIRBRAKE DEPLOYMENT ALTITUDE (metres AGL)
// Set based on OpenRocket sim targeting 10,000 ft apogee.
// 9000 ft = 2740 m. Deploying at this altitude gives ~1500 ft descent distance for airbrake control.
// Must be >= kAltitudeLockoutM to comply with IREC 7.4.1.3.2.
static constexpr float kDeployAltM = 2740.0f;

// ALTITUDE LOCKOUT (metres AGL)
// Per IREC 7.4.1.3.2: airbrakes must be locked (retracted) below this altitude.
// 10K flight = 2500 m. Dual petals cannot deploy until well above this.
// Condition #1 for airbrake deployment: altitude >= this value.
static constexpr float kAltitudeLockoutM = 2500.0f;

// ALTITUDE LOCKOUT HYSTERESIS (metres AGL)
// Once deployed, airbrakes retract if altitude drops below (kDeployAltM - kDeployHysteresisM).
// Prevents chatter if altitude oscillates near deployment threshold.
static constexpr float kDeployHysteresisM = 5.0f;

// TILT SAFETY (degrees from vertical)
// Per IREC 7.3.1: retract airbrakes immediately if tilt exceeds this.
// 30° = airbrake extension must not exceed 30° from vertical for safe deployment.
// Condition #2 for airbrake deployment: tilt <= this value.
static constexpr float kMaxTiltDeg = 30.0f;

// ─── Liftoff / burnout detection ─────────────────────────────────────────────
// Dual-confirm strategy prevents false triggers from pad vibration or baro noise.

// LIFTOFF ACCELERATION THRESHOLD (m/s²)
// 550 lb thrust / 42 lb rocket = ~13G off pad. BNO055 clips at ~4G (39.2 m/s²).
// Threshold set to 2G (19.6 m/s²) — below saturation, reliable.
// Condition #1 for PAD->BOOST: IMU accel_y must exceed this.
static constexpr float kLiftoffAccelMs2 = 20.0f;

// LIFTOFF ALTITUDE CONFIRM (metres AGL)
// Altitude must rise at least this much for baro confirmation of liftoff.
// Prevents false trigger from a pad bump detected by accel alone.
// Condition #2 for PAD->BOOST (must be true WITH accel condition): altitude >= 20m AGL.
static constexpr float kLiftoffAltM = 20.0f;

// BURNOUT ACCELERATION THRESHOLD (m/s²)
// Motor burning detection: accel drops below 2 m/s² = motor out.
// Trigger from BOOST->COAST when accel goes near zero.
static constexpr float kBurnoutAccelMs2 = 2.0f;

// BURNOUT BACKUP TIMER (milliseconds)
// Safety backstop: even if accel never drops, BOOST->COAST after this time.
// Motor burn time = 4 seconds. Margin = 100ms. Total = 4100ms.
// Ensures we don't stay in BOOST state if sensors fail.
static constexpr uint32_t kBurnoutBackstopMs = 4100;

// ─── Landing detection ────────────────────────────────────────────────────────
// DESCENDING → DESCENDED when velocity stays below threshold for 3 seconds.
// Prevents false landing detection from momentary velocity dips during descent.
static constexpr float    kLandedVelThreshMs = 2.0f;    // m/s
static constexpr uint32_t kLandedConfirmMs   = 3000;    // ms
static uint32_t           landedTimerStart   = 0;
static bool               landedTimerRunning = false;

// ─── Barometer timing ─────────────────────────────────────────────────────────
// BMP280 at a slow 1 Hz cadence to match telemetry and reduce read failures.
// Kalman only runs on fresh baro reads to prevent velocity drift.
static constexpr uint32_t kBaroPeriodMs = 1000;
static uint32_t lastBaroMs   = 0;
static uint32_t lastKalmanMs = 0;

#ifdef ENABLE_OTA
static constexpr uint32_t kWifiActiveWindowMs = 1UL * 60UL * 1000UL;
#endif

// ─── Flight state machine ─────────────────────────────────────────────────────

enum class FlightState {
    PAD,         // Sitting on launch pad. Waiting for liftoff.
    BOOST,       // Motor burning. CAS locked neutral per IREC 7.4.1.
    COAST,       // Motor out, ascending. Airbrakes active if conditions met.
    DESCENDING,  // Past apogee, falling. Airbrakes retracted.
    DESCENDED    // On the ground. Flight over. Velocity forced to zero (ZUPT).
};
