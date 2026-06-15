#include <Arduino.h>
#include "common.h"
#include "imu.h"
#include "transmitter.h"
#include "bar.h"
#include "servo.h"
#include "kalman.h"
#include "telemetry.h"
#include "verbose.h"
#include "logger.h"

#ifdef ENABLE_OTA
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "wifi_credentials.h"
#endif

#ifdef ENABLE_OTA // OTA enabled via build flag in platformio.ini
static bool otaEnabled = false;
static uint32_t otaStartMs = 0;
#endif

// README: How to run:

// pio run -t upload
// -> Get IP Address from console:
// pio run -t upload --upload-port ADDR (e.g., 192.168.1.50)


// ─── Flight state initialization ──────────────────────────────────────────────
// System boots in PAD state, waiting for liftoff.
// Change this ONLY for bench testing in different states.
static FlightState state        = FlightState::PAD;
static uint32_t    boostStartMs = 0;
static bool        airbrakeOut  = false;
static bool        commandDeployRequested = false;
static float       lastAltM     = 0.0f;
static uint32_t apogeeConditionStartMs = 0;
static constexpr uint32_t kApogeeConfirmMs = 2000; // 3 seconds

// Cache last Kalman state — always have something to transmit
static KfState lastKf = {0.0f, 0.0f};
static float baroBuffer[5] = {0};
static int baroIdx = 0;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static const char* stateLabel(FlightState s)
{
    switch (s) {
        case FlightState::PAD:        return "PAD";
        case FlightState::BOOST:      return "BOOST";
        case FlightState::COAST:      return "COAST";
        case FlightState::DESCENDING: return "DESCENDING";
        case FlightState::DESCENDED:  return "DESCENDED";
    }
    return "UNKNOWN";
}

static void retractAirbrakes(const char* reason)
{
    if (airbrakeOut) {
        servo_set_angle(kServoChannel, kNeutralAngle);
        airbrakeOut = false;
        Serial.printf("[CTRL] Airbrakes RETRACTED — %s\n", reason);
    }
}

static void deployAirbrakes()
{
    if (!airbrakeOut) {
        servo_set_angle(kServoChannel, kAirbrakeAngle);
        airbrakeOut = true;
        Serial.println("[CTRL] Airbrakes DEPLOYED");
    }
}

static float median5(const float values[5])
{
    float sorted[5];

    for (int i = 0; i < 5; i++) {
        sorted[i] = values[i];
    }

    // Simple sort for 5 values
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 5; j++) {
            if (sorted[j] < sorted[i]) {
                float temp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = temp;
            }
        }
    }

    return sorted[2]; // middle value
}

static void printFlashStorageInfo()
{
    uint32_t flashBytes = ESP.getFlashChipSize();
    uint32_t sketchBytes = ESP.getFreeSketchSpace();

    Serial.printf("[FLASH] Flash chip size: %u bytes (%.2f MB)\n",
                  flashBytes, flashBytes / 1048576.0f);
    Serial.printf("[FLASH] Free sketch space: %u bytes (%.2f MB)\n",
                  sketchBytes, sketchBytes / 1048576.0f);
}

#ifdef ENABLE_OTA
static void shutdownOTA()
{
    if (!otaEnabled) {
        return;
    }

    VLOG("[OTA] WiFi window expired — shutting wifi down");
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    otaEnabled = false;
    logger_init();  // begin new flight log only after old one could be downloaded
}

static void initOTA()
{
    WiFi.mode(WIFI_AP);

    IPAddress apIp(192, 168, 4, 1);
    IPAddress apGateway(192, 168, 4, 1);
    IPAddress apSubnet(255, 255, 255, 0);

    if (!WiFi.softAPConfig(apIp, apGateway, apSubnet)) {
        Serial.println("[OTA] softAPConfig failed — OTA disabled");
        otaEnabled = false;
        return;
    }

    if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD)) {
        Serial.println("[OTA] softAP start failed — OTA disabled");
        otaEnabled = false;
        return;
    }

    VLOG("[OTA] Access point started");
    VLOGF("[OTA] SSID: %s\n", WIFI_AP_SSID);
    VLOGF("[OTA] IP address: %s\n", WiFi.softAPIP().toString().c_str());

    ArduinoOTA.setHostname("esp32-flight");
    ArduinoOTA.setPassword(WIFI_AP_PASSWORD);

    ArduinoOTA.begin();
    otaEnabled = true;
    otaStartMs = millis();

    VLOG("[OTA] Ready");
}
#endif

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("booting...");
    printFlashStorageInfo();
#ifdef ENABLE_OTA
    logger_mount_fs();  // mount FS so old log is served during WiFi window
#else
    logger_init();      // no WiFi window — start logging immediately
#endif

    if (!imuInit()) {
        Serial.println("[MAIN] IMU unavailable — continuing without IMU.");
    }

    if (!transmitterInit()) {
        Serial.println("[MAIN] Transmitter init failed, halting.");
        while (true) { delay(1000); }
    }

    // bar_init() owns Wire1.begin(18,19) — must run before servo_init()
    if (!bar_init()) {
        Serial.println("[MAIN] BMP280 init failed, halting.");
        while (true) { delay(1000); }
    }

    // 10-second ground pressure calibration — board must be still on pad
    float groundPressure = bar_calibrate();
    if (groundPressure < 0.0f) {
        Serial.println("[MAIN] Barometer calibration failed, halting.");
        while (true) { delay(1000); }
    }

    // servo_init() shares Wire1 — Wire1.begin() already called by bar_init()
    if (!servo_init()) {
        Serial.println("[MAIN] Servo init failed, halting.");
        while (true) { delay(1000); }
    }

    // 7.3.1 — ensure neutral state at boot before anything else
    servo_set_angle(kServoChannel, kNeutralAngle);
    airbrakeOut = false;

    // ── Kalman init ───────────────────────────────────────────────────────────
    // Average 20 baro readings after calibration settles.
    // Fixes the 2-3m ground offset from single noisy sample.
    VLOG("[MAIN] Settling baro for Kalman init...");
    delay(500);
    double initSum   = 0.0;
    int    initCount = 0;
    for (int i = 0; i < 20; i++) {
        float t, p, a;
        if (bar_read_all(t, p, a)) {
            initSum += a;
            initCount++;
        }
        delay(25);
    }
    float initAlt = (initCount > 0) ? (float)(initSum / initCount) : 0.0f;
    VLOGF("[MAIN] Kalman init altitude: %.2f m (%d samples)\n",
          initAlt, initCount);

    kf_init(initAlt);
    lastKf = {initAlt, 0.0f};
    // Start in PAD tuning
    kf_set_phase(KF_PHASE_PAD);
    for (int i = 0; i < 5; i++) {
        baroBuffer[i] = initAlt;
    }
    baroIdx = 0;

    lastBaroMs   = millis();
    lastKalmanMs = millis();

    #ifdef ENABLE_OTA
    initOTA();
    logger_start_http();
    #endif

    Serial.println("[MAIN] System ready.");
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void loop()
{
    transmitterPoll();

    #ifdef ENABLE_OTA
    if (otaEnabled) {
        if (millis() - otaStartMs >= kWifiActiveWindowMs) {
            shutdownOTA();
        } else {
            ArduinoOTA.handle();
            logger_handle_http();
        }
    }
    #endif

    uint32_t now = millis();

    // ── IMU read (every loop — non-blocking) ──────────────────────────────────
    ImuSample imu;
    bool imuOk = imuRead(imu);

    // ── Barometer + Kalman (only at ~40 Hz baro rate) ─────────────────────────
    bool baroFresh = (now - lastBaroMs) >= kBaroPeriodMs;
    float temp = 0.0f, pressure = 0.0f, baroAlt = 0.0f;
    bool barOk = false;

    if (baroFresh) {
        barOk      = bar_read_all(temp, pressure, baroAlt);
        lastBaroMs = now;

        if (barOk) {
            float dt_s = (now - lastKalmanMs) / 1000.0f;
            lastKalmanMs = now;
            if (dt_s <= 0.0f || dt_s > 0.5f) dt_s = kBaroPeriodMs / 1000.0f;

            // IMU ACCELERATION INPUT FOR KALMAN FILTER
            // On PAD and DESCENDED: disable IMU input (accelInput = 0.0f)
            //   PAD: Board sitting still — bench noise would corrupt velocity estimate
            //   DESCENDED: Flight complete, velocity forced to zero (ZUPT) anyway
            // During BOOST/COAST/DESCENT: use IMU accel_y with deadband
            //   Deadband 0.25 m/s² filters out small noise/bias
            float accelInput = 0.0f;

            // During powered flight and descent, feed IMU acceleration to Kalman
            if (imuOk && state != FlightState::PAD && state != FlightState::DESCENDED) {
                accelInput = imu.accel_y;

                // Small signal deadband — kill bias/noise near zero
                if (fabsf(accelInput) < 0.25f) {
                    accelInput = 0.0f;
                }
            }

            baroBuffer[baroIdx] = baroAlt;
            baroIdx = (baroIdx + 1) % 5;

            // Feed median to Kalman instead of raw reading
            float baroFiltered = median5(baroBuffer);
            lastKf = kf_update(baroFiltered, accelInput, true, dt_s);

        }
    }

    // ── ZUPT — Zero Velocity Update ───────────────────────────────────────────
    // Force velocity to zero once landed. PAD is left free-running so bench
    // motion and attitude changes are visible during testing.
    if (state == FlightState::DESCENDED) {
        kf_zero_velocity();
        lastKf = kf_get_state();
    }

    // ── State machine ─────────────────────────────────────────────────────────
    switch (state) {

        // ── PAD: wait for liftoff ─────────────────────────────────────────────
        // Dual confirm required:
        //   1. IMU accel_y >= 2G (19.6 m/s2) — below BNO055 4G fusion clip
        //   2. Kalman altitude >= 20m AGL — baro confirmation
        // Both must be true simultaneously to prevent false triggers.
        case FlightState::PAD:
            retractAirbrakes("PAD state");
            {
                bool accelTriggered = imuOk && (imu.accel_y >= kLiftoffAccelMs2);
                bool baroTriggered  = (lastKf.altitude_m >= kLiftoffAltM);

                if (accelTriggered && baroTriggered) {
                    state        = FlightState::BOOST;
                    boostStartMs = millis();
                    // Switch Kalman to boost tuning
                    kf_set_phase(KF_PHASE_BOOST);
                    Serial.printf("[SM] PAD -> BOOST (accel_y=%.1f m/s2 alt=%.1fm vel=%.1fm/s)\n",
                                  imu.accel_y, lastKf.altitude_m, lastKf.velocity_ms);
                }
            }
            break;

        // ── BOOST: motor burning, CAS locked neutral per IREC 7.4.1 ──────────
        case FlightState::BOOST:
            retractAirbrakes("BOOST phase");
            {
                uint32_t burnElapsedMs = millis() - boostStartMs;
                bool accelBurnout = imuOk && (imu.accel_y < kBurnoutAccelMs2);
                bool timerBurnout = (burnElapsedMs >= kBurnoutBackstopMs);

                if (accelBurnout || timerBurnout) {
                    state = FlightState::COAST;
                    // Enter COAST tuning
                    kf_set_phase(KF_PHASE_COAST);
                    Serial.printf("[SM] BOOST -> COAST (%s at %lums alt=%.1fm vel=%.1fm/s)\n",
                                  accelBurnout ? "accel" : "timer",
                                  (unsigned long)burnElapsedMs,
                                  lastKf.altitude_m, lastKf.velocity_ms);
                }
            }
            break;

        // ── COAST: ascending after burnout, airbrakes active ──────────────────
        // AIRBRAKE DEPLOYMENT: All THREE conditions must be true simultaneously.
        // If ANY condition becomes false, airbrakes retract.
        case FlightState::COAST:
            {
                float altNow = lastKf.altitude_m;

                // CONDITION #1 — IREC 7.4.1.3.2: Altitude Lockout
                // Airbrakes must NOT deploy until well above minimum safe altitude.
                // For 10K flights: 2000m AGL. Below this = locked retracted.
                bool altOk = (altNow >= kAltitudeLockoutM);

                // CONDITION #2 — IREC 7.3.1: Tilt Safety
                // Airbrakes must NOT deploy if rocket is tilted > 30° from vertical.
                // Excessive tilt = asymmetric airbrake load = unstable.
                // If IMU unavailable, assume tilt OK (can't confirm violation).
                bool tiltOk = !imuOk || (imu.tilt_deg <= kMaxTiltDeg);

                // CONDITION #3 — Deployment Altitude Reached
                // Airbrake CAN deploy once we pass the target deployment altitude (9000 ft).
                // Combined with ground command: either auto-deploy or explicit request.
                bool deployOk = (altNow >= kDeployAltM);

                // Ground-station deploy request (latched from RX window)
                // Allows ground to force deployment if auto-logic fails or testing.
                bool commandOk = commandDeployRequested;

                // 7.3.1 — tilt exceeded: retract IMMEDIATELY
                if (imuOk && imu.tilt_deg > kMaxTiltDeg) {
                    retractAirbrakes("tilt > 30deg");
                }
                // DEPLOY: All THREE conditions must be TRUE to deploy
                //   1. altOk          = altitude >= lockout altitude (2000m)
                //   2. tiltOk         = tilt <= 30° from vertical
                //   3. deployOk || commandOk = deployment altitude reached OR ground command
                //   4. !airbrakeOut   = not already deployed
                // If any condition fails, deployment is blocked.
                else if (altOk && tiltOk && (deployOk || commandOk) && !airbrakeOut) {
                    deployAirbrakes();
                    if (commandOk) {
                        commandDeployRequested = false;
                        Serial.println("[CTRL] Ground deploy command accepted");
                    }
                }
                // Hysteresis retract: keep airbrakes out until altitude drops below deployment
                // altitude minus hysteresis margin (prevents chatter near deployment threshold).
                else if (airbrakeOut && altNow <= (kDeployAltM - kDeployHysteresisM)) {
                    retractAirbrakes("below deploy alt");
                }

                // Apogee candidate: velocity negative AND altitude falling
                // Must stay true continuously for X seconds before changing state
                bool apogeeCondition = (lastKf.velocity_ms < 0.0f && altNow < lastAltM);

                if (apogeeCondition) {
                    if (apogeeConditionStartMs == 0) {
                        apogeeConditionStartMs = millis();
                        Serial.println("[SM] Apogee condition detected — starting 3s confirmation timer");
                    }

                    if (millis() - apogeeConditionStartMs >= kApogeeConfirmMs) {
                        retractAirbrakes("apogee reached");
                        state = FlightState::DESCENDING;
                        // Enter descent tuning
                        kf_set_phase(KF_PHASE_DESCENT);
                        apogeeConditionStartMs = 0;

                        Serial.printf("[SM] COAST -> DESCENDING (alt=%.1fm vel=%.1fm/s)\n",
                                    lastKf.altitude_m, lastKf.velocity_ms);
                    }
                } else {
                    // Reset timer if the condition breaks even once
                    if (apogeeConditionStartMs != 0) {
                        VLOG("[SM] Apogee condition lost — confirmation timer reset");
                    }

                    apogeeConditionStartMs = 0;
                }
            }
            break;

        // ── DESCENDING: falling after apogee ──────────────────────────────────
        // Airbrakes retracted. Waiting for landing detection.
        case FlightState::DESCENDING:
                    retractAirbrakes("DESCENDING");
            {
                float absVel = fabsf(lastKf.velocity_ms);

                // Landing confirmed when velocity stays near zero for 3 seconds
                if (absVel < kLandedVelThreshMs) {
                    if (!landedTimerRunning) {
                        landedTimerStart   = millis();
                        landedTimerRunning = true;
                    } else if ((millis() - landedTimerStart) >= kLandedConfirmMs) {
                        state              = FlightState::DESCENDED;
                        // Set landing/ground tuning on confirmed touchdown
                        kf_set_phase(KF_PHASE_LANDING);
                        landedTimerRunning = false;
                        logger_flush_to_fs();
                        Serial.printf("[SM] DESCENDING -> DESCENDED (alt=%.1fm vel=%.1fm/s)\n",
                                    lastKf.altitude_m, lastKf.velocity_ms);
                    }
                } else {
                    // Still moving — reset landing timer
                    landedTimerRunning = false;
                }
            }
            break;

        // ── DESCENDED: on the ground, flight over ─────────────────────────────
        // ZUPT above forces velocity permanently to zero.
        // Airbrakes stay retracted.
        case FlightState::DESCENDED:
            retractAirbrakes("DESCENDED");
            break;
    }

    if (barOk) lastAltM = lastKf.altitude_m;

    // ── Transmit at fixed interval ────────────────────────────────────────────
    if (now - lastTxMs < kTransmitIntervalMs) return;
    lastTxMs = now;

    // ── Serial debug ──────────────────────────────────────────────────────────
    VLOGF(
        "[DATA] %-11s | AltKF=%6.1fm BaroAlt=%6.1fm | VelKF=%6.3fm/s | "
        "AccY=%6.2f Tilt=%5.1f° | Brake=%d\n",
        stateLabel(state),
        lastKf.altitude_m,
        barOk ? baroAlt : -999.0f,
        lastKf.velocity_ms,
        imuOk ? imu.accel_y : -999.0f,
        imuOk ? imu.tilt_deg : -999.0f,
        airbrakeOut ? 1 : 0
    );

    uint8_t pktBuf[TELEM_PACKET_BYTES];
    telem_pack_buf(pktBuf,
        now,
        static_cast<uint8_t>(state),
        airbrakeOut,
        barOk,
        imuOk,
        lastKf.altitude_m,
        lastKf.velocity_ms,
        barOk  ? baroAlt      : 0.0f,
        imuOk  ? imu.accel_y  : 0.0f,
        imuOk  ? imu.tilt_deg : 0.0f,
        imuOk  ? imu.roll     : 0.0f,
        imuOk  ? imu.pitch    : 0.0f,
        imuOk  ? imu.yaw      : 0.0f,
        barOk  ? temp         : 0.0f
    );

    transmitterSend(telem_buf_to_hex(pktBuf, TELEM_PACKET_BYTES));
    logger_record(pktBuf);

    if (transmitterReceiveDeployCommandWindow(kCommandRxWindowMs)) {
        if (state == FlightState::COAST) { // Maybe need to change this to PAD for testing?
            commandDeployRequested = true;
            Serial.println("[CTRL] Ground deploy command latched");
        } else {
            Serial.printf("[CTRL] Ground deploy command ignored in %s\n", stateLabel(state));
        }
    }

    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'clrmem') {  // Type 'clrmem' to clear calibration
            Serial.println("[IMU] Clearing saved calibration...");
            EEPROM.begin(64);
            uint32_t magic = 0;
            EEPROM.put(0, magic);  // Clear magic number
            EEPROM.end();
            Serial.println("[IMU] Calibration cleared. Reboot to recalibrate.");
        }
    }
}

/*
    

    
 */