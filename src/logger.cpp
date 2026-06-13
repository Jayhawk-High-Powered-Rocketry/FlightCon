#include "logger.h"
#include <LittleFS.h>
#include <WebServer.h>

// Binary log format — 31 bytes per record:
//   Bytes 0-3:  ts_ms  uint32_t LE
//   Bytes 4-30: telemetry packet (TELEM_PACKET_BYTES = 27)
//
// File header: 4 magic bytes { 'F','C','1', TELEM_PACKET_BYTES }
// At 1 Hz: 31 bytes/s → 4 MB PSRAM ≈ 37 hours, 1.5 MB LittleFS ≈ 13 hours.

static constexpr size_t kLogMaxBytes = 4 * 1024 * 1024;
static constexpr size_t kRecordLen   = 4 + TELEM_PACKET_BYTES; // 31 bytes

static const uint8_t kMagic[4] = { 'F', 'C', '1', (uint8_t)TELEM_PACKET_BYTES };

// PSRAM path
static uint8_t* sBuf     = nullptr;
static size_t   sLen     = 0;

// Direct-to-FS path (no PSRAM)
static File  sLogFile;
static bool  sDirectMode = false;

static bool sFlushed = false;

static WebServer sHttpServer(80);

// ─── Internal ─────────────────────────────────────────────────────────────────

static void flush_internal()
{
    if (sDirectMode) {
        if (sLogFile) {
            sLogFile.flush();
            sLogFile.close();
        }
        sFlushed = true;
        Serial.println("[LOG] Direct mode: log closed and ready to download");
        return;
    }

    if (!sBuf || sLen == 0 || sFlushed) return;

    File f = LittleFS.open("/flight.log", "w");
    if (!f) {
        Serial.println("[LOG] Failed to open /flight.log for write");
        return;
    }

    size_t written = f.write(sBuf, sLen);
    f.close();
    sFlushed = true;

    Serial.printf("[LOG] Flushed %u bytes to /flight.log (%u records)\n",
                  (unsigned)written,
                  (unsigned)((sLen > sizeof(kMagic)) ? (sLen - sizeof(kMagic)) / kRecordLen : 0));
}

// ─── Public API ───────────────────────────────────────────────────────────────

void logger_init()
{
    if (!LittleFS.begin(true)) {
        Serial.println("[LOG] LittleFS mount failed");
        return;
    }

    if (psramInit() && ESP.getPsramSize() > 0) {
        sBuf = (uint8_t*)ps_malloc(kLogMaxBytes);
        if (sBuf) {
            memcpy(sBuf, kMagic, sizeof(kMagic));
            sLen = sizeof(kMagic);
            Serial.printf("[LOG] Ready — %u KB PSRAM buffer (~%u hrs at 1 Hz)\n",
                          (unsigned)(kLogMaxBytes / 1024),
                          (unsigned)(kLogMaxBytes / kRecordLen / 3600));
            return;
        }
        Serial.println("[LOG] PSRAM alloc failed — falling back to direct FS write");
    }

    sLogFile = LittleFS.open("/flight.log", "w");
    if (!sLogFile) {
        Serial.println("[LOG] Failed to open /flight.log — logging disabled");
        return;
    }
    sLogFile.write(kMagic, sizeof(kMagic));
    sDirectMode = true;
    Serial.println("[LOG] Ready — direct LittleFS mode (no PSRAM)");
}

void logger_record(uint32_t ts_ms, const uint8_t* pkt)
{
    uint8_t rec[kRecordLen];
    rec[0] = (uint8_t)(ts_ms & 0xFF);
    rec[1] = (uint8_t)((ts_ms >>  8) & 0xFF);
    rec[2] = (uint8_t)((ts_ms >> 16) & 0xFF);
    rec[3] = (uint8_t)((ts_ms >> 24) & 0xFF);
    memcpy(rec + 4, pkt, TELEM_PACKET_BYTES);

    if (sDirectMode) {
        if (sLogFile) sLogFile.write(rec, kRecordLen);
        return;
    }

    if (!sBuf || sLen + kRecordLen > kLogMaxBytes) return;
    memcpy(sBuf + sLen, rec, kRecordLen);
    sLen += kRecordLen;
}

void logger_flush_to_fs()
{
    flush_internal();
}

void logger_start_http()
{
    sHttpServer.on("/log", HTTP_GET, []() {
        flush_internal();
        if (!LittleFS.exists("/flight.log")) {
            sHttpServer.send(404, "text/plain", "No log on disk\n");
            return;
        }
        File f = LittleFS.open("/flight.log", "r");
        sHttpServer.streamFile(f, "application/octet-stream");
        f.close();
    });

    sHttpServer.on("/log/status", HTTP_GET, []() {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "mode=%s  buf_bytes=%u  records=%u  flushed=%d  fs_exists=%d\n",
                 sDirectMode ? "direct" : "psram",
                 (unsigned)sLen,
                 (unsigned)((sLen > sizeof(kMagic)) ? (sLen - sizeof(kMagic)) / kRecordLen : 0),
                 sFlushed ? 1 : 0,
                 LittleFS.exists("/flight.log") ? 1 : 0);
        sHttpServer.send(200, "text/plain", msg);
    });

    sHttpServer.on("/log/clear", HTTP_GET, []() {
        LittleFS.remove("/flight.log");
        sFlushed = false;

        if (sDirectMode) {
            sLogFile = LittleFS.open("/flight.log", "w");
            if (sLogFile) sLogFile.write(kMagic, sizeof(kMagic));
        } else {
            if (sBuf) {
                memcpy(sBuf, kMagic, sizeof(kMagic));
                sLen = sizeof(kMagic);
            }
        }
        sHttpServer.send(200, "text/plain", "Log cleared — ready for next flight\n");
    });

    sHttpServer.begin();
    Serial.println("[LOG] HTTP server on port 80");
    Serial.println("[LOG]   GET /log         — download binary log");
    Serial.println("[LOG]   GET /log/status  — buffer stats");
    Serial.println("[LOG]   GET /log/clear   — reset for next flight");
}

void logger_handle_http()
{
    sHttpServer.handleClient();
}
