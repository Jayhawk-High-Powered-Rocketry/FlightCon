#include "logger.h"
#include <LittleFS.h>
#include <WebServer.h>

// Binary log format — TELEM_PACKET_BYTES (31) bytes per record.
// ts_ms is packed at bytes 0-3 of each record (same layout as the LoRa payload).
//
// File header: 4 magic bytes { 'F','C','1', TELEM_PACKET_BYTES }
// At 1 Hz: 31 bytes/s → 4 MB PSRAM ≈ 37 hours, 1.5 MB LittleFS ≈ 13 hours.

static constexpr size_t kLogMaxBytes = 4 * 1024 * 1024;
static constexpr size_t kRecordLen   = TELEM_PACKET_BYTES; // 31 bytes

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

// Snapshot current state to disk for HTTP download. Does NOT set sFlushed —
// logging continues after return. In direct mode, closes and reopens the file
// (LittleFS can't have a write and read handle open simultaneously).
static void sync_to_fs_for_download()
{
    if (sDirectMode) {
        if (sLogFile) {
            sLogFile.flush();
            sLogFile.close();
        }
        return;
    }

    if (!sBuf || sLen == 0) return;
    File f = LittleFS.open("/flight.log", "w");
    if (!f) {
        Serial.println("[LOG] sync: failed to open /flight.log");
        return;
    }
    f.write(sBuf, sLen);
    f.close();
}

// Called on DESCENDED transition. Idempotent — only the first call writes.
static void flush_internal()
{
    if (sDirectMode) {
        if (sLogFile) {
            sLogFile.flush();
            sLogFile.close();
        }
        sFlushed = true;
        Serial.println("[LOG] Direct mode: log closed");
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

void logger_mount_fs()
{
    if (!LittleFS.begin(true)) {
        Serial.println("[LOG] LittleFS mount failed");
    }
}

void logger_init()
{
    // LittleFS.begin() is idempotent — safe to call even if logger_mount_fs()
    // already ran.
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

void logger_record(const uint8_t* pkt)
{
    if (sDirectMode) {
        if (sLogFile) sLogFile.write(pkt, kRecordLen);
        return;
    }

    if (!sBuf || sLen + kRecordLen > kLogMaxBytes) return;
    memcpy(sBuf + sLen, pkt, kRecordLen);
    sLen += kRecordLen;
}

void logger_flush_to_fs()
{
    flush_internal();
}

void logger_start_http()
{
    sHttpServer.on("/log", HTTP_GET, []() {
        sync_to_fs_for_download();
        if (!LittleFS.exists("/flight.log")) {
            sHttpServer.send(404, "text/plain", "No log on disk\n");
            if (sDirectMode && !sFlushed) {
                sLogFile = LittleFS.open("/flight.log", "a");
            }
            return;
        }
        File f = LittleFS.open("/flight.log", "r");
        sHttpServer.streamFile(f, "application/octet-stream");
        f.close();
        // Reopen write handle so direct-mode recording continues after download
        if (sDirectMode && !sFlushed) {
            sLogFile = LittleFS.open("/flight.log", "a");
            if (!sLogFile) Serial.println("[LOG] Warning: failed to reopen log after download");
        }
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
