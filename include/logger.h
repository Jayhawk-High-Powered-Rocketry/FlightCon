#pragma once
#include <Arduino.h>
#include "telemetry.h"

// Initialize PSRAM buffer and mount LittleFS. Call once in setup().
void logger_init();

// Append one binary record: 4-byte ts_ms (LE) + 27-byte telemetry packet = 31 bytes.
// pkt must be exactly TELEM_PACKET_BYTES (from telem_pack_buf).
// Called once per transmit cycle (1 Hz) so log and downlink are byte-identical.
void logger_record(uint32_t ts_ms, const uint8_t* pkt);

// Write PSRAM buffer to /flight.log on LittleFS. Idempotent — safe to call
// multiple times; only the first call writes. Call on DESCENDED transition.
void logger_flush_to_fs();

// Register HTTP routes and start server on port 80.
// Call after WiFi AP is up (after initOTA).
void logger_start_http();

// Drive the HTTP server. Call every loop() alongside ArduinoOTA.handle().
void logger_handle_http();
