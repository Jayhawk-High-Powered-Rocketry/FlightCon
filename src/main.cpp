#include <Arduino.h>

#include "imu.h"
#include "transmitter.h"
#include "bar.h"
#include "servo.h"

static constexpr uint32_t kTransmitIntervalMs = 50;
static uint32_t lastTxMs = 0;
// Servo altitude trigger settings
static constexpr float kServoAltTrigger = 242.3f; // meters (adjust as needed)
static constexpr float kServoAltHysteresis = 1.0f; // meters to avoid chattering
static bool servo_triggered = false;
void setup() {
  Serial.begin(115200);
  delay(200);

  // if (!imuInit()) {
  //   Serial.println("IMU init failed, halting.");
  //   while (true) {
  //     delay(1000);
  //   }
  // }

  if (!transmitterInit()) {
    Serial.println("Transmitter init failed, halting.");
    while (true) {
      delay(1000);
    }
  }

  if (!bar_init()) {
    Serial.println("BMP280 init failed, halting.");
    while (true) {
      delay(1000);
    }
  }

  if (!servo_init()) {
    Serial.println("Servo init failed, halting.");
    while (true) {
      delay(1000);
    }
  }
  servo_set_angle(0, 90); // Test: Move channel 0 to 90° on startup

  Serial.println("System ready: IMU -> LoRa transmitter");
}

void loop() {
  transmitterPoll();

  float temp, pressure, altitude;
  if (bar_read_all(temp, pressure, altitude)) {
    Serial.printf("Temp: %.2f °C, Pressure: %.2f hPa, Altitude: %.2f m\n", temp, pressure, altitude);
  } else {
    Serial.println("Failed to read from BMP280");
  }

  // ImuSample sample;
  // if (!imuRead(sample)) {
  //   return;
  // }

  const uint32_t now = millis();
  if (now - lastTxMs < kTransmitIntervalMs) {
    return;
  }
  lastTxMs = now;

  String payload;
  payload.reserve(96);
  //payload = "Roll: " + String(sample.roll, 2) + ", Pitch: " + String(sample.pitch, 2) + ", Yaw: " + String(sample.yaw, 2);
  payload += ", Temp: " + String(temp, 2) + ", Pressure: " + String(pressure, 2) + ", Altitude: " + String(altitude, 2);
  payload += ", ServoTriggered: " + String(servo_triggered ? 1 : 0);
  Serial.println(payload);
  transmitterSend(payload);

  // Altitude-based servo control with hysteresis
  if (altitude >= kServoAltTrigger && !servo_triggered) {
    servo_set_angle(0, 45); // triggered position
    servo_triggered = true;
    Serial.println("Servo triggered by altitude");
  } else if (servo_triggered && altitude <= (kServoAltTrigger - kServoAltHysteresis)) {
    servo_set_angle(0, 0); // reset position
    servo_triggered = false;
    Serial.println("Servo reset by altitude drop");
  }
}