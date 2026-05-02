#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include "imu.h"

// --------- Pins (ESP32 default I2C) ----------
#define SDA_PIN 25
#define SCL_PIN 26

// --------- I2C addresses ----------
#define BNO_ADDR  0x4B

// --------- IMU ----------
BNO080 myIMU;

static bool imuReady = false;

bool imuInit() {
  delay(200);

  // Start I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Start BNO080 at 0x4B
  if (!myIMU.begin(BNO_ADDR, Wire)) {
    Serial.println("BNO080 not found at 0x4B");
    Serial.println("Check wiring and I2C address");
    imuReady = false;
    return false;
  }

  // Enable Euler angles (via Rotation Vector)
  myIMU.enableRotationVector(50); // 50ms (20Hz)

  imuReady = true;
  Serial.println("IMU initialized");
  return true;
}

bool imuRead(ImuSample &sample) {
  if (!imuReady || !myIMU.dataAvailable()) {
    return false;
  }

  // Convert radians to degrees
  sample.roll = myIMU.getRoll() * 180.0f / PI;
  sample.pitch = myIMU.getPitch() * 180.0f / PI;
  sample.yaw = myIMU.getYaw() * 180.0f / PI;
  return true;
}