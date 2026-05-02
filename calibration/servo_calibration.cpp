#include <Arduino.h>

#include "servo.h"

static constexpr uint8_t kCalibrationChannel = 0;
static constexpr size_t kLineBufferSize = 64;

struct CalibrationStep {
  const char *label;
  float angle;
};

static constexpr CalibrationStep kSteps[] = {
    {"MIN", 0.0f},
    {"MIDDLE", 90.0f},
    {"MAX", 180.0f},
};

static char kLineBuffer[kLineBufferSize];
static size_t kLineLength = 0;

static uint16_t angleToPulseUs(float degrees) {
  degrees = constrain(degrees, 0.0f, 180.0f);
  return (uint16_t)map(degrees,
                       0.0f, 180.0f,
                       (float)SERVO_MIN_US, (float)SERVO_MAX_US);
}

static void moveToStep(const CalibrationStep &step) {
  const uint16_t pulseUs = angleToPulseUs(step.angle);
  Serial.printf("[cal] %s -> %.1f deg (%u us)\n", step.label, step.angle, pulseUs);
  servo_set_angle(kCalibrationChannel, step.angle);
}

static void printHelp() {
  Serial.println("[cal] Commands:");
  Serial.println("[cal]   min        -> move to 0 deg");
  Serial.println("[cal]   mid        -> move to 90 deg");
  Serial.println("[cal]   max        -> move to 180 deg");
  Serial.println("[cal]   angle <n>   -> move to an angle in degrees");
  Serial.println("[cal]   pulse <us>  -> move to a raw pulse width");
  Serial.println("[cal]   sweep      -> cycle min/mid/max once");
  Serial.println("[cal]   help        -> show this help");
}

static void runSweep() {
  for (const CalibrationStep &step : kSteps) {
    moveToStep(step);
    delay(1500);
  }
}

static void handleCommand(char *command) {
  while (*command == ' ' || *command == '\t') {
    ++command;
  }

  if (*command == '\0') {
    return;
  }

  if (!strcmp(command, "help") || !strcmp(command, "h")) {
    printHelp();
    return;
  }

  if (!strcmp(command, "min")) {
    moveToStep(kSteps[0]);
    return;
  }

  if (!strcmp(command, "mid") || !strcmp(command, "middle")) {
    moveToStep(kSteps[1]);
    return;
  }

  if (!strcmp(command, "max")) {
    moveToStep(kSteps[2]);
    return;
  }

  if (!strcmp(command, "sweep")) {
    runSweep();
    return;
  }

  char *argument = strchr(command, ' ');
  if (argument != nullptr) {
    *argument++ = '\0';
    while (*argument == ' ' || *argument == '\t') {
      ++argument;
    }

    if (!strcmp(command, "angle")) {
      const float angle = atof(argument);
      Serial.printf("[cal] angle -> %.1f deg (%u us)\n", angle, angleToPulseUs(angle));
      servo_set_angle(kCalibrationChannel, angle);
      return;
    }

    if (!strcmp(command, "pulse")) {
      const int pulseUs = atoi(argument);
      Serial.printf("[cal] pulse -> %d us\n", pulseUs);
      servo_set_pulse_us(kCalibrationChannel, (uint16_t)pulseUs);
      return;
    }
  }

  Serial.printf("[cal] Unknown command: %s\n", command);
  printHelp();
}

static void pollSerial() {
  while (Serial.available() > 0) {
    const char incoming = (char)Serial.read();

    if (incoming == '\r' || incoming == '\n') {
      if (kLineLength == 0) {
        continue;
      }

      kLineBuffer[kLineLength] = '\0';
      handleCommand(kLineBuffer);
      kLineLength = 0;
      continue;
    }

    if (kLineLength + 1 < kLineBufferSize) {
      kLineBuffer[kLineLength++] = incoming;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("[cal] Servo calibration mode");
  Serial.printf("[cal] Channel: %u\n", kCalibrationChannel);
  Serial.printf("[cal] Range: %u us -> %u us\n", SERVO_MIN_US, SERVO_MAX_US);

  if (!servo_init()) {
    Serial.println("[cal] Servo init failed, halting.");
    while (true) {
      delay(1000);
    }
  }

  printHelp();
  moveToStep(kSteps[1]);
}

void loop() {
  pollSerial();
}