#pragma once

struct ImuSample {
  float roll;
  float pitch;
  float yaw;
};

bool imuInit();
bool imuRead(ImuSample &sample);
