#pragma once

#include <Arduino.h>

bool transmitterInit();
bool transmitterSend(const String &payload);
void transmitterPoll();
