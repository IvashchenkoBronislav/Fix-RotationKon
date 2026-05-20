#pragma once

#include <Arduino.h>

enum SystemError : uint8_t {
  ERROR_NONE = 0,
  ERROR_DISPLAY_INIT = 1 << 0,
  ERROR_PULSE_NO_SIGNAL = 1 << 1,
  ERROR_AZIMUTH = 1 << 2,
};

void clearAllErrors();
void setErrorState(SystemError error, bool active);
bool hasError(SystemError error);
uint8_t getErrorFlags();
const char *getErrorSummary();
