#include "errors.h"

namespace {
uint8_t errorFlags = ERROR_NONE;
}

void clearAllErrors() {
  errorFlags = ERROR_NONE;
}

void setErrorState(SystemError error, bool active) {
  if (active) {
    errorFlags |= error;
  } else {
    errorFlags &= static_cast<uint8_t>(~error);
  }
}

bool hasError(SystemError error) {
  return (errorFlags & error) != 0;
}

uint8_t getErrorFlags() {
  return errorFlags;
}

const char *getErrorSummary() {
  if (errorFlags == ERROR_NONE) {
    return "NONE";
  }

  if (hasError(ERROR_DISPLAY_INIT) && hasError(ERROR_PULSE_NO_SIGNAL) && hasError(ERROR_AZIMUTH)) {
    return "DISPLAY,PULSE,AZIMUTH";
  }

  if (hasError(ERROR_PULSE_NO_SIGNAL) && hasError(ERROR_AZIMUTH)) {
    return "PULSE,AZIMUTH";
  }

  if (hasError(ERROR_DISPLAY_INIT) && hasError(ERROR_PULSE_NO_SIGNAL)) {
    return "DISPLAY,PULSE";
  }

  if (hasError(ERROR_DISPLAY_INIT) && hasError(ERROR_AZIMUTH)) {
    return "DISPLAY,AZIMUTH";
  }

  if (hasError(ERROR_DISPLAY_INIT)) {
    return "DISPLAY";
  }

  if (hasError(ERROR_PULSE_NO_SIGNAL)) {
    return "PULSE";
  }

  if (hasError(ERROR_AZIMUTH)) {
    return "AZIMUTH";
  }

  return "UNKNOWN";
}
