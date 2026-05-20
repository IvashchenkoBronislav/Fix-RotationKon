#include "status.h"

#include <Arduino.h>

#include "azimuth.h"
#include "buttons.h"
#include "config.h"
#include "display.h"
#include "errors.h"
#include "motor.h"
#include "pulse.h"
namespace {
void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }

  Serial.print(value, HEX);
}
}

void printStatus() {
  const int angleHalf = getAzimuthHalfDegrees();
  Serial.println("[STATUS]");
  Serial.print("state=");
  Serial.println(motorStateToString(getMotorState()));
  Serial.print("real=");
  Serial.print(angleHalf / 2);
  Serial.println((angleHalf % 2) == 0 ? ".0" : ".5");
  Serial.print("K1=");
  Serial.println(getRelayK1State());
  Serial.print("K2=");
  Serial.println(getRelayK2State());
  Serial.print("K3=");
  Serial.println(getRelayK3State());
  Serial.print("buttons=");
  Serial.println(buttonModeToString(getButtonMode()));
  Serial.print("pulses=");
  Serial.println(getPulseCount());
  Serial.print("lastPulseMs=");
  Serial.println(getLastPulseMs());
  Serial.print("pulseIntervalMs=");
  Serial.println(getLastPulseIntervalMs());
  Serial.print("pulseLevel=");
  Serial.println(getPulseRawValue());
  Serial.print("pulseSignal=");
  Serial.println(isPulseSignalTimedOut() ? "NO SIGNAL" : "OK");
  Serial.print("errors=");
  Serial.println(getErrorSummary());
}

void refreshStatusAndDisplay() {
  printStatus();
  updateDisplay();
}
