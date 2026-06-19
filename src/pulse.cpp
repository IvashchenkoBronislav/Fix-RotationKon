#include "pulse.h"

#include <Arduino.h>

#include "azimuth.h"
#include "config.h"
#include "errors.h"
#include "motor.h"

namespace {
volatile uint32_t pulseCount = 0;
volatile uint32_t lastPulseMs = 0;
volatile uint32_t lastPulseIntervalMs = 0;
volatile uint32_t lastEdgeMs = 0;
volatile bool contactClosed = false;
bool signalTimedOut = false;
MotorState lastMotorState = STOP;
uint32_t motionStartedMs = 0;
uint32_t lastDebugPulseCount = 0;

bool IRAM_ATTR isPulseActiveLevel(uint16_t rawValue) {
  return PULSE_ACTIVE_LOW ? rawValue == LOW : rawValue == HIGH;
}

void handlePulseLossDuringMotion(MotorState motorState) {
  (void)motorState;
  cancelAzimuthMove();
  setStop();
  Serial.println("[AZ] move stopped: no pulses");
}

void IRAM_ATTR onPulseChange() {
  const uint32_t nowMs = millis();
  const bool isClosed = isPulseActiveLevel(digitalRead(PIN_PULSE_INPUT));

  // Ignore bounce on either edge for dry-contact style sensors.
  if ((nowMs - lastEdgeMs) < PULSE_DEBOUNCE_MS) {
    return;
  }

  lastEdgeMs = nowMs;
  contactClosed = isClosed;
  if (lastPulseMs != 0) {
    lastPulseIntervalMs = nowMs - lastPulseMs;
  }
  lastPulseMs = nowMs;
  pulseCount++;
}
}

void pulseInit() {
  pinMode(PIN_PULSE_INPUT, INPUT_PULLUP);
  contactClosed = isPulseActiveLevel(digitalRead(PIN_PULSE_INPUT));
  lastEdgeMs = millis();
  lastPulseMs = 0;
  lastPulseIntervalMs = 0;
  signalTimedOut = false;
  lastDebugPulseCount = 0;
  setErrorState(ERROR_PULSE_NO_SIGNAL, false);
  attachInterrupt(digitalPinToInterrupt(PIN_PULSE_INPUT), onPulseChange, CHANGE);

  Serial.print("Pulse input: ");
  Serial.println(PIN_PULSE_INPUT);
  Serial.print("Pulse active level: ");
  Serial.println(PULSE_ACTIVE_LOW ? "LOW" : "HIGH");
  Serial.print("Pulse debounce ms: ");
  Serial.println(PULSE_DEBOUNCE_MS);
}

void pulseUpdate() {
  const uint32_t nowMs = millis();
  const uint32_t count = getPulseCount();
  const uint32_t lastPulse = getLastPulseMs();
  const MotorState motorState = getMotorState();
  const bool hadSignalTimeout = signalTimedOut;

  if (ENABLE_PULSE_INTERVAL_DEBUG && count != lastDebugPulseCount) {
    lastDebugPulseCount = count;
    Serial.print("[PULSE] p=");
    Serial.print(count);
    Serial.print(" dt=");
    Serial.print(getLastPulseIntervalMs());
    Serial.print(" t=");
    Serial.println(lastPulse);
  }

  if (motorState != lastMotorState) {
    if (motorState == CW || motorState == CCW) {
      motionStartedMs = nowMs;
    } else {
      motionStartedMs = 0;
    }

    lastMotorState = motorState;
  }

  if (motorState == STOP) {
    signalTimedOut = false;
    setErrorState(ERROR_PULSE_NO_SIGNAL, false);
    return;
  }

  const bool startupGraceActive =
      motionStartedMs > 0 && (nowMs - motionStartedMs) < PULSE_STARTUP_GRACE_MS;

  if (startupGraceActive) {
    signalTimedOut = false;
    setErrorState(ERROR_PULSE_NO_SIGNAL, false);
    return;
  }

  signalTimedOut =
      (count == 0 && motionStartedMs > 0 &&
       (nowMs - motionStartedMs) >= PULSE_NO_SIGNAL_TIMEOUT_MS) ||
      (count > 0 && (nowMs - lastPulse) > PULSE_NO_SIGNAL_TIMEOUT_MS);

  setErrorState(ERROR_PULSE_NO_SIGNAL, signalTimedOut);

  if (signalTimedOut && !hadSignalTimeout) {
    setErrorState(ERROR_AZIMUTH, true);
    if (motorState == CW || motorState == CCW) {
      handlePulseLossDuringMotion(motorState);
    }
  }
}

uint32_t getPulseCount() {
  noInterrupts();
  const uint32_t value = pulseCount;
  interrupts();
  return value;
}

uint32_t getLastPulseMs() {
  noInterrupts();
  const uint32_t value = lastPulseMs;
  interrupts();
  return value;
}

uint32_t getLastPulseIntervalMs() {
  noInterrupts();
  const uint32_t value = lastPulseIntervalMs;
  interrupts();
  return value;
}

uint16_t getPulseRawValue() {
  noInterrupts();
  const bool value = contactClosed;
  interrupts();
  return value ? (PULSE_ACTIVE_LOW ? 0 : 1) : (PULSE_ACTIVE_LOW ? 1 : 0);
}

bool isPulseSignalTimedOut() {
  return signalTimedOut;
}

bool isPulseContactClosed() {
  noInterrupts();
  const bool value = contactClosed;
  interrupts();
  return value;
}
