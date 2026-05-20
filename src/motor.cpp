#include "motor.h"

#include <Arduino.h>

#include "config.h"

namespace {
MotorState currentState = STOP;
uint8_t relayK1State = LOW;
uint8_t relayK2State = LOW;
uint8_t relayK3State = LOW;

uint8_t applyRelayInvert(uint8_t logicalLevel, bool invert) {
  if (!invert) {
    return logicalLevel;
  }

  return logicalLevel == LOW ? HIGH : LOW;
}

void writeRelayK1(uint8_t logicalLevel) {
  relayK1State = logicalLevel;
  digitalWrite(PIN_RELAY_K1, applyRelayInvert(logicalLevel, RELAY_K1_INVERT));
}

void writeRelayK2(uint8_t logicalLevel) {
  relayK2State = logicalLevel;
  digitalWrite(PIN_RELAY_K2, applyRelayInvert(logicalLevel, RELAY_K2_INVERT));
}

void writeRelayK3(uint8_t logicalLevel) {
  relayK3State = logicalLevel;
  digitalWrite(PIN_RELAY_K3, applyRelayInvert(logicalLevel, RELAY_K3_INVERT));
}
}

void motorInit() {
  digitalWrite(PIN_RELAY_K1, applyRelayInvert(LOW, RELAY_K1_INVERT));
  digitalWrite(PIN_RELAY_K2, applyRelayInvert(LOW, RELAY_K2_INVERT));
  digitalWrite(PIN_RELAY_K3, applyRelayInvert(LOW, RELAY_K3_INVERT));
  pinMode(PIN_RELAY_K1, OUTPUT);
  pinMode(PIN_RELAY_K2, OUTPUT);
  pinMode(PIN_RELAY_K3, OUTPUT);
}

MotorState getMotorState() {
  return currentState;
}

uint8_t getRelayK1State() {
  return relayK1State;
}

uint8_t getRelayK2State() {
  return relayK2State;
}

uint8_t getRelayK3State() {
  return relayK3State;
}

const char *motorStateToString(MotorState state) {
  switch (state) {
    case STOP:
      return "STOP";
    case CW:
      return "CW";
    case CCW:
      return "CCW";
    default:
      return "UNKNOWN";
  }
}

void setStop() {
  const bool motorPowerWasOn = relayK3State == HIGH;
  writeRelayK3(LOW);
  if (motorPowerWasOn && RELAY_K3_STOP_LEAD_MS > 0) {
    delay(RELAY_K3_STOP_LEAD_MS);
  }
  writeRelayK1(LOW);
  writeRelayK2(LOW);
  currentState = STOP;
}

void setCW() {
  writeRelayK1(HIGH);
  writeRelayK2(LOW);
  writeRelayK3(HIGH);
  currentState = CW;
}

void setCCW() {
  writeRelayK1(LOW);
  writeRelayK2(HIGH);
  writeRelayK3(HIGH);
  currentState = CCW;
}

void requestCW() {
  if (currentState == CCW) {
    setStop();
    delay(DIRECTION_CHANGE_DELAY_MS);
  }

  if (currentState != CW) {
    setCW();
  }
}

void requestCCW() {
  if (currentState == CW) {
    setStop();
    delay(DIRECTION_CHANGE_DELAY_MS);
  }

  if (currentState != CCW) {
    setCCW();
  }
}
