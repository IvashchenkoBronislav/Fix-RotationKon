#pragma once

#include <Arduino.h>

enum MotorState {
  STOP = 0,
  CW,
  CCW
};

void motorInit();
void setStop();
void setCW();
void setCCW();
void requestCW();
void requestCCW();
MotorState getMotorState();
uint8_t getRelayK1State();
uint8_t getRelayK2State();
uint8_t getRelayK3State();
const char *motorStateToString(MotorState state);
