#pragma once

#include <Arduino.h>

void pulseInit();
void pulseUpdate();
uint32_t getPulseCount();
uint32_t getLastPulseMs();
uint32_t getLastPulseIntervalMs();
uint16_t getPulseRawValue();
bool isPulseSignalTimedOut();
bool isPulseContactClosed();
