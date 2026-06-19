#include "azimuth.h"

#include <Arduino.h>
#include <Preferences.h>

#include "config.h"
#include "errors.h"
#include "motor.h"
#include "pulse.h"

namespace {
Preferences preferences;
int realAzimuthHalfDegrees = 0;
int targetAzimuthHalfDegrees = 0;
uint32_t lastSaveMs = 0;
uint32_t azimuthMoveStartedMs = 0;
uint32_t lastProcessedPulseCount = 0;
uint32_t azimuthMoveStartPulseCount = 0;
uint32_t azimuthMoveRequiredPulses = 0;
uint32_t controlLockUntilMs = 0;
MotorState lastPulseDirection = STOP;
MotorState azimuthMoveDirection = STOP;
bool azimuthDirty = false;
bool azimuthMoveActive = false;

int normalizeAzimuth(int value);
int normalizeAzimuthHalf(int value);
int clampAzimuthHalf(int value);
int degreesToHalfDegrees(int degrees);
int halfDegreesToDegrees(int halfDegrees);
int shortestHalfDegreeDiff(int fromHalfDegrees, int toHalfDegrees);
void printAzimuthHalf(int halfDegrees);
void saveAzimuthIfNeeded(bool forceSave);

void printMoveSnapshot(const char *tag,
                       uint32_t startPulse,
                       uint32_t pulseCount,
                       uint32_t requiredPulses,
                       uint32_t completionPulses,
                       int currentAngleHalf,
                       int targetAngleHalf,
                       MotorState direction) {
  const uint32_t traveledPulses = pulseCount - startPulse;
  Serial.print("[AZ/");
  Serial.print(tag);
  Serial.print("] a=");
  printAzimuthHalf(currentAngleHalf);
  Serial.print("->");
  printAzimuthHalf(targetAngleHalf);
  Serial.print(" dir=");
  Serial.print(direction == CW ? "CW" : direction == CCW ? "CCW" : "STOP");
  Serial.print(" p=");
  Serial.print(startPulse);
  Serial.print("->");
  Serial.print(pulseCount);
  Serial.print(" t=");
  Serial.print(traveledPulses);
  Serial.print("/");
  Serial.print(requiredPulses);
  Serial.print(" stop@");
  Serial.println(completionPulses);
}

void clearMoveState() {
  azimuthMoveActive = false;
  azimuthMoveStartedMs = 0;
  azimuthMoveRequiredPulses = 0;
  controlLockUntilMs = 0;
  azimuthMoveDirection = STOP;
}

void finishCompletedMove() {
  clearMoveState();
  setStop();
  lastPulseDirection = STOP;
  lastProcessedPulseCount = getPulseCount();
  azimuthDirty = true;
  saveAzimuthIfNeeded(true);
}

void startPulseBudgetMove(uint32_t requiredPulses, MotorState direction, int targetDegrees, const char *tag) {
  targetAzimuthHalfDegrees = degreesToHalfDegrees(targetDegrees);
  azimuthMoveRequiredPulses = requiredPulses;
  azimuthMoveDirection = requiredPulses > 0 ? direction : STOP;
  azimuthMoveActive = requiredPulses > 0;
  azimuthMoveStartedMs = azimuthMoveActive ? millis() : 0;
  azimuthMoveStartPulseCount = getPulseCount();

  if (!azimuthMoveActive) {
    controlLockUntilMs = 0;
    azimuthMoveDirection = STOP;
    return;
  }

  uint32_t pulseIntervalMs = getLastPulseIntervalMs();
  if (pulseIntervalMs == 0) {
    pulseIntervalMs = AZIMUTH_FALLBACK_PULSE_INTERVAL_MS;
  }

  controlLockUntilMs =
      azimuthMoveStartedMs + (azimuthMoveRequiredPulses * pulseIntervalMs) + AZIMUTH_MOVE_TIMEOUT_MARGIN_MS;

  const uint32_t completionPulses =
      azimuthMoveRequiredPulses > AZIMUTH_STOP_LEAD_PULSES
          ? (azimuthMoveRequiredPulses - AZIMUTH_STOP_LEAD_PULSES)
          : azimuthMoveRequiredPulses;
  Serial.print("[AZ/");
  Serial.print(tag);
  Serial.print("] a=");
  printAzimuthHalf(realAzimuthHalfDegrees);
  Serial.print("->");
  printAzimuthHalf(targetAzimuthHalfDegrees);
  Serial.print(" dir=");
  Serial.print(azimuthMoveDirection == CW ? "CW" : azimuthMoveDirection == CCW ? "CCW" : "STOP");
  Serial.print(" need=");
  Serial.print(azimuthMoveRequiredPulses);
  Serial.print(" stop@");
  Serial.print(completionPulses);
  Serial.print(" pulseMs=");
  Serial.print(pulseIntervalMs);
  Serial.print(" lockMs=");
  Serial.println(controlLockUntilMs - azimuthMoveStartedMs);
}

int normalizeAzimuth(int value) {
  int normalized = value % 360;
  if (normalized < 0) {
    normalized += 360;
  }

  return normalized;
}

int normalizeAzimuthHalf(int value) {
  int normalized = value % 720;
  if (normalized < 0) {
    normalized += 720;
  }

  return normalized;
}

// This rotator has a hard mechanical stop at the 0/360 boundary - it cannot
// physically rotate through it. Real position tracking must clamp there
// instead of wrapping (e.g. overshoot pulses past 0 must stop at 0, not
// wrap around to ~359), unlike target/diff math elsewhere which treats the
// range as a normal 0-359.5 circle.
int clampAzimuthHalf(int value) {
  if (value < 0) {
    return 0;
  }
  if (value > 719) {
    return 719;
  }
  return value;
}

int degreesToHalfDegrees(int degrees) {
  return normalizeAzimuthHalf(normalizeAzimuth(degrees) * 2);
}

int halfDegreesToDegrees(int halfDegrees) {
  return normalizeAzimuthHalf(halfDegrees) / 2;
}

int shortestHalfDegreeDiff(int fromHalfDegrees, int toHalfDegrees) {
  int diff = normalizeAzimuthHalf(toHalfDegrees) - normalizeAzimuthHalf(fromHalfDegrees);

  if (diff > 360) {
    diff -= 720;
  } else if (diff < -360) {
    diff += 720;
  }

  return diff;
}

void printAzimuthHalf(int halfDegrees) {
  const int normalized = normalizeAzimuthHalf(halfDegrees);
  Serial.print(normalized / 2);
  Serial.print((normalized % 2) == 0 ? ".0" : ".5");
}

void saveAzimuthIfNeeded(bool forceSave) {
  if (!azimuthDirty) {
    return;
  }

  const uint32_t now = millis();
  if (!forceSave && (now - lastSaveMs) < AZIMUTH_SAVE_INTERVAL_MS) {
    return;
  }

  preferences.putInt("real_half", realAzimuthHalfDegrees);
  lastSaveMs = now;
  azimuthDirty = false;
}

}

void azimuthInit() {
  preferences.begin("rotor", false);
  if (preferences.isKey("real_half")) {
    realAzimuthHalfDegrees = normalizeAzimuthHalf(preferences.getInt("real_half", 0));
  } else {
    realAzimuthHalfDegrees = degreesToHalfDegrees(preferences.getInt("real_angle", 0));
  }
  targetAzimuthHalfDegrees = realAzimuthHalfDegrees;
  lastSaveMs = millis();
  lastProcessedPulseCount = getPulseCount();
  lastPulseDirection = getMotorState();
  azimuthDirty = false;
  azimuthMoveActive = false;
  azimuthMoveStartedMs = 0;
  azimuthMoveStartPulseCount = lastProcessedPulseCount;
  azimuthMoveRequiredPulses = 0;
  controlLockUntilMs = 0;
  azimuthMoveDirection = STOP;
}

int getAzimuthDegrees() {
  return halfDegreesToDegrees(realAzimuthHalfDegrees);
}

int getAzimuthHalfDegrees() {
  return realAzimuthHalfDegrees;
}

void setAzimuthDegrees(int value) {
  realAzimuthHalfDegrees = degreesToHalfDegrees(value);
  targetAzimuthHalfDegrees = realAzimuthHalfDegrees;
  clearMoveState();
  azimuthMoveStartPulseCount = getPulseCount();
  azimuthDirty = true;
  lastProcessedPulseCount = getPulseCount();
  saveAzimuthIfNeeded(true);
}

void adjustAzimuthDegrees(int delta) {
  if (delta == 0) {
    return;
  }

  realAzimuthHalfDegrees = clampAzimuthHalf(realAzimuthHalfDegrees + (delta * 2));
  azimuthDirty = true;
}

void resetAzimuthSystem() {
  realAzimuthHalfDegrees = 0;
  targetAzimuthHalfDegrees = 0;
  clearMoveState();
  azimuthMoveStartPulseCount = getPulseCount();
  azimuthDirty = true;
  lastProcessedPulseCount = getPulseCount();
  saveAzimuthIfNeeded(true);
}

void startAzimuthMove(int targetDegrees) {
  const int startAngleHalf = realAzimuthHalfDegrees;
  const int normalizedTargetHalf = degreesToHalfDegrees(targetDegrees);
  const int diff = AZIMUTH_ALLOW_ZERO_CROSSING
                       ? shortestHalfDegreeDiff(startAngleHalf, normalizedTargetHalf)
                       : (normalizedTargetHalf - startAngleHalf);
  const uint32_t requiredPulses = static_cast<uint32_t>(abs(diff));
  const MotorState direction = diff > 0 ? CW : CCW;
  startPulseBudgetMove(requiredPulses, direction, targetDegrees, "START");
}

void cancelAzimuthMove() {
  targetAzimuthHalfDegrees = realAzimuthHalfDegrees;
  clearMoveState();
  azimuthMoveStartPulseCount = getPulseCount();
}

bool isAzimuthMoveActive() {
  return azimuthMoveActive;
}

bool isAzimuthMoveLocked() {
  return azimuthMoveActive && millis() < controlLockUntilMs;
}

int getTargetAzimuthDegrees() {
  return halfDegreesToDegrees(targetAzimuthHalfDegrees);
}

int getTargetAzimuthHalfDegrees() {
  return targetAzimuthHalfDegrees;
}

void azimuthUpdate() {
  const MotorState state = getMotorState();
  const uint32_t pulseCount = getPulseCount();

  if (state == CW || state == CCW) {
    lastPulseDirection = state;
  }

  if (pulseCount != lastProcessedPulseCount) {
    const uint32_t deltaPulses = pulseCount - lastProcessedPulseCount;
    lastProcessedPulseCount = pulseCount;

    if (lastPulseDirection == CW) {
      realAzimuthHalfDegrees = clampAzimuthHalf(realAzimuthHalfDegrees + static_cast<int>(deltaPulses));
      azimuthDirty = true;
    } else if (lastPulseDirection == CCW) {
      realAzimuthHalfDegrees = clampAzimuthHalf(realAzimuthHalfDegrees - static_cast<int>(deltaPulses));
      azimuthDirty = true;
    }
  }

  if (azimuthMoveActive) {
    const uint32_t now = millis();
    const bool graceExpired =
        azimuthMoveStartedMs > 0 && (now - azimuthMoveStartedMs) >= AZIMUTH_PULSE_GRACE_MS;
    const uint32_t traveledPulses = pulseCount - azimuthMoveStartPulseCount;
    const uint32_t completionPulses =
        azimuthMoveRequiredPulses > AZIMUTH_STOP_LEAD_PULSES
            ? (azimuthMoveRequiredPulses - AZIMUTH_STOP_LEAD_PULSES)
            : azimuthMoveRequiredPulses;
    const uint32_t remainingPulses =
        azimuthMoveRequiredPulses > traveledPulses ? (azimuthMoveRequiredPulses - traveledPulses) : 0;
    uint32_t pulseIntervalMs = getLastPulseIntervalMs();

    if (pulseIntervalMs == 0) {
      pulseIntervalMs = AZIMUTH_FALLBACK_PULSE_INTERVAL_MS;
    }

    if (remainingPulses > 0) {
      const uint32_t dynamicLockUntil =
          now + (remainingPulses * pulseIntervalMs) + AZIMUTH_MOVE_TIMEOUT_MARGIN_MS;
      if (dynamicLockUntil > controlLockUntilMs) {
        controlLockUntilMs = dynamicLockUntil;
      }
    }

    if (graceExpired && isPulseSignalTimedOut()) {
      printMoveSnapshot(
          "NOPULSE",
          azimuthMoveStartPulseCount,
          pulseCount,
          azimuthMoveRequiredPulses,
          completionPulses,
          realAzimuthHalfDegrees,
          targetAzimuthHalfDegrees,
          azimuthMoveDirection);
      clearMoveState();
      setStop();
      saveAzimuthIfNeeded(true);
      return;
    }

    if (traveledPulses >= completionPulses) {
      printMoveSnapshot(
          "DONE",
          azimuthMoveStartPulseCount,
          pulseCount,
          azimuthMoveRequiredPulses,
          completionPulses,
          realAzimuthHalfDegrees,
          targetAzimuthHalfDegrees,
          azimuthMoveDirection);
      finishCompletedMove();
      return;
    }

    if (now >= controlLockUntilMs) {
      printMoveSnapshot(
          "TIMEOUT",
          azimuthMoveStartPulseCount,
          pulseCount,
          azimuthMoveRequiredPulses,
          completionPulses,
          realAzimuthHalfDegrees,
          targetAzimuthHalfDegrees,
          azimuthMoveDirection);
      clearMoveState();
      setStop();
      setErrorState(ERROR_AZIMUTH, true);
      saveAzimuthIfNeeded(true);
      return;
    }

    if (azimuthMoveDirection == CW) {
      requestCW();
    } else if (azimuthMoveDirection == CCW) {
      requestCCW();
    }
  }

  saveAzimuthIfNeeded(getMotorState() == STOP);
}

void startAzimuthMoveForced(int targetDegrees, MotorState direction) {
  const int startHalf = realAzimuthHalfDegrees;
  const int targetHalf = degreesToHalfDegrees(targetDegrees);

  const int cw = (normalizeAzimuthHalf(targetHalf) - normalizeAzimuthHalf(startHalf) + 720) % 720;
  const int ccw = (normalizeAzimuthHalf(startHalf) - normalizeAzimuthHalf(targetHalf) + 720) % 720;

  const uint32_t requiredPulses = static_cast<uint32_t>(direction == CW ? cw : ccw);
  startPulseBudgetMove(requiredPulses, direction, targetDegrees, "FORCED");
}