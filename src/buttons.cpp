#include "buttons.h"

#include <Arduino.h>

#include "azimuth.h"
#include "config.h"
#include "display.h"
#include "errors.h"
#include "motor.h"
#include "pulse.h"
#include "status.h"

struct ButtonState {
  uint8_t pin;
  bool stableLevel;
  bool lastReading;
  uint32_t lastChangeMs;
};

namespace {
ButtonState buttonCW = {PIN_BUTTON_CW, HIGH, HIGH, 0};
ButtonState buttonCCW = {PIN_BUTTON_CCW, HIGH, HIGH, 0};
ButtonState buttonStop = {PIN_BUTTON_STOP, HIGH, HIGH, 0};
ButtonMode currentButtonMode = BUTTON_IDLE;
bool stopPending = false;
}

static bool isPressed(const ButtonState &button) {
  return button.stableLevel == LOW;
}

ButtonMode getButtonMode() {
  return currentButtonMode;
}

const char *buttonModeToString(ButtonMode mode) {
  switch (mode) {
    case BUTTON_IDLE:
      return "IDLE";
    case BUTTON_CW:
      return "CW";
    case BUTTON_CCW:
      return "CCW";
    case BUTTON_STOP:
      return "STOP";
    case BUTTON_CONFLICT:
      return "CONFLICT";
    default:
      return "UNKNOWN";
  }
}

static ButtonMode evaluateButtonMode() {
  const bool stopPressed = ENABLE_STOP_BUTTON && isPressed(buttonStop);
  const bool cwPressed = isPressed(buttonCW);
  const bool ccwPressed = isPressed(buttonCCW);

  if (stopPressed) {
    return BUTTON_STOP;
  }

  if (cwPressed && ccwPressed) {
    return BUTTON_CONFLICT;
  }

  if (cwPressed) {
    return BUTTON_CW;
  }

  if (ccwPressed) {
    return BUTTON_CCW;
  }

  return BUTTON_IDLE;
}

static void applyButtonUiRefresh() {
  const ButtonMode previousMode = currentButtonMode;
  currentButtonMode = evaluateButtonMode();

  if (currentButtonMode != previousMode) {
    refreshStatusAndDisplay();
  } else {
    updateDisplay();
  }
}

static void requestSafeStop() {
  if (isPulseContactClosed()) {
    stopPending = true;
    return;
  }

  stopPending = false;
  setStop();
}

static void handleButtonEvent(ButtonState &button, bool pressed) {
  notifyDisplayActivity();

  cancelAzimuthMove();

  if (button.pin == PIN_BUTTON_CW) {
    if (pressed) {
      Serial.println("[BTN] CW press");
      stopPending = false;
      requestCW();
    } else {
      Serial.println("[BTN] CW release");
      requestSafeStop();
    }
  }

  if (button.pin == PIN_BUTTON_CCW) {
    if (pressed) {
      Serial.println("[BTN] CCW press");
      stopPending = false;
      requestCCW();
    } else {
      Serial.println("[BTN] CCW release");
      requestSafeStop();
    }
  }

  if (button.pin == PIN_BUTTON_STOP && pressed) {
    Serial.println("[BTN] STOP press");
    clearAllErrors();

    if (getMotorState() == STOP) {
      resetAzimuthSystem();
    }

    requestSafeStop();
  }

  applyButtonUiRefresh();
}

static void updateButton(ButtonState &button) {
  const bool reading = digitalRead(button.pin);
  const uint32_t now = millis();

  if (reading != button.lastReading) {
    button.lastReading = reading;
    button.lastChangeMs = now;
  }

  if ((now - button.lastChangeMs) >= BUTTON_DEBOUNCE_MS && reading != button.stableLevel) {
    button.stableLevel = reading;
    handleButtonEvent(button, button.stableLevel == LOW);
  }
}

void buttonsInit() {
  pinMode(PIN_BUTTON_CW, INPUT_PULLUP);
  pinMode(PIN_BUTTON_CCW, INPUT_PULLUP);
  if (ENABLE_STOP_BUTTON) {
    pinMode(PIN_BUTTON_STOP, INPUT_PULLUP);
  }

  const uint32_t now = millis();

  buttonCW.lastReading = digitalRead(buttonCW.pin);
  buttonCW.stableLevel = buttonCW.lastReading;
  buttonCW.lastChangeMs = now;

  buttonCCW.lastReading = digitalRead(buttonCCW.pin);
  buttonCCW.stableLevel = buttonCCW.lastReading;
  buttonCCW.lastChangeMs = now;

  buttonStop.lastReading = HIGH;
  buttonStop.stableLevel = HIGH;
  buttonStop.lastChangeMs = now;

  if (ENABLE_STOP_BUTTON) {
    buttonStop.lastReading = digitalRead(buttonStop.pin);
    buttonStop.stableLevel = buttonStop.lastReading;
  }

  currentButtonMode = evaluateButtonMode();
  stopPending = false;
}

void updateButtons() {
  updateButton(buttonCW);
  updateButton(buttonCCW);
  if (ENABLE_STOP_BUTTON) {
    updateButton(buttonStop);
  }

  if (evaluateButtonMode() == BUTTON_CONFLICT) {
    requestSafeStop();
    applyButtonUiRefresh();
  }

  if (stopPending && !isPulseContactClosed()) {
    stopPending = false;
    setStop();
    refreshStatusAndDisplay();
  }
}
