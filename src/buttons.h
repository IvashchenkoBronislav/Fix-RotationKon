#pragma once

enum ButtonMode {
  BUTTON_IDLE = 0,
  BUTTON_CW,
  BUTTON_CCW,
  BUTTON_STOP,
  BUTTON_CONFLICT
};

void buttonsInit();
void updateButtons();
ButtonMode getButtonMode();
const char *buttonModeToString(ButtonMode mode);
