#pragma once

#include <Arduino.h>

// Serial number of this physical rotation unit.
// Keep this value aligned with RotationKonWT DEVICE_ID for the same device.
constexpr const char *DEVICE_ID = "r1p-005";

// Relay GPIO pins on the main controller.
// K1 is the CW direction relay.
constexpr uint8_t PIN_RELAY_K1 = 5;
// K2 is the CCW direction relay.
constexpr uint8_t PIN_RELAY_K2 = 4;
// K3 is the motor power relay.
constexpr uint8_t PIN_RELAY_K3 = 21;

// Relay output inversion.
// false: LOW stays LOW, HIGH stays HIGH.
// true: LOW becomes HIGH, HIGH becomes LOW.
// Use true for active-low relay boards.
constexpr bool RELAY_K1_INVERT = true;
constexpr bool RELAY_K2_INVERT = true;
constexpr bool RELAY_K3_INVERT = false;

// Button GPIO pins. Buttons are expected to pull the input to GND when pressed.
// Local CW/manual-right button.
constexpr uint8_t PIN_BUTTON_CW = 27;
// Local CCW/manual-left button.
constexpr uint8_t PIN_BUTTON_CCW = 26;
// Local STOP/zero button.
constexpr uint8_t PIN_BUTTON_STOP = 32;
// Pulse sensor input. GPIO35 is input-only on ESP32 and needs external wiring/pullup.
constexpr uint8_t PIN_PULSE_INPUT = 35;
// Set false to ignore the STOP button completely in firmware.
constexpr bool ENABLE_STOP_BUTTON = true;

// OLED I2C pins.
constexpr uint8_t PIN_OLED_SDA = 17;
constexpr uint8_t PIN_OLED_SCL = 16;

// UART link pins for the external Ethernet/Wi-Fi bridge board.
// RX receives data from the bridge TX.
constexpr uint8_t PIN_UART_LINK_RX = 19;
// TX sends data to the bridge RX.
constexpr uint8_t PIN_UART_LINK_TX = 18;
// UART speed for packets between RotationKon and the bridge.
constexpr uint32_t UART_LINK_BAUD = 9600;

// Delay before changing direction after the opposite direction was active.
// This protects the relays and motor driver from instant reverse switching.
constexpr uint32_t DIRECTION_CHANGE_DELAY_MS = 250;
// On stop, K3 motor power is switched off first; K1/K2 are released after this delay.
constexpr uint32_t RELAY_K3_STOP_LEAD_MS = 160;
// Button debounce time for local manual buttons.
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;
// Startup logo display time before the live status screen appears.
constexpr uint32_t OLED_SPLASH_DELAY_MS = 1200;
// Minimum interval between OLED redraws while the display is awake.
constexpr uint32_t OLED_REFRESH_MS = 80;
// Display sleep timeout after no local button activity.
constexpr uint32_t OLED_IDLE_TIMEOUT_MS = 20000;
// Minimum interval for saving azimuth to flash during movement.
constexpr uint32_t AZIMUTH_SAVE_INTERVAL_MS = 1000;
// Reserved delay for automatic azimuth start behavior.
constexpr uint32_t AZIMUTH_AUTO_START_DELAY_MS = 2000;
// Grace period after auto-move start before pulse-loss can stop the move.
constexpr uint32_t AZIMUTH_PULSE_GRACE_MS = 1000;
// Used for move timeout estimation until a real pulse interval is known.
constexpr uint32_t AZIMUTH_FALLBACK_PULSE_INTERVAL_MS = 250;
// Extra safety time added to estimated automatic move duration.
constexpr uint32_t AZIMUTH_MOVE_TIMEOUT_MARGIN_MS = 400;
// Number of counted pulse edges to stop before the target.
// With half-degree counting, 1 means 0.5 degree early; 0 means no early stop.
constexpr uint32_t AZIMUTH_STOP_LEAD_PULSES = 0;
// false: use direct target-current difference and do not wrap across 0/360.
// true: use the shortest path and allow movement across 0/360.
constexpr bool AZIMUTH_ALLOW_ZERO_CROSSING = true;
// Reserved repeat interval for target adjustment controls.
constexpr uint32_t TARGET_ADJUST_REPEAT_MS = 180;
// Debounce time for pulse input edges.
constexpr uint32_t PULSE_DEBOUNCE_MS = 5;
// Short startup grace period after motor start before checking for missing pulses.
constexpr uint32_t PULSE_STARTUP_GRACE_MS = 240;
// Minimum no-pulse timeout while the motor is moving.
constexpr uint32_t PULSE_NO_SIGNAL_TIMEOUT_MS = 240;
// Extra time added to dynamic pulse timeout calculated from the last interval.
constexpr uint32_t PULSE_TIMEOUT_MARGIN_MS = 80;
// Dynamic timeout multiplier based on the last measured pulse interval.
constexpr uint32_t PULSE_TIMEOUT_INTERVAL_MULTIPLIER = 3;
// true: LOW means pulse contact/sensor is active. false: HIGH means active.
constexpr bool PULSE_ACTIVE_LOW = true;
// Print every counted pulse interval to Serial for debugging.
constexpr bool ENABLE_PULSE_INTERVAL_DEBUG = true;
// Maximum UART bytes processed per main loop tick.
constexpr uint8_t UART_LINK_MAX_READS_PER_LOOP = 32;

// OLED settings.
// Display width in pixels.
constexpr int OLED_WIDTH = 128;
// Display height in pixels.
constexpr int OLED_HEIGHT = 64;
// I2C address of the OLED module.
constexpr uint8_t OLED_ADDRESS = 0x3C;

// OLED driver selection. Enable exactly one driver.
// Set to 1 for SSD1306 OLED modules.
#define CONFIG_OLED_1306 1
// Set to 1 for SH1106 OLED modules.
#define CONFIG_OLED_1106 0
