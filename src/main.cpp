#include <Arduino.h>

#include "azimuth.h"
#include "buttons.h"
#include "config.h"
#include "display.h"
#include "errors.h"
#include "motor.h"
#include "pulse.h"
#include "serial_cli.h"
#include "status.h"
#include "uart_link.h"

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("Rotor controller");
  Serial.print("DEVICE_ID: ");
  Serial.println(DEVICE_ID);
  Serial.println("Starting...");
  Serial.println("Init: motor");

  clearAllErrors();
  motorInit();
  setStop();
  azimuthInit();
  pulseInit();

  Serial.println("Init: buttons");
  buttonsInit();

  Serial.println("Init: display");
  displayInit();

  Serial.println("Init: serial cli");
  serialCliInit();

  Serial.println("Init: uart");
  uartLinkInit();

  Serial.println("Rotor controller start");
  Serial.println("Board: WT32-ETH01");
  Serial.println("Ready");

  refreshStatusAndDisplay();
}

void loop() {
  displayLoop();
  updateButtons();
  pulseUpdate();
  azimuthUpdate();
  serialCliUpdate();
  uartLinkUpdate();
}
