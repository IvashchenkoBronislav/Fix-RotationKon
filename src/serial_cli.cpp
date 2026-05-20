#include "serial_cli.h"

#include <Arduino.h>
#include <string.h>

#include "status.h"

namespace {
constexpr size_t SERIAL_BUFFER_SIZE = 32;

char serialBuffer[SERIAL_BUFFER_SIZE];
size_t serialLength = 0;

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  help   - show this help");
  Serial.println("  status - print current WT32 status");
}

void handleCommand() {
  serialBuffer[serialLength] = '\0';

  if (strcmp(serialBuffer, "help") == 0) {
    printHelp();
    return;
  }

  if (strcmp(serialBuffer, "status") == 0) {
    printStatus();
    return;
  }

  Serial.print("[CLI] Unknown command: ");
  Serial.println(serialBuffer);
}
}

void serialCliInit() {
  serialLength = 0;
}

void serialCliUpdate() {
  while (Serial.available() > 0) {
    const char value = static_cast<char>(Serial.read());

    if (value == '\r') {
      continue;
    }

    if (value == '\n') {
      if (serialLength > 0) {
        handleCommand();
        serialLength = 0;
      }
      continue;
    }

    if (serialLength < (SERIAL_BUFFER_SIZE - 1)) {
      serialBuffer[serialLength++] = value;
    }
  }
}
