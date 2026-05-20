#include "uart_link.h"

#include <Arduino.h>
#include <HardwareSerial.h>

#include "azimuth.h"
#include "config.h"
#include "errors.h"
#include "motor.h"

namespace {
constexpr uint8_t FRAME_SYNC_1 = 0xAA;
constexpr uint8_t FRAME_SYNC_2 = 0x55;
constexpr uint8_t PACKET_PING = 0x01;
constexpr uint8_t PACKET_STATUS = 0x10;
constexpr uint8_t PACKET_GET_ANGLE = 0x20;
constexpr uint8_t PACKET_GOTO_AZIMUTH = 0x21;
constexpr uint8_t PACKET_STOP = 0x22;
constexpr uint8_t PACKET_PEER_IP = 0x30;
constexpr uint8_t PACKET_ACK = 0x7F;
constexpr size_t MAX_PAYLOAD_SIZE = 16;

struct Packet {
  uint8_t type;
  uint8_t length;
  uint8_t payload[MAX_PAYLOAD_SIZE];
};

enum RxState : uint8_t {
  RX_WAIT_SYNC_1 = 0,
  RX_WAIT_SYNC_2,
  RX_WAIT_TYPE,
  RX_WAIT_LENGTH,
  RX_WAIT_PAYLOAD,
  RX_WAIT_CHECKSUM
};

HardwareSerial uartLink(2);
RxState rxState = RX_WAIT_SYNC_1;
Packet rxPacket = {};
uint8_t rxPayloadIndex = 0;
uint8_t rxChecksum = 0;

MotorState lastSentState = STOP;
int lastSentAngle = -1;
uint8_t lastSentErrors = 0xFF;
bool statusDirty = true;
uint8_t lastRxCommand = 0;
uint32_t lastRxCommandMs = 0;
char peerIpText[MAX_PAYLOAD_SIZE + 1] = "";

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }

  Serial.print(value, HEX);
}

uint8_t computeChecksum(const Packet &packet) {
  uint8_t checksum = packet.type ^ packet.length;

  for (uint8_t i = 0; i < packet.length; ++i) {
    checksum ^= packet.payload[i];
  }

  return checksum;
}

void sendPacket(const Packet &packet) {
  uartLink.write(FRAME_SYNC_1);
  uartLink.write(FRAME_SYNC_2);
  uartLink.write(packet.type);
  uartLink.write(packet.length);

  for (uint8_t i = 0; i < packet.length; ++i) {
    uartLink.write(packet.payload[i]);
  }

  uartLink.write(computeChecksum(packet));
}

void sendAck(uint8_t commandType) {
  Packet packet = {};
  packet.type = PACKET_ACK;
  packet.length = 1;
  packet.payload[0] = commandType;
  sendPacket(packet);

  Serial.print("[UART] TX ack 0x");
  printHexByte(commandType);
  Serial.println();
}

void sendStatusPacket() {
  Packet packet = {};
  const int angle = getAzimuthDegrees();
  const int targetAngle = getTargetAzimuthDegrees();
  const uint8_t errorFlags = getErrorFlags();
  const MotorState state = getMotorState();
  const uint8_t flags = isAzimuthMoveActive() ? 0x01 : 0x00;

  packet.type = PACKET_STATUS;
  packet.length = 7;
  packet.payload[0] = static_cast<uint8_t>(state);
  packet.payload[1] = static_cast<uint8_t>(angle & 0xFF);
  packet.payload[2] = static_cast<uint8_t>((angle >> 8) & 0xFF);
  packet.payload[3] = errorFlags;
  packet.payload[4] = flags;
  packet.payload[5] = static_cast<uint8_t>(targetAngle & 0xFF);
  packet.payload[6] = static_cast<uint8_t>((targetAngle >> 8) & 0xFF);
  sendPacket(packet);

  lastSentState = state;
  lastSentAngle = angle;
  lastSentErrors = errorFlags;
  statusDirty = false;

  Serial.print("[UART] TX status state=");
  Serial.print(motorStateToString(state));
  Serial.print(" angle=");
  Serial.print(angle);
  Serial.print(" target=");
  Serial.print(targetAngle);
  Serial.print(" exec=");
  Serial.print((flags & 0x01) ? "ON" : "OFF");
  Serial.print(" errors=0x");
  printHexByte(errorFlags);
  Serial.println();
}

void rememberCommand(uint8_t commandType) {
  lastRxCommand = commandType;
  lastRxCommandMs = millis();
}

void handlePacket(const Packet &packet) {
  if (packet.type == PACKET_PING) {
    rememberCommand(PACKET_PING);
    Serial.println("[UART] RX ping");
    sendAck(PACKET_PING);
    statusDirty = true;
    return;
  }

  if (packet.type == PACKET_GET_ANGLE) {
    rememberCommand(PACKET_GET_ANGLE);
    Serial.println("[UART] RX get angle");
    sendAck(PACKET_GET_ANGLE);
    statusDirty = true;
    return;
  }

  if (packet.type == PACKET_GOTO_AZIMUTH) {
    if (packet.length < 2) {
      Serial.println("[UART] RX goto invalid length");
      return;
    }

    const int targetAngle = static_cast<int>(
        static_cast<uint16_t>(packet.payload[0]) |
        (static_cast<uint16_t>(packet.payload[1]) << 8));

    rememberCommand(PACKET_GOTO_AZIMUTH);
    if (isAzimuthMoveLocked()) {
      Serial.print("[UART] RX goto ignored during lock target=");
      Serial.println(targetAngle);
      sendAck(PACKET_GOTO_AZIMUTH);
      statusDirty = true;
      return;
    }

    startAzimuthMove(targetAngle);
    Serial.print("[UART] RX goto target=");
    Serial.println(getTargetAzimuthDegrees());
    sendAck(PACKET_GOTO_AZIMUTH);
    statusDirty = true;
    return;
  }

  if (packet.type == PACKET_STOP) {
    rememberCommand(PACKET_STOP);
    cancelAzimuthMove();
    setStop();
    Serial.println("[UART] RX stop");
    sendAck(PACKET_STOP);
    statusDirty = true;
    return;
  }

  if (packet.type == PACKET_PEER_IP) {
    const size_t copyLength = packet.length < MAX_PAYLOAD_SIZE ? packet.length : MAX_PAYLOAD_SIZE;

    for (size_t i = 0; i < copyLength; ++i) {
      peerIpText[i] = static_cast<char>(packet.payload[i]);
    }
    peerIpText[copyLength] = '\0';

    rememberCommand(PACKET_PEER_IP);
    Serial.print("[UART] RX peer ip=");
    Serial.println(peerIpText);
    sendAck(PACKET_PEER_IP);
    return;
  }

  if (packet.type == PACKET_ACK) {
    Serial.print("[UART] RX unexpected ack 0x");
    if (packet.length > 0) {
      printHexByte(packet.payload[0]);
    } else {
      Serial.print("00");
    }
    Serial.println();
    return;
  }

  if (packet.type == PACKET_STATUS) {
    Serial.println("[UART] RX unexpected status");
    return;
  }

  Serial.print("[UART] RX unknown type=0x");
  printHexByte(packet.type);
  Serial.println();
}

void processIncomingByte(uint8_t value) {
  switch (rxState) {
    case RX_WAIT_SYNC_1:
      if (value == FRAME_SYNC_1) {
        rxState = RX_WAIT_SYNC_2;
      }
      break;

    case RX_WAIT_SYNC_2:
      rxState = (value == FRAME_SYNC_2) ? RX_WAIT_TYPE : RX_WAIT_SYNC_1;
      break;

    case RX_WAIT_TYPE:
      rxPacket.type = value;
      rxChecksum = value;
      rxState = RX_WAIT_LENGTH;
      break;

    case RX_WAIT_LENGTH:
      rxPacket.length = value;
      rxChecksum ^= value;
      rxPayloadIndex = 0;

      if (rxPacket.length > MAX_PAYLOAD_SIZE) {
        Serial.println("[UART] RX packet too long");
        rxState = RX_WAIT_SYNC_1;
      } else if (rxPacket.length == 0) {
        rxState = RX_WAIT_CHECKSUM;
      } else {
        rxState = RX_WAIT_PAYLOAD;
      }
      break;

    case RX_WAIT_PAYLOAD:
      rxPacket.payload[rxPayloadIndex++] = value;
      rxChecksum ^= value;

      if (rxPayloadIndex >= rxPacket.length) {
        rxState = RX_WAIT_CHECKSUM;
      }
      break;

    case RX_WAIT_CHECKSUM:
      if (value == rxChecksum) {
        handlePacket(rxPacket);
      } else {
        Serial.print("[UART] RX checksum mismatch rx=0x");
        printHexByte(value);
        Serial.print(" calc=0x");
        printHexByte(rxChecksum);
        Serial.println();
      }

      rxState = RX_WAIT_SYNC_1;
      break;
  }
}
}

void uartLinkInit() {
  pinMode(PIN_UART_LINK_RX, INPUT_PULLUP);
  uartLink.begin(UART_LINK_BAUD, SERIAL_8N1, PIN_UART_LINK_RX, PIN_UART_LINK_TX);

  Serial.print("UART link RX=");
  Serial.println(PIN_UART_LINK_RX);
  Serial.print("UART link TX=");
  Serial.println(PIN_UART_LINK_TX);
  Serial.print("UART link baud=");
  Serial.println(UART_LINK_BAUD);

  statusDirty = true;
  lastSentState = STOP;
  lastSentAngle = -1;
  lastSentErrors = 0xFF;
  lastRxCommand = 0;
  lastRxCommandMs = 0;
}

void uartLinkUpdate() {
  uint8_t reads = 0;
  while (uartLink.available() > 0 && reads < UART_LINK_MAX_READS_PER_LOOP) {
    processIncomingByte(static_cast<uint8_t>(uartLink.read()));
    reads++;
  }

  const MotorState currentState = getMotorState();
  const int currentAngle = getAzimuthDegrees();
  const uint8_t currentErrors = getErrorFlags();

  if (currentState != lastSentState || currentAngle != lastSentAngle || currentErrors != lastSentErrors) {
    statusDirty = true;
  }

  if (statusDirty) {
    sendStatusPacket();
  }
}

const char *uartLinkGetPeerIp() {
  return peerIpText;
}
