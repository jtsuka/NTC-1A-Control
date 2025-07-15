/**********************************************************************
  ESP32-S3 TC Repeater - Fixed-Length Packet, BitBang 300bps <-> UART
  安定版 固定長中継機 (FreeRTOS 使用)

  - GPIO2: BitBang TX (to TC)
  - GPIO3: BitBang RX (未使用)
  - GPIO8: UART TX (to Pi)
  - GPIO9: UART RX (from Pi)

  - 固定長: 6バイトパケット中継
  - 受信 → 中継 → 返信応答までの精度重視構成

  更新日: 2025-07-11
**********************************************************************/

#include <Arduino.h>
#include "driver/uart.h"

#define BB_TX_PIN 2
#define BB_BAUD 300
#define BIT_US 3333
#define PACKET_LEN 6

#define UART_RX_PIN 9
#define UART_TX_PIN 8
#define UART_BAUD 9600

uint8_t pktBuf[PACKET_LEN];

// BitBang送信 (LSBファースト)
void sendBitBangPacket(const uint8_t *data) {
  for (int i = 0; i < PACKET_LEN; i++) {
    uint8_t b = data[i];
    noInterrupts();
    digitalWrite(BB_TX_PIN, LOW);  // start bit
    delayMicroseconds(BIT_US);
    for (int j = 0; j < 8; j++) {
      digitalWrite(BB_TX_PIN, (b >> j) & 0x01);
      delayMicroseconds(BIT_US);
    }
    digitalWrite(BB_TX_PIN, HIGH); // stop bit
    delayMicroseconds(BIT_US);
    interrupts();
    delayMicroseconds(BIT_US);     // inter-byte gap
  }
}

void setup() {
  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);

  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  Serial.println("[INFO] Fixed-length TC Repeater started");
}

void loop() {
  if (Serial2.available() >= PACKET_LEN) {
    int n = Serial2.readBytes(pktBuf, PACKET_LEN);
    if (n == PACKET_LEN) {
      Serial.print("[RX UART] ");
      for (int i = 0; i < PACKET_LEN; i++) {
        Serial.printf("%02X ", pktBuf[i]);
      }
      Serial.println();

      // BitBang送信
      sendBitBangPacket(pktBuf);
      Serial.println("[TX BB] sent");
    }
  }
}
