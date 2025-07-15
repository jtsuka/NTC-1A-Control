/**********************************************************************
  NTC-1A TC Emulator (ESP32-S3 Bit-Bang UART 300bps Test Sender)
  - GPIO2: TX (TC side)
  - Sends fixed 6-byte test packet repeatedly
  - Uses esp_rom_delay_us() for timing precision

  Packet: 01 06 05 00 00 0C
**********************************************************************/

#include <Arduino.h>
#include "esp_rom_sys.h"  // for esp_rom_delay_us()

#define BB_TX_PIN 2
#define BB_BAUD 300
#define BIT_DURATION_US 3333  // 1bit = 3333us for 300bps
#define BYTE_GAP_US    (BIT_DURATION_US * 3)  // Inter-byte delay

#define BIT_PAT true  // true = LSB first

uint8_t testPacket[] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};

uint8_t reverseBits(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

void bitBangSendByte(uint8_t b) {
  if (BIT_PAT) b = reverseBits(b);  // LSB-first: reverse bits beforehand

  noInterrupts();

  // Start bit (LOW)
  digitalWrite(BB_TX_PIN, LOW);
  esp_rom_delay_us(BIT_DURATION_US);

  // Data bits (MSB first)
  for (int i = 7; i >= 0; --i) {
    digitalWrite(BB_TX_PIN, (b >> i) & 0x01);
    esp_rom_delay_us(BIT_DURATION_US);
  }

  // Stop bit (HIGH)
  digitalWrite(BB_TX_PIN, HIGH);
  esp_rom_delay_us(BIT_DURATION_US);  // stop
//  esp_rom_delay_us(BIT_DURATION_US * 2);  // stop + gap

  interrupts();
}

void sendTestPacket() {
  for (uint8_t i = 0; i < sizeof(testPacket); i++) {
    bitBangSendByte(testPacket[i]);
    esp_rom_delay_us(BYTE_GAP_US);  // Gap between bytes
  }
  esp_rom_delay_us(50000);  // Extra wait before next send
}

void setup() {
  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);
  delay(1000);  // Wait for logic analyzer ready
}

void loop() {
  sendTestPacket();
  delay(2000);  // Send every 2 seconds
}
