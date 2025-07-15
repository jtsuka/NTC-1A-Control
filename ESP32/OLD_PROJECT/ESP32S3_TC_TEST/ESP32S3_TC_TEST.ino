#include <Arduino.h>
#include "esp_rom_sys.h"

#define TX_PIN 2
#define TEST_PIN 8

#define BIT_PAT true  // true=LSB first
#define BAUD_RATE 300
#define BIT_DURATION_US 3333  // 調整可
#define BYTE_GAP_US 10000

// ---------------------------
// LSB/MSB 反転用
uint8_t reverseBits(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

// ---------------------------
// 1バイトをBitBang送信
void bitBangSendByte(uint8_t b) {
  if (BIT_PAT) b = reverseBits(b);

  noInterrupts();
  digitalWrite(TX_PIN, LOW);  // start bit
  esp_rom_delay_us(BIT_DURATION_US);

  for (int i = 0; i < 8; i++) {
    digitalWrite(TX_PIN, (b >> i) & 0x01);
    esp_rom_delay_us(BIT_DURATION_US);
  }

  digitalWrite(TX_PIN, HIGH);  // stop bit
  esp_rom_delay_us(BIT_DURATION_US * 2);  // stop + ギャップ
  interrupts();
}

// ---------------------------
// パケット送信
void sendTestPacket() {
  uint8_t testPacket[] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};

  for (uint8_t i = 0; i < sizeof(testPacket); i++) {
    bitBangSendByte(testPacket[i]);
    esp_rom_delay_us(BYTE_GAP_US);
  }
}

void setup() {
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, HIGH);

  pinMode(TEST_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println("BitBang TX Test Ready");
}

void loop() {
  if (digitalRead(TEST_PIN) == LOW) {
    Serial.println("Sending test packet...");
    sendTestPacket();
    delay(2000);  // デバウンス用
  }

  delay(50);  // CPU節約
}
