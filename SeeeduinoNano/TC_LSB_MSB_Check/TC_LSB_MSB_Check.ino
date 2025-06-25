/**********************************************************************
  NeoSWSerial_Receiver_LSB_MSB_Compare.ino
  - Seeeduino Nano / Arduino UNO対応
  - D2: RX（TCからの受信信号）
  - D3: TX（未使用）
  - 通信速度：300〜330bps
  - 7バイト固定パケット（0xAA〜0x7F）をLSB/MSB両視点で解析・表示
**********************************************************************/

#include <NeoSWSerial.h>

#define RX_PIN 2
#define TX_PIN 3

NeoSWSerial mySerial(RX_PIN, TX_PIN);

#define PACKET_SIZE 7

void setup() {
  Serial.begin(115200);
  mySerial.begin(300);  // 必要に応じて330に調整
  Serial.println(F("NeoSWSerial Receiver (LSB vs MSB)"));
}

void loop() {
  static uint8_t buffer[PACKET_SIZE];
  static uint8_t idx = 0;

  if (mySerial.available()) {
    uint8_t b = mySerial.read();
    buffer[idx++] = b;

    if (idx == PACKET_SIZE) {
      Serial.println(F("\n==== PACKET RECEIVED ===="));

      // LSB視点
      Serial.print(F("[LSB] "));
      for (uint8_t i = 0; i < PACKET_SIZE; i++) {
        Serial.print(buffer[i], HEX); Serial.print(" ");
      }

      uint8_t chk_lsb = (buffer[1] + buffer[2] + buffer[3] + buffer[4]) & 0xFF;
      Serial.print((chk_lsb == buffer[5]) ? " [OK]" : " [NG]");

      // MSB視点（ビット反転したものを表示）
      Serial.print(F("\n[MSB] "));
      for (uint8_t i = 0; i < PACKET_SIZE; i++) {
        uint8_t flipped = reverse_bits(buffer[i]);
        Serial.print(flipped, HEX); Serial.print(" ");
      }

      uint8_t chk_msb = (reverse_bits(buffer[1]) + reverse_bits(buffer[2]) +
                         reverse_bits(buffer[3]) + reverse_bits(buffer[4])) & 0xFF;
      uint8_t chk_target = reverse_bits(buffer[5]);
      Serial.print((chk_msb == chk_target) ? " [OK]" : " [NG]");

      Serial.println();
      idx = 0;
    }
  }
}

// ビット順を反転（例: 0b00010000 → 0b00001000）
uint8_t reverse_bits(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}
