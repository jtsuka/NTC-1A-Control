/*
  Nano Every用 5V擬似TC送信スケッチ
  -----------------------------------
  目的:
    - ESP32-S3受信スニファー基板の最初の受信確認用
    - 5Vレベルの単純な300bps信号を周期送信する
    - 実TC106完全再現ではなく、最初の物理疎通確認を優先

  送信仕様:
    - 300bps
    - 8N1
    - idle HIGH
    - start bit LOW
    - data 8bit LSB first
    - stop bit HIGH

  接続例:
    Nano Every D10  -> ESP32-S3基板のTC受信入力側
    Nano Every GND  -> ESP32-S3基板 GND

  注意:
    - これは「5V擬似TC信号」の最初の確認用です
    - 単線OC完全再現ではありません
*/

#include <Arduino.h>

static const uint8_t TX_PIN = 10;
static const uint32_t BAUD_RATE = 300;
static const uint32_t BIT_US = 1000000UL / BAUD_RATE;  // 約3333us

// 実TC106の6バイト返送の雰囲気に寄せたテストパターン
uint8_t frameA[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x7F};
uint8_t frameB[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x7F};
uint8_t frameC[6] = {0x34, 0x12, 0x18, 0x09, 0x00, 0x7F};

void sendByte300(uint8_t b) {
  // start bit
  digitalWrite(TX_PIN, LOW);
  delayMicroseconds(BIT_US);

  // data bits (LSB first)
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(TX_PIN, (b & 0x01) ? HIGH : LOW);
    delayMicroseconds(BIT_US);
    b >>= 1;
  }

  // stop bit
  digitalWrite(TX_PIN, HIGH);
  delayMicroseconds(BIT_US);
}

void sendFrame300(const uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i++) {
    sendByte300(buf[i]);
  }
}

void printFrame(const char *label, const uint8_t *buf, size_t len) {
  Serial.print(label);
  Serial.print(" : ");
  for (size_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    if (i != len - 1) Serial.print(' ');
  }
  Serial.println();
}

void setup() {
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, HIGH);  // idle HIGH

  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== Nano Every 5V pseudo-TC sender start ===");
  Serial.print("TX_PIN = D");
  Serial.println(TX_PIN);
  Serial.print("BAUD   = ");
  Serial.println(BAUD_RATE);
  Serial.println("Frame length = 6 bytes");
  Serial.println();
}

void loop() {
  static uint8_t mode = 0;

  switch (mode) {
    case 0:
      printFrame("[TX frameA]", frameA, sizeof(frameA));
      sendFrame300(frameA, sizeof(frameA));
      break;

    case 1:
      printFrame("[TX frameB]", frameB, sizeof(frameB));
      sendFrame300(frameB, sizeof(frameB));
      break;

    case 2:
      // 毎回少し変化させる
      frameC[0]++;
      frameC[3]++;
      printFrame("[TX frameC]", frameC, sizeof(frameC));
      sendFrame300(frameC, sizeof(frameC));
      break;
  }

  mode++;
  if (mode >= 3) mode = 0;

  delay(1000);
}