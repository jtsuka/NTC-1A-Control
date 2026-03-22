/*
  ESP32-S3用 受信スニファースケッチ
  ---------------------------------
  目的:
    - Nano Everyからの5V擬似TC信号を受信して可視化する
    - 基板完成直後の最初の受信確認用
    - まずは300bpsの単純な8N1受信で確認する

  受信仕様:
    - 300bps
    - 8N1
    - idle HIGH
    - start bit LOW
    - data 8bit LSB first
    - stop bit HIGH

  ログ例:
    [RX] 11 22 33 44 55 7F
    [TIMEOUT] partial=3 : 11 22 33

  注意:
    - これは最初の受信確認用スケッチ
    - 実TC106のOC単線プロトコル完全再現ではない
    - まず5V擬似信号がESP32-S3まで届くかを見る
*/

#include <Arduino.h>

static const int RX_PIN = 44;             // 必要に応じて変更
static const uint32_t BAUD_RATE = 300;
static const uint32_t BIT_US = 1000000UL / BAUD_RATE;  // 約3333us

static const size_t FRAME_LEN = 6;
static const uint32_t FRAME_GAP_TIMEOUT_MS = 80;  // 6バイト途中で止まったらpartial判定

uint8_t rxBuf[FRAME_LEN];
size_t rxCount = 0;
uint32_t lastByteMs = 0;

bool readByte300(uint8_t &outByte) {
  // start bit待ち（LOWになるのを待つ）
  if (digitalRead(RX_PIN) != LOW) {
    return false;
  }

  // start bit中央付近 + 1bit先で最初のデータを読む
  delayMicroseconds(BIT_US + (BIT_US / 2));

  uint8_t value = 0;
  for (uint8_t i = 0; i < 8; i++) {
    int bitVal = digitalRead(RX_PIN);
    if (bitVal) {
      value |= (1 << i);   // LSB first
    }
    delayMicroseconds(BIT_US);
  }

  // stop bit位置まで進める
  delayMicroseconds(BIT_US);

  outByte = value;
  return true;
}

void printHexByte(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

void printFrame(const char *prefix, const uint8_t *buf, size_t len) {
  Serial.print(prefix);
  for (size_t i = 0; i < len; i++) {
    printHexByte(buf[i]);
    if (i != len - 1) Serial.print(' ');
  }
  Serial.println();
}

void printPartialTimeout(const uint8_t *buf, size_t len) {
  Serial.print("[TIMEOUT] partial=");
  Serial.print(len);
  Serial.print(" : ");
  for (size_t i = 0; i < len; i++) {
    printHexByte(buf[i]);
    if (i != len - 1) Serial.print(' ');
  }
  Serial.println();
}

void setup() {
  pinMode(RX_PIN, INPUT_PULLUP);  // idle HIGH想定
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== ESP32-S3 TC RX Sniffer start ===");
  Serial.print("RX_PIN = GPIO");
  Serial.println(RX_PIN);
  Serial.print("BAUD   = ");
  Serial.println(BAUD_RATE);
  Serial.print("FRAME_LEN = ");
  Serial.println(FRAME_LEN);
  Serial.println();
}

void loop() {
  uint8_t b = 0;

  if (readByte300(b)) {
    if (rxCount < FRAME_LEN) {
      rxBuf[rxCount++] = b;
      lastByteMs = millis();
    }

    // 6バイト揃ったら表示
    if (rxCount == FRAME_LEN) {
      printFrame("[RX] ", rxBuf, FRAME_LEN);
      rxCount = 0;
    }

    // 同じスタートビットを二重に拾いにくくするため少し待つ
    delayMicroseconds(BIT_US / 2);
  }

  // 途中まで来て止まったら partial 表示
  if (rxCount > 0) {
    if (millis() - lastByteMs > FRAME_GAP_TIMEOUT_MS) {
      printPartialTimeout(rxBuf, rxCount);
      rxCount = 0;
    }
  }
}