/*
  TC-Emulator  – Arduino Nano Every
  --------------------------------
  • BitBang TX:  pin 2   (to ESP32 RXB)
  • BitBang RX:  pin 3   (from ESP32 TXB) ← 今回は未使用
  • Baud: 300 bps, LSB-first, 8N1
  --------------------------------
  2025-07-15  – “CMD1” を 1 秒ごとに自動送信する改訂版
*/

#include <Arduino.h>

#define BB_TX_PIN 2
#define BB_RX_PIN 3          // 受信は使わないが念のため宣言
#define BIT_DELAY_US 3333    // 300 bps ≅ 3333 µs/bit

// ─────────────────────────────
// 1) CMD1 = 01 06 05 00 00 0C
//    LSB-firstで送るのでビット反転した値を保持
// ─────────────────────────────
static const uint8_t CMD1[6] = {
  0x80, 0x60, 0xA0, 0x00, 0x00, 0x30   // rev8(01)=80, rev8(06)=60, …
};

// ── 8bit ビット反転 (汎用) ──
static inline uint8_t rev8(uint8_t v)
{
  v = (v >> 4) | (v << 4);
  v = ((v & 0xCC) >> 2) | ((v & 0x33) << 2);
  v = ((v & 0xAA) >> 1) | ((v & 0x55) << 1);
  return v;
}

// ── 1バイト LSB-first 送信 ──
void bitBangSendByte(uint8_t b)
{
  digitalWrite(BB_TX_PIN, LOW);               // start
  delayMicroseconds(BIT_DELAY_US);

  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(BB_TX_PIN, (b >> i) & 1);
    delayMicroseconds(BIT_DELAY_US);
  }

  digitalWrite(BB_TX_PIN, HIGH);              // stop
  delayMicroseconds(BIT_DELAY_US);
}

// ── 6バイト一括送信 ──
void sendCmd1()
{
  Serial.print(F("[EMU] TX:"));
  for (uint8_t i = 0; i < 6; i++) {
    bitBangSendByte(CMD1[i]);
    Serial.printf(" %02X", CMD1[i]);
  }
  Serial.println();
}

// ── SETUP ────────────────────
void setup()
{
  Serial.begin(115200);

  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);   // idle HIGH
  pinMode(BB_RX_PIN, INPUT_PULLUP);

  Serial.println(F("TC-Emulator ready – sending CMD1 every 1 s"));
}

// ── LOOP ─────────────────────
void loop()
{
  static unsigned long prevSend = 0;

  if (millis() - prevSend >= 1000) {   // 1 s 間隔
    prevSend = millis();
    sendCmd1();
  }
}
