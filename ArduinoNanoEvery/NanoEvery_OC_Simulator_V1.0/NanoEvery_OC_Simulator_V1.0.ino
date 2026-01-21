/*
 * ===========================================================================================
 * Project:  FOR-TC-OC Ver 1.3.2 - Open Collector Bus Simulator
 * Platform: Arduino Nano Every
 * Hardware: SIG_PIN (D10) / DRV_PIN (D11)
 * Logic:    Standard UART (300 bps, 9-bit, LSB First)
 * ===========================================================================================
 */

#include <Arduino.h>

// 1. ピンアサイン定義
const uint8_t SIG_PIN = 10;   // [RX] 5V-SIG監視用
const uint8_t DRV_PIN = 11;   // [TX] Q2 MOSFET ゲート駆動用

// 2. 通信タイミング設定
const uint32_t BAUD   = 300;
const uint32_t BIT_US = 1000000UL / BAUD; // $BIT\_US \approx 3,333\,\mu\text{s}$

// 3. 動作モード選択 (1:送信 / 2:受信 / 3:エコー)
#define MODE 1

// 4. バス制御用インライン関数
static inline void busRelease() { digitalWrite(DRV_PIN, LOW); }
static inline void busPullLow() { digitalWrite(DRV_PIN, HIGH); }
static inline bool busIsHigh() { return digitalRead(SIG_PIN) == HIGH; }

static inline void sendBit(bool bit1) {
  if (bit1) busRelease();
  else      busPullLow();
  delayMicroseconds(BIT_US);
}

// 5. 通信エンジン (9-bit Bit-Bang)
void send9(uint16_t v9) {
  busRelease(); delayMicroseconds(BIT_US); 
  sendBit(false); // Start
  for (int i = 0; i < 8; i++) { sendBit((v9 >> i) & 1); } // Data
  sendBit((v9 >> 8) & 1); // 9th bit
  sendBit(true); // Stop
  delayMicroseconds(BIT_US * 2);
}

bool recv9(uint16_t &out) {
  uint32_t t0;
  t0 = millis();
  while (!busIsHigh()) { if (millis() - t0 > 1000) return false; }
  t0 = millis();
  while (busIsHigh()) { if (millis() - t0 > 1000) return false; }

  delayMicroseconds(BIT_US + (BIT_US / 2));
  uint16_t v = 0;
  for (int i = 0; i < 9; i++) {
    if (busIsHigh()) v |= (1U << i);
    delayMicroseconds(BIT_US);
  }
  delayMicroseconds(BIT_US / 2);
  if (!busIsHigh()) return false;
  out = v;
  return true;
}

// 6. メインロジック
void setup() {
  Serial.begin(115200);
  pinMode(SIG_PIN, INPUT);
  pinMode(DRV_PIN, OUTPUT);
  busRelease();
  
  Serial.println(F("\n=============================================="));
  Serial.println(F("  Nano Every TC106 Simulator Ver 1.3.2"));
  Serial.print(F("  Current Mode: "));
  Serial.println(MODE);
  Serial.println(F("=============================================="));
}

void loop() {
#if MODE == 1
  const uint16_t patterns[] = {0x055, 0x0AA, 0x000, 0x0FF, (uint16_t)(0x100 | 0x55)};
  for (auto v : patterns) {
    Serial.print(F("TX -> 0x"));
    if (v < 0x100) Serial.print(F("0")); // 3桁16進数表示の整形
    if (v < 0x10) Serial.print(F("0"));
    Serial.println(v, HEX);
    send9(v);
    delay(500);
  }

#elif MODE == 2
  uint16_t v;
  if (recv9(v)) {
    Serial.print(F("RX <- 0x"));
    Serial.print(v & 0x1FF, HEX);
    Serial.print(F(" (b9="));
    Serial.print((v >> 8) & 1);
    Serial.println(F(")"));
  }

#elif MODE == 3
  uint16_t v;
  if (recv9(v)) {
    Serial.print(F("RX <- 0x"));
    Serial.print(v, HEX);
    Serial.println(F(" : Echoing..."));
    delayMicroseconds(BIT_US); 
    send9(v);
  }
#endif
}