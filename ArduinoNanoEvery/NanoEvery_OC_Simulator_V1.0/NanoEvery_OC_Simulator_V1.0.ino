/*
 * ===========================================================================================
 * Project:  FOR-TC-OC Ver 1.3.2 - OC Bus Simulator (Final Test Edition)
 * Hardware: Arduino Nano Every
 * Pins:     SIG=D10 (Monitor), DRV=D11 (Q2 Driver)
 * Logic:    Standard UART (300 bps, 9-bit, LSB First)
 * ===========================================================================================
 */

#include <Arduino.h>

// --- 配線設定 (Ver 1.3.2 準拠) ---
const uint8_t SIG_PIN = 10;   // [RX] 5V-SIG監視
const uint8_t DRV_PIN = 11;   // [TX] Q2 MOSFETゲート駆動 (HIGH=ON=PullLow) [cite: 52]

// --- 通信条件 ---
const uint32_t BAUD   = 300;
const uint32_t BIT_US = 1000000UL / BAUD; // 約 3333 us

// --- 動作モード切替 ---
// 1: 送信単体 (ESP32スニファ確認用)
// 2: 受信単体 (ESP32送信テスト用)
// 3: エコーバック (相互通信用)
#define MODE 1

// --- デバッグ用内部プルアップ設定 ---
#define USE_INTERNAL_PULLUP 0 // 外部プルアップ(R6)故障疑い時のみ 1 にする 

// --- バス制御関数 ---
static inline void busRelease() { digitalWrite(DRV_PIN, LOW); }  // Q2 OFF -> バス解放 (High) [cite: 52]
static inline void busPullLow() { digitalWrite(DRV_PIN, HIGH); } // Q2 ON  -> バス引下 (Low) [cite: 90]
static inline bool busIsHigh()  { return digitalRead(SIG_PIN) == HIGH; }

/** 3桁16進数表示用ヘルパー (Nano Everyのprintf欠如対策) */
void printHex3(uint16_t v9) {
  uint16_t v = (v9 & 0x1FF); // 9bitマスク
  Serial.print(F("0x"));
  if (v < 0x100) Serial.print('0');
  if (v < 0x010) Serial.print('0');
  Serial.print(v, HEX);
}

/** 1ビット出力 (論理1=High, 0=Low) */
static inline void sendBit(bool bit1) {
  if (bit1) busRelease();
  else      busPullLow();
  delayMicroseconds(BIT_US);
}

/** 9bitフレーム送信 (LSB First) */
void send9(uint16_t v9) {
  busRelease(); delayMicroseconds(BIT_US); // アイドル確保

  sendBit(false); // Start (Low)
  for (int i = 0; i < 8; i++) sendBit((v9 >> i) & 1); // Data 0-7
  sendBit((v9 >> 8) & 1); // 9th Bit
  sendBit(true);          // Stop (High)

  busRelease(); delayMicroseconds(BIT_US * 2); // フレーム間隔
}

/** 9bitフレーム受信 */
bool recv9(uint16_t &out) {
  uint32_t t0;
  t0 = millis();
  while (!busIsHigh()) { if (millis() - t0 > 1000) return false; } // Idle待ち
  t0 = millis();
  while (busIsHigh())  { if (millis() - t0 > 1000) return false; } // Startエッジ待ち

  delayMicroseconds(BIT_US + (BIT_US / 2)); // Bit0の中央へ

  uint16_t v = 0;
  for (int i = 0; i < 9; i++) {
    if (busIsHigh()) v |= (1U << i);
    delayMicroseconds(BIT_US);
  }

  delayMicroseconds(BIT_US / 2);
  if (!busIsHigh()) return false; // Stop bitエラー

  out = v;
  return true;
}

void setup() {
  Serial.begin(115200);

  // 安全初期化：先にLOWを確定させてからOUTPUT化
  digitalWrite(DRV_PIN, LOW);
  pinMode(DRV_PIN, OUTPUT);
  busRelease();

  #if USE_INTERNAL_PULLUP
    pinMode(SIG_PIN, INPUT_PULLUP);
  #else
    pinMode(SIG_PIN, INPUT);
  #endif

  Serial.println(F("\n=== Nano Every OC Simulator (Ver 1.3.2) ==="));
  Serial.print(F("MODE: ")); Serial.println(MODE);
}

void loop() {
#if MODE == 1
  const uint16_t patterns[] = {0x055, 0x0AA, 0x000, 0x0FF, (uint16_t)(0x100 | 0x55)};
  for (auto v : patterns) {
    Serial.print(F("TX -> ")); printHex3(v);
    Serial.print(F(" (b9=")); Serial.print((v >> 8) & 1); Serial.println(F(")"));
    send9(v);
    delay(500);
  }

#elif MODE == 2
  uint16_t v;
  if (recv9(v)) {
    Serial.print(F("RX <- ")); printHex3(v);
    Serial.print(F(" (b9=")); Serial.print((v >> 8) & 1); Serial.println(F(")"));
  }

#elif MODE == 3
  uint16_t v;
  if (recv9(v)) {
    Serial.print(F("RX <- ")); printHex3(v); Serial.println(F(" : Echo"));
    delayMicroseconds(BIT_US); 
    send9(v);
  }
#endif
}