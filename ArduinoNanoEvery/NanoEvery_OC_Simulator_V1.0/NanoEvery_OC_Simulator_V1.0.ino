/*
 * Arduino Nano Every: TC106 OC Emulator (MODE Switchable)
 * Hardware: FOR-TC-OC Ver 1.3.1 / 1.3.2 専用
 * Pins: D10(RX/SIG), D11(TX/DRV via Q2)
 */

#include <Arduino.h>

// ====== 配線設定 (Ver 1.3.2 基板準拠) ======
const uint8_t SIG_PIN = 10;   // 受信 (5V-SIG監視)
const uint8_t DRV_PIN = 11;   // 送信 (Q2ゲート駆動)

// ====== 通信条件 ======
const uint32_t BAUD = 300;
const uint32_t BIT_US = 1000000UL / BAUD; // 約3333us

// ====== 動作モード切替 ======
// 1: 送信テスト（ESP32のスニファで受信確認）
// 2: 受信テスト（シリアルモニタでNanoの受信確認）
// 3: エコーバック（受信したデータをそのまま返す）
#define MODE 1

// --- 通信エンジン (Ver 1.3.2 回路用) ---

// バスを解放 (Highにする): Q2をOFFにする
static inline void busRelease() {
  digitalWrite(DRV_PIN, LOW); 
}

// バスを引き下げる (Lowにする): Q2をONにする
static inline void busPullLow() {
  digitalWrite(DRV_PIN, HIGH);
}

// 現在のバスの物理電圧を確認 (Highならtrue)
static inline bool busIsHigh() {
  return digitalRead(SIG_PIN) == HIGH;
}

// 9bit送信 (標準UART論理: 1=High, 0=Low)
void send9(uint16_t v9) {
  busRelease(); delayMicroseconds(BIT_US); // Gap
  busPullLow(); delayMicroseconds(BIT_US); // Start Bit (0)
  
  for (int i = 0; i < 8; i++) { // Data Bits (0..7)
    if ((v9 >> i) & 1) busRelease(); else busPullLow();
    delayMicroseconds(BIT_US);
  }
  
  if ((v9 >> 8) & 1) busRelease(); else busPullLow(); // Bit 9
  delayMicroseconds(BIT_US);
  
  busRelease(); delayMicroseconds(BIT_US * 2); // Stop Bit (1) & Gap
}

// 9bit受信 (標準UART論理)
bool recv9(uint16_t &out) {
  uint32_t t0 = millis();
  while (!busIsHigh()) { if (millis() - t0 > 1000) return false; } // Idle待ち
  while (busIsHigh()) { if (millis() - t0 > 2000) return false; } // Startエッジ待ち

  delayMicroseconds(BIT_US + (BIT_US / 2)); // 1.5bit移動

  uint16_t v = 0;
  for (int i = 0; i < 9; i++) {
    if (busIsHigh()) v |= (1U << i);
    delayMicroseconds(BIT_US);
  }
  
  delayMicroseconds(BIT_US / 2);
  bool ok = busIsHigh(); // Stop bit確認
  if (ok) out = v;
  return ok;
}

void setup() {
  Serial.begin(115200);
  pinMode(SIG_PIN, INPUT);
  pinMode(DRV_PIN, OUTPUT);
  busRelease(); // 初期状態は Idle(High)
  Serial.println("\n--- Nano Every OC Test (MODE: " + String(MODE) + ") ---");
}

void loop() {
#if MODE == 1
  // ①送信テスト: ESP32側のスニファ V3.3 で正しく読めるか確認
  uint16_t test_val = 0x055; // 01010101
  Serial.print("TX: 0x"); Serial.println(test_val, HEX);
  send9(test_val);
  delay(1000);

#elif MODE == 2
  // ②受信テスト: ESP32から送ったデータがNanoのモニタに出るか確認
  uint16_t v;
  if (recv9(v)) {
    Serial.print("RX: 0x"); Serial.println(v, HEX);
  }

#elif MODE == 3
  // ③エコーバック: 実践テスト
  uint16_t v;
  if (recv9(v)) {
    Serial.print("RX: 0x"); Serial.print(v, HEX); Serial.println(" -> Echo");
    delay(10); // 衝突回避の短いウェイト
    send9(v);
  }
#endif
}