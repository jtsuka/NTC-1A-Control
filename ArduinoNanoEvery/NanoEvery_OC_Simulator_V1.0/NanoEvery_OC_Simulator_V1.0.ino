/*
 * ===========================================================================================
 * Project:  FOR-TC-OC Ver 1.3.2 - Open Collector Bus Simulator
 * Platform: Arduino Nano Every
 * Author:   Junichi (Planning Office) & Gemini Thought Partner
 * Date:     2026.01.21
 * * [Hardware Context]
 * - Board:  FOR-TC-OC Ver 1.3.2 (Custom PCB)
 * - Input:  SIG_PIN (D10) - Monitors the 5V-SIG line directly.
 * - Output: DRV_PIN (D11) - Drives N-ch MOSFET (Q2) Gate to pull bus to GND.
 * * [Communication Specs]
 * - Baud:   300 bps (1 bit length: approx 3333 us)
 * - Logic:  Standard UART (Idle=High, Start=Low, 8-bit Data, 9th Bit, 1-bit Stop)
 * - Format: LSB First
 * ===========================================================================================
 */

#include <Arduino.h>

// -------------------------------------------------------------------------------------------
// 1. ピンアサイン定義 (Ver 1.3.2 回路図準拠)
// -------------------------------------------------------------------------------------------
const uint8_t SIG_PIN = 10;   // [RX] 5V-SIG監視用
const uint8_t DRV_PIN = 11;   // [TX] Q2 MOSFET ゲート駆動用

// -------------------------------------------------------------------------------------------
// 2. 通信タイミング設定
// -------------------------------------------------------------------------------------------
const uint32_t BAUD   = 300;
const uint32_t BIT_US = 1000000UL / BAUD; // 1ビットのパルス幅 ($BIT\_US \approx 3333\mu s$)

// -------------------------------------------------------------------------------------------
// 3. 動作モード選択
// -------------------------------------------------------------------------------------------
// 1: 送信単体テスト (Nano -> ESP32) - スニファの受信確認用
// 2: 受信単体テスト (ESP32 -> Nano) - Nanoの受信ロジック確認用
// 3: エコーバック   (ESP32 <-> Nano) - 相互通信・半二重の実戦確認用
#define MODE 1

// -------------------------------------------------------------------------------------------
// 4. バス制御用インライン関数
// -------------------------------------------------------------------------------------------

/** バスを解放し、High（Idle）状態にする (Q2: OFF) */
static inline void busRelease() {
  digitalWrite(DRV_PIN, LOW);
}

/** バスをGNDへ引き下げ、Low状態にする (Q2: ON) */
static inline void busPullLow() {
  digitalWrite(DRV_PIN, HIGH);
}

/** 現在のバスの物理電圧を確認する (Highならtrue) */
static inline bool busIsHigh() {
  return digitalRead(SIG_PIN) == HIGH;
}

/** 1ビット分の論理信号をバスに出力する (bit1=trueならHigh, falseならLow) */
static inline void sendBit(bool bit1) {
  if (bit1) busRelease();
  else      busPullLow();
  delayMicroseconds(BIT_US);
}

// -------------------------------------------------------------------------------------------
// 5. 通信エンジン (9-bit Bit-Bang)
// -------------------------------------------------------------------------------------------

/** 9ビットフレームを送信する */
void send9(uint16_t v9) {
  busRelease(); delayMicroseconds(BIT_US); // 送信前ギャップ

  // [Start Bit] 常にLow
  sendBit(false);

  // [Data Bits 0-7] LSBから順に送信
  for (int i = 0; i < 8; i++) {
    sendBit((v9 >> i) & 1);
  }

  // [9th Bit] コマンド/データの識別用等
  sendBit((v9 >> 8) & 1);

  // [Stop Bit] 常にHigh
  sendBit(true);

  // フレーム間インターバル
  delayMicroseconds(BIT_US * 2);
}

/** 9ビットフレームを受信する (成功時true) */
bool recv9(uint16_t &out) {
  uint32_t t0;

  // 1. バスの張り付き回避：Idle(High)に戻るのを待つ (1秒制限)
  t0 = millis();
  while (!busIsHigh()) {
    if (millis() - t0 > 1000) return false;
  }

  // 2. スタートビット(High->Low)の検出 (1秒制限)
  t0 = millis();
  while (busIsHigh()) {
    if (millis() - t0 > 1000) return false;
  }

  // 3. サンプリングポイントの移動 (スタートビットの1.5ビット分後 = Bit0の中央)
  delayMicroseconds(BIT_US + (BIT_US / 2));

  // 4. 9ビット分のデータ取得
  uint16_t v = 0;
  for (int i = 0; i < 9; i++) {
    if (busIsHigh()) v |= (1U << i);
    delayMicroseconds(BIT_US);
  }

  // 5. ストップビットの中央判定
  delayMicroseconds(BIT_US / 2);
  if (!busIsHigh()) return false; // ストップビットが不正(Low)ならエラー

  out = v;
  return true;
}

// -------------------------------------------------------------------------------------------
// 6. メインロジック
// -------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  pinMode(SIG_PIN, INPUT);
  pinMode(DRV_PIN, OUTPUT);
  busRelease(); // バスの解放を初期状態とする
  
  Serial.println("\n==============================================");
  Serial.println("  Nano Every TC106 Simulator Ver 1.3.2");
  Serial.printf ("  Current Mode: %d\n", MODE);
  Serial.println("==============================================");
}

void loop() {
#if MODE == 1
  // --- MODE 1: 固定パターンの定期送信 ---
  const uint16_t patterns[] = {0x055, 0x0AA, 0x000, 0x0FF, (uint16_t)(0x100 | 0x55)};
  for (auto v : patterns) {
    Serial.printf("TX -> 0x%03X\n", v);
    send9(v);
    delay(500); // 確認しやすいよう少し長めの間隔
  }

#elif MODE == 2
  // --- MODE 2: 受信モニタリング ---
  uint16_t v;
  if (recv9(v)) {
    Serial.printf("RX <- 0x%03X (b9=%d)\n", v & 0x1FF, (v >> 8) & 1);
  }

#elif MODE == 3
  // --- MODE 3: 受信エコーバック (実践用) ---
  uint16_t v;
  if (recv9(v)) {
    Serial.printf("RX <- 0x%03X : Echoing...\n", v);
    // 相手(ESP32)が受信完了し、バスがIdleに戻るのをわずかに待つ
    delayMicroseconds(BIT_US); 
    send9(v);
  }
#endif
}