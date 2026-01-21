/*
 * ===========================================================================================
 * Project:  FOR-TC-OC Ver 1.3.2 - ESP32-S3 Sniffer
 * Platform: XIXO ESP32-S3 (Custom PCB)
 * Pins:     TX = GPIO 4 (Q1 Driver), RX = GPIO 9 (Input Protected)
 * Logic:    Standard UART (Idle=High, Start=Low, 300bps, 9-bit)
 * ===========================================================================================
 */

#include <Arduino.h>

// --- 回路図 (Ver 1.3.2) に基づくピン定義 --- [cite: 191, 197-201]
const int RX_PIN = 9;  // 受信保護回路 (D1, D2, R3) 経由
const int TX_PIN = 4;  // Q1-2N7000 MOSFET ゲート駆動用

// --- 通信設定 ---
const uint32_t BAUD   = 300;
const uint32_t BIT_US = 1000000UL / BAUD; // 約3333us

#define USE_INTERNAL_PULLUP 0 // 外部プルアップ(R6等)が正常なら0

// 統計用カウンター
uint32_t total_attempts = 0;  // スタートエッジを検出した総数
uint32_t good_frames    = 0;  // 正常に受信完了した数
uint32_t framing_errors = 0;  // ストップビット異常の数

/** 物理的なバスの電圧状態を確認 (High = true) */
static inline bool busPhysicalHigh() {
  return digitalRead(RX_PIN) == HIGH;
}

void setup() {
  Serial.begin(115200);

  // 1. ChatGPT推奨の安全な初期化順序：先にLOW(OFF)を確定させてからOUTPUT化
  digitalWrite(TX_PIN, LOW);   // MOSFET Q1 を確実にOFF（バス解放）に固定
  pinMode(TX_PIN, OUTPUT);
  
  #if USE_INTERNAL_PULLUP
    pinMode(RX_PIN, INPUT_PULLUP);
  #else
    pinMode(RX_PIN, INPUT); 
  #endif

  Serial.println("\n--- TC106 Sniffer V3.3 (GPIO 4/9) FINAL ---");
}

uint16_t read9BitFrame() {
  uint32_t t0;

  // 2. Bus Stuck LOW 監視 (200ms)
  t0 = millis();
  while (!busPhysicalHigh()) {
    if (millis() - t0 > 200) {
      Serial.println("[ERROR] Bus stuck LOW (Check Pull-up/Q1/Q2)");
      return 0xFFFF;
    }
  }

  // 3. No START 監視 (1秒)
  t0 = millis();
  while (busPhysicalHigh()) {
    if (millis() - t0 > 1000) {
      Serial.println("[ERROR] No START edge (Sender power OFF / Wiring?)");
      return 0xFFFE;
    }
  }

  // フレーム受信開始
  total_attempts++;

  // 4. サンプリング点へ移動 (スタートビット1.5倍分後)
  delayMicroseconds(BIT_US + (BIT_US / 2));

  uint16_t data = 0;
  for (int i = 0; i < 9; i++) {
    if (busPhysicalHigh()) data |= (1U << i);
    delayMicroseconds(BIT_US);
  }

  // 5. ストップビット中央判定と復帰処理
  delayMicroseconds(BIT_US / 2); 
  if (!busPhysicalHigh()) {
    framing_errors++;
    Serial.printf("[WARNING] Framing Error (#%lu): Recovery...\n", (unsigned long)framing_errors);
    
    uint32_t t1 = millis();
    while (!busPhysicalHigh()) {
      if (millis() - t1 > 200) return 0xFFFF; // 復帰失敗
      delay(1);
    }
  } else {
    good_frames++; // 正常終了
  }

  return data;
}

void loop() {
  uint16_t v = read9BitFrame();
  
  if (v != 0xFFFF && v != 0xFFFE) {
    // 成功時のみ詳細を表示（型指定を%luで安全に表示）
    Serial.printf("RX: 0x%03X (b9=%d) [Att:%lu Good:%lu Err:%lu]\n", 
                  (unsigned int)(v & 0x1FF),
                  (int)((v >> 8) & 1),
                  (unsigned long)total_attempts,
                  (unsigned long)good_frames,
                  (unsigned long)framing_errors);
  }
}