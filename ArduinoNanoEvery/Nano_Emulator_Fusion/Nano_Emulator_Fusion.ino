#include <Arduino.h>
#include "tc_packet.hpp"

/*
  TC Emulator Fusion Final V1.2.6
  - ESP32 側との安定通信のため D10(RX) / D11(TX) を使用
  - 9ビット UART 形式を維持しつつ、反転ロジックを最適化
*/

// ピン定義を修正
#define PIN_RX 10 // D10: 受信
#define PIN_TX 11 // D11: 送信

// 通信設定
#define TC_INVERT_LOGIC true
#define ECHO_REPLY true

// 300bps: 精密な 3334us に設定 (ESP32側と同期)
#define BIT_US 3334 

// --- ビット制御関数 ---

void tcWrite(bool logical) {
  // 論理反転：true(1)なら物理LOW、false(0)なら物理HIGH
  bool out = TC_INVERT_LOGIC ? !logical : logical;
  digitalWrite(PIN_TX, out ? HIGH : LOW);
  delayMicroseconds(BIT_US);
}

bool tcRead() {
  bool val = (digitalRead(PIN_RX) == HIGH);
  // 論理反転：物理HIGHなら論理false(0)として返す
  return TC_INVERT_LOGIC ? !val : val;
}

// 9ビット送信（データ8bit + コマンドフラグ1bit）
void send9bit(uint8_t d, bool isCmd) {
  tcWrite(false); // Start bit (論理0)
  for (int i = 0; i < 8; i++) {
    tcWrite((d >> i) & 0x01); // Data 8bits (LSB first)
  }
  tcWrite(isCmd); // 9th bit (Command Flag)
  tcWrite(true);  // Stop bit (論理1 / Idle)
  tcWrite(true);  // Gap
}

// --- メイン処理 ---

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // Nano Every のシリアル準備待ち

  pinMode(PIN_RX, INPUT_PULLUP); // 浮き防止のため Pullup 推奨
  pinMode(PIN_TX, OUTPUT);
  
  // 初期状態：Idle HIGH (反転ロジック時は物理LOW)
  digitalWrite(PIN_TX, TC_INVERT_LOGIC ? LOW : HIGH);
  
  Serial.println(F("\n--- [Nano TC Emulator v1.2.6 (D10/D11)] ---"));
  Serial.print(F("Pins: RX=D10, TX=D11 | Baud: 300bps ("));
  Serial.print(BIT_US); Serial.println(F("us)"));
  Serial.println(F("Ready to emulate TC Controller..."));
}

void loop() {
  static uint8_t data[tc::TC_DATA_LEN];
  static uint8_t di = 0;
  
  // スタートビット検出：反転時は物理HIGHが論理false(0)
  if (tcRead() == false) { 
    delayMicroseconds(BIT_US / 2); // ビット中央まで待機
    
    if (tcRead() == true) return; // ノイズ除去（デグリッチ）
    
    delayMicroseconds(BIT_US); // 最初のデータビットの中央へ移動
    
    // データ 8ビット読み取り
    uint8_t d = 0;
    for (int i = 0; i < 8; i++) {
      if (tcRead()) d |= (1 << i);
      delayMicroseconds(BIT_US);
    }
    
    // 9ビット目（コマンドフラグ）の判定
    bool isCmd = tcRead(); 
    
    if (!isCmd) {
      // データバイトの場合：バッファに蓄積
      if (di < tc::TC_DATA_LEN) {
        data[di++] = d;
        Serial.print(d, HEX); Serial.print(" ");
      }
    } else {
      // コマンドバイトの場合：パケット完成とみなしてエコーバック
      Serial.print(F("\n[COMMAND] 0x")); Serial.println(d, HEX);
      
      if (ECHO_REPLY && di == tc::TC_DATA_LEN) {
        // 返信前のわずかなギャップ
        for (int i = 0; i < 3; i++) tcWrite(true); 
        
        // 蓄積したデータとコマンドをそのまま返信
        for (int i = 0; i < tc::TC_DATA_LEN; i++) {
          send9bit(data[i], false);
        }
        send9bit(d, true); 
        Serial.println(F("[ECHO] Sent response to ESP32."));
      }
      di = 0; // バッファリセット
    }
    
    // ストップビットを待機して同期をリセット
    uint32_t t0 = micros();
    while(tcRead() == false && (micros() - t0) < (BIT_US * 2));
  }
}