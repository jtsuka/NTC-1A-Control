#include <Arduino.h>
#include "tc_packet.hpp"

/* --- ピン設定 --- */
#define PIN_TC_RX 3  // ESP32のTXから接続 (レベルシフタ経由)
#define PIN_TC_TX 2  // ESP32のRXへ接続 (レベルシフタ経由)

/* --- 通信パラメータ --- */
#define TC_BAUD 300
#define BIT_US (1000000 / TC_BAUD) // 3333us
#define INVERT_LOGIC true          // 実機ソースに基づき反転

/* --- グローバル変数 --- */
uint8_t ring[tc::RING_SIZE];
uint8_t head = 0;

// 論理レベルを書き込む関数（反転対応）
void tcWrite(bool level) {
  digitalWrite(PIN_TC_TX, (INVERT_LOGIC ? !level : level) ? HIGH : LOW);
}

// 論理レベルを読み込む関数（反転対応）
bool tcRead() {
  bool val = (digitalRead(PIN_TC_RX) == HIGH);
  return INVERT_LOGIC ? !val : val;
}

// 9ビットフレーム送信 (Start + 8Data + 1Cmd + Stop)
void send9bit(uint8_t data, bool isCmd) {
  tcWrite(false); delayMicroseconds(BIT_US); // Start
  for (int i=0; i<8; i++) {
    tcWrite((data >> i) & 0x01);
    delayMicroseconds(BIT_US);
  }
  tcWrite(isCmd); delayMicroseconds(BIT_US); // 9th bit
  tcWrite(true);  delayMicroseconds(BIT_US); // Stop
}

void setup() {
  Serial.begin(115200); // 高速デバッグ用
  pinMode(PIN_TC_RX, INPUT_PULLUP);
  pinMode(PIN_TC_TX, OUTPUT);
  tcWrite(true); // Idle (HIGH)

  Serial.println("--- Nano Every TC Emulator (9-bit BitBang) ---");
}

void loop() {
  // スタートビット待ち (反転考慮で論理LOWを検出)
  if (tcRead() == false) {
    // ビットの中央を狙うため、1.5ビット分待機
    delayMicroseconds(BIT_US + (BIT_US / 2));

    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
      if (tcRead()) data |= (1 << i);
      delayMicroseconds(BIT_US);
    }

    bool isCmd = tcRead(); // 9ビット目（コマンドフラグ）
    
    // ストップビットを待たずに即時格納（300bpsなら十分な余裕あり）
    if (!isCmd) {
      ring[head] = data;
      head = (head + 1) % tc::RING_SIZE;
      
      // パケットが成立したかチェック
      if (auto p = tc::PacketFactory::tryParse(ring, head)) {
        Serial.print("[RECV PKT] CMD ID: "); Serial.println(p->cmd());
        // ここで返信（例：エコーバック）
        for(int i=0; i<p->len-1; i++) send9bit(p->buf[i], false);
        send9bit(0x7F, true); // ダミーの完了コマンド
      }
    } else {
      // コマンドフラグ単体の受信時
      Serial.print("[COMMAND FRAME] ID: 0x"); Serial.println(data, HEX);
    }
    
    // 次のスタートビットとの誤認を防ぐためのクールダウン
    delayMicroseconds(BIT_US);
  }
}