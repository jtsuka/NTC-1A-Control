/**
 * Nano Every: TC Controller Emulator (Echo Back Test)
 * --------------------------------------------------
 * 【本番基板（Swap配線）専用設定】
 * 接続先：XIAO ESP32-S3 (GPIO 1, 2) ※レベルシフタ経由
 * * 役割：
 * 1. ESP32から送られてくる 300bps の信号を受信
 * 2. 6バイト揃ったら LEDを5回点滅（受信確認）
 * 3. 同じ6バイトをESP32へ送り返す（エコーバック）
 * * なぜ D10/D11 か：
 * Nano EveryのハードウェアUART(Serial1)の干渉を避け、
 * Swap回路で安定通信が確認された SoftwareSerial 用のピン。
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

// --- 通信設定 ---
static const uint32_t BAUD_USB = 115200; // PCデバッグ用
static const uint32_t BAUD_TC  = 300;    // 本番機（PIC24F）互換の超低速レート

// --- パケット設定 ---
static const uint8_t  PACKET_LEN = 6;      // 固定長パケット
static const uint16_t RX_GAP_TIMEOUT_MS = 150; // 300bpsでは1文字33msかかるため長めに設定

// --- ピンアサイン（本番Swap回路用） ---
static const uint8_t  PIN_SOFT_RX = 10; // D10: ESP32 TX(GPIO 1) からの信号を受信
static const uint8_t  PIN_SOFT_TX = 11; // D11: ESP32 RX(GPIO 2) へ信号を送信

SoftwareSerial TcSoft(PIN_SOFT_RX, PIN_SOFT_TX); 

uint8_t  buf[PACKET_LEN];
uint8_t  idx = 0;
uint32_t t_last_rx = 0;
uint32_t pkt_count = 0;

// LED 5回点滅：300bpsテスト用（この間、通信は一時停止するが検証用には十分）
static void blink5() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(50);
    digitalWrite(LED_BUILTIN, LOW);  delay(50);
  }
}

// 16進数表示ヘルパー
static void printHex6(const uint8_t *p) {
  for (int i = 0; i < 6; i++) {
    if (p[i] < 0x10) Serial.print('0');
    Serial.print(p[i], HEX);
    Serial.print(i == 5 ? "" : " ");
  }
}

void setup() {
  Serial.begin(BAUD_USB);
  while (!Serial) {}

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // ソフトウェアシリアル開始
  TcSoft.begin(BAUD_TC);

  Serial.println("=== Nano Every TC Emulator (Swap Board Mode) ===");
  Serial.println("Config: SoftwareSerial RX=D10, TX=D11 @300bps");
  Serial.println("Action: Waiting for 6 bytes from ESP32...");
}

void loop() {
  // バイト間ギャップ監視：途切れたパケットを破棄して同期ズレを防ぐ
  if (idx > 0 && (millis() - t_last_rx) > RX_GAP_TIMEOUT_MS) {
    Serial.print("[TIMEOUT] partial bytes discarded: ");
    Serial.println(idx);
    idx = 0;
  }

  while (TcSoft.available() > 0) {
    uint8_t b = (uint8_t)TcSoft.read();
    t_last_rx = millis();

    buf[idx++] = b;

    // 6バイト揃った時の処理
    if (idx >= PACKET_LEN) {
      pkt_count++;

      // 視覚的フィードバック（LED点滅）
      blink5();

      // 受信ログ出力
      Serial.print("[RX#"); Serial.print(pkt_count); Serial.print("] ");
      printHex6(buf); Serial.println();

      // --- Echo back 送信 ---
      TcSoft.write(buf, PACKET_LEN);
      TcSoft.flush();

      // 送信ログ出力
      Serial.print("[TX#"); Serial.print(pkt_count); Serial.print("] ");
      printHex6(buf); Serial.println();

      idx = 0; // バッファリセット
    }
  }
}