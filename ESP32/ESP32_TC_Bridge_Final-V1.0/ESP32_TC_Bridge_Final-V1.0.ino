/**
 * ESP32 TC Bridge (SWAP配線確認用) - 6バイト 簡易ブリッジ
 * --------------------------------------------------
 * 修正内容：
 * 1. 二重定義エラー (redefinition of 'struct tc::Fixed6') を解消
 * 2. 不要な cite タグを完全に排除
 * 3. 本番基板の SWAP配線（TC=1,2 / Pi=43,44）を維持
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet.hpp"

// ※ Fixed6 の定義は tc_packet.hpp にあるものを使用するため、ここでは書きません。

// ========================================================
// 1. レベルシフタ (SN74LVC16T245) 制御ピン設定
// ========================================================
static constexpr int PIN_LSHIFT_OE1  = 6;  // LOWで有効 (Nano側)
static constexpr int PIN_LSHIFT_DIR1 = 5;  // HIGHで A(ESP) -> B(Nano) 方向
static constexpr int PIN_LSHIFT_OE2  = 7;  // LOWで有効 (Pi側)
static constexpr int PIN_LSHIFT_DIR2 = 8;  // LOWで B(Nano) -> A(ESP) 方向

// ========================================================
// 2. SWAP配線マッピング (本番基板仕様)
// ========================================================
static constexpr int PIN_TC_TX = 1;  // ESPからNanoのRXへ送信
static constexpr int PIN_TC_RX = 2;  // NanoのTXからESPで受信
static constexpr int PIN_PI_TX = 43; // ESPからPiのRXへ送信
static constexpr int PIN_PI_RX = 44; // PiのTXからESPで受信

// 通信速度設定
static constexpr uint32_t PI_BAUD   = 9600;
static constexpr uint32_t NANO_BAUD = 300;
static constexpr uint32_t NANO_RX_TIMEOUT_MS = 450;

HardwareSerial SerialPi(1);
HardwareSerial SerialNano(2);

// ========================================================
// 3. 補助関数
// ========================================================

/**
 * タイムアウト付きで固定6バイトを受信する関数
 */
static bool readFixed6WithTimeout(Stream& s, tc::Fixed6& out, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  uint8_t idx = 0;
  while (idx < tc::Fixed6::LEN) {
    if (s.available() > 0) {
      out.b[idx++] = (uint8_t)s.read();
      t0 = millis(); // 1バイト受信するごとにタイマーリセット
    } else {
      if ((millis() - t0) > timeoutMs) return false;
      delay(1);
    }
  }
  return true;
}

static void dumpFixed6(const char* tag, const tc::Fixed6& p) {
  Serial.print(tag);
  Serial.print(' ');
  p.dumpTo(Serial);
  Serial.println();
}

// ========================================================
// 4. セットアップ
// ========================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- ESP32 SWAP 疎通テスト (Final Fix) ---");

  // レベルシフタの安全起動
  pinMode(PIN_LSHIFT_OE1, OUTPUT);  digitalWrite(PIN_LSHIFT_OE1, HIGH);
  pinMode(PIN_LSHIFT_OE2, OUTPUT);  digitalWrite(PIN_LSHIFT_OE2, HIGH);
  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH); // ESP -> Nano
  pinMode(PIN_LSHIFT_DIR2, OUTPUT); digitalWrite(PIN_LSHIFT_DIR2, LOW);  // Nano -> ESP

  // 各シリアルの初期化
  SerialPi.setRxBufferSize(256);
  SerialPi.begin(PI_BAUD, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);

  SerialNano.setRxBufferSize(128);
  SerialNano.begin(NANO_BAUD, SERIAL_8N1, PIN_TC_RX, PIN_TC_TX);

  delay(100);
  // レベルシフタを有効化
  digitalWrite(PIN_LSHIFT_OE1, LOW);
  digitalWrite(PIN_LSHIFT_OE2, LOW);

  Serial.printf("Pi UART  : RX=%d TX=%d @%lu\n", PIN_PI_RX, PIN_PI_TX, (unsigned long)PI_BAUD);
  Serial.printf("Nano UART: RX=%d TX=%d @%lu\n", PIN_TC_RX, PIN_TC_TX, (unsigned long)NANO_BAUD);
}

// ========================================================
// 5. メインループ
// ========================================================
void loop() {
  // 1) Piからの入力を待機
  tc::Fixed6 tx{};
  if (!readFixed6WithTimeout(SerialPi, tx, 2000)) {
    delay(1);
    return;
  }

  // 2) 受信バッファをクリア
  while (SerialNano.available() > 0) (void)SerialNano.read();

  // 3) Nanoへデータを転送
  SerialNano.write(tx.b, tc::Fixed6::LEN);
  SerialNano.flush();

  // 4) Nanoからのエコーを待機
  tc::Fixed6 rx{};
  if (!readFixed6WithTimeout(SerialNano, rx, NANO_RX_TIMEOUT_MS)) {
    Serial.println("[TIMEOUT] Nanoからの応答がありません");
    dumpFixed6("[PI DATA]", tx);
    
    // タイムアウト時はPiへ0パケットを返送
    tc::Fixed6 z{};
    for(int i=0; i<6; i++) z.b[i] = 0;
    SerialPi.write(z.b, tc::Fixed6::LEN);
    return;
  }

  // 5) Piへ返送
  SerialPi.write(rx.b, tc::Fixed6::LEN);

  // デバッグログ出力
  dumpFixed6("[TX to Nano]", tx);
  dumpFixed6("[RX fr Nano]", rx);
}