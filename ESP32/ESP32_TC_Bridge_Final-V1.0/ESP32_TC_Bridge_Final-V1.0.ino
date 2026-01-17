/**
 * ESP32 TC Bridge (Non-SWAP / 本番配線) - 6バイト 簡易ブリッジ
 * --------------------------------------------------
 * 目的:
 *   Pi(USB-TTL, 9600bps, 6バイト) → ESP32 → Nano(D10/D11, 300bps, 6バイト) → ESP32 → Pi
 *
 * 前提(今回の配線):
 *   Pi側UART   : ESP TX=GPIO1  (→ Pi RX)
 *               ESP RX=GPIO2  (← Pi TX)
 *   Nano側UART : ESP TX=GPIO43 (→ Nano D10 RX)
 *               ESP RX=GPIO44 (← Nano D11 TX)
 *
 * 注意:
 *   - Fixed6 構造体は tc_packet_fixed.hpp を使用
 *   - レベルシフタ(SN74LVC16T245)の OE/DIR は確定値で固定
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet_fixed.hpp"

// ========================================================
// 1. レベルシフタ (SN74LVC16T245) 制御ピン（確定・変更しない）
// ========================================================
static constexpr int PIN_LSHIFT_OE1  = 6;  // LOWで有効 (Bank1)
static constexpr int PIN_LSHIFT_DIR1 = 5;  // HIGHで A(ESP) -> B(5V側)
static constexpr int PIN_LSHIFT_OE2  = 7;  // LOWで有効 (Bank2)
static constexpr int PIN_LSHIFT_DIR2 = 8;  // LOWで B(5V側) -> A(ESP)

// ========================================================
// 2. 非SWAP配線マッピング（今回の目的どおり）
// ========================================================
static constexpr int PIN_PI_TX = 1;   // ESP→Pi RX
static constexpr int PIN_PI_RX = 2;   // Pi TX→ESP
static constexpr int PIN_TC_TX = 43;  // ESP→Nano D10(RX)
static constexpr int PIN_TC_RX = 44;  // Nano D11(TX)→ESP

// 通信速度設定
static constexpr uint32_t PI_BAUD   = 9600;
static constexpr uint32_t NANO_BAUD = 300;

// Nanoのエコー待ち（300bpsなので余裕を持たせる）
static constexpr uint32_t NANO_RX_TIMEOUT_MS = 600;

// UART1=Pi, UART2=Nano
HardwareSerial SerialPi(1);
HardwareSerial SerialNano(2);

// ========================================================
// 3. 補助関数
// ========================================================

/**
 * タイムアウト付きで固定6バイトを受信する
 * - 1バイト受信する度にタイマーをリセット（バイト間タイムアウト）
 */
static bool readFixed6WithTimeout(Stream& s, tc::Fixed6& out, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  uint8_t idx = 0;
  while (idx < tc::Fixed6::LEN) {
    if (s.available() > 0) {
      out.b[idx++] = (uint8_t)s.read();
      t0 = millis();
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
  Serial.println("\n--- ESP32 Pi<->Nano Bridge (Non-SWAP) ---");

  // レベルシフタ安全起動: まずHi-Z
  pinMode(PIN_LSHIFT_OE1, OUTPUT);  digitalWrite(PIN_LSHIFT_OE1, HIGH);
  pinMode(PIN_LSHIFT_OE2, OUTPUT);  digitalWrite(PIN_LSHIFT_OE2, HIGH);
  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH); // A->B
  pinMode(PIN_LSHIFT_DIR2, OUTPUT); digitalWrite(PIN_LSHIFT_DIR2, LOW);  // B->A

  // UART初期化
  SerialPi.setRxBufferSize(256);
  SerialPi.begin(PI_BAUD, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);

  SerialNano.setRxBufferSize(256);
  SerialNano.begin(NANO_BAUD, SERIAL_8N1, PIN_TC_RX, PIN_TC_TX);

  delay(100);
  // UART準備後に有効化
  digitalWrite(PIN_LSHIFT_OE1, LOW);
  digitalWrite(PIN_LSHIFT_OE2, LOW);

  Serial.printf("Pi UART   : RX=%d TX=%d @%lu\n", PIN_PI_RX, PIN_PI_TX, (unsigned long)PI_BAUD);
  Serial.printf("Nano UART : RX=%d TX=%d @%lu\n", PIN_TC_RX, PIN_TC_TX, (unsigned long)NANO_BAUD);
}

// ========================================================
// 5. メインループ
// ========================================================
void loop() {
  // 1) Piから6バイト受信
  tc::Fixed6 tx{};
  if (!readFixed6WithTimeout(SerialPi, tx, 2000)) {
    delay(1);
    return;
  }

  // 2) Nano側受信バッファをクリア（残骸対策）
  while (SerialNano.available() > 0) (void)SerialNano.read();

  // 3) Nanoへ転送
  SerialNano.write(tx.b, tc::Fixed6::LEN);
  SerialNano.flush();

  // 4) Nanoから6バイト受信（エコー待ち）
  tc::Fixed6 rx{};
  if (!readFixed6WithTimeout(SerialNano, rx, NANO_RX_TIMEOUT_MS)) {
    Serial.println("[TIMEOUT] Nanoからの応答がありません");
    dumpFixed6("[PI DATA]", tx);

    // タイムアウト時はPiへ 0 パケットを返す（切り分け用）
    tc::Fixed6 z{};
    for (int i = 0; i < 6; i++) z.b[i] = 0;
    SerialPi.write(z.b, tc::Fixed6::LEN);
    return;
  }

  // 5) Piへ返送
  SerialPi.write(rx.b, tc::Fixed6::LEN);

  // ログ
  dumpFixed6("[TX to Nano]", tx);
  dumpFixed6("[RX fr Nano]", rx);
}
