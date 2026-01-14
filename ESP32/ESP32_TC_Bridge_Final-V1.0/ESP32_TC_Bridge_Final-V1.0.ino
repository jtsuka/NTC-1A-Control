/**
 * ESP32 TC Bridge (SWAP配線確認用) - 6バイト 簡易ブリッジ
 * * 【目的】
 * - 本番基板の SWAP配線（Pi/Nanoの交差）における電気的な接続確認。
 * - データフロー: Pi(9600bps) -> ESP32 -> Nano(300bps) -> (Echo) -> ESP32 -> Pi
 * * 【注意】
 * - これは「9ビット通信プロトコル」の実装ではありません。
 * - あくまで「標準的な8ビット通信」で信号が正しくレベルシフタを通るかを確認します。 [cite: 48, 49]
 * Ver 2.4.7 at 2026.1.14
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet.hpp"

// ========================================================
// レベルシフタ (SN74LVC16T245) 制御ピン設定
// ========================================================
static constexpr int PIN_LSHIFT_OE1  = 6;  // LOWで有効 (Nano側) [cite: 50]
static constexpr int PIN_LSHIFT_DIR1 = 5;  // HIGHで A(ESP) -> B(Nano) 方向
static constexpr int PIN_LSHIFT_OE2  = 7;  // LOWで有効 (Pi側への予備/その他) [cite: 51]
static constexpr int PIN_LSHIFT_DIR2 = 8;  // LOWで B(Nano) -> A(ESP) 方向

// ========================================================
// SWAP配線マッピング
//  - Nano/TC側: GPIO1(TX) / GPIO2(RX) [cite: 52]
//  - Pi側:      GPIO43(TX) / GPIO44(RX) [cite: 53, 54]
// ========================================================
static constexpr int PIN_TC_TX = 1;  // ESPからNanoのRXへ送信 [cite: 52]
static constexpr int PIN_TC_RX = 2;  // NanoのTXからESPで受信
static constexpr int PIN_PI_TX = 43; // ESPからPiのRXへ送信 [cite: 53]
static constexpr int PIN_PI_RX = 44; // PiのTXからESPで受信 [cite: 54]

// 通信速度設定
static constexpr uint32_t PI_BAUD   = 9600;
static constexpr uint32_t NANO_BAUD = 300;
// 300bpsは非常に低速なため、受信タイムアウトを長め(450ms)に設定 [cite: 55]
static constexpr uint32_t NANO_RX_TIMEOUT_MS = 450;

HardwareSerial SerialPi(1);
HardwareSerial SerialNano(2);

/**
 * タイムアウト付きで固定6バイトを受信する関数 [cite: 56]
 */
static bool readFixed6WithTimeout(Stream& s, tc::Fixed6& out, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  uint8_t idx = 0;
  while (idx < tc::Fixed6::LEN) { [cite: 57]
    if (s.available() > 0) {
      out.b[idx++] = (uint8_t)s.read(); [cite: 58]
      t0 = millis(); // 1バイト受信するごとにタイマーリセット
    } else {
      if ((millis() - t0) > timeoutMs) return false;
      delay(1); [cite: 59]
    }
  }
  return true;
}

static void dumpFixed6(const char* tag, const tc::Fixed6& p) {
  Serial.print(tag);
  Serial.print(' ');
  p.dumpTo(Serial); [cite: 60]
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n--- ESP32 SWAP 疎通テスト (tc::Fixed6) ---");

  // レベルシフタの安全起動: まずは全てHIGH(無効)から開始 [cite: 61]
  pinMode(PIN_LSHIFT_OE1, OUTPUT);  digitalWrite(PIN_LSHIFT_OE1, HIGH);
  pinMode(PIN_LSHIFT_OE2, OUTPUT);  digitalWrite(PIN_LSHIFT_OE2, HIGH);
  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH); // 1系は ESP -> Nano
  pinMode(PIN_LSHIFT_DIR2, OUTPUT); digitalWrite(PIN_LSHIFT_DIR2, LOW);  // 2系は Nano -> ESP [cite: 62]

  // 各シリアルの初期化
  SerialPi.setRxBufferSize(256);
  SerialPi.begin(PI_BAUD, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);

  SerialNano.setRxBufferSize(128);
  SerialNano.begin(NANO_BAUD, SERIAL_8N1, PIN_TC_RX, PIN_TC_TX);

  delay(100);
  // レベルシフタを有効化 (LOW) [cite: 63]
  digitalWrite(PIN_LSHIFT_OE1, LOW);
  digitalWrite(PIN_LSHIFT_OE2, LOW);

  Serial.printf("Pi UART  : RX=%d TX=%d @%lu\n", PIN_PI_RX, PIN_PI_TX, (unsigned long)PI_BAUD); [cite: 64]
  Serial.printf("Nano UART: RX=%d TX=%d @%lu\n", PIN_TC_RX, PIN_TC_TX, (unsigned long)NANO_BAUD); [cite: 64]
  Serial.println("フロー: Piからの6byte送信待ち -> Nanoへ転送 -> NanoからのEchoをPiへ返送"); [cite: 65]
}

void loop() {
  // 1) Piからの6バイト入力を待機 [cite: 66]
  tc::Fixed6 tx{};
  if (!readFixed6WithTimeout(SerialPi, tx, 2000)) {
    delay(1);
    return; [cite: 67]
  }

  // 2) 受信バッファをクリア（ゴミデータの混入防止） [cite: 68]
  while (SerialNano.available() > 0) (void)SerialNano.read();

  // 3) Nanoへデータを転送
  SerialNano.write(tx.b, tc::Fixed6::LEN);
  SerialNano.flush();

  // 4) Nanoからの折り返し（エコーバック）を待機 [cite: 69]
  tc::Fixed6 rx{};
  if (!readFixed6WithTimeout(SerialNano, rx, NANO_RX_TIMEOUT_MS)) {
    Serial.println("[TIMEOUT] Nanoからの応答がありません"); [cite: 70]
    dumpFixed6("[PIからの送信データ]", tx);
    
    // タイムアウト時はPiへ0埋めパケットを返して同期を維持 [cite: 71]
    tc::Fixed6 z{};
    SerialPi.write(z.b, tc::Fixed6::LEN);
    return;
  }

  // 5) 正常に受信できればPiへ返送
  SerialPi.write(rx.b, tc::Fixed6::LEN);

  // デバッグログ出力
  dumpFixed6("[送信]", tx);
  dumpFixed6("[受信]", rx);
}