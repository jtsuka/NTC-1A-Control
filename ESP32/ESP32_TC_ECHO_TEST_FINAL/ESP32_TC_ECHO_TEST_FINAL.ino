/**
 * ESP32-S3 Pi <-> ESP 疎通確認用エコーバック・スケッチ
 * --------------------------------------------------
 * 【目的】
 * - Raspberry Pi (USB-TTL) と ESP32-S3 間の物理的な接続と電圧の安定性を確認する。
 * - 6バイトの固定パケットを受信し、そのまま送り返すことで通信の整合性を検証する。
 * * 【修正・改善内容】
 * 1. 名前衝突の回避: Arduino標準の円周率マクロ(PI)との衝突を防ぐため、インスタンス名を SerialPi に変更。
 * 2. タイムアウトの二段構え: バイト間(30ms)と全体(300ms)のダブル監視で、ノイズによるハングアップを防止。
 * 3. 産業用シーケンス: 本格運用を見据え、初期化時のレベルシフタ制御（OEピン）の枠組みを維持。
 */

#include <Arduino.h>
#include <HardwareSerial.h>

// --- 通信設定 ---
static const uint32_t BAUD_RATE = 9600; // Raspberry Pi 側と合わせる
static const size_t   PKT_LEN   = 6;    // 通信プロトコル規定の6バイト

// --- ピンアサイン (本番基板 / SWAP配線仕様) ---
static const int PIN_ESP_TX_TO_PI = 1;  // ESP32から出力（PiのRXへ）
static const int PIN_ESP_RX_FR_PI = 2;  // Piから入力（ESP32のRXへ）

// --- 制御ピン（レベルシフタ用 / 単体テスト時は未使用でも定義維持） ---
static const int PIN_LSHIFT_OE1  = 6;
static const int PIN_LSHIFT_DIR1 = 5;

// --- タイムアウト設定 ---
static const uint16_t INTERBYTE_TIMEOUT_MS = 30;   // バイト間の最大空き時間
static const uint16_t TOTAL_TIMEOUT_MS     = 300;  // 1パケット受信の最大許容時間

// UART1を使用してPiと通信
HardwareSerial SerialPi(1);

static uint8_t buf[PKT_LEN];

/**
 * 受信データを16進数でデバッグ出力する補助関数
 */
static void dumpHex(const char* tag, const uint8_t* p, size_t n) {
  Serial.print(tag);
  for (size_t i = 0; i < n; i++) {
    if (p[i] < 0x10) Serial.print('0');
    Serial.print(p[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void setup() {
  // 1. USBデバッグ用のシリアル開始
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n--- [ESP32-S3] Pi Echo Test (9600bps) ---"));

  // 2. レベルシフタの安全制御（起動時のノイズ遮断）
  pinMode(PIN_LSHIFT_OE1, OUTPUT); digitalWrite(PIN_LSHIFT_OE1, HIGH); // 最初は無効(Hi-Z)
  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH); // A->B方向に固定

  // 3. Pi用シリアルの初期化（ピン番号を明示的に指定）
  SerialPi.begin(BAUD_RATE, SERIAL_8N1, PIN_ESP_RX_FR_PI, PIN_ESP_TX_TO_PI);

  // 4. 受信バッファの掃除
  while (SerialPi.available()) (void)SerialPi.read();
  
  // 5. 準備ができたら接続を有効化
  delay(100);
  digitalWrite(PIN_LSHIFT_OE1, LOW); // 有効化
  
  Serial.println(F("Ready: Waiting for 6-byte packet from Pi..."));
}

void loop() {
  size_t got = 0;
  uint32_t t_first = 0;
  uint32_t t_last  = millis();

  // パケット受信ループ：指定サイズが揃うまで待機
  while (got < PKT_LEN) {
    if (SerialPi.available()) {
      if (got == 0) t_first = millis(); // 最初の1バイト目を受信した時刻
      buf[got++] = (uint8_t)SerialPi.read();
      t_last = millis(); // 最後にデータを受信した時刻
    } else {
      // 1バイトでも受信を開始している場合のみタイムアウト判定を行う
      if (got > 0) {
        if ((millis() - t_last)  > INTERBYTE_TIMEOUT_MS) break; // バイト間が空きすぎ
        if ((millis() - t_first) > TOTAL_TIMEOUT_MS)     break; // 全体時間がかかりすぎ
      }
      delay(1);
    }
  }

  // データが届いていない場合は何もしない
  if (got == 0) return;

  // 指定サイズ(6バイト)に満たない場合はエラーとして処理
  if (got != PKT_LEN) {
    Serial.print(F("[TIMEOUT] partial bytes received: "));
    Serial.println(got);
    while (SerialPi.available()) (void)SerialPi.read(); // バッファの残骸を掃除
    return;
  }

  // 【正常受信】
  dumpHex("[RX fr Pi] ", buf, PKT_LEN);

  // 送信（エコーバック）：受信したデータをそのまま送り返す
  SerialPi.write(buf, PKT_LEN);
  SerialPi.flush();

  dumpHex("[TX to Pi] ", buf, PKT_LEN);
}