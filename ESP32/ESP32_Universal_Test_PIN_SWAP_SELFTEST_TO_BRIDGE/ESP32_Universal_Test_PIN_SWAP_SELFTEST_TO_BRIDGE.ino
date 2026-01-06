/**
 * TC Bridge (XIAO ESP32-S3) v2.4.6 MASTER_STABLE_FIX
 * * 役割：Raspberry Pi(9600bps) と Nano Every(300bps/9bit) の双方向通信ブリッジ
 * 特徴：
 * - Core 0: Pi側のUARTパケット解析（HardwareSerial + PacketFactory）
 * - Core 1: Nano側の非常に重いBitBang送受信（300bps/1bitあたり約3.3ms占有）
 * - 堅牢性: 5データ+1コマンドの順序厳守、チェックサム検証、自己受信防止フラグ
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet.hpp"

// ========================================================
// 1. ピンアサイン（暫定：ユニバーサル基板配線に合わせる）
// ========================================================

// 秋月レベルシフタ (SN74LVC16T245) 制御用
static constexpr int PIN_LSHIFT_OE1  = 5;   // 出力有効化1: LOWでBank1有効 (XIAO -> Nano)
static constexpr int PIN_LSHIFT_DIR1 = 6;   // 方向1: HIGHで A(3.3V) -> B(5V)
static constexpr int PIN_LSHIFT_OE2  = 7;   // 出力有効化2: LOWでBank2有効 (Nano -> XIAO)
static constexpr int PIN_LSHIFT_DIR2 = 8;   // 方向2: LOWで B(5V) -> A(3.3V)

// TC/Nano Every 側（レベルシフタ 3.3V側）BitBang 通信ピン
static constexpr int PIN_TC_TX = 1;         // (暫定) レベルシフタ側へ送信: GPIO1 -> TC/NanoのRX
static constexpr int PIN_TC_RX = 2;         // (暫定) レベルシフタ側から受信: GPIO2 <- TC/NanoのTX

// Raspberry Pi側 通信ピン (HardwareSerial1) ※暫定: GPIO43/44を使用
static constexpr int PIN_PI_UART_RX = 44;   // PiのTXから受信 (GPIO44)
static constexpr int PIN_PI_UART_TX = 43;   // PiのRXへ送信   (GPIO43)

// ========================================================
// 2. 通信パラメータ
// ========================================================
static constexpr uint32_t TC_BAUD = 300;
static constexpr uint32_t BIT_US  = (1000000UL + (TC_BAUD / 2)) / TC_BAUD; // 1ビットの持続時間(約3333us)
static constexpr uint32_t PI_BAUD = 9600;

// ========================================================
// 2.5 セルフテスト設定（Nano Every 6バイト8N1エコー）
//    ※SELFTEST後に通常のBitBang(9bit)ブリッジへ移行
// ========================================================
static constexpr bool     ENABLE_SELFTEST        = true;
static constexpr uint32_t SELF_TEST_BAUD         = 300;
static constexpr uint32_t SELF_TEST_DURATION_MS  = 5UL * 60UL * 1000UL; // 5分
static constexpr uint32_t SELF_TEST_INTERVAL_MS  = 500;                // 送信周期
static constexpr uint32_t SELF_TEST_RX_TIMEOUT_MS= 450;                // 6byte受信待ち


HardwareSerial SerialPi(1);
HardwareSerial SerialSelf(2); // SelfTest用 (300bps 8N1)
static QueueHandle_t qToTC = nullptr;       // Piから受信したジョブを送信タスクへ送る
static QueueHandle_t qToPi = nullptr;       // Nanoから受信したパケットをPi送信タスクへ送る

static volatile bool gTcTxActive = false;    // 送信中は受信を停止させる「半二重」制御フラグ

// =====================
// 3. BitBang 補助関数 (Core 1で使用)
// =====================

// 指定レベルを一定時間(BIT_US)出力する
static inline void writeLevel(bool high) {
  digitalWrite(PIN_TC_TX, high ? HIGH : LOW);
  delayMicroseconds(BIT_US);
}

// 入力ピンの現在値を読む
static inline bool readLevel() {
  return (digitalRead(PIN_TC_RX) == HIGH);
}

// 9bitフレーム送信 [Start(L) + 8bit-Data + 9th(isCmd) + Stop(H)]
static void send9bitFrame(uint8_t data, bool isCmd) {
  writeLevel(false); // スタートビット(LOW)
  for (int i = 0; i < 8; i++) writeLevel(((data >> i) & 0x01) != 0); // 8bitデータ
  writeLevel(isCmd); // 9番目のビット (0:データ, 1:コマンド)
  writeLevel(true);  // ストップビット(HIGH)
  writeLevel(true);  // ガードタイム
}

// 9bitフレーム受信 (スタートビット検出後に中央をサンプリング)
static bool read9bitFrame(uint8_t &data, bool &isCmd) {
  if (readLevel()) return false; // アイドル状態(HIGH)なら即終了
  
  delayMicroseconds(BIT_US / 2); // スタートビットの中央まで待機
  if (readLevel()) return false; // 偽のスタート（ノイズ）なら棄却
  delayMicroseconds(BIT_US);     // 最初のデータビット(Bit0)の中央へ

  data = 0;
  for (int i = 0; i < 8; i++) {
    if (readLevel()) data |= (1U << i);
    delayMicroseconds(BIT_US);
  }
  isCmd = readLevel();           // 9thビット(isCmd)
  delayMicroseconds(BIT_US);     // ストップビット位置へ
  if (!readLevel()) return false; // ストップビットがHIGHでなければエラー
  delayMicroseconds(BIT_US);     // ガード分
  return true;
}

// ========================================================
// 3.5 Nano Every向けセルフテスト (6バイト 8N1 エコーバック)
// ========================================================
static void runSelfTestToNano() {
  if (!ENABLE_SELFTEST) return;

  Serial.println("--- [SELFTEST] Start (Nano Every echo 6 bytes, 8N1/300bps) ---");
  Serial.println("--- [SELFTEST] After test, switch to normal BRIDGE (BitBang 9bit) ---");

  // SerialSelf: RX=PIN_TC_RX, TX=PIN_TC_TX
  SerialSelf.setRxBufferSize(128);
  SerialSelf.begin(SELF_TEST_BAUD, SERIAL_8N1, PIN_TC_RX, PIN_TC_TX);
  delay(50);

  uint32_t ok = 0, timeout = 0, mismatch = 0;
  uint8_t seq = 0;
  const uint32_t t0 = millis();
  uint32_t nextSend = t0;

  while ((millis() - t0) < SELF_TEST_DURATION_MS) {
    const uint32_t now = millis();
    if ((int32_t)(now - nextSend) >= 0) {
      nextSend += SELF_TEST_INTERVAL_MS;

      // 送信パケット（6バイト固定）
      uint8_t tx[6];
      tx[0] = 0xA5;
      tx[1] = 0x5A;
      tx[2] = seq;
      tx[3] = (uint8_t)~seq;
      tx[4] = (uint8_t)(tx[0] + tx[1] + tx[2] + tx[3]);
      tx[5] = 0x0D;
      seq++;

      // 受信バッファを掃除（前回の残骸対策）
      while (SerialSelf.available() > 0) (void)SerialSelf.read();

      SerialSelf.write(tx, 6);
      SerialSelf.flush();

      // 6バイト受信待ち
      uint8_t rx[6];
      uint8_t idx = 0;
      const uint32_t waitStart = millis();
      while (idx < 6 && (millis() - waitStart) < SELF_TEST_RX_TIMEOUT_MS) {
        if (SerialSelf.available() > 0) {
          rx[idx++] = (uint8_t)SerialSelf.read();
        } else {
          delay(1);
        }
      }

      if (idx < 6) {
        timeout++;
        Serial.printf("[SELFTEST] timeout (got=%u) ok=%lu timeout=%lu mismatch=%lu\n", idx, (unsigned long)ok, (unsigned long)timeout, (unsigned long)mismatch);
        continue;
      }

      bool same = true;
      for (int i = 0; i < 6; i++) {
        if (rx[i] != tx[i]) { same = false; break; }
      }
      if (same) {
        ok++;
      } else {
        mismatch++;
        Serial.printf("[SELFTEST] mismatch ok=%lu timeout=%lu mismatch=%lu\n", (unsigned long)ok, (unsigned long)timeout, (unsigned long)mismatch);
        Serial.print("  TX:");
        for (int i = 0; i < 6; i++) { Serial.printf(" %02X", tx[i]); }
        Serial.println();
        Serial.print("  RX:");
        for (int i = 0; i < 6; i++) { Serial.printf(" %02X", rx[i]); }
        Serial.println();
      }
    }

    // USBシリアルから 'b' が来たら早期終了
    if (Serial.available() > 0) {
      int c = Serial.read();
      if (c == 'b' || c == 'B') {
        Serial.println("[SELFTEST] Break by user.");
        break;
      }
    }

    delay(1);
  }

  Serial.println("--- [SELFTEST] Done ---");
  Serial.printf("[SELFTEST] result: ok=%lu timeout=%lu mismatch=%lu\n", (unsigned long)ok, (unsigned long)timeout, (unsigned long)mismatch);

  SerialSelf.end();
  delay(20);

  // BitBangへ戻す（ピン状態を明示）
  pinMode(PIN_TC_TX, OUTPUT); digitalWrite(PIN_TC_TX, HIGH);
  pinMode(PIN_TC_RX, INPUT_PULLUP);
  delay(20);
}

// ========================================================
// 4. FreeRTOS タスク
// ========================================================

// 【Core 1】Piから来た命令をNanoへ送信するタスク
static void taskTCSend(void *pv) {
  tc::TcFrames f{};
  while (true) {
    if (xQueueReceive(qToTC, &f, portMAX_DELAY) == pdTRUE) {
      gTcTxActive = true; // 送信開始：受信タスクを抑制
      for (int i = 0; i < 3; i++) writeLevel(true); // 送信前の安定化パルス
      // 5つのデータフレームを順に送信
      for (int i = 0; i < tc::TC_DATA_LEN; i++) send9bitFrame(f.data[i], false);
      // 最後にコマンドフレーム(9th=1)を送信
      send9bitFrame(f.cmd, true);
      gTcTxActive = false; // 送信終了
    }
  }
}

// 【Core 1】Nanoからの信号を受け取り、整合性を確認してPiへ送るタスク
static void taskTCRx(void *pv) {
  uint8_t data[tc::TC_DATA_LEN]{};
  uint8_t n = 0;
  bool haveCmd = false;
  uint8_t cmd = 0;
  uint32_t lastUs = 0;

  while (true) {
    // 自身の送信中、または送信直後の場合はバッファをリセットして無視する（自己受信防止）
    if (gTcTxActive) { n = 0; haveCmd = false; vTaskDelay(1); continue; }

    uint8_t b; bool isCmd;
    if (read9bitFrame(b, isCmd)) {
      lastUs = micros();

      if (!isCmd) { // --- 9bit=0 (データフレーム) 受信 ---
        if (haveCmd) { n = 0; haveCmd = false; continue; } // すでにCMDがあるのにデータが来たら同期異常
        if (n < tc::TC_DATA_LEN) data[n++] = b;
        else { n = 0; haveCmd = false; } // 溢れたらリセット
        continue;
      }

      // --- 9bit=1 (コマンドフレーム) 受信 ---
      if (n != tc::TC_DATA_LEN) { n = 0; haveCmd = false; continue; } // データが5つ揃っていないなら棄却

      cmd = b;
      haveCmd = true;

      // チェックサム検証
      uint8_t checkBuf[5] = { cmd, data[0], data[1], data[2], data[3] };
      const uint8_t expected = tc::checksum7(checkBuf, 5);

      if (data[4] == expected) { // TC側の末尾バイトと計算値が一致するか
        tc::Packet p{};                 // ゼロ初期化
        p.len = tc::PI_LEN_SEND;        // Pi向けの長さ(6)を設定
        p.buf[0] = cmd;
        for (int i = 0; i < tc::TC_DATA_LEN; i++) p.buf[1 + i] = data[i];

        (void)xQueueSend(qToPi, &p, 0); // Pi送信キューへ（溢れたら捨てる）
      }
      n = 0; haveCmd = false; // 次のパケットのためにリセット
    } else {
      // 150ms以上通信がなければ、不完全なパケットを破棄してリセット
      if ((n > 0 || haveCmd) && (micros() - lastUs) > 150000UL) { n = 0; haveCmd = false; }
      vTaskDelay(1);
    }
  }
}

// 【Core 0】PiからのUART受信・解析タスク
static void taskPiRx(void *pv) {
  uint8_t ring[tc::RING_SIZE]{};
  uint8_t head = 0;
  uint64_t lastSig = 0;
  while (true) {
    while (SerialPi.available() > 0) {
      ring[head] = (uint8_t)SerialPi.read();
      head = (uint8_t)((head + 1) & (tc::RING_SIZE - 1));
      // リングバッファから有効なパケット(6/8/12バイト)を切り出す
      if (const tc::Packet* p = tc::PacketFactory::tryParse(ring, head, lastSig)) {
        tc::TcFrames f = tc::toTcFrames(*p); // TcFrames構造体に変換
        (void)xQueueSend(qToTC, &f, pdMS_TO_TICKS(10));
      }
    }
    vTaskDelay(1);
  }
}

// 【Core 0】PiへのUART送信タスク
static void taskPiTx(void *pv) {
  tc::Packet p{};
  while (true) {
    if (xQueueReceive(qToPi, &p, portMAX_DELAY) == pdTRUE) {
      SerialPi.write(p.buf, p.len); // 6バイトパケットをPiの受信ポートへ射出
    }
  }
}

// ========================================================
// 5. 初期設定 (setup)
// ========================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n--- [TC Bridge v2.4.6 MASTER_STABLE_FIX] ---");

  // --- レベルシフタ安全起動シーケンス ---
  pinMode(PIN_LSHIFT_OE1, OUTPUT);  digitalWrite(PIN_LSHIFT_OE1, HIGH); // 最初は遮断(HIGH)
  pinMode(PIN_LSHIFT_OE2, OUTPUT);  digitalWrite(PIN_LSHIFT_OE2, HIGH);
  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH); // A->B
  pinMode(PIN_LSHIFT_DIR2, OUTPUT); digitalWrite(PIN_LSHIFT_DIR2, LOW);  // B->A

  pinMode(PIN_TC_TX, OUTPUT); digitalWrite(PIN_TC_TX, HIGH); // アイドルHIGH
  pinMode(PIN_TC_RX, INPUT_PULLUP);

  // UART RXバッファを拡大してバースト受信に対応
  SerialPi.setRxBufferSize(256);
  SerialPi.begin(PI_BAUD, SERIAL_8N1, PIN_PI_UART_RX, PIN_PI_UART_TX);

  delay(100);
  digitalWrite(PIN_LSHIFT_OE1, LOW); // 設定が安定してからゲートを開く(LOW)
  digitalWrite(PIN_LSHIFT_OE2, LOW);

  // --- セルフテスト（Nano Every エコー）→完了後に通常BRIDGEへ ---
  runSelfTestToNano();

  // SELFTEST中にPi側からゴミが入っていた場合に備え、開始前に捨てる
  while (SerialPi.available() > 0) (void)SerialPi.read();

  qToTC = xQueueCreate(16, sizeof(tc::TcFrames));
  qToPi = xQueueCreate(16, sizeof(tc::Packet));

  // --- コア割り当て：PiはCore 0、TCはCore 1 ---
  xTaskCreatePinnedToCore(taskPiRx,   "PiRx",   4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(taskPiTx,   "PiTx",   4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(taskTCSend, "TCSend", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(taskTCRx,   "TCRx",   4096, nullptr, 5, nullptr, 1);

  Serial.println("--- [READY] Roles: Core0(Pi) / Core1(TC) ---");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000)); // メインループは空で待機
}