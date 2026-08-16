/*
  ESP32-S3 TC Bridge RTOS Phase3 Skeleton
  --------------------------------------
  V0.9からOLED/I2Cを完全撤去したRTOS土台。

  方針:
    Core0 : TC106側 300bps相当 BitBang/OC 物理層専用
    Core1 : Raspberry Pi側 9600bps UART / PacketFactory / ログ
    Queue : Core間のTcMessage受け渡し
    OLED  : 使用しない。デバッグは USB Serial + Pi側ログ + LED。

  注意:
    これはPhase3用の「ひな型」です。
    TC106実機の17タイムスロット送受信は、tcSendFrame()/tcReceiveFrame()
    の中身を実測結果に合わせて差し替えてください。
*/

#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet_phase3.hpp"
#include "tc_message.hpp"

// ======================================================
// ピン設定
// ======================================================
// Pi側 UART。基板・配線に合わせて最終確認すること。
// 注: XIAO ESP32S3 のパッド名 (D0〜D10) を使用。
//     これは Seeed の variant ファイルが提供するマクロで、
//     XIAO_ESP32S3 ボード選択時のみ有効。
//     ボードパッケージ v2.0.8 以降が必要。
static constexpr int PIN_PI_RX = D0;  // Pi_Tx_MCU -> ESP32 RX  (= GPIO1)
static constexpr int PIN_PI_TX = D1;  // ESP32 TX -> Pi_Rx_MCU  (= GPIO2)

// TC106側。現行Snifar回路図の読み直しでは RX=D3(GPIO4), TX_TRIG=D10(GPIO9) が有力。
static constexpr int PIN_TC_RX      = D3;   // TC_MCU_RX <- JP7 <- クランプ <- 分圧  (= GPIO4)
static constexpr int PIN_TC_TX_TRIG = D10;  // MCU_TX_TRIG -> JP5 -> R12 -> Q2 gate  (= GPIO9)

// 開発用LED。XIAO ESP32S3のオンボードLEDは環境で異なるため必要に応じて変更。
static constexpr int PIN_STATUS_LED = LED_BUILTIN;

// ======================================================
// 通信パラメータ
// ======================================================
static constexpr uint32_t PI_BAUD = 9600;
static constexpr uint32_t TC_BAUD = 300;
static constexpr uint32_t BIT_US  = 1000000UL / TC_BAUD;

// TC側はOCドライバ想定。
// Q2 gate HIGH -> Q2 ON -> bus LOW、Q2 gate LOW -> bus released。
static constexpr bool TC_OC_ACTIVE_LOW = true;

// ======================================================
// Phase1.1: 17スロットBitBangプロトコル定数
// ======================================================
// Nano Every側(NanoEvery_TCEmulator_V2_2.ino)と同じ値。
// slot0=HIGHプリアンブル、slot1-7=7bitデータ(LSB first, bit0->HIGH, bit1->LOW)、
// slot8-16=LOW footer(9スロット)。
static constexpr uint32_t P11_SLOT_US       = 3300;
static constexpr uint8_t  P11_DATA_SLOTS    = 7;
static constexpr uint8_t  P11_SLOTS_PER_BYTE = 17;
static constexpr uint8_t  P11_FRAME_BYTES   = 6;   // 6バイト/フレーム
static constexpr uint32_t P11_BYTE_US       = P11_SLOT_US * P11_SLOTS_PER_BYTE; // 56100us

// footer(9スロット=29700us)と判定するための下限しきい値。
// データ部だけで作れる最長連続LOWは7スロット(=全data bit=1, 23100us)なので、
// その間(26000us=26ms)に設定すれば両者を確実に区別できる。
static constexpr uint32_t P11_FOOTER_MIN_US = 26000;

// 1フレーム分のエッジ(信号変化)を記録するバッファ。
// 1バイトあたり最大17エッジ×6バイト分の余裕を持たせる。
static constexpr uint16_t P11_EDGE_BUF_SIZE = 300;

static volatile uint32_t p11EdgeTimes[P11_EDGE_BUF_SIZE];
static volatile uint8_t  p11EdgeLevels[P11_EDGE_BUF_SIZE]; // そのエッジの後に確定するレベル
static volatile uint16_t p11EdgeCount = 0;

// GPIO4(PIN_TC_RX)のレベルが変化するたびに呼ばれる。
// 記録するだけで、Serial等の重い処理は一切行わない(ISR内厳禁のため)。
static void IRAM_ATTR p11TcRxIsr() {
  if (p11EdgeCount < P11_EDGE_BUF_SIZE) {
    p11EdgeTimes[p11EdgeCount]  = micros();
    p11EdgeLevels[p11EdgeCount] = (uint8_t)digitalRead(PIN_TC_RX);
    p11EdgeCount++;
  }
}

// Queue設定
static constexpr uint8_t QUEUE_DEPTH_PI_TO_TC = 8;
static constexpr uint8_t QUEUE_DEPTH_TC_TO_PI = 8;

// TC応答待ちタイムアウト。実機確認後に調整。
static constexpr uint32_t TC_RESPONSE_TIMEOUT_MS = 150;

HardwareSerial SerialPi(1);

static QueueHandle_t qPiToTc = nullptr;
static QueueHandle_t qTcToPi = nullptr;

// TCバス送信を複数箇所から同時に叩かないための保険。
// 現時点ではTCタスクのみが使うが、将来拡張用に残す。
static SemaphoreHandle_t tcBusMutex = nullptr;

// ======================================================
// ユーティリティ
// ======================================================
static void debugDumpMessage(const char* tag, const tc::TcMessage& m) {
  Serial.print(tag);
  Serial.print(" len=");
  Serial.print(m.len);
  Serial.print(" flags=0x");
  Serial.print(m.flags, HEX);
  Serial.print(" data=");
  m.dumpTo(Serial);
  Serial.println();
}

static void setStatusLed(bool on) {
#ifdef LED_BUILTIN
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
#endif
}

// ======================================================
// TC106物理層スタブ
// ======================================================

// OCバスへ1ビット相当を出す。
// logicalHigh=true  : バスを解放してHIGHへ戻す想定
// logicalHigh=false : バスをLOWへ引き下げる想定
static void tcWriteLogicalBit(bool logicalHigh) {
  if (TC_OC_ACTIVE_LOW) {
    // Q2 gate LOW  = release
    // Q2 gate HIGH = pull-down
    digitalWrite(PIN_TC_TX_TRIG, logicalHigh ? LOW : HIGH);
  } else {
    digitalWrite(PIN_TC_TX_TRIG, logicalHigh ? HIGH : LOW);
  }

  delayMicroseconds(BIT_US);
}

// 旧V0.9互換の9bit送信スタブ。
// 実機用17タイムスロット実装へ差し替える場所。
static void tcSend9BitByte(uint8_t data, bool isCommand) {
  tcWriteLogicalBit(false);  // start

  for (uint8_t i = 0; i < 8; i++) {
    tcWriteLogicalBit((data >> i) & 0x01);
  }

  tcWriteLogicalBit(isCommand);
  tcWriteLogicalBit(true);   // stop/release
}

// Phase3差し替えポイント。
// 現在は Packet -> TcFrames -> 9bit風送信の仮実装。
static bool tcSendFrame(const tc::TcMessage& msg) {
  if (msg.len == 0) return false;

  tc::Packet p;
  p.len = min<uint8_t>(msg.len, tc::PI_MAX);
  memcpy(p.buf, msg.data, p.len);

  tc::TcFrames f = tc::toTcFrames(p);

  for (uint8_t i = 0; i < tc::TC_DATA_LEN; i++) {
    tcSend9BitByte(f.data[i], false);
  }

  tcSend9BitByte(f.cmd, true);

  return true;
}

// ======================================================
// Phase1.1: 17スロット受信の実装
// ======================================================
//
// 課題: アイドル時(HIGH)とslot0プリアンブル(HIGH)が同一レベルなので、
// 「バイト1がいつ始まったか」はレベルの変化だけでは判定できない。
//
// 解決策:
//  1. GPIO4の変化(エッジ)を割り込みで全部記録しておく
//  2. 記録の中から「9スロット分(29700us)続く連続LOW」= footerを探す
//     (データ部だけで作れる最長連続LOWは7スロット=23100usなので、
//      しきい値26000usで確実に区別できる)
//  3. footerの終わり(LOWからHIGHに戻る瞬間)は「次のバイトのslot0開始」
//     と確定的に分かる
//  4. byte1だけはfooterが手前に無い(idleから直接続く)ので、
//     byte2の開始時刻から1バイト分(56100us)引いて逆算する
//  5. 各バイトの開始時刻が分かれば、あとは各データスロットの中央を
//     サンプリングするだけでビットを復元できる
//
// 1回の呼び出しで6バイト(1フレーム)を復元する。
static bool p11TryReceiveFrame(uint8_t out[P11_FRAME_BYTES], uint32_t timeoutMs) {
  const uint32_t giveUpAtMs = millis() + timeoutMs;

  // 記録バッファをリセットしてから待つ
  noInterrupts();
  p11EdgeCount = 0;
  interrupts();

  // 最初のエッジ(何か信号が動き出した)を待つ
  while (true) {
    if ((int32_t)(millis() - giveUpAtMs) > 0) return false;
    noInterrupts();
    uint16_t n = p11EdgeCount;
    interrupts();
    if (n > 0) break;
    vTaskDelay(1);
  }

  // エッジが増えなくなる(=送信が一段落した)まで待つ
  const uint32_t QUIET_MS = 50;
  uint16_t lastCount = 0;
  uint32_t lastChangeAtMs = millis();
  while (true) {
    if ((int32_t)(millis() - giveUpAtMs) > 0) return false;

    noInterrupts();
    uint16_t n = p11EdgeCount;
    interrupts();

    if (n != lastCount) {
      lastCount = n;
      lastChangeAtMs = millis();
    } else if (millis() - lastChangeAtMs > QUIET_MS) {
      break; // 一連のバースト(1フレーム分)が終わったとみなす
    }
    vTaskDelay(1);
  }

  // スナップショットをローカルにコピー(volatileのままだと扱いにくいため)
  static uint32_t times[P11_EDGE_BUF_SIZE];
  static uint8_t  levels[P11_EDGE_BUF_SIZE];
  uint16_t n;
  noInterrupts();
  n = p11EdgeCount;
  for (uint16_t i = 0; i < n; i++) {
    times[i]  = p11EdgeTimes[i];
    levels[i] = p11EdgeLevels[i];
  }
  p11EdgeCount = 0; // 次回に備えてクリア
  interrupts();

  if (n < 2) return false;

  // footer(連続LOWが26000us以上)の「終わりの時刻」を集める。
  // これがbyte2〜byte6それぞれのslot0開始時刻になる。
  uint32_t footerEnd[P11_FRAME_BYTES];
  uint8_t footerCount = 0;
  for (uint16_t i = 0; i + 1 < n && footerCount < P11_FRAME_BYTES; i++) {
    if (levels[i] == LOW) {
      uint32_t dur = times[i + 1] - times[i];
      if (dur >= P11_FOOTER_MIN_US) {
        footerEnd[footerCount++] = times[i + 1];
      }
    }
  }

  // byte1<->2, 2<->3, 3<->4, 4<->5, 5<->6 の5個の境界が最低限必要。
  if (footerCount < P11_FRAME_BYTES - 1) {
    return false; // フッターを十分見つけられなかった(ノイズ/未接続など)
  }

  uint32_t byteT0[P11_FRAME_BYTES];
  byteT0[0] = footerEnd[0] - P11_BYTE_US; // byte1は逆算
  for (uint8_t b = 1; b < P11_FRAME_BYTES; b++) {
    byteT0[b] = footerEnd[b - 1];
  }

  // 指定時刻における信号レベルを、記録したエッジ列から求める
  auto levelAt = [&](uint32_t t) -> int {
    if (n == 0 || t < times[0]) return HIGH; // 最初のエッジより前はidle(HIGH)とみなす
    int lvl = HIGH;
    for (uint16_t i = 0; i < n; i++) {
      if (times[i] <= t) {
        lvl = levels[i];
      } else {
        break;
      }
    }
    return lvl;
  };

  // 各バイトのデータスロット(1〜7)の中央をサンプリングしてビット復元
  // bit=0->HIGH, bit=1->LOW, LSB first
  for (uint8_t b = 0; b < P11_FRAME_BYTES; b++) {
    uint8_t value = 0;
    for (uint8_t k = 1; k <= P11_DATA_SLOTS; k++) {
      uint32_t target = byteT0[b] + (uint32_t)k * P11_SLOT_US + (P11_SLOT_US / 2);
      if (levelAt(target) == LOW) {
        value |= (uint8_t)(1u << (k - 1));
      }
    }
    out[b] = value;
  }

  return true;
}

// ======================================================
// Phase1.2: 17スロット送信の実装
// ======================================================
//
// 受信側(p11TryReceiveFrame)と対になる送信ロジック。
// Q2はオープンドレイン駆動(TC_OC_ACTIVE_LOW=true)なので、
// bit=0(HIGH) -> Q2 gate LOW  -> Q2 OFF -> バス解放(プルアップでHIGHへ)
// bit=1(LOW)  -> Q2 gate HIGH -> Q2 ON  -> バスをLOWへ引き込む
// という極性になる(tcWriteLogicalBit()と同じ考え方)。
//
// delayMicroseconds()の累積ではなく、micros()の絶対時刻を基準に
// 各スロットの終わりまで待つ方式にして、ドリフト(誤差の蓄積)を防ぐ。

static void p11SetBusLogical(bool logicalHigh) {
  if (TC_OC_ACTIVE_LOW) {
    digitalWrite(PIN_TC_TX_TRIG, logicalHigh ? LOW : HIGH);
  } else {
    digitalWrite(PIN_TC_TX_TRIG, logicalHigh ? HIGH : LOW);
  }
}

// targetMicros に達するまでビジーウェイトする。
// スロット長(3300us)が十分長いので、busy-waitが最も正確。
static void p11WaitUntilMicros(uint32_t targetMicros) {
  while ((int32_t)(micros() - targetMicros) < 0) {
    // busy wait
  }
}

// 1バイト(17スロット)を送信する。
// slot0=HIGHプリアンブル、slot1-7=7bitデータ(LSB first, bit0->HIGH, bit1->LOW)、
// slot8-16=LOW footer(9スロット、まとめて1回のウェイトでOK)。
static void p11SendByte17Slot(uint8_t value) {
  const uint32_t t0 = micros();

  // slot0: プリアンブル(HIGH)
  p11SetBusLogical(true);
  p11WaitUntilMicros(t0 + P11_SLOT_US);

  // slot1-7: データ7bit、LSB first
  for (uint8_t k = 1; k <= P11_DATA_SLOTS; k++) {
    const bool bitIsOne = (value >> (k - 1)) & 0x01;
    p11SetBusLogical(!bitIsOne); // bit=0->HIGH(true), bit=1->LOW(false)
    p11WaitUntilMicros(t0 + (uint32_t)(k + 1) * P11_SLOT_US);
  }

  // slot8-16: footer(LOW、9スロットまとめて待つ)
  p11SetBusLogical(false);
  p11WaitUntilMicros(t0 + (uint32_t)P11_SLOTS_PER_BYTE * P11_SLOT_US);
}

// 6バイト(1フレーム)を送信する。送信後はバスをidle(HIGH/解放)に戻す。
static void p11SendFrame17Slot(const uint8_t data[P11_FRAME_BYTES]) {
  for (uint8_t i = 0; i < P11_FRAME_BYTES; i++) {
    p11SendByte17Slot(data[i]);
  }
  p11SetBusLogical(true); // idle解放
}


// Phase3差し替えポイント(コマンド/応答モデル用)。
// Phase1.1/1.2ではそれぞれ p11TryReceiveFrame()/p11SendFrame17Slot() を
// taskTcBus から直接使うため、この関数は呼ばれない(下の TC_TEST_MODE 参照)。
// Phase3再開時に、ここへ17スロット応答受信ロジックを合わせ込む。
//
// 注意: tc::TcMessage/tc::MsgType 経由のラップは tc_message.hpp の
// 正確なAPI(MsgTypeの列挙子名など)をこちらで確認できていないため、
// ここでは未実装のままにしてある(Phase3再開時、tc_message.hppの
// 中身と突き合わせて実装すること)。
static bool tcReceiveFrame(tc::TcMessage& out, uint32_t timeoutMs) {
  (void)out;
  (void)timeoutMs;
  return false; // Phase3再開時に実装
}

// ======================================================
// Core1: Raspberry Pi側タスク
// ======================================================
static void taskPiUart(void* pv) {
  uint8_t ring[tc::RING_SIZE]{};
  uint8_t head = 0;
  uint64_t lastSig = 0;

  Serial.println("[PiTask] start on Core1");

  while (true) {
    // Pi -> ESP32
    while (SerialPi.available()) {
      ring[head] = static_cast<uint8_t>(SerialPi.read());
      head = (head + 1) & (tc::RING_SIZE - 1);

      tc::Packet p;
      if (tc::PacketFactory::tryParse(ring, head, lastSig, p)) {
        tc::TcMessage msg = tc::TcMessage::fromPacket(
          p, tc::MsgSource::Pi, tc::MsgType::Command
        );

        if (xQueueSend(qPiToTc, &msg, 0) != pdTRUE) {
          Serial.println("[PiTask] qPiToTc full, dropping packet");
        } else {
          debugDumpMessage("[PiTask RX]", msg);
        }
      }
    }

    // TC -> Pi
    tc::TcMessage rx;
    while (xQueueReceive(qTcToPi, &rx, 0) == pdTRUE) {
      debugDumpMessage("[PiTask TX]", rx);

      // Phase3ではPi側プロトコルを最終確定する。
      // 現時点では6/8/12byte応答をそのまま返す。
      if (rx.len > 0) {
        SerialPi.write(rx.data, rx.len);
        SerialPi.flush();
      }
    }

    vTaskDelay(1);
  }
}

// ======================================================
// Core0: TC106側タスク
// ======================================================
//
// TC_TEST_MODE で動作モードを切り替える:
//   1 = Phase1.1: 受信専用テスト(Nano Every -> ESP32、GPIO4を受信し続けてログ出力)
//   2 = Phase1.2: 送信専用テスト(ESP32 -> Nano Every、テストパターンを周期送信)
//   0 = Phase3  : Piからのコマンド/応答モデル(元のロジック)
//
// Phase1.2実施時は、JP3=2-3・JP5=閉に設定し、Nano側は
// NanoEvery_TCEmulator_V2_2.ino の MODE_PERIODIC=0 に変更しておくこと。
// 受信確認はNano側のシリアルモニタで行う(D13で受信したバイト列を見る)。
#define TC_TEST_MODE 2

#if TC_TEST_MODE == 2
// Nano側の4パターンと同じものを使う(相互比較しやすいように)。
static const uint8_t kP12TestFrames[][P11_FRAME_BYTES] = {
  {0x00, 0x64, 0x00, 0x64, 0x00, 0x7F},
  {0x11, 0x22, 0x33, 0x44, 0x55, 0x7F},
  {0x01, 0x00, 0x00, 0x1E, 0x00, 0x7F},
  {0x64, 0x00, 0x00, 0x32, 0x00, 0x7F},
};
static constexpr uint8_t kP12TestFrameCount =
  sizeof(kP12TestFrames) / sizeof(kP12TestFrames[0]);
#endif

static void taskTcBus(void* pv) {
  Serial.println("[TcTask] start on Core0");

  // OCバスをidle/release状態へ
  digitalWrite(PIN_TC_TX_TRIG, LOW);

#if TC_TEST_MODE == 1
  Serial.println("[TcTask] Phase1.1 standalone listen mode (Pi queue is ignored)");

  uint8_t frame[P11_FRAME_BYTES];
  while (true) {
    // タイムアウトは長めに。Nano側の送信間隔(1000ms)より余裕を持たせる。
    if (p11TryReceiveFrame(frame, 3000)) {
      setStatusLed(true);
      Serial.print("[TcTask RX] ");
      for (uint8_t i = 0; i < P11_FRAME_BYTES; i++) {
        if (frame[i] < 0x10) Serial.print('0');
        Serial.print(frame[i], HEX);
        Serial.print(' ');
      }
      Serial.println();
      setStatusLed(false);
    }
    // 受信できなくてもループを回し続け、次のフレームを待つ。
  }

#elif TC_TEST_MODE == 2
  Serial.println("[TcTask] Phase1.2 standalone send mode (Pi queue is ignored)");
  Serial.println("[TcTask] Check Nano Every's serial monitor for received bytes.");

  uint8_t idx = 0;
  while (true) {
    const uint8_t* f = kP12TestFrames[idx];

    setStatusLed(true);
    p11SendFrame17Slot(f);
    setStatusLed(false);

    Serial.print("[TcTask TX] ");
    for (uint8_t i = 0; i < P11_FRAME_BYTES; i++) {
      if (f[i] < 0x10) Serial.print('0');
      Serial.print(f[i], HEX);
      Serial.print(' ');
    }
    Serial.println();

    idx = (idx + 1) % kP12TestFrameCount;
    vTaskDelay(pdMS_TO_TICKS(1000)); // Nano側と同じく1000ms周期
  }

#else
  while (true) {
    tc::TcMessage cmd;

    if (xQueueReceive(qPiToTc, &cmd, portMAX_DELAY) == pdTRUE) {
      debugDumpMessage("[TcTask CMD]", cmd);

      if (tcBusMutex) xSemaphoreTake(tcBusMutex, portMAX_DELAY);

      setStatusLed(true);
      const bool sent = tcSendFrame(cmd);

      tc::TcMessage resp;
      const bool gotResp = sent && tcReceiveFrame(resp, TC_RESPONSE_TIMEOUT_MS);

      setStatusLed(false);

      if (tcBusMutex) xSemaphoreGive(tcBusMutex);

      if (!gotResp) {
        resp = tc::TcMessage::timeout(tc::MsgSource::Tc);
      }

      if (xQueueSend(qTcToPi, &resp, pdMS_TO_TICKS(10)) != pdTRUE) {
        Serial.println("[TcTask] qTcToPi full, dropping response");
      }
    }
  }
#endif
}

// ======================================================
// setup / loop
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== ESP32-S3 TC Bridge RTOS Phase3 Skeleton v0.1 ===");
  Serial.println("OLED disabled. Debug via USB Serial / Pi log / LED.");
  Serial.println("Core0=TC BitBang/OC, Core1=Pi UART.");

  pinMode(PIN_STATUS_LED, OUTPUT);
  setStatusLed(false);

  pinMode(PIN_TC_TX_TRIG, OUTPUT);
  digitalWrite(PIN_TC_TX_TRIG, LOW);  // OC release
  pinMode(PIN_TC_RX, INPUT_PULLUP);

  // Phase1.1: 17スロット受信用のエッジキャプチャ割り込みを常時有効化
  attachInterrupt(digitalPinToInterrupt(PIN_TC_RX), p11TcRxIsr, CHANGE);

  SerialPi.begin(PI_BAUD, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);

  // Pi 未接続時のフローティング防止 (基板/Pi 未接続時のゴーストパケット対策)
  pinMode(PIN_PI_RX, INPUT_PULLUP);

  // 起動時に入り込んだノイズを捨てる
  delay(100);
  while (SerialPi.available()) SerialPi.read();

  qPiToTc = xQueueCreate(QUEUE_DEPTH_PI_TO_TC, sizeof(tc::TcMessage));
  qTcToPi = xQueueCreate(QUEUE_DEPTH_TC_TO_PI, sizeof(tc::TcMessage));
  tcBusMutex = xSemaphoreCreateMutex();

  if (!qPiToTc || !qTcToPi || !tcBusMutex) {
    Serial.println("[FATAL] RTOS object creation failed");
    while (true) {
      setStatusLed(true);
      delay(100);
      setStatusLed(false);
      delay(100);
    }
  }

  // ESP32 ArduinoではCore1にArduino loopがいるため、スタックと優先度は控えめにする。
  // TC側はタイミング重視のためCore0へ固定。
  xTaskCreatePinnedToCore(taskTcBus,  "TcBus",  4096, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(taskPiUart, "PiUart", 4096, nullptr, 3, nullptr, 1);

  Serial.println("[setup] tasks created");
}
void loop() {
  // 何もしない。タスク側で処理する。
  vTaskDelay(portMAX_DELAY);
}
