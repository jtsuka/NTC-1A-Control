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

// Phase3差し替えポイント。
// 実機ではここに17タイムスロット/RMT/エッジ時刻受信を入れる。
// 現時点ではタイムアウトを返す。
static bool tcReceiveFrame(tc::TcMessage& out, uint32_t timeoutMs) {
  const uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    // TODO:
    //  1. GPIO4のエッジ検出
    //  2. start bit検出
    //  3. bit中央サンプリング
    //  4. 17タイムスロット対応
    //  5. 6byte応答へ復元
    vTaskDelay(1);
  }

  out = tc::TcMessage::timeout(tc::MsgSource::Tc);
  return false;
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
static void taskTcBus(void* pv) {
  Serial.println("[TcTask] start on Core0");

  // OCバスをidle/release状態へ
  digitalWrite(PIN_TC_TX_TRIG, LOW);

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
