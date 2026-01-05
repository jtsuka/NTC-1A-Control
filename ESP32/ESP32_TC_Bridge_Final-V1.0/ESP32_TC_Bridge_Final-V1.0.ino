/*
 * TC Bridge (XIAO ESP32-S3) v2.4.5 "FINAL_STABLE_CORE_SPLIT"
 * ---------------------------------------------------------------------------
 * Pi  <-> XIAO : HW UART1 9600bps 8N1  (RX=GPIO2, TX=GPIO1)
 * Nano<-> XIAO : 9-bit BitBang 300bps (TX=GPIO43, RX=GPIO44) via SN74LVC16T245
 *
 * 重要:
 * - BitBang RX/TX は重い(300bpsでもフレーム読取はブロッキング)ので Core1 に隔離
 * - Pi UART処理は Core0 で回して取りこぼしを防止
 * - 送信中は受信タスクを止めて自己受信を防止（半二重前提）
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet.hpp"

// =====================
// ピンアサイン（最新回路図 固定）
// =====================
static constexpr int PIN_LSHIFT_OE1  = 5;   // LOW=enable (Bank1: XIAO->Nano)
static constexpr int PIN_LSHIFT_DIR1 = 6;   // HIGH A->B
static constexpr int PIN_LSHIFT_OE2  = 7;   // LOW=enable (Bank2: Nano->XIAO)
static constexpr int PIN_LSHIFT_DIR2 = 8;   // LOW  B->A

static constexpr int PIN_TC_TX = 43;        // XIAO -> Nano (TXA)
static constexpr int PIN_TC_RX = 44;        // Nano -> XIAO (RXA)

static constexpr int PIN_PI_UART_RX = 2;    // XIAO RX (Pi TX)
static constexpr int PIN_PI_UART_TX = 1;    // XIAO TX (Pi RX)

// =====================
// 通信パラメータ
// =====================
static constexpr uint32_t TC_BAUD = 300;
static constexpr uint32_t BIT_US  = (1000000UL + (TC_BAUD / 2)) / TC_BAUD; // ≒3333us
static constexpr uint32_t PI_BAUD = 9600;

HardwareSerial SerialPi(1);

// キュー
static QueueHandle_t qToTC = nullptr; // Pi -> TC (tc::TcFrames)
static QueueHandle_t qToPi = nullptr; // TC -> Pi (tc::Packet)

// デバッグ用カウンタ（必要ならSerialで表示してOK）
static volatile uint32_t gTcCsumOk = 0;
static volatile uint32_t gTcCsumNg = 0;

// 送信中フラグ（TC RXの自己受信防止）
static volatile bool gTcTxActive = false;

// =====================
// 9-bit BitBang
// =====================
static inline void writeLevel(bool high) {
  digitalWrite(PIN_TC_TX, high ? HIGH : LOW);
  delayMicroseconds(BIT_US);
}

static inline bool readLevel() {
  return (digitalRead(PIN_TC_RX) == HIGH);
}

// 9bitフレーム送信 [Start(0), 8-data, 9th(isCmd), Stop(1), Guard(1)]
static void send9bitFrame(uint8_t data, bool isCmd) {
  writeLevel(false); // Start
  for (int i = 0; i < 8; i++) writeLevel(((data >> i) & 0x01) != 0);
  writeLevel(isCmd); // 9th bit
  writeLevel(true);  // Stop
  writeLevel(true);  // Guard
}

static bool read9bitFrame(uint8_t &data, bool &isCmd) {
  // アイドル(HIGH)なら何もしない
  if (readLevel()) return false;

  // Startの中央へ
  delayMicroseconds(BIT_US / 2);
  if (readLevel()) return false; // 偽スタート棄却

  // bit0中央へ
  delayMicroseconds(BIT_US);

  data = 0;
  for (int i = 0; i < 8; i++) {
    if (readLevel()) data |= (1U << i);
    delayMicroseconds(BIT_US);
  }

  isCmd = readLevel();           // 9th
  delayMicroseconds(BIT_US);     // Stop位置へ

  if (!readLevel()) return false; // StopがHIGHでないならフレーム不正

  delayMicroseconds(BIT_US);      // Guard
  return true;
}

// =====================
// タスク
// =====================

// ---- Core1: Pi->TC 送信（BitBang）
static void taskTCSend(void *pv) {
  tc::TcFrames f{};
  while (true) {
    if (xQueueReceive(qToTC, &f, portMAX_DELAY) == pdTRUE) {
      gTcTxActive = true;

      // 送信前のMark（安定化）
      for (int i = 0; i < 3; i++) writeLevel(true);

      // data[4]には Piのチェックサムが入っている前提（tc_packet.hpp の toTcFrames 仕様）
      for (int i = 0; i < tc::TC_DATA_LEN; i++) send9bitFrame(f.data[i], false);
      send9bitFrame(f.cmd, true);

      gTcTxActive = false;
    }
  }
}

// ---- Core0: Pi UART 受信解析（PacketFactory使用）
static void taskPiRx(void *pv) {
  uint8_t ring[tc::RING_SIZE]{};
  uint8_t head = 0;
  uint64_t lastSig = 0;

  while (true) {
    while (SerialPi.available() > 0) {
      ring[head] = (uint8_t)SerialPi.read();
      head = (uint8_t)((head + 1) & (tc::RING_SIZE - 1));

      if (const tc::Packet* p = tc::PacketFactory::tryParse(ring, head, lastSig)) {
        tc::TcFrames f = tc::toTcFrames(*p);
        xQueueSend(qToTC, &f, pdMS_TO_TICKS(10));
      }
    }
    vTaskDelay(1);
  }
}

// ---- Core1: TC(BitBang)受信 -> checksum検証 -> Piへ
static void taskTCRx(void *pv) {
  uint8_t data[tc::TC_DATA_LEN]{};
  uint8_t n = 0;
  bool haveCmd = false;
  uint8_t cmd = 0;
  uint32_t lastUs = 0;

  while (true) {
    // 送信中は受信しない（自己受信・誤検出防止）
    if (gTcTxActive) {
      n = 0;
      haveCmd = false;
      vTaskDelay(1);
      continue;
    }

    uint8_t b; bool isCmd;
    if (read9bitFrame(b, isCmd)) {
      lastUs = micros();

      if (!isCmd) {
        if (n < tc::TC_DATA_LEN) data[n++] = b;
        else { n = 0; haveCmd = false; } // オーバーフローは同期崩れ扱い
      } else {
        // CMDが来た時点で data が揃ってないなら同期崩れなので捨てる
        if (n != tc::TC_DATA_LEN) {
          n = 0;
          haveCmd = false;
          continue;
        }
        cmd = b;
        haveCmd = true;
      }

      // data5本が揃っていて CMD を受けたらチェックサム検証
      if (haveCmd && n == tc::TC_DATA_LEN) {
        // Pi送信6バイトの構造: [cmd, d0, d1, d2, d3, checksum]
        const uint8_t expected = tc::checksum7((const uint8_t[5]){cmd, data[0], data[1], data[2], data[3]}, 5);

        if (data[4] == expected) {
          tc::Packet p{};
          p.len = tc::PI_LEN_SEND; // 6
          p.buf[0] = cmd;
          for (int i = 0; i < tc::TC_DATA_LEN; i++) p.buf[1 + i] = data[i]; // data[4]がchecksum
          xQueueSend(qToPi, &p, pdMS_TO_TICKS(20));
          gTcCsumOk++;
        } else {
          gTcCsumNg++;
        }

        // 次へ
        n = 0;
        haveCmd = false;
      }
    } else {
      // 無通信で同期クリア
      if ((n > 0 || haveCmd) && (micros() - lastUs) > 150000UL) {
        n = 0;
        haveCmd = false;
      }
      vTaskDelay(1);
    }
  }
}

// ---- Core0: Pi UART 送信
static void taskPiTx(void *pv) {
  tc::Packet p{};
  while (true) {
    if (xQueueReceive(qToPi, &p, portMAX_DELAY) == pdTRUE) {
      SerialPi.write(p.buf, p.len);
    }
  }
}

// =====================
// setup
// =====================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n--- [TC Bridge v2.4.5 FINAL] ---");

  // レベルシフタ：無効→方向固定→有効
  pinMode(PIN_LSHIFT_OE1, OUTPUT);  digitalWrite(PIN_LSHIFT_OE1, HIGH);
  pinMode(PIN_LSHIFT_OE2, OUTPUT);  digitalWrite(PIN_LSHIFT_OE2, HIGH);
  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH); // A->B
  pinMode(PIN_LSHIFT_DIR2, OUTPUT); digitalWrite(PIN_LSHIFT_DIR2, LOW);  // B->A

  // TC側
  pinMode(PIN_TC_TX, OUTPUT); digitalWrite(PIN_TC_TX, HIGH); // idle HIGH
  pinMode(PIN_TC_RX, INPUT_PULLUP);

  // Pi UART（取りこぼし対策でRXバッファ拡大）
  SerialPi.setRxBufferSize(256);
  SerialPi.begin(PI_BAUD, SERIAL_8N1, PIN_PI_UART_RX, PIN_PI_UART_TX);

  delay(100);
  digitalWrite(PIN_LSHIFT_OE1, LOW);
  digitalWrite(PIN_LSHIFT_OE2, LOW);

  qToTC = xQueueCreate(16, sizeof(tc::TcFrames));
  qToPi = xQueueCreate(16, sizeof(tc::Packet));

  // Core割り当て
  xTaskCreatePinnedToCore(taskPiRx,   "PiRx",   4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(taskPiTx,   "PiTx",   4096, nullptr, 3, nullptr, 0);

  xTaskCreatePinnedToCore(taskTCSend, "TCSend", 4096, nullptr, 6, nullptr, 1);
  xTaskCreatePinnedToCore(taskTCRx,   "TCRx",   4096, nullptr, 5, nullptr, 1);

  Serial.println("--- [READY] Pins: 1/2 (Pi), 43/44 (Nano), 5/6/7/8 (LSHIFT) ---");
}

void loop() {
  // 必要ならここで統計表示も可（今は何もしない）
  vTaskDelay(pdMS_TO_TICKS(1000));
}
