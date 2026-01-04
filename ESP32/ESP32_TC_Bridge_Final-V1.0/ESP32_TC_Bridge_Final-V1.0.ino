/*
 * TC Bridge (XIAO ESP32-S3) v2.3.1 "HEADER_FUSED_FINAL"
 * ------------------------------------------------------------
 * 【配線（固定）】
 *  Pi <-> XIAO : HW UART 9600bps
 *    Pi TX  -> GPIO44 (XIAO RX)
 *    Pi RX <- GPIO43 (XIAO TX)
 *
 *  Nano Every(TC側) <-> XIAO : 9bit BitBang 300bps
 *    XIAO TX (GPIO1) -> LevelShifter -> Nano RX
 *    Nano TX -> LevelShifter -> XIAO RX (GPIO2)
 *
 *  LevelShifter(AE-LLCNV-LVCH16T245) 制御（固定）
 *    OE1=GPIO5 LOW enable / DIR1=GPIO6 HIGH (A->B) : XIAO->Nano
 *    OE2=GPIO7 LOW enable / DIR2=GPIO8 LOW  (B->A) : Nano->XIAO
 *
 * 【tc_packet.hpp 整合ポイント】
 *  - tc::PI_LEN_SEND = 6 （末尾1バイトがchecksum）
 *  - toTcFrames() は data[0..4]=buf[1..5] なので
 *      data[0..3]=実データ4バイト
 *      data[4]=checksum（cmd+data0..3の下位7bit）
 *  - よって TCへは「data5本（最後checksum）＋cmd」の9bitで送るのが正解
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet.hpp"

// =====================
// ピン（固定）
// =====================
static constexpr int PIN_LSHIFT_OE1  = 5;
static constexpr int PIN_LSHIFT_DIR1 = 6;
static constexpr int PIN_LSHIFT_OE2  = 7;
static constexpr int PIN_LSHIFT_DIR2 = 8;

static constexpr int PIN_TC_TX = 1;   // XIAO -> Nano
static constexpr int PIN_TC_RX = 2;   // Nano -> XIAO

static constexpr int PIN_PI_UART_RX = 44; // XIAO RX  (Pi TX)
static constexpr int PIN_PI_UART_TX = 43; // XIAO TX  (Pi RX)

// =====================
// 通信定数
// =====================
static constexpr bool     TC_INVERT_LOGIC = false; // 反転回路があるなら true
static constexpr uint32_t TC_BAUD = 300;
static constexpr uint32_t BIT_US  = (1000000UL + (TC_BAUD / 2)) / TC_BAUD;

static constexpr uint32_t PI_BAUD = 9600;

// =====================
// FreeRTOS / UART
// =====================
HardwareSerial SerialPi(1);

static QueueHandle_t qToTC = nullptr; // Pi -> TC (tc::TcFrames)
static QueueHandle_t qToPi = nullptr; // TC -> Pi (tc::Packet)

static portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

// =====================
// BitBang補助
// =====================
static inline bool applyInvert(bool logical) {
  return TC_INVERT_LOGIC ? !logical : logical;
}

static inline void writeLevel(bool logicalHigh) {
  const bool out = applyInvert(logicalHigh);
  digitalWrite(PIN_TC_TX, out ? HIGH : LOW);
  delayMicroseconds(BIT_US);
}

static inline bool readLevel() {
  const bool raw = (digitalRead(PIN_TC_RX) == HIGH);
  return applyInvert(raw);
}

// 9bit送信（Start + 8bit + 9th + Stop + Guard）
static void send9bitFrame(uint8_t data, bool isCmd) {
  // 300bpsはフレームが長いのでクリティカルも長くなる点に注意。
  // まずはタイミング優先で固定（不安定なら次にRMT化が有効）。
  portENTER_CRITICAL(&gMux);

  writeLevel(false); // Start
  for (int i = 0; i < 8; i++) writeLevel(((data >> i) & 0x01) != 0);
  writeLevel(isCmd); // 9th
  writeLevel(true);  // Stop
  writeLevel(true);  // Guard

  portEXIT_CRITICAL(&gMux);
}

// 9bit受信（簡易：スタート検出→中央サンプル）
// 成功時 true（Stop=HIGH確認済）
static bool read9bitFrame(uint8_t &data, bool &isCmd) {
  // IdleはHIGH
  if (readLevel() == true) return false;

  // Start中央へ
  delayMicroseconds(BIT_US / 2);
  if (readLevel() != false) return false; // ノイズ

  // bit0 を 1.5bit位置で読む
  delayMicroseconds(BIT_US);

  data = 0;
  for (int i = 0; i < 8; i++) {
    if (readLevel()) data |= (1U << i);
    delayMicroseconds(BIT_US);
  }

  // 9th
  isCmd = readLevel();
  delayMicroseconds(BIT_US);

  // Stop（HIGH必須）
  const bool stop = readLevel();
  if (!stop) return false;

  // Guard分進める
  delayMicroseconds(BIT_US);
  return true;
}

// =====================
// タスク
// =====================

// Core1: TC(Nano)へ送信（Pi->TC）
static void taskTCSend(void *pv) {
  tc::TcFrames f{};
  while (true) {
    if (xQueueReceive(qToTC, &f, portMAX_DELAY) == pdTRUE) {
      // Markのプリアンブル
      for (int i = 0; i < 3; i++) writeLevel(true);

      // data[0..4]（最後はchecksum）を 9th=0 で送る
      for (int i = 0; i < tc::TC_DATA_LEN; i++) {
        send9bitFrame(f.data[i], false);
      }

      // cmd を 9th=1 で送る
      send9bitFrame(f.cmd, true);
    }
  }
}

// Core0: Pi受信（PacketFactoryで厳密パース）→ qToTC
static void taskPiRx(void *pv) {
  uint8_t ring[tc::RING_SIZE]{};
  uint8_t head = 0;
  uint64_t lastSig = 0;

  while (true) {
    while (SerialPi.available() > 0) {
      const int v = SerialPi.read();
      if (v < 0) break;

      ring[head] = (uint8_t)v;
      head = (uint8_t)((head + 1) & (tc::RING_SIZE - 1));

      if (const tc::Packet* p = tc::PacketFactory::tryParse(ring, head, lastSig)) {
        // 6/8/12 のいずれかで checksum が合ったパケットだけ来る
        tc::TcFrames f = tc::toTcFrames(*p);

        // 8/12だった場合は追加情報が捨てられる（必要なら将来拡張）
        if (f.isTruncated) {
          // 必要ならデバッグログ（うるさければコメントアウト）
          // Serial.printf("[PiRx] truncated len=%u\n", p->len);
        }

        (void)xQueueSend(qToTC, &f, pdMS_TO_TICKS(10));
      }
    }
    vTaskDelay(1);
  }
}

// Core0: TC(Nano)受信 → 6バイト（PI_LEN_SEND）でパケット化 → qToPi
static void taskTCRx(void *pv) {
  uint8_t data[tc::TC_DATA_LEN]{};
  uint8_t n = 0;
  uint8_t cmd = 0;
  bool haveCmd = false;

  uint32_t lastUs = 0;

  while (true) {
    uint8_t b = 0;
    bool isCmd = false;

    if (read9bitFrame(b, isCmd)) {
      lastUs = micros();

      if (!isCmd) {
        if (n < tc::TC_DATA_LEN) {
          data[n++] = b;
        } else {
          // データが多すぎる → 同期崩れ扱いでリセット
          n = 0;
          haveCmd = false;
        }
      } else {
        cmd = b;
        haveCmd = true;
      }

      // DATA5本 + CMD が揃ったときだけ Piへ返す
      if (haveCmd && n == tc::TC_DATA_LEN) {
        tc::Packet p{};
        p.len = tc::PI_LEN_SEND; // 6固定

        // buf[0]=cmd
        p.buf[0] = cmd;

        // buf[1..5]=data[0..4]（data[4]はchecksum想定）
        for (int i = 0; i < tc::TC_DATA_LEN; i++) {
          p.buf[1 + i] = data[i];
        }

        // 念のため checksum を再計算して buf[5] を整合させる
        // （data[4]が既にchecksumのはずだが、ノイズで崩れた場合の安全策）
        p.buf[tc::PI_LEN_SEND - 1] = tc::checksum7(p.buf, tc::PI_LEN_SEND - 1);

        (void)xQueueSend(qToPi, &p, pdMS_TO_TICKS(20));

        // 次に備えてリセット
        n = 0;
        haveCmd = false;
      }

    } else {
      // 途中で途切れたら破棄（同期回復）
      if ((n > 0 || haveCmd) && lastUs != 0) {
        if ((uint32_t)(micros() - lastUs) > 150000UL) { // 150ms
          n = 0;
          haveCmd = false;
        }
      }
      vTaskDelay(1);
    }
  }
}

// Core0: Pi送信（SerialPiへのwriteはここ1本に固定）
static void taskPiTx(void *pv) {
  tc::Packet p{};
  while (true) {
    if (xQueueReceive(qToPi, &p, portMAX_DELAY) == pdTRUE) {
      SerialPi.write(p.buf, p.len);
    }
  }
}

// =====================
// setup / loop
// =====================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("--- [TC Bridge v2.3.1 HEADER_FUSED_FINAL] ---");
  Serial.printf("Pi UART : RX=%d TX=%d @%lu\n", PIN_PI_UART_RX, PIN_PI_UART_TX, (unsigned long)PI_BAUD);
  Serial.printf("TC 9bit : TX=%d RX=%d @%lu (BIT_US=%lu)\n", PIN_TC_TX, PIN_TC_RX, (unsigned long)TC_BAUD, (unsigned long)BIT_US);

  // レベルシフタ：起動直後は無効→DIR確定→有効（起動ノイズ低減）
  pinMode(PIN_LSHIFT_OE1, OUTPUT);  digitalWrite(PIN_LSHIFT_OE1, HIGH);
  pinMode(PIN_LSHIFT_OE2, OUTPUT);  digitalWrite(PIN_LSHIFT_OE2, HIGH);
  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH); // A->B
  pinMode(PIN_LSHIFT_DIR2, OUTPUT); digitalWrite(PIN_LSHIFT_DIR2, LOW);  // B->A

  // TC側 I/O
  pinMode(PIN_TC_TX, OUTPUT);
  digitalWrite(PIN_TC_TX, applyInvert(true) ? HIGH : LOW); // idle=HIGH（論理）
  pinMode(PIN_TC_RX, INPUT_PULLUP); // 浮き防止（idle HIGH）

  // Pi UART開始
  SerialPi.begin(PI_BAUD, SERIAL_8N1, PIN_PI_UART_RX, PIN_PI_UART_TX);

  delay(100);
  digitalWrite(PIN_LSHIFT_OE1, LOW);
  digitalWrite(PIN_LSHIFT_OE2, LOW);

  // キュー
  qToTC = xQueueCreate(16, sizeof(tc::TcFrames));
  qToPi = xQueueCreate(16, sizeof(tc::Packet));
  if (!qToTC || !qToPi) {
    Serial.println("[FATAL] Queue create failed.");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // タスク
  xTaskCreatePinnedToCore(taskTCSend, "TCSend", 4096, nullptr, 5, nullptr, 1); // Core1
  xTaskCreatePinnedToCore(taskTCRx,   "TCRx",   4096, nullptr, 6, nullptr, 0); // Core0
  xTaskCreatePinnedToCore(taskPiRx,   "PiRx",   4096, nullptr, 3, nullptr, 0); // Core0
  xTaskCreatePinnedToCore(taskPiTx,   "PiTx",   4096, nullptr, 3, nullptr, 0); // Core0

  Serial.println("--- [READY] GPIO: Pi(43/44) Nano(1/2) LS(5/6/7/8) ---");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
