/**
 * TC Bridge (XIAO ESP32-S3) v2.4.6 MASTER_STABLE_FIX
 * * 修正内容：
 * 1. ピンアサインを「本番基板（SWAP配線）」に完全準拠：TC=GPIO1,2 / Pi=GPIO43,44
 * 2. 不足していた TcPacket 構造体の定義を追加（コンパイルエラーの解消）
 * 3. 300bps BitBang 送受信の安定化ロジックを統合
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet.hpp"

// =====================
// 0. 足りなかった型定義 (コンパイルエラー対策)
// =====================
struct TcPacket {
  static constexpr int LEN = 6; // コマンド1 + データ5
  uint8_t b[LEN];

  void dumpTo(Print &p) const {
    for (int i = 0; i < LEN; i++) {
      if (b[i] < 0x10) p.print('0');
      p.print(b[i], HEX);
      if (i < LEN - 1) p.print(' ');
    }
  }
};

// =====================
// 1. ピンアサイン (本番基板の SWAP 配線に固定) [cite: 483, 488-491]
// =====================

// 秋月レベルシフタ (SN74LVC16T245) 制御用
static constexpr int PIN_LSHIFT_OE1  = 6;
static constexpr int PIN_LSHIFT_DIR1 = 5;
static constexpr int PIN_LSHIFT_OE2  = 7;
static constexpr int PIN_LSHIFT_DIR2 = 8;

// TC側 (Nano Every) 通信ピン: SWAP配線 
static constexpr int PIN_TC_TX = 1; // ESP32 TX -> Nano RX
static constexpr int PIN_TC_RX = 2; // Nano TX -> ESP32 RX

// Pi側 (Raspberry Pi) 通信ピン: SWAP配線 
static constexpr int PIN_PI_UART_RX = 43; // Pi TX -> ESP32 RX
static constexpr int PIN_PI_UART_TX = 44; // ESP32 TX -> Pi RX

// =====================
// 2. 通信パラメータ
// =====================
static constexpr uint32_t TC_BAUD = 300;
static constexpr uint32_t BIT_US  = 3334; // 1/300s
static constexpr uint32_t PI_BAUD = 9600;

HardwareSerial SerialPi(1); 
static QueueHandle_t qToTC = nullptr;
static QueueHandle_t qToPi = nullptr;
static volatile bool gTcTxActive = false;

// =====================
// 3. BitBang 補助関数 (Core 1で使用) [cite: 495-503]
// =====================

static inline void writeLevel(bool high) {
  digitalWrite(PIN_TC_TX, high ? HIGH : LOW);
  delayMicroseconds(BIT_US);
}

static inline bool readLevel() {
  return (digitalRead(PIN_TC_RX) == HIGH);
}

static void send9bitFrame(uint8_t data, bool isCmd) {
  writeLevel(false); // Start bit (LOW)
  for (int i = 0; i < 8; i++) writeLevel(((data >> i) & 0x01) != 0);
  writeLevel(isCmd); // 9th bit
  writeLevel(true);  // Stop bit (HIGH)
  writeLevel(true);  // Guard
}

static bool read9bitFrame(uint8_t &data, bool &isCmd) {
  if (readLevel()) return false; 
  delayMicroseconds(BIT_US / 2); 
  if (readLevel()) return false; 
  delayMicroseconds(BIT_US);

  data = 0;
  for (int i = 0; i < 8; i++) {
    if (readLevel()) data |= (1U << i);
    delayMicroseconds(BIT_US);
  }
  isCmd = readLevel(); 
  delayMicroseconds(BIT_US); 
  if (!readLevel()) return false;
  delayMicroseconds(BIT_US);
  return true;
}

// =====================
// 4. FreeRTOS タスク [cite: 504-532]
// =====================

static void taskTCSend(void *pv) {
  tc::TcFrames f{};
  while (true) {
    if (xQueueReceive(qToTC, &f, portMAX_DELAY) == pdTRUE) {
      gTcTxActive = true;
      for (int i = 0; i < 3; i++) writeLevel(true);
      for (int i = 0; i < tc::TC_DATA_LEN; i++) send9bitFrame(f.data[i], false);
      send9bitFrame(f.cmd, true);
      gTcTxActive = false;
    }
  }
}

static void taskTCRx(void *pv) {
  uint8_t data[tc::TC_DATA_LEN]{};
  uint8_t n = 0;
  bool haveCmd = false;
  uint32_t lastUs = 0;

  while (true) {
    if (gTcTxActive) { n = 0; haveCmd = false; vTaskDelay(1); continue; }

    uint8_t b; bool isCmd;
    if (read9bitFrame(b, isCmd)) {
      lastUs = micros();
      if (!isCmd) {
        if (n < tc::TC_DATA_LEN) data[n++] = b;
        else n = 0;
        continue;
      }
      // コマンド受信時
      if (n == tc::TC_DATA_LEN) {
        TcPacket p{};
        p.b[0] = b; // cmd
        for (int i = 0; i < tc::TC_DATA_LEN; i++) p.b[1 + i] = data[i];
        (void)xQueueSend(qToPi, &p, 0);
      }
      n = 0;
    } else {
      if (n > 0 && (micros() - lastUs) > 150000UL) n = 0;
      vTaskDelay(1);
    }
  }
}

static void taskPiRx(void *pv) {
  uint8_t ring[tc::RING_SIZE]{};
  uint8_t head = 0;
  uint64_t lastSig = 0;
  while (true) {
    while (SerialPi.available() > 0) {
      ring[head] = (uint8_t)SerialPi.read();
      head = (uint8_t)((head + 1) & (tc::RING_SIZE - 1));
      if (const tc::Packet* p = tc::PacketFactory::tryParse(ring, head, lastSig)) {
        // --- ここにログ出力を追加 ---
        Serial.print("[Pi -> ESP] Received: ");
        for (int i = 0; i < p->len; i++) {
          if (p->buf[i] < 0x10) Serial.print('0');
          Serial.print(p->buf[i], HEX);
          Serial.print(' ');
        }
        Serial.println();
        // ------------------------
        tc::TcFrames f = tc::toTcFrames(*p);
        (void)xQueueSend(qToTC, &f, pdMS_TO_TICKS(10));
      }
    }
    vTaskDelay(1);
  }
}

static void taskPiTx(void *pv) {
  TcPacket p{};
  while (true) {
    if (xQueueReceive(qToPi, &p, portMAX_DELAY) == pdTRUE) {
      SerialPi.write(p.b, TcPacket::LEN);
    }
  }
}

// =====================
// 5. 初期設定 (setup) [cite: 533-540]
// =====================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n--- [TC Bridge v2.4.6 MASTER (SWAP Wiring)] ---");

  // レベルシフタ安全起動
  pinMode(PIN_LSHIFT_OE1, OUTPUT);  digitalWrite(PIN_LSHIFT_OE1, HIGH);
  pinMode(PIN_LSHIFT_OE2, OUTPUT);  digitalWrite(PIN_LSHIFT_OE2, HIGH);
  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH);
  pinMode(PIN_LSHIFT_DIR2, OUTPUT); digitalWrite(PIN_LSHIFT_DIR2, LOW);

  pinMode(PIN_TC_TX, OUTPUT); digitalWrite(PIN_TC_TX, HIGH); 
  pinMode(PIN_TC_RX, INPUT_PULLUP);

  SerialPi.begin(PI_BAUD, SERIAL_8N1, PIN_PI_UART_RX, PIN_PI_UART_TX);

  delay(100);
  digitalWrite(PIN_LSHIFT_OE1, LOW); 
  digitalWrite(PIN_LSHIFT_OE2, LOW);

  qToTC = xQueueCreate(16, sizeof(tc::TcFrames));
  qToPi = xQueueCreate(16, sizeof(TcPacket));

  xTaskCreatePinnedToCore(taskPiRx,   "PiRx",   4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(taskPiTx,   "PiTx",   4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(taskTCSend, "TCSend", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(taskTCRx,   "TCRx",   4096, nullptr, 5, nullptr, 1);

  Serial.println("--- [READY] SWAP pins: TC(1/2) Pi(43/44) ---");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}