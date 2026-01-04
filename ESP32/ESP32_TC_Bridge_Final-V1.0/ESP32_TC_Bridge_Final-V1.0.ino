/*
 * TC Bridge (XIAO ESP32-S3) v2.2.1 "FINAL_FIXED"
 * ---------------------------------------------------------------------------
 * 【接続構成】
 * 1. Pi <-> XIAO: HW UART (9600bps, 8N1)
 * - Pi TX  -> GPIO 44 (XIAO RX)
 * - Pi RX <- GPIO 43 (XIAO TX)
 * 2. Nano Every <-> XIAO: 9bit Software UART (300bps)
 * - Nano RX <- レベルシフタB側 <- GPIO 1 (XIAO TX)
 * - Nano TX  -> レベルシフタB側 -> GPIO 2 (XIAO RX)
 * 3. レベルシフタ(AE-LLCNV-LVCH16T245) 制御
 * - GPIO 5, 6, 7, 8
 * ---------------------------------------------------------------------------
 */

#include <Arduino.h>
#include <HardwareSerial.h>

// ========================================================
// 1. ピンアサイン定義 (GPIO番号直指定)
// ===================== ===================================
// 秋月レベルシフタ制御ピン [cite: 49-91, 112]
static constexpr int PIN_LSHIFT_OE1  = 5;  // 1OE: Bank1(XIAO->Nano)有効化。LOWで通電 [cite: 387-388]
static constexpr int PIN_LSHIFT_DIR1 = 6;  // 1DIR: Bank1の方向。HIGHで A(3.3V)からB(5V)へ [cite: 381-382]
static constexpr int PIN_LSHIFT_OE2  = 7;  // 2OE: Bank2(Nano->XIAO)有効化。LOWで通電 [cite: 390-391]
static constexpr int PIN_LSHIFT_DIR2 = 8;  // 2DIR: Bank2の方向。LOWで B(5V)からA(3.3V)へ [cite: 384-386]

// Nano Every (TC側) 通信ピン [cite: 172-177, 247-248]
static constexpr int PIN_TC_TX = 1;        // XIAOからNanoへ送る信号 (300bps/9bit)
static constexpr int PIN_TC_RX = 2;        // NanoからXIAOへ受ける信号 (300bps/9bit)

// Raspberry Pi 通信ピン (Hardware UART)
static constexpr int PIN_PI_UART_RX = 44;  // XIAOの受信口 (PiのTXを接続)
static constexpr int PIN_PI_UART_TX = 43;  // XIAOの送信口 (PiのRXを接続)

// ========================================================
// 2. 通信定数
// ========================================================
static constexpr bool     TC_INVERT_LOGIC = false; // 論理反転(74HC04等)がない場合はfalse
static constexpr uint32_t TC_BAUD         = 300;   // Nano側の特殊低速レート
static constexpr uint32_t BIT_US          = (1000000UL + (TC_BAUD / 2)) / TC_BAUD; // 1ビット長(約3333us)

static constexpr uint32_t PI_BAUD         = 9600;  // Pi側の標準レート
static constexpr uint8_t  PI_PKT_LEN      = 6;     // Piとの1パケット長(CMD+DATA5)
static constexpr uint8_t  TC_DATA_LEN     = 5;     // 9bitフレームのデータ構成数

// ========================================================
// 3. FreeRTOS オブジェクト
// ========================================================
HardwareSerial SerialPi(1); // Pi通信用シリアル(UART1)
static QueueHandle_t qToTC = nullptr; // Piから受信したジョブを送信タスクへ渡す
static QueueHandle_t qToPi = nullptr; // TCから受信したデータを転送タスクへ渡す

struct JobTxToTC { uint8_t cmd; uint8_t data[TC_DATA_LEN]; };
struct JobTxToPi { uint8_t b[PI_PKT_LEN]; }; // Piへ送る6バイト形式 [cmd, d0, d1, d2, d3, d4]

static portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED; // ビットタイミング保護用

// ========================================================
// 4. 通信補助関数
// ========================================================
static inline void writeLevel(bool logicalHigh) {
  const bool out = TC_INVERT_LOGIC ? !logicalHigh : logicalHigh;
  digitalWrite(PIN_TC_TX, out ? HIGH : LOW);
  delayMicroseconds(BIT_US);
}

static inline bool readLevel() {
  const bool raw = (digitalRead(PIN_TC_RX) == HIGH);
  return TC_INVERT_LOGIC ? !raw : raw;
}

// 9bitフレーム送信 (スタートビット + データ8bit + モード1bit + ストップビット)
static void send9bitFrame(uint8_t data, bool isCmd) {
  portENTER_CRITICAL(&gMux); // 割り込みによるタイミングのズレを防ぐ
  writeLevel(false); // Start bit
  for (int i = 0; i < 8; i++) writeLevel(((data >> i) & 0x01) != 0);
  writeLevel(isCmd); // 9th bit (0=Data, 1=Command)
  writeLevel(true);  // Stop bit
  writeLevel(true);  // Guard
  portEXIT_CRITICAL(&gMux);
}

// 9bitフレーム受信
static bool read9bitFrame(uint8_t &data, bool &isCmd) {
  if (readLevel() == true) return false; // アイドル状態(HIGH)
  delayMicroseconds(BIT_US / 2); // スタートビットの中央まで待機
  if (readLevel() != false) return false; // ノイズ判定
  delayMicroseconds(BIT_US);

  data = 0;
  for (int i = 0; i < 8; i++) {
    if (readLevel()) data |= (1U << i);
    delayMicroseconds(BIT_US);
  }
  isCmd = readLevel(); // 9th bitの読み取り
  delayMicroseconds(BIT_US);
  return readLevel(); // ストップビットがHIGHなら成功
}

// ========================================================
// 5. 並列タスク定義
// ========================================================

// 【Core 1】Nano Everyへの送信 (最高優先度)
static void taskTCSend(void *pv) {
  JobTxToTC j{};
  while (true) {
    if (xQueueReceive(qToTC, &j, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < 3; i++) writeLevel(true); // プリアンブル
      for (int i = 0; i < TC_DATA_LEN; i++) send9bitFrame(j.data[i], false);
      send9bitFrame(j.cmd, true);
    }
  }
}

// 【Core 0】Piからの受信解析
static void taskPiRx(void *pv) {
  uint8_t buf[PI_PKT_LEN]{};
  uint8_t idx = 0;
  uint32_t lastMs = 0;
  while (true) {
    while (SerialPi.available() > 0) {
      buf[idx++] = (uint8_t)SerialPi.read();
      lastMs = millis();
      if (idx >= PI_PKT_LEN) {
        JobTxToTC j{};
        j.cmd = buf[0];
        for (int i = 0; i < TC_DATA_LEN; i++) j.data[i] = buf[1 + i];
        xQueueSend(qToTC, &j, pdMS_TO_TICKS(10));
        idx = 0;
      }
    }
    if (idx > 0 && (millis() - lastMs) > 20) idx = 0; // タイムアウトでバッファクリア
    vTaskDelay(1);
  }
}

// 【Core 0】Nano Everyからの受信パケット組み立て
static void taskTCRx(void *pv) {
  uint8_t data[TC_DATA_LEN]{};
  uint8_t n = 0; uint8_t cmd = 0; bool haveCmd = false;
  uint32_t lastUs = 0;
  while (true) {
    uint8_t b; bool isCmd;
    if (read9bitFrame(b, isCmd)) {
      lastUs = micros();
      if (!isCmd) { if (n < TC_DATA_LEN) data[n++] = b; }
      else { cmd = b; haveCmd = true; }
      if (haveCmd && n == TC_DATA_LEN) {
        JobTxToPi out{}; out.b[0] = cmd;
        for (int i = 0; i < TC_DATA_LEN; i++) out.b[1 + i] = data[i];
        xQueueSend(qToPi, &out, pdMS_TO_TICKS(20));
        n = 0; haveCmd = false;
      }
    } else {
      if ((n > 0 || haveCmd) && (micros() - lastUs) > 150000UL) { n = 0; haveCmd = false; }
      vTaskDelay(1);
    }
  }
}

// 【Core 0】Piへの転送送信
static void taskPiTx(void *pv) {
  JobTxToPi out{};
  while (true) {
    if (xQueueReceive(qToPi, &out, portMAX_DELAY) == pdTRUE) {
      SerialPi.write(out.b, PI_PKT_LEN);
    }
  }
}

// ========================================================
// 6. メイン設定
// ========================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- [TC Bridge v2.2.1 FINAL] ---");

  // レベルシフタ初期化: 突入ノイズを避けるため無効化→方向決定→有効化の順 [cite: 381-391]
  pinMode(PIN_LSHIFT_OE1, OUTPUT);  digitalWrite(PIN_LSHIFT_OE1, HIGH);
  pinMode(PIN_LSHIFT_OE2, OUTPUT);  digitalWrite(PIN_LSHIFT_OE2, HIGH);
  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH); // A->B (3.3V->5V)
  pinMode(PIN_LSHIFT_DIR2, OUTPUT); digitalWrite(PIN_LSHIFT_DIR2, LOW);  // B->A (5V->3.3V)

  pinMode(PIN_TC_TX, OUTPUT); digitalWrite(PIN_TC_TX, HIGH); // アイドルはHIGH
  pinMode(PIN_TC_RX, INPUT_PULLUP); // 浮き防止

  SerialPi.begin(PI_BAUD, SERIAL_8N1, PIN_PI_UART_RX, PIN_PI_UART_TX);

  delay(100);
  digitalWrite(PIN_LSHIFT_OE1, LOW); // Bank1 有効化
  digitalWrite(PIN_LSHIFT_OE2, LOW); // Bank2 有効化

  qToTC = xQueueCreate(16, sizeof(JobTxToTC));
  qToPi = xQueueCreate(16, sizeof(JobTxToPi));

  // タスクの割り当て: タイミングがシビアな送信をCore1、その他をCore0で分担
  xTaskCreatePinnedToCore(taskTCSend, "TCSend", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(taskTCRx,   "TCRx",   4096, nullptr, 6, nullptr, 0);
  xTaskCreatePinnedToCore(taskPiRx,   "PiRx",   4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(taskPiTx,   "PiTx",   4096, nullptr, 3, nullptr, 0);

  Serial.println("--- [READY] GPIO: 43/44, 1/2, 5/6/7/8 ---");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}