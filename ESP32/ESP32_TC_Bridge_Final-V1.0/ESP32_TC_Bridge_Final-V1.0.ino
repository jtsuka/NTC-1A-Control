#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet.hpp"

// ========================================================
// 【完全固定】GPIO ピンアサイン (XIAO視点の命名に統一)
// ========================================================
// --- レベルシフタ制御系 ---
#define PIN_LSHIFT_OE1    5   // 1OE: LOWで有効 (Bank1: XIAO->Nano)
#define PIN_LSHIFT_DIR1   6   // 1DIR: HIGHで A->B 方向固定
#define PIN_LSHIFT_OE2    7   // 2OE: LOWで有効 (Bank2: Nano->XIAO)
#define PIN_LSHIFT_DIR2   8   // 2DIR: LOWで B->A 方向固定

// --- 信号通信系 ---
#define PIN_TC_UART_TX    43  // Nanoへの送信 (GPIO43)
#define PIN_TC_UART_RX    44  // Nanoからの受信 (GPIO44)
#define PIN_PI_UART_RX    2   // Piからの信号受信 (Pi TXから入力)
#define PIN_PI_UART_TX    1   // Piへの信号送信 (Pi RXへ出力)

// --- 通信パラメータ ---
#define TC_INVERT_LOGIC   false 
#define TC_BAUD           300
#define BIT_US            ((1000000UL + (TC_BAUD/2)) / TC_BAUD) 

HardwareSerial SerialPi(1);
static QueueHandle_t qToTC; // Pi -> TC 送信待ち
static QueueHandle_t qToPi; // TC -> Pi 転送待ち (双方向用)

struct Job { tc::TcFrames f; };

// ========================================================
// 9bit UART 送受信処理 (ソフトウェア・ビットバン)
// ========================================================

// 1ビット送信 (クリティカルセクションでジッタ抑制)
void writeLevel(bool logical) {
  bool out = TC_INVERT_LOGIC ? !logical : logical;
  digitalWrite(PIN_TC_UART_TX, out ? HIGH : LOW);
  delayMicroseconds(BIT_US);
}

// 9bitフレーム送信 (5データ + 1コマンド)
void send9bitFrame(uint8_t data, bool isCmd) {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&mux);
  writeLevel(false); // Start
  for (int i = 0; i < 8; i++) writeLevel((data >> i) & 0x01);
  writeLevel(isCmd); // 9th bit
  writeLevel(true);  // Stop
  writeLevel(true);  // Guard
  portEXIT_CRITICAL(&mux);
}

// 9bitフレーム受信 (簡易実装: スタートビット検出後にサンプリング)
// ※実運用ではタイマー割り込みが理想ですが、300bpsならこれでも追従可能です
bool read9bitFrame(uint8_t &data, bool &isCmd) {
  if (digitalRead(PIN_TC_UART_RX) == HIGH) return false; // アイドル

  uint32_t start = micros();
  delayMicroseconds(BIT_US / 2); // スタートビットの中央まで待機
  
  if (digitalRead(PIN_TC_UART_RX) != LOW) return false; // 偽のスタートビット

  data = 0;
  for (int i = 0; i < 8; i++) {
    delayMicroseconds(BIT_US);
    if (digitalRead(PIN_TC_UART_RX)) data |= (1 << i);
  }
  delayMicroseconds(BIT_US);
  isCmd = digitalRead(PIN_TC_UART_RX);
  
  delayMicroseconds(BIT_US); // ストップビット分待機
  return true;
}

// ========================================================
// FreeRTOS タスク
// ========================================================

// Core 1: Nanoへの送信タスク
void taskTCSend(void* pv) {
  Job j;
  while (true) {
    if (xQueueReceive(qToTC, &j, portMAX_DELAY)) {
      for (int i=0; i<3; i++) writeLevel(true); // プリアンブル
      for (int i=0; i<tc::TC_DATA_LEN; i++) send9bitFrame(j.f.data[i], false);
      send9bitFrame(j.f.cmd, true); 
    }
  }
}

// Core 1: Nanoからの受信・Piへの転送タスク
void taskTCRead(void* pv) {
  uint8_t data;
  bool isCmd;
  while (true) {
    if (read9bitFrame(data, isCmd)) {
      // 受信した生データをPiへそのまま転送 (必要ならパケット化)
      SerialPi.write(data);
      if (isCmd) SerialPi.write(0xFF); // コマンドフラグ代わりのマーカーなど
    }
    vTaskDelay(1);
  }
}

// Core 0: Piからの受信解析
void taskPiRead(void* pv) {
  uint8_t ring[tc::RING_SIZE]{};
  uint8_t head = 0;
  uint64_t lastSig = 0;
  while (true) {
    while (SerialPi.available()) {
      ring[head] = (uint8_t)SerialPi.read();
      head = (uint8_t)((head + 1) & (tc::RING_SIZE - 1));
      if (const tc::Packet* p = tc::PacketFactory::tryParse(ring, head, lastSig)) {
        Job j; j.f = tc::toTcFrames(*p);
        // キューが一杯なら10ms待機して確実に送る
        xQueueSend(qToTC, &j, pdMS_TO_TICKS(10));
      }
    }
    vTaskDelay(1); 
  }
}

// ========================================================
// メイン設定
// ========================================================
void setup() {
  Serial.begin(115200);
  delay(1000); 

  // --- レベルシフタ制御設定 ---
  // 起動時のノイズ回避のため、まずはOEをHIGH(無効)にしてから方向を固める
  pinMode(PIN_LSHIFT_OE1, OUTPUT);  digitalWrite(PIN_LSHIFT_OE1, HIGH);
  pinMode(PIN_LSHIFT_OE2, OUTPUT);  digitalWrite(PIN_LSHIFT_OE2, HIGH);

  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH); // A->B [cite: 381-382]
  pinMode(PIN_LSHIFT_DIR2, OUTPUT); digitalWrite(PIN_LSHIFT_DIR2, LOW);  // B->A [cite: 385-386]

  // 方向が安定してからOEを有効化 [cite: 388, 391]
  delay(100); 
  digitalWrite(PIN_LSHIFT_OE1, LOW); 
  digitalWrite(PIN_LSHIFT_OE2, LOW); 

  // --- 信号ピン設定 ---
  pinMode(PIN_TC_UART_TX, OUTPUT); digitalWrite(PIN_TC_UART_TX, HIGH);
  pinMode(PIN_TC_UART_RX, INPUT);

  // PiとのUART通信 (命名整理版)
  SerialPi.begin(9600, SERIAL_8N1, PIN_PI_UART_RX, PIN_PI_UART_TX);
  
  qToTC = xQueueCreate(16, sizeof(Job));
  
  xTaskCreatePinnedToCore(taskPiRead, "PiRead", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskTCSend, "TCSend", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(taskTCRead, "TCRead", 4096, NULL, 5, NULL, 1);

  Serial.println("--- [Bridge v2.1.0] Bi-Directional Ready ---");
}

void loop() { vTaskDelay(1000); }