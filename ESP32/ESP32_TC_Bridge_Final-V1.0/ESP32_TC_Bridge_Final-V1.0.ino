#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet.hpp"

// --- 基本設定 ---
#define TC_INVERT_LOGIC true // 負論理出力
#define TC_BAUD 300
#define BIT_US ((1000000UL + (TC_BAUD/2)) / TC_BAUD) 

// --- 【重要】物理ピン定義 (昨夜のテストで確定済) ---
#define PIN_TC_TX 2    // 物理TXB (D1)
#define PIN_TC_RX 3    // 物理RXB (D2)
#define UART_PI_TX 43  // D6
#define UART_PI_RX 44  // D7

#define PIN_OE1  8     // OEA
#define PIN_DIR1 6     // DIRA
#define PIN_OE2  7     // OEB
#define PIN_DIR2 5     // DIRB

HardwareSerial SerialPi(1);
static QueueHandle_t qJobs;
struct Job { tc::TcFrames f; };

// 物理レベル出力
void writeLevel(bool logical) {
  // logical=true (アイドル) のとき、反転時は out=false (物理LOW) になる
  bool out = TC_INVERT_LOGIC ? !logical : logical;
  digitalWrite(PIN_TC_TX, out ? HIGH : LOW);
  delayMicroseconds(BIT_US);
}

void send9bitFrame(uint8_t data, bool isCmd) {
  writeLevel(false); // Start bit
  for (int i = 0; i < 8; i++) writeLevel((data >> i) & 0x01);
  writeLevel(isCmd); // 9th bit
  writeLevel(true);  // Stop bit
  writeLevel(true);  // Guard
}

// 送信タスク (Core 1)
void taskTC(void* pv) {
  Job j;
  while (true) {
    if (xQueueReceive(qJobs, &j, portMAX_DELAY)) {
      for (int i=0; i<3; i++) writeLevel(true); // Idle padding
      for (int i=0; i<tc::TC_DATA_LEN; i++) send9bitFrame(j.f.data[i], false);
      send9bitFrame(j.f.cmd, true); 
    }
  }
}

// 受信タスク (Core 0)
void taskPi(void* pv) {
  uint8_t ring[tc::RING_SIZE]{};
  uint8_t head = 0;
  uint64_t lastSig = 0;
  while (true) {
    while (SerialPi.available()) {
      ring[head] = (uint8_t)SerialPi.read();
      head = (uint8_t)((head + 1) & (tc::RING_SIZE - 1));
      if (const tc::Packet* p = tc::PacketFactory::tryParse(ring, head, lastSig)) {
        Job j; j.f = tc::toTcFrames(*p);
        if (xQueueSend(qJobs, &j, 0) == pdTRUE) {
          // 【修正】チャッピー案：生データ B0, B1 を表示
          Serial.printf("[PiRX] Len:%d B0:0x%02X B1:0x%02X %s\n", 
                        p->len, p->buf[0], p->buf[1], j.f.isTruncated ? "(TRUNC)" : "");
        }
      }
    }
    vTaskDelay(1); 
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); 

  Serial.println("\n--- [System Initialization v1.3.1] ---");

  // レベルシフタ初期化 (昨夜のテスト結果を反映)
  pinMode(PIN_OE1, OUTPUT);  digitalWrite(PIN_OE1, LOW);  // 門 A 開ける
  pinMode(PIN_DIR1, OUTPUT); digitalWrite(PIN_DIR1, HIGH); // A 送信方向
  pinMode(PIN_OE2, OUTPUT);  digitalWrite(PIN_OE2, LOW);  // 門 B 開ける
  pinMode(PIN_DIR2, OUTPUT); digitalWrite(PIN_DIR2, LOW);  // B 受信方向

  pinMode(PIN_TC_RX, INPUT_PULLUP);
  pinMode(PIN_TC_TX, OUTPUT);

  // 【致命傷の修正】チャッピー指摘のアイドル物理レベル
  // logical=true(アイドル) を writeLevel のロジックに合わせる
  digitalWrite(PIN_TC_TX, TC_INVERT_LOGIC ? LOW : HIGH); 

  SerialPi.begin(9600, SERIAL_8N1, UART_PI_RX, UART_PI_TX);
  qJobs = xQueueCreate(16, sizeof(Job));
  
  xTaskCreatePinnedToCore(taskPi, "Pi", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskTC, "TC", 4096, NULL, 5, NULL, 1);

  Serial.println("--- [READY] v1.3.1: Physical Idle Level Fixed ---\n");
}

void loop() { vTaskDelay(portMAX_DELAY); }