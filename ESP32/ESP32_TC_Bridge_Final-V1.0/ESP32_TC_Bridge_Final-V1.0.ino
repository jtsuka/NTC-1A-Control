#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet.hpp"

/*
   ESP32 TC Bridge Final V1.2.1
*/

// ---- Pins (固定) ----
#define PIN_TC_TX 2
#define PIN_TC_RX 3
#define UART_PI_TX 43
#define UART_PI_RX 44
#define PIN_OE1 5
#define PIN_DIR1 6
#define PIN_OE2 7
#define PIN_DIR2 8

// ---- Config ----
#define TC_BAUD 300
#define BIT_US (1000000UL / TC_BAUD)
#define TC_INVERT_LOGIC true 

HardwareSerial SerialPi(1);
static QueueHandle_t qJobs;
struct Job { tc::TcFrames f; };

// 論理レベル関数：送信中のビットタイミング生成
void writeLevel(bool logical) {
  bool out = TC_INVERT_LOGIC ? !logical : logical;
  digitalWrite(PIN_TC_TX, out ? HIGH : LOW);
  delayMicroseconds(BIT_US);
}

// 9bitフレーム：start(0) + 8data + 1cmd + stop(1)
void send9bitFrame(uint8_t data, bool isCmd) {
  writeLevel(false); // Start
  for (int i = 0; i < 8; i++) writeLevel((data >> i) & 0x01);
  writeLevel(isCmd); // 9th bit
  writeLevel(true);  // Stop
  writeLevel(true);  // 1bit Idle Gap
}

/* --- Core 1: TC送信タスク --- */
void taskTC(void* pv) {
  Job j;
  while (true) {
    if (xQueueReceive(qJobs, &j, portMAX_DELAY)) {
      // 送信前アイドル（論理HIGH）
      for (int i=0; i<3; i++) writeLevel(true); 
      
      for (int i=0; i<tc::TC_DATA_LEN; i++) send9bitFrame(j.f.data[i], false);
      send9bitFrame(j.f.cmd, true); 
      
      Serial.printf("[TcTX] CMD:0x%02X Sent.\n", j.f.cmd);
    }
  }
}

/* --- Core 0: Pi受信タスク --- */
void taskPi(void* pv) {
  uint8_t ring[tc::RING_SIZE]{};
  uint8_t head = 0;
  uint64_t lastSig = 0;
  while (true) {
    while (SerialPi.available()) {
      ring[head] = SerialPi.read();
      head = (head + 1) & (tc::RING_SIZE - 1);
      if (const tc::Packet* p = tc::PacketFactory::tryParse(ring, head, lastSig)) {
        Job j; j.f = tc::toTcFrames(*p);
        xQueueSend(qJobs, &j, 0);
        
        Serial.printf("[PiRX] Len:%d CMD:0x%02X %s\n", 
                      p->len, p->cmd(), j.f.isTruncated ? "(TRUNCATED)" : "");
      }
    }
    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200);
  SerialPi.begin(9600, SERIAL_8N1, UART_PI_RX, UART_PI_TX);
  
  // レベルシフタ完全固定
  pinMode(PIN_OE1, OUTPUT); digitalWrite(PIN_OE1, LOW);
  pinMode(PIN_DIR1, OUTPUT); digitalWrite(PIN_DIR1, HIGH);
  pinMode(PIN_OE2, OUTPUT); digitalWrite(PIN_OE2, LOW);
  pinMode(PIN_DIR2, OUTPUT); digitalWrite(PIN_DIR2, LOW);

  // setup内では delayMicroseconds を含む writeLevel(true) を避ける
  pinMode(PIN_TC_TX, OUTPUT); 
  digitalWrite(PIN_TC_TX, TC_INVERT_LOGIC ? LOW : HIGH); // 論理HIGHの電位

  qJobs = xQueueCreate(8, sizeof(Job));

  xTaskCreatePinnedToCore(taskPi, "Pi", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskTC, "TC", 4096, NULL, 5, NULL, 1);
  Serial.println("ESP32 Fusion v1.2.1 READY");
}

void loop() { vTaskDelay(portMAX_DELAY); }