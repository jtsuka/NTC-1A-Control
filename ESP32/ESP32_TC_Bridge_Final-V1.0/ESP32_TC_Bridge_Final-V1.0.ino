#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet.hpp"

// ========================================================
// 【沼脱出】J1, J2, J6, J7 の独立ピンを 6 本フル活用
// ========================================================
#define PIN_OEB    D0   // J1 黄
#define PIN_DIRB   D1   // J1 白
#define UART_PI_RX D2   // J6 白 (Piからの受信)
#define PIN_DIRA   D3   // J7 黄
#define PIN_OEA    D4   // J7 白
#define PIN_TC_TX  D5   // J2 白 (Nanoへの送信)

#define TC_INVERT_LOGIC false // アイドルHIGHの標準UART
#define TC_BAUD 300
#define BIT_US ((1000000UL + (TC_BAUD/2)) / TC_BAUD) 

HardwareSerial SerialPi(1);
static QueueHandle_t qJobs;
struct Job { tc::TcFrames f; };

void writeLevel(bool logical) {
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

void taskTC(void* pv) {
  Job j;
  while (true) {
    if (xQueueReceive(qJobs, &j, portMAX_DELAY)) {
      for (int i=0; i<3; i++) writeLevel(true); 
      for (int i=0; i<tc::TC_DATA_LEN; i++) send9bitFrame(j.f.data[i], false);
      send9bitFrame(j.f.cmd, true); 
    }
  }
}

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
        xQueueSend(qJobs, &j, 0);
      }
    }
    vTaskDelay(1); 
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000); 
  Serial.println("\n--- [System Initialization v1.4.4-FINAL_Clean] ---");

  // 全ピンを物理配線に合わせて初期化
  pinMode(PIN_OEA, OUTPUT);  digitalWrite(PIN_OEA, HIGH); // 門を開ける
  pinMode(PIN_OEB, OUTPUT);  digitalWrite(PIN_OEB, HIGH); 
  pinMode(PIN_DIRA, OUTPUT); digitalWrite(PIN_DIRA, HIGH); // ESP->Nano
  pinMode(PIN_DIRB, OUTPUT); digitalWrite(PIN_DIRB, LOW);  // Nano->ESP
  
  pinMode(PIN_TC_TX, OUTPUT);
  digitalWrite(PIN_TC_TX, HIGH); // アイドルHIGH

  SerialPi.begin(9600, SERIAL_8N1, UART_PI_RX, -1);
  qJobs = xQueueCreate(16, sizeof(Job));
  
  xTaskCreatePinnedToCore(taskPi, "Pi", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskTC, "TC", 4096, NULL, 5, NULL, 1);

  Serial.println("--- [READY] CLEAN 6-PIN ASSIGNMENT ---");
}

void loop() {
  delay(3000);
  Serial.println("[TEST] Sending F0 01 02 03 04 05 to D5...");
  Job j;
  j.f.data[0]=0xF0; j.f.data[1]=0x01; j.f.data[2]=0x02;
  j.f.data[3]=0x03; j.f.data[4]=0x04; j.f.data[5]=0x05;
  j.f.cmd = 0xF0;
  xQueueSend(qJobs, &j, 0);
}