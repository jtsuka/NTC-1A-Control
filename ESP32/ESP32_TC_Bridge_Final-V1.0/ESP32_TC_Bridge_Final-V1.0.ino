#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet.hpp"

#define DEBUG_TC_TX 0        
#define TC_INVERT_LOGIC true 
#define USE_TC_RX_PULLUP 1   

#define PIN_TC_TX 2
#define PIN_TC_RX 3
#define UART_PI_TX 43
#define UART_PI_RX 44
#define PIN_OE1 5
#define PIN_DIR1 6
#define PIN_OE2 7
#define PIN_DIR2 8

#define TC_BAUD 300
#define BIT_US ((1000000UL + (TC_BAUD/2)) / TC_BAUD) 

HardwareSerial SerialPi(1);
static QueueHandle_t qJobs;
struct Job { tc::TcFrames f; };

static uint32_t droppedCount = 0;
static uint32_t lastReportTime = 0;

void writeLevel(bool logical) {
  bool out = TC_INVERT_LOGIC ? !logical : logical;
  digitalWrite(PIN_TC_TX, out ? HIGH : LOW);
  delayMicroseconds(BIT_US);
}

void send9bitFrame(uint8_t data, bool isCmd) {
  writeLevel(false); 
  for (int i = 0; i < 8; i++) writeLevel((data >> i) & 0x01);
  writeLevel(isCmd); 
  writeLevel(true);  
  writeLevel(true);  
}

void taskTC(void* pv) {
  Job j;
  while (true) {
    if (xQueueReceive(qJobs, &j, portMAX_DELAY)) {
      for (int i=0; i<3; i++) writeLevel(true); 
      for (int i=0; i<tc::TC_DATA_LEN; i++) send9bitFrame(j.f.data[i], false);
      send9bitFrame(j.f.cmd, true); 
      #if DEBUG_TC_TX
      Serial.printf("[TcTX] Sent CMD:0x%02X\n", j.f.cmd);
      #endif
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
        if (xQueueSend(qJobs, &j, 0) != pdTRUE) {
          droppedCount++;
        } else {
          Serial.printf("[PiRX] Len:%d CMD:0x%02X %s\n", 
                        p->len, p->cmd(), j.f.isTruncated ? "(TRUNC)" : "");
        }
      }
    }
    uint32_t now = millis();
    if (now - lastReportTime > 1000) {
      if (droppedCount > 0) {
        Serial.printf("[WARN] Dropped %u packets in 1s.\n", droppedCount);
        droppedCount = 0;
      }
      lastReportTime = now;
    }
    vTaskDelay(1); 
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); 

  Serial.println("\n--- [System Initialization v1.2.5-Final] ---");

  pinMode(PIN_OE1, OUTPUT);  digitalWrite(PIN_OE1, LOW);
  pinMode(PIN_DIR1, OUTPUT); digitalWrite(PIN_DIR1, HIGH);
  pinMode(PIN_OE2, OUTPUT);  digitalWrite(PIN_OE2, LOW);
  pinMode(PIN_DIR2, OUTPUT); digitalWrite(PIN_DIR2, LOW);

  #if USE_TC_RX_PULLUP
    pinMode(PIN_TC_RX, INPUT_PULLUP);
  #else
    pinMode(PIN_TC_RX, INPUT);
  #endif

  Serial.println("[CHECK] Software Internal Pin Logic State:");
  bool results[] = {
    digitalRead(PIN_OE1) == LOW, digitalRead(PIN_DIR1) == HIGH,
    digitalRead(PIN_OE2) == LOW, digitalRead(PIN_DIR2) == LOW
  };
  const char* labels[] = {"OE1(LOW)", "DIR1(HIGH)", "OE2(LOW)", "DIR2(LOW)"};
  for(int i=0; i<4; i++) {
    Serial.printf("  - %-12s: %s\n", labels[i], results[i] ? "OK" : "!! FAIL !!");
  }

  pinMode(PIN_TC_TX, OUTPUT);
  digitalWrite(PIN_TC_TX, TC_INVERT_LOGIC ? LOW : HIGH);
  Serial.printf("[INIT] TC TX IDLE (GPIO %d, BIT_US:%lu)\n", PIN_TC_TX, (unsigned long)BIT_US);

  SerialPi.begin(9600, SERIAL_8N1, UART_PI_RX, UART_PI_TX);
  qJobs = xQueueCreate(16, sizeof(Job)); // キューを16へ拡張
  if (qJobs == NULL) {
    Serial.println("[CRITICAL] Queue Create Failed!");
    while(1) { delay(1000); }
  }

  xTaskCreatePinnedToCore(taskPi, "Pi", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskTC, "TC", 4096, NULL, 5, NULL, 1);

  Serial.println("--- [READY] v1.2.5-Final Running ---\n");
}

void loop() { vTaskDelay(portMAX_DELAY); }