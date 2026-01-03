#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet.hpp"

// ========================================================
// 【決定版】XIAO ESP32-S3 ピンアサイン (GPIO番号直指定)
// ========================================================
#define PIN_OE1    5   // OE1  -> LOWで有効 (秋月基板 1OE)
#define PIN_DIR1   6   // DIR1 -> HIGHで A->B (秋月基板 1DIR)
#define PIN_OE2    7   // OE2  -> LOWで有効 (秋月基板 2OE)
#define PIN_DIR2   8   // DIR2 -> LOWで B->A (秋月基板 2DIR)
#define PIN_TC_TX  43  // TXA  -> Nanoへの送信 (9bit bit-bang用)
#define PIN_TC_RX  44  // RXA  -> Nanoからの受信 (今回検査用にトグル可能)
#define PIN_PI_TX  2   // PI_TX -> PiのTXを受けるピン (XIAOのRX)
#define PIN_PI_RX  1   // PI_RX -> PiのRXへ送るピン (今回検査用にトグル可能)

#define TC_INVERT_LOGIC false 
#define TC_BAUD 300
#define BIT_US ((1000000UL + (TC_BAUD/2)) / TC_BAUD) 

HardwareSerial SerialPi(1);
static QueueHandle_t qJobs;
struct Job { tc::TcFrames f; };

// 9bit UART用の信号出力
void writeLevel(bool logical) {
  bool out = TC_INVERT_LOGIC ? !logical : logical;
  digitalWrite(PIN_TC_TX, out ? HIGH : LOW);
  delayMicroseconds(BIT_US);
}

// 9bitフレーム送信
void send9bitFrame(uint8_t data, bool isCmd) {
  writeLevel(false); // Start
  for (int i = 0; i < 8; i++) writeLevel((data >> i) & 0x01);
  writeLevel(isCmd); // 9th bit
  writeLevel(true);  // Stop
  writeLevel(true);  // Guard
}

// Nano側送信タスク (Core 1)
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

// Raspberry Pi側受信タスク (Core 0)
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
  Serial.println("\n--- [System Initialization v1.5.0-FINAL] ---");

  // --- レベルシフタ制御ピンの初期設定 ---
  // 秋月基板のOEはLOWで有効（門が開く）です
  pinMode(PIN_OE1, OUTPUT);  digitalWrite(PIN_OE1, LOW);  // OE1有効
  pinMode(PIN_OE2, OUTPUT);  digitalWrite(PIN_OE2, LOW);  // OE2有効

  // DIRA: HIGH(A->B) / DIRB: LOW(B->A)
  pinMode(PIN_DIR1, OUTPUT); digitalWrite(PIN_DIR1, HIGH); 
  pinMode(PIN_DIR2, OUTPUT); digitalWrite(PIN_DIR2, LOW);  

  // --- 信号ピンの初期設定 ---
  pinMode(PIN_TC_TX, OUTPUT);
  digitalWrite(PIN_TC_TX, HIGH); // アイドルHIGH

  // 今回のデバッグ用トグル対象ピンを出力に設定
  pinMode(PIN_TC_RX, OUTPUT);
  pinMode(PIN_PI_RX, OUTPUT);

  // --- Piとの通信開始 (XIAO RX = PIN_PI_TX) ---
  SerialPi.begin(9600, SERIAL_8N1, PIN_PI_TX, -1);
  
  qJobs = xQueueCreate(16, sizeof(Job));
  
  xTaskCreatePinnedToCore(taskPi, "Pi", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskTC, "TC", 4096, NULL, 5, NULL, 1);

  Serial.println("--- [READY] PIN_FIXED_GPIO_MAPPING ---");
}

void loop() {
  // 通信テスト: 3秒おきにパケットを Nano(D5/GPIO43) へ送信
  delay(3000);
  Serial.println("[TEST] Sending Frame to Nano (GPIO43)...");
  
  // 検査用のトグル動作 (RXAとPI_RXを交互に反転)
  static bool toggle = false;
  toggle = !toggle;
  digitalWrite(PIN_TC_RX, toggle ? HIGH : LOW);
  digitalWrite(PIN_PI_RX, toggle ? LOW : HIGH);

  Job j;
  j.f.data[0]=0xF0; j.f.data[1]=0x01; j.f.data[2]=0x02;
  j.f.data[3]=0x03; j.f.data[4]=0x04; j.f.data[5]=0x05;
  j.f.cmd = 0xF0;
  xQueueSend(qJobs, &j, 0);
}