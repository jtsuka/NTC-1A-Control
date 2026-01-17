// ESP32-S3 (XIAO) TestC Loopback (暫定配線版)
// - LevelShifter固定（OE1/2, DIR1/2）
// - TC UART: TX=GPIO1, RX=GPIO2 @300bps 8N1
// - 6 bytes send, wait echo 6 bytes

#include <Arduino.h>
#include <HardwareSerial.h>
#include "driver/gpio.h"

// レベルシフタ制御（あなたのv2.4.6確定）
static constexpr int PIN_LSHIFT_OE1  = 6;
static constexpr int PIN_LSHIFT_DIR1 = 5;
static constexpr int PIN_LSHIFT_OE2  = 7;
static constexpr int PIN_LSHIFT_DIR2 = 8;

// ★暫定デバッグ配線（必要ならここだけ変える）
// static constexpr int PIN_TC_TX = 1; // ESP->Nano（往路）
// static constexpr int PIN_TC_RX = 2; // Nano->ESP（復路）
static constexpr int PIN_TC_TX = 43;  // ESP32 -> Nano RX
static constexpr int PIN_TC_RX = 44;  // Nano TX -> ESP32


static constexpr uint32_t BAUD_USB = 115200;
static constexpr uint32_t BAUD_TC  = 300;

HardwareSerial SerialTC(2);

static const uint8_t pkt[6] = { 0xA5, 0x5A, 0x08, 0xF7, 0xFE, 0x0D };

static void printHex6(const uint8_t* p) {
  for (int i = 0; i < 6; i++) {
    if (p[i] < 0x10) Serial.print('0');
    Serial.print(p[i], HEX);
    Serial.print(i == 5 ? "" : " ");
  }
}

void setup() {
  Serial.begin(BAUD_USB);
  delay(200);

  // シフタ固定（トグル禁止）
  pinMode(PIN_LSHIFT_OE1, OUTPUT);  digitalWrite(PIN_LSHIFT_OE1, LOW);
  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH);
  pinMode(PIN_LSHIFT_OE2, OUTPUT);  digitalWrite(PIN_LSHIFT_OE2, LOW);
  pinMode(PIN_LSHIFT_DIR2, OUTPUT); digitalWrite(PIN_LSHIFT_DIR2, LOW);

  SerialTC.begin(BAUD_TC, SERIAL_8N1, PIN_TC_RX, PIN_TC_TX);

  Serial.println("=== ESP32 TestC Loopback (TEMP pins) ===");
  Serial.printf("TC UART: TX=%d RX=%d @%lu\n", PIN_TC_TX, PIN_TC_RX, BAUD_TC);
}

void loop() {
  // send
  SerialTC.write(pkt, 6);
  SerialTC.flush();
  Serial.print("[ESP TX] "); printHex6(pkt); Serial.println();

  // read back 6 bytes
  uint8_t rx[6];
  int got = 0;
  const uint32_t t0 = millis();
  while (got < 6 && (millis() - t0) < 1200) { // 余裕
    if (SerialTC.available()) rx[got++] = (uint8_t)SerialTC.read();
  }

  if (got == 6) {
    Serial.print("[ESP RX] "); printHex6(rx); Serial.println();
  } else {
    Serial.print("[TIMEOUT] got="); Serial.println(got);
    while (SerialTC.available()) (void)SerialTC.read();
  }

  delay(1000);
}
