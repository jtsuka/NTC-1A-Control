// Nano Every: SoftwareSerial RX6 + Echo + LED(5x)
// RX=D10, TX=D11, 300bps 8N1

#include <Arduino.h>
#include <SoftwareSerial.h>

static const uint32_t BAUD_USB = 115200;
static const uint32_t BAUD_TC  = 300;

static const uint8_t  PACKET_LEN = 6;
static const uint16_t RX_GAP_TIMEOUT_MS = 150;

static const uint8_t  PIN_SOFT_RX = 10; // D10
static const uint8_t  PIN_SOFT_TX = 11; // D11

SoftwareSerial TcSoft(PIN_SOFT_RX, PIN_SOFT_TX); // RX, TX

uint8_t  buf[PACKET_LEN];
uint8_t  idx = 0;
uint32_t t_last_rx = 0;
uint32_t pkt_count = 0;

// LED 5回点滅（ブロッキングでOK：300bpsのテスト用途）
static void blink5() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(50);
    digitalWrite(LED_BUILTIN, LOW);  delay(50);
  }
}

static void printHex6(const uint8_t *p) {
  for (int i = 0; i < 6; i++) {
    if (p[i] < 0x10) Serial.print('0');
    Serial.print(p[i], HEX);
    Serial.print(i == 5 ? "" : " ");
  }
}

void setup() {
  Serial.begin(BAUD_USB);
  while (!Serial) {}

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  TcSoft.begin(BAUD_TC);

  Serial.println("=== Nano Every SoftUART Test ===");
  Serial.println("SoftUART: RX=D10, TX=D11 @300bps");
  Serial.println("LED blinks 5x ONLY when 6 bytes received.");
}

void loop() {
  // バイト間ギャップでpartial破棄
  if (idx > 0 && (millis() - t_last_rx) > RX_GAP_TIMEOUT_MS) {
    Serial.print("[TIMEOUT] partial=");
    Serial.println(idx);
    idx = 0;
  }

  while (TcSoft.available() > 0) {
    uint8_t b = (uint8_t)TcSoft.read();
    t_last_rx = millis();

    buf[idx++] = b;

    if (idx >= PACKET_LEN) {
      pkt_count++;

      blink5();

      Serial.print("[RX#"); Serial.print(pkt_count); Serial.print("] ");
      printHex6(buf); Serial.println();

      // Echo back
      TcSoft.write(buf, PACKET_LEN);
      TcSoft.flush();

      Serial.print("[TX#"); Serial.print(pkt_count); Serial.print("] ");
      printHex6(buf); Serial.println();

      idx = 0;
    }
  }
}
