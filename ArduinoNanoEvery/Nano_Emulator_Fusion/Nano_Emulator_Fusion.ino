// Nano Every (ATmega4809)
// 300bps, 6-byte fixed packet echo-back
// RX=D10, TX=D11 (SoftwareSerial)
//
// Wiring:
//   ESP32 TX (GPIO43) -> Nano D10 (RX)
//   ESP32 RX (GPIO44) <- Nano D11 (TX)
//   GND common

#include <Arduino.h>
#include <SoftwareSerial.h>

static const uint8_t PIN_RX = 10;
static const uint8_t PIN_TX = 11;

static const uint32_t BAUD = 300;
static const size_t PKT_LEN = 6;

// 300bps: 1byte(10bit) ≒ 33ms。余裕を見て50ms。
static const uint16_t INTERBYTE_TIMEOUT_MS = 50;

SoftwareSerial tcSerial(PIN_RX, PIN_TX); // RX, TX
static uint8_t buf[PKT_LEN];

static void dumpHex(const char* tag, const uint8_t* p, size_t n) {
  Serial.print(tag);
  for (size_t i = 0; i < n; i++) {
    if (p[i] < 0x10) Serial.print('0');
    Serial.print(p[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  tcSerial.begin(BAUD);

  Serial.println(F("[Nano] Echo-back ready: SoftSerial D10(RX)/D11(TX) 300bps, 6 bytes"));
}

void loop() {
  size_t got = 0;
  uint32_t last = millis();

  while (got < PKT_LEN) {
    if (tcSerial.available()) {
      buf[got++] = (uint8_t)tcSerial.read();
      last = millis();
    } else {
      if ((millis() - last) > INTERBYTE_TIMEOUT_MS) break;
    }
  }

  if (got == 0) return;

  if (got != PKT_LEN) {
    Serial.print(F("[Nano][TIMEOUT] partial="));
    Serial.println(got);
    while (tcSerial.available()) (void)tcSerial.read();
    return;
  }

  dumpHex("[Nano][RX] ", buf, PKT_LEN);
  tcSerial.write(buf, PKT_LEN);
  dumpHex("[Nano][TX] ", buf, PKT_LEN);
}
