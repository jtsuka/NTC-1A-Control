// ESP32-S3  (Pi <-> ESP echo test)
// Pi side UART: ESP32 TX=GPIO1 (to Pi RX), ESP32 RX=GPIO2 (from Pi TX)
// 9600bps, 6-byte fixed packet echo back
//
// Wiring (production):
//   ESP GPIO1 (TX) -> Pi RXD0 (GPIO15 / pin10)
//   ESP GPIO2 (RX) <- Pi TXD0 (GPIO14 / pin8)
//   GND common
//
// Note: On Raspberry Pi, enable UART and disable serial console if needed.
// ESP32-S3  (Pi <-> ESP echo test)
// Pi side UART: ESP32 TX=GPIO1 (to Pi RX), ESP32 RX=GPIO2 (from Pi TX)
// 9600bps, 6-byte fixed packet echo back
//
// Wiring (production):
//   ESP GPIO1 (TX) -> Pi RXD0 (GPIO15 / pin10)
//   ESP GPIO2 (RX) <- Pi TXD0 (GPIO14 / pin8)
//   GND common
//
// Note: On Raspberry Pi, enable UART and disable serial console if needed.

#include <Arduino.h>

static const uint32_t BAUD = 9600;
static const size_t   PKT_LEN = 6;

static const int PI_TX_PIN = 1;  // ESP32 -> Pi RX
static const int PI_RX_PIN = 2;  // Pi TX -> ESP32

static const uint16_t INTERBYTE_TIMEOUT_MS = 30; // generous for 9600bps

// Use UART1 for Pi side (safe / portable in Arduino-ESP32)
HardwareSerial PI(1);

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
  // USB-Serial log
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("[ESP32] Pi <-> ESP Echo Test (9600bps, 6 bytes)"));

  // Pi UART (UART1) with reassigned pins
  PI.begin(BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);

  while (PI.available()) (void)PI.read();
}

void loop() {
  size_t got = 0;
  uint32_t last = millis();

  // Receive exactly 6 bytes with inter-byte timeout
  while (got < PKT_LEN) {
    if (PI.available()) {
      buf[got++] = (uint8_t)PI.read();
      last = millis();
    } else {
      if ((millis() - last) > INTERBYTE_TIMEOUT_MS) break;
      delay(1);
    }
  }

  if (got == 0) return;

  if (got != PKT_LEN) {
    Serial.print(F("[ESP32][TIMEOUT] partial="));
    Serial.println(got);
    while (PI.available()) (void)PI.read();
    return;
  }

  dumpHex("[ESP32][RX from Pi] ", buf, PKT_LEN);

  // Echo back exactly same 6 bytes
  PI.write(buf, PKT_LEN);
  PI.flush();

  dumpHex("[ESP32][TX to Pi]   ", buf, PKT_LEN);
}

#include <Arduino.h>

static const uint32_t BAUD = 9600;
static const size_t PKT_LEN = 6;

static const int PI_TX_PIN = 1;  // ESP32 -> Pi RX
static const int PI_RX_PIN = 2;  // Pi TX -> ESP32

static const uint16_t INTERBYTE_TIMEOUT_MS = 30; // 9600bps: ~1ms/byte, 30ms is generous

HardwareSerial &PI = Serial0; // On ESP32 Arduino, Serial is usually UART0. We'll use Serial0 explicitly.

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
  // USB-Serial log
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("[ESP32] Pi <-> ESP Echo Test (9600bps, 6 bytes)"));

  // Pi UART (use UART0 with reassigned pins)
  PI.begin(BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);

  while (PI.available()) (void)PI.read();
}

void loop() {
  size_t got = 0;
  uint32_t last = millis();

  while (got < PKT_LEN) {
    if (PI.available()) {
      buf[got++] = (uint8_t)PI.read();
      last = millis();
    } else {
      if ((millis() - last) > INTERBYTE_TIMEOUT_MS) break;
      delay(1);
    }
  }

  if (got == 0) return;

  if (got != PKT_LEN) {
    Serial.print(F("[ESP32][TIMEOUT] partial="));
    Serial.println(got);
    while (PI.available()) (void)PI.read();
    return;
  }

  dumpHex("[ESP32][RX from Pi] ", buf, PKT_LEN);

  // Echo back exactly same 6 bytes
  PI.write(buf, PKT_LEN);
  PI.flush();

  dumpHex("[ESP32][TX to Pi]   ", buf, PKT_LEN);
}
