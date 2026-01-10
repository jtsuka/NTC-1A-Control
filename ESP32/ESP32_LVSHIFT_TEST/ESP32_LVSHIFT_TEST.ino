// ESP32-S3 (XIAO) TestC Loopback (v2.4.6 pin-fixed)
// - Fix SN74LVC16T245 DIR/OE
// - Use UART on TC pins: RX=GPIO44, TX=GPIO43 @300bps (8N1 for wiring test)
// - Send 6 bytes periodically, read back 6 bytes echo from Nano
// - Pi pins set Hi-Z to avoid interference

#include <Arduino.h>
#include <HardwareSerial.h>
#include "driver/gpio.h"

// Level shifter control pins (FIXED by v2.4.6)
static constexpr int PIN_LSHIFT_OE1  = 6;  // LOW enable (Bank1: XIAO->Nano)
static constexpr int PIN_LSHIFT_DIR1 = 5;  // HIGH: A(3.3V)->B(5V)
static constexpr int PIN_LSHIFT_OE2  = 7;  // LOW enable (Bank2: Nano->XIAO)
static constexpr int PIN_LSHIFT_DIR2 = 8;  // LOW:  B(5V)->A(3.3V)

// TC-side UART pins (FIXED by v2.4.6)
static constexpr int PIN_TC_TX = 1; // ESP32 -> Nano RX
static constexpr int PIN_TC_RX = 2; // Nano TX -> ESP32

// Pi-side UART pins (FIXED by v2.4.6) — isolate during test
static constexpr int PIN_PI_UART_RX = 44; // Pi TX -> ESP RX
static constexpr int PIN_PI_UART_TX = 43; // ESP TX -> Pi RX

static constexpr uint32_t BAUD_USB = 115200;
static constexpr uint32_t BAUD_TC  = 300;

HardwareSerial SerialTC(2);

static const uint8_t pkt[6] = { 0xA5, 0x5A, 0x08, 0xF7, 0xFE, 0x0D };

static void setHiZNoPull(int pin) {
  gpio_reset_pin((gpio_num_t)pin);
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
  gpio_pullup_dis((gpio_num_t)pin);
  gpio_pulldown_dis((gpio_num_t)pin);
}

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

  // Fix level shifter states
  pinMode(PIN_LSHIFT_OE1, OUTPUT);  digitalWrite(PIN_LSHIFT_OE1, LOW);
  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH);
  pinMode(PIN_LSHIFT_OE2, OUTPUT);  digitalWrite(PIN_LSHIFT_OE2, LOW);
  pinMode(PIN_LSHIFT_DIR2, OUTPUT); digitalWrite(PIN_LSHIFT_DIR2, LOW);

  // Isolate Pi-side pins during this test
  setHiZNoPull(PIN_PI_UART_RX);
  setHiZNoPull(PIN_PI_UART_TX);

  // Start TC UART (wiring test = 8N1)
  SerialTC.begin(BAUD_TC, SERIAL_8N1, PIN_TC_RX, PIN_TC_TX);

  Serial.println("=== ESP32 TestC Loopback (v2.4.6 pins) ===");
  Serial.printf("LSHIFT OE1=%d(L) DIR1=%d(H) OE2=%d(L) DIR2=%d(L)\n",
                PIN_LSHIFT_OE1, PIN_LSHIFT_DIR1, PIN_LSHIFT_OE2, PIN_LSHIFT_DIR2);
  Serial.printf("TC UART: RX=%d TX=%d @%lu bps\n", PIN_TC_RX, PIN_TC_TX, BAUD_TC);
  Serial.printf("Pi pins Hi-Z: RX=%d TX=%d\n", PIN_PI_UART_RX, PIN_PI_UART_TX);
}

void loop() {
  // Send
  SerialTC.write(pkt, 6);
  SerialTC.flush();

  Serial.print("[ESP->Nano TX] ");
  printHex6(pkt);
  Serial.println();

  // Read back 6 bytes (timeout)
  uint8_t rx[6];
  int got = 0;
  const uint32_t t0 = millis();
  while (got < 6 && (millis() - t0) < 800) {  // 300bps往復を余裕で待つ
    if (SerialTC.available()) {
      rx[got++] = (uint8_t)SerialTC.read();
    }
  }

  if (got == 6) {
    Serial.print("[Nano->ESP RX] ");
    printHex6(rx);
    Serial.println();
  } else {
    Serial.print("[TIMEOUT] got=");
    Serial.println(got);
    while (SerialTC.available()) (void)SerialTC.read();
  }

  delay(1000);
}
