// ESP32-S3 (XIAO) TX-only to Nano (for Nano Serial1 RX verification)
// - Fix SN74LVC16T245 DIR/OE to known states
// - Set Pi-side pins Hi-Z
// - Send 6 bytes every 1s on TC TX pin @300bps (8N1)
// - No receive (we focus on Nano receiving)

#include <Arduino.h>
#include <HardwareSerial.h>
#include "driver/gpio.h"

// ====== ここだけ切替 ======
// 0: 本番ピン (TC: TX=43 RX=44 / Pi: TX=1 RX=2)
// 1: ユニバーサル暫定 (TCとPiを入替: TC: TX=1 RX=2 / Pi: TX=43 RX=44)
#define DEBUG_SWAP_TC_PI 1
// ==========================

// Level shifter control pins (v2.4.6 固定)
static constexpr int PIN_LSHIFT_OE1  = 6;  // LOW enable (Bank1: XIAO->Nano)
static constexpr int PIN_LSHIFT_DIR1 = 5;  // HIGH: A(3.3V)->B(5V)
static constexpr int PIN_LSHIFT_OE2  = 7;  // LOW enable (Bank2: Nano->XIAO)
static constexpr int PIN_LSHIFT_DIR2 = 8;  // LOW:  B(5V)->A(3.3V)

#if DEBUG_SWAP_TC_PI == 0
static constexpr int PIN_TC_TX = 43;        // ESP32 -> Nano RX
static constexpr int PIN_TC_RX = 44;        // Nano TX -> ESP32
static constexpr int PIN_PI_UART_TX = 1;    // ESP32 -> Pi RX
static constexpr int PIN_PI_UART_RX = 2;    // Pi TX -> ESP32
#else
static constexpr int PIN_TC_TX = 1;         // (暫定) ESP32 -> Nano RX
static constexpr int PIN_TC_RX = 2;         // (暫定) Nano TX -> ESP32
static constexpr int PIN_PI_UART_TX = 43;   // (暫定) ESP32 -> Pi RX
static constexpr int PIN_PI_UART_RX = 44;   // (暫定) Pi TX -> ESP32
#endif

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

  // Level shifter: 固定（絶対トグルしない）
  pinMode(PIN_LSHIFT_OE1, OUTPUT);  digitalWrite(PIN_LSHIFT_OE1, LOW);
  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH);
  pinMode(PIN_LSHIFT_OE2, OUTPUT);  digitalWrite(PIN_LSHIFT_OE2, LOW);
  pinMode(PIN_LSHIFT_DIR2, OUTPUT); digitalWrite(PIN_LSHIFT_DIR2, LOW);

  // Pi side: 完全Hi-Z（混線防止）
  setHiZNoPull(PIN_PI_UART_RX);
  setHiZNoPull(PIN_PI_UART_TX);

  // TC UART begin (8N1で配線確認)
  SerialTC.begin(BAUD_TC, SERIAL_8N1, PIN_TC_RX, PIN_TC_TX);

  Serial.println("=== ESP TX-only @300bps ===");
  Serial.printf("Swap mode=%d\n", DEBUG_SWAP_TC_PI);
  Serial.printf("TC UART: TX=%d RX=%d\n", PIN_TC_TX, PIN_TC_RX);
}

void loop() {
  SerialTC.write(pkt, 6);
  SerialTC.flush();

  Serial.print("[ESP TX] ");
  printHex6(pkt);
  Serial.println();

  delay(1000);
}
