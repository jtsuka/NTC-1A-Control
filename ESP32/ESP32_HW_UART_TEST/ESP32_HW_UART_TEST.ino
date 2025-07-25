#include <Arduino.h>

// UART定義
#define UART_PI_RX    44
#define UART_PI_TX    43
#define UART_TC_RX     3
#define UART_TC_TX     2

#define PACKET_LEN     6
#define TIMEOUT_MS    100

HardwareSerial SerialPi(1);  // UART1 ←→ Pi
HardwareSerial SerialTC(2);  // UART2 ←→ TC

// LVC16T245 制御ピン定義
#define PIN_DIR1       6   // TX方向：ESP32→TC → HIGH固定
#define PIN_DIR2       8   // RX方向：TC→ESP32 → LOW固定
#define PIN_OE1        5   // TX出力有効 → 常時 LOW
#define PIN_OE2        7   // RX出力有効 → 常時 LOW

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("[BOOT] TC Repeater (OE=常時LOW)");

  // UART初期化
  SerialPi.begin(9600, SERIAL_8N1, UART_PI_RX, UART_PI_TX);
  SerialTC.begin(300, SERIAL_8N1, UART_TC_RX, UART_TC_TX);

  // DIRピン固定（TX方向: HIGH, RX方向: LOW）
  pinMode(PIN_DIR1, OUTPUT); digitalWrite(PIN_DIR1, HIGH);
  pinMode(PIN_DIR2, OUTPUT); digitalWrite(PIN_DIR2, LOW);

  // OEピン常時LOW（出力有効化）
  pinMode(PIN_OE1, OUTPUT); digitalWrite(PIN_OE1, LOW);
  pinMode(PIN_OE2, OUTPUT); digitalWrite(PIN_OE2, LOW);
}

void loop() {
  static uint8_t piBuf[PACKET_LEN];
  static uint8_t tcBuf[PACKET_LEN];

  // Pi → TC
  if (SerialPi.available() >= PACKET_LEN) {
    SerialPi.readBytes(piBuf, PACKET_LEN);

    Serial.print("[Pi → TC] ");
    for (int i = 0; i < PACKET_LEN; ++i) Serial.printf("%02X ", piBuf[i]);
    Serial.println();

    SerialTC.write(piBuf, PACKET_LEN);
    SerialTC.flush();
  }

  // TC → Pi 応答
  if (SerialTC.available()) {
    unsigned long start = millis();
    int received = 0;

    while (received < PACKET_LEN && millis() - start < TIMEOUT_MS) {
      if (SerialTC.available()) {
        tcBuf[received++] = SerialTC.read();
      }
    }

    if (received == PACKET_LEN) {
      Serial.print("[TC → Pi] ");
      for (int i = 0; i < PACKET_LEN; ++i) Serial.printf("%02X ", tcBuf[i]);
      Serial.println();

      SerialPi.write(tcBuf, PACKET_LEN);
      SerialPi.flush();
    } else {
      Serial.println("[WARN] TC応答なし or 不完全");
    }
  }
}
