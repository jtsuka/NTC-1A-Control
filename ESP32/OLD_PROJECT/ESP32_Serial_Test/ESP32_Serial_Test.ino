#include <HardwareSerial.h>

#define PI_UART_TX_PIN 43  // ESP32から見て TX
#define PI_UART_RX_PIN 44  // 今回は使用しないが必要なので指定
#define UART_BAUD_PI   9600

HardwareSerial SerialPI(2);

const uint8_t test_packet[6] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x15 };

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("ESP32 -> Pi テスト送信開始");

  // UART2 初期化（GPIO43=TX, GPIO44=RX）
  SerialPI.begin(UART_BAUD_PI, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
}

#if 1
void loop() {
  SerialPI.write(test_packet, sizeof(test_packet));
  Serial.print("Sent: ");
  for (int i = 0; i < 6; i++) {
    if (test_packet[i] < 0x10) Serial.print("0");
    Serial.print(test_packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  delay(1000); // 1秒間隔
}
#else
void loop() {
  while (true) {
    // 何もせず停止
    delay(1000);
  }
}
#endif
