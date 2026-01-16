// ESP32S3 <-> UART 信号線チェック
// 9600bps 8N1
// Pi + USB-TTL <-> ESP32S3

#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial PiUart(1);

// GPIO43/44 使用（今の検証用）
static const int PIN_PI_RX = 43;  // ESP RX（USB-TTL TX）
static const int PIN_PI_TX = 44;  // ESP TX（USB-TTL RX）

void setup() {
  Serial.begin(115200);   // USBモニタ
  delay(200);             // USB安定待ち

  PiUart.begin(9600, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);

  Serial.printf("UART Echo Ready. RX=%d TX=%d\n",
                PIN_PI_RX, PIN_PI_TX);
}

void loop() {
  if (PiUart.available()) {
    int c = PiUart.read();
    PiUart.write(c);      // Piへエコー
    Serial.write(c);      // USBへ表示
  }
}
