// ESP32S3 <-> UART 信号線チェック
// 9600bps 8N
// for Pi+USBシリアル<-> ESP32S3 GPIO 43, 44

#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial PiUart(1);

// ★この2行だけ、あなたの配線に合わせて変える★
// SWAP配線（あなたのスケッチの注記どおり）
static const int PIN_PI_RX = 43;  // ESPが受ける（ここへケーブルTXが来る）
static const int PIN_PI_TX = 44;  // ESPが出す（ここからケーブルRXへ行く）

void setup() {
  Serial.begin(115200); // USBモニタ
  PiUart.begin(9600, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);

  Serial.printf("UART Echo Ready. RX=%d TX=%d\n", PIN_PI_RX, PIN_PI_TX);
  PiUart.println("ESP32 ready");
}

void loop() {
  while (PiUart.available()) {
    int c = PiUart.read();
    PiUart.write(c);   // Piへエコー
    Serial.write(c);   // USBにも表示（確認用）
  }
}
