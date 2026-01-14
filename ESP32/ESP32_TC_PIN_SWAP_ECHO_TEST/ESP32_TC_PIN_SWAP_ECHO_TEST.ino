/*
 * TTL-232R-3V3（USBシリアル変換ケーブル）Pi<->ESP32S3 テスト用
*/

#include <Arduino.h>
#include <HardwareSerial.h>
#include <tc_packet.hpp> // せっかく作った共通ライブラリを使用

// PCB/スケッチ定義に準拠 
static constexpr int PIN_PI_TX = 43; // ESP32のTX（PiのRXへ）
static constexpr int PIN_PI_RX = 44; // ESP32のRX（PiのTXから）
static constexpr uint32_t PI_BAUD = 9600;

HardwareSerial SerialPi(1);

void setup() {
  Serial.begin(115200); // USBデバッグ用
  delay(500);
  
  // Pi側UARTの開始
  SerialPi.begin(PI_BAUD, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);
  
  Serial.println("--- 通信テスト開始 (Byte Echo Mode) ---");
  SerialPi.println("READY: ESP32-S3 is waiting for bytes...");
}

void loop() {
  // Pi側からデータが来たら、そのまま送り返す
  while (SerialPi.available() > 0) {
    uint8_t c = SerialPi.read();
    
    // 1. Piへエコーバック
    SerialPi.write(c); 
    
    // 2. USBシリアル（PC）にも表示して確認
    Serial.print("Received from Pi: ");
    Serial.print((char)c);
    Serial.print(" [0x"); Serial.print(c, HEX); Serial.println("]");
  }
}