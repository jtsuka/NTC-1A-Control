/**********************************************************************
  TC Emulator – Arduino Nano Every 用 (SoftwareSerial 版)
  - TX: D2 (TX_PIN)
  - RX: D3 (RX_PIN)
  - 300bps エコーバック
  - OLED 表示あり（I2C SSD1306）
**********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>

// ピン定義（BitBangと同じ）
#define TX_PIN 2  // D2
#define RX_PIN 3  // D3
#define OLED_ADDR 0x3C
#define PACKET_LEN 6
#define UART_BAUD 300

Adafruit_SSD1306 display(128, 64, &Wire);
SoftwareSerial tcSerial(RX_PIN, TX_PIN);  // RX, TX の順

uint8_t rx_buffer[PACKET_LEN];
uint8_t rx_index = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // ソフトウェアUART 初期化
  tcSerial.begin(UART_BAUD);

  // OLED 初期化
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("OLED init failed"));
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("TC Emulator Start"));
  display.display();
  delay(1000);
}

void loop() {
  while (tcSerial.available()) {
    uint8_t b = tcSerial.read();
    rx_buffer[rx_index++] = b;

    if (rx_index >= PACKET_LEN) {
      // OLED 表示
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("RECV: ");
      for (int i = 0; i < PACKET_LEN; i++) {
        display.printf("%02X ", rx_buffer[i]);
      }
      display.display();

      // シリアルモニタに表示
      Serial.print("[ECHO] ");
      for (int i = 0; i < PACKET_LEN; i++) {
        Serial.printf("%02X ", rx_buffer[i]);
      }
      Serial.println();

      // エコーバック送信
      tcSerial.write(rx_buffer, PACKET_LEN);

      rx_index = 0;  // バッファリセット
    }
  }
}
