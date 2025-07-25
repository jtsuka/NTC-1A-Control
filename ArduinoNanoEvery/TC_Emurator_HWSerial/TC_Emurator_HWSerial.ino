/**********************************************************************
  TC Emulator – Arduino Nano Every / Seeeduino Nano 用
  - SoftwareSerial (TX: D2, RX: D3)
  - 300bps 通信（6バイト固定長）
  - OLED 表示（I2C SSD1306）
  - printf 未対応環境向けに sprintf + print に修正済み
**********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>

// === ピン定義 ===
#define BB_TX_PIN 2  // D2
#define BB_RX_PIN 3  // D3
#define OLED_ADDR 0x3C
#define PACKET_LEN 6
#define UART_BAUD 300

// === OLED初期化（128x64）===
Adafruit_SSD1306 display(128, 64, &Wire);

// === SoftwareSerial 初期化 ===
SoftwareSerial tcSerial(BB_RX_PIN, BB_TX_PIN);  // RX, TX の順

uint8_t rx_buffer[PACKET_LEN];
uint8_t rx_index = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // ソフトUART開始
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
      // === OLED 表示 ===
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("RECV: ");
      for (int i = 0; i < PACKET_LEN; i++) {
        char buf[5];
        sprintf(buf, "%02X ", rx_buffer[i]);
        display.print(buf);
      }
      display.display();

      // === シリアル出力 ===
      Serial.print("[ECHO] ");
      for (int i = 0; i < PACKET_LEN; i++) {
        char buf[5];
        sprintf(buf, "%02X ", rx_buffer[i]);
        Serial.print(buf);
      }
      Serial.println();

      // === エコーバック送信 ===
      tcSerial.write(rx_buffer, PACKET_LEN);
      rx_index = 0;
    }
  }
}
