/**********************************************************************
  NTC-1A TC Emulator – ESP32-S3版 (Bit-Bang UART 300bps)
  TX: GPIO2  RX: GPIO3  OLED(I2C) 128×64

  - TCエミュレーター中継機からのBitBangで受信したものを中継機にエコーバック
  - OLEDに受信内容と送信内容を16進で表示
  - FreeRTOSベース、スレッドセーフOLED対応
  - 更新日: 2025-06-30
**********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "driver/gpio.h"

#define BB_RX 44
#define BB_TX 43
#define PACKET_SIZE 6
#define BAUD_RATE 300
#define BIT_DELAY_US (1000000UL / BAUD_RATE)
#define HALF_DELAY_US (BIT_DELAY_US / 2 + 20)
#define BYTE_GAP_US 800

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
SemaphoreHandle_t oledMutex;

uint8_t recv_buf[PACKET_SIZE];

// ======== OLED表示関数 =========
void displayHexLine(const char* label, const uint8_t* data, uint8_t row) {
  if (xSemaphoreTake(oledMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    display.setCursor(0, row * 8);
    display.printf("%s:", label);
    for (int i = 0; i < PACKET_SIZE; i++) {
      display.printf(" %02X", data[i]);
    }
    display.display();
    xSemaphoreGive(oledMutex);
  }
}

// ======== BitBang受信 =========
bool receive_packet(uint8_t* buf) {
  unsigned long t_start = millis();

  for (int i = 0; i < PACKET_SIZE; i++) {
    // Start bit検出
    while (digitalRead(BB_RX) == HIGH) {
      if (millis() - t_start > 1000) return false;
    }

    delayMicroseconds(HALF_DELAY_US);
    if (digitalRead(BB_RX) != LOW) return false;

    // データビット読み取り（LSBファースト）
    uint8_t val = 0;
    for (int b = 0; b < 8; b++) {
      delayMicroseconds(BIT_DELAY_US - 2);
      val |= (digitalRead(BB_RX) << b);
      delayMicroseconds(30); // 安定化
    }

    buf[i] = val;
    delayMicroseconds(BIT_DELAY_US + 100); // Stopビット余白
  }

  return true;
}

// ======== BitBang送信 =========
void send_packet(const uint8_t* buf) {
  for (int i = 0; i < PACKET_SIZE; i++) {
    send_byte(buf[i]);
    digitalWrite(BB_TX, HIGH);
    delayMicroseconds(BYTE_GAP_US);
  }
}

void send_byte(uint8_t val) {
  noInterrupts();
  digitalWrite(BB_TX, LOW);  // Start bit
  delayMicroseconds(BIT_DELAY_US);

  for (int i = 0; i < 8; i++) {
    digitalWrite(BB_TX, (val >> i) & 0x01);
    delayMicroseconds(BIT_DELAY_US);
  }

  digitalWrite(BB_TX, HIGH);  // Stop bit
  delayMicroseconds(BIT_DELAY_US * 2);
  interrupts();
}

// ======== メインタスク =========
void task_main(void* arg) {
  while (true) {
    if (receive_packet(recv_buf)) {
      displayHexLine("RECV", recv_buf, 0);
      delayMicroseconds(BIT_DELAY_US * 4); // 安定化
      send_packet(recv_buf);
      displayHexLine("SEND", recv_buf, 2);
    } else {
      delay(10);
    }
  }
}

// ======== 初期化 =========
void setup() {
  pinMode(BB_RX, INPUT_PULLUP);
  pinMode(BB_TX, OUTPUT);
  digitalWrite(BB_TX, HIGH);

  Serial.begin(115200);
  Wire.begin();
  oledMutex = xSemaphoreCreateMutex();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("TC Emu Ready");
  display.display();

  xTaskCreatePinnedToCore(task_main, "TCEmuMain", 4096, NULL, 1, NULL, 1);
}

void loop() {
  delay(100);
}
