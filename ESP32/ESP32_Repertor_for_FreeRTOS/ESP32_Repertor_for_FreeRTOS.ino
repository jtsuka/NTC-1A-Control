// =========================================
// ESP32 リピーター統合スケッチ（RTOSバッファ分離モデル）
// - FreeRTOSベース、安全なProducer/Consumer方式
// - 割り込み禁止なし、キューでデータ受け渡し
// - UART（Pi）<-> BitBang（TC）中継
// - テストパケット送信対応（GPIO8）
// - OLED表示は初期化時のみ使用
// - 日付: 2025.07.08
// =========================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_SSD1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ピン定義
#define PI_UART_TX_PIN 43
#define PI_UART_RX_PIN 44
#define TC_UART_TX_PIN 2
#define TC_UART_RX_PIN 3
#define TEST_PIN        8
#define LED_PIN        21

// 通信設定
#define UART_BAUDRATE 9600
#define BITBANG_BPS   300
#define BIT_DELAY_US  (1000000 / BITBANG_BPS)

// テストパケット
const uint8_t testPacket[6] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};

// キュー
QueueHandle_t txQueue;
#define TX_QUEUE_LENGTH 4
#define PACKET_SIZE 6

// BitBang送信
void bitbangSendByte(uint8_t b) {
  digitalWrite(TC_UART_TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY_US);
  for (int i = 0; i < 8; i++) {
    digitalWrite(TC_UART_TX_PIN, (b >> i) & 1);
    delayMicroseconds(BIT_DELAY_US);
  }
  digitalWrite(TC_UART_TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY_US);
}

void bitbangSendPacket(const uint8_t* packet, int len) {
  for (int i = 0; i < len; i++) {
    bitbangSendByte(packet[i]);
  }
}

// UART受信タスク（Pi -> ESP）
void uartReceiverTask(void* pv) {
  while (1) {
    if (Serial2.available() >= 6) {
      uint8_t buf[6];
      Serial2.readBytes(buf, 6);
      xQueueSend(txQueue, buf, portMAX_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// BitBang送信タスク（ESP -> TC）
void bitbangSenderTask(void* pv) {
  uint8_t packet[6];
  while (1) {
    if (xQueueReceive(txQueue, packet, portMAX_DELAY) == pdTRUE) {
      bitbangSendPacket(packet, 6);
    }
  }
}

// OLED表示（初期化時のみ）
void showOLEDMessage(const char* line1, const char* line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  display.println(line2);
  display.display();
}

// setup
void setup() {
  Serial.begin(115200);
  pinMode(TC_UART_TX_PIN, OUTPUT);
  digitalWrite(TC_UART_TX_PIN, HIGH);
  pinMode(TC_UART_RX_PIN, INPUT);
  pinMode(TEST_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // OLED
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  showOLEDMessage("TC Repeater Mode", "RTOS Safe Model");

  // UART
  Serial2.begin(UART_BAUDRATE, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);

  // キューとタスク起動
  txQueue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(uint8_t) * PACKET_SIZE);
  xTaskCreatePinnedToCore(uartReceiverTask, "UARTRecv", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(bitbangSenderTask, "BitBangSend", 4096, NULL, 1, NULL, 1);
}

// loop
void loop() {
  static unsigned long lastSend = 0;
  static bool ledState = false;
  if (digitalRead(TEST_PIN) == HIGH) {
    if (millis() - lastSend > 1000) {
      uint8_t buf[6];
      memcpy(buf, testPacket, 6);
      xQueueSend(txQueue, buf, 0);         // BitBang送信用
      Serial2.write(buf, 6);               // Piへ送信
      Serial2.flush();
      lastSend = millis();
      digitalWrite(LED_PIN, HIGH);
    }
  } else {
    if (millis() - lastSend > 500) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastSend = millis();
    }
  }
}
