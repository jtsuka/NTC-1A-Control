/*
  ESP32-S3 BitBang <-> UART Bridge (FreeRTOS)
  - UART RX (from Pi) -> BitBang TX (to TC)
  - BitBang RX (from TC) -> UART TX (to Pi)
  - OLED + Serial log
  - Grove Shield pin assignment
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "freertos/queue.h"
#include "freertos/task.h"

// === Pin Definitions ===
#define PIN_UART_RX     9  // GPIO9 (from Pi)
#define PIN_UART_TX     8  // GPIO8 (to Pi)
#define PIN_BITBANG_RX  3  // GPIO3 (from TC)
#define PIN_BITBANG_TX  2  // GPIO2 (to TC)
#define PIN_OLED_SDA    4  // Grove I2C
#define PIN_OLED_SCL    5

// === Constants ===
#define BAUD_UART       9600
#define BIT_DELAY_US    3333  // Approx. 300bps
#define MAX_PACKET_SIZE 16

// === Queues ===
QueueHandle_t uartRxQueue;
QueueHandle_t bitbangTxQueue;
QueueHandle_t bitbangRxQueue;
QueueHandle_t uartTxQueue;

// === OLED Setup ===
Adafruit_SSD1306 display(128, 64, &Wire, -1);

void logOLED(const char* title, const char* msg) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(title);
  display.setCursor(0, 20);
  display.println(msg);
  display.display();
}

void logSerial(const char* prefix, const uint8_t* data, size_t len) {
  Serial.printf("%s:", prefix);
  for (size_t i = 0; i < len; i++) {
    Serial.printf(" %02X", data[i]);
  }
  Serial.println();
}

// === BitBang TX ===
void bitBangSendByte(uint8_t b) {
  digitalWrite(PIN_BITBANG_TX, LOW);  // Start bit
  delayMicroseconds(BIT_DELAY_US);
  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_BITBANG_TX, (b >> i) & 0x01);
    delayMicroseconds(BIT_DELAY_US);
  }
  digitalWrite(PIN_BITBANG_TX, HIGH);  // Stop bit
  delayMicroseconds(BIT_DELAY_US);
}

void bitBangSendPacket(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    bitBangSendByte(data[i]);
  }
  logSerial("BitBang TX", data, len);
}

// === BitBang RX ===
uint8_t bitBangReceiveByte() {
  while (digitalRead(PIN_BITBANG_RX));  // Wait for start bit
  delayMicroseconds(BIT_DELAY_US * 1.5);
  uint8_t b = 0;
  for (int i = 0; i < 8; i++) {
    b |= (digitalRead(PIN_BITBANG_RX) << i);
    delayMicroseconds(BIT_DELAY_US);
  }
  delayMicroseconds(BIT_DELAY_US);  // Stop bit
  return b;
}

// === Tasks ===
void TaskUARTReceive(void *pvParameters) {
  uint8_t b;
  while (1) {
    if (Serial2.available()) {
      b = Serial2.read();
      xQueueSend(uartRxQueue, &b, portMAX_DELAY);
    }
  }
}

void TaskBitBangSend(void *pvParameters) {
  uint8_t b;
  while (1) {
    if (xQueueReceive(uartRxQueue, &b, portMAX_DELAY) == pdTRUE) {
      bitBangSendByte(b);
    }
  }
}

void TaskBitBangReceive(void *pvParameters) {
  uint8_t b;
  while (1) {
    b = bitBangReceiveByte();
    xQueueSend(bitbangRxQueue, &b, portMAX_DELAY);
  }
}

void TaskUARTSend(void *pvParameters) {
  uint8_t b;
  while (1) {
    if (xQueueReceive(bitbangRxQueue, &b, portMAX_DELAY) == pdTRUE) {
      Serial2.write(b);
    }
  }
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  Serial2.begin(BAUD_UART, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

  pinMode(PIN_BITBANG_TX, OUTPUT);
  pinMode(PIN_BITBANG_RX, INPUT_PULLUP);
  digitalWrite(PIN_BITBANG_TX, HIGH);

  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("UART <-> BitBang Bridge");
  display.display();

  uartRxQueue = xQueueCreate(32, sizeof(uint8_t));
  bitbangTxQueue = xQueueCreate(32, sizeof(uint8_t));
  bitbangRxQueue = xQueueCreate(32, sizeof(uint8_t));
  uartTxQueue = xQueueCreate(32, sizeof(uint8_t));

  xTaskCreatePinnedToCore(TaskUARTReceive, "UART RX", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskBitBangSend,  "BB Send", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskBitBangReceive,"BB Recv", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskUARTSend,     "UART TX", 2048, NULL, 1, NULL, 0);

  Serial.println("UART <-> BitBang Bridge Ready");
}

void loop() {
  // OLED heartbeat or debug log (optional)
  delay(1000);
}
