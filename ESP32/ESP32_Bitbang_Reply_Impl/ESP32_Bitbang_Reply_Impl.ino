// UART→BitBang Repeater (ESP32-S3 + FreeRTOS)
// - UART: GPIO44 (RX), GPIO43 (TX), 9600bps
// - BitBang TX: GPIO2 (300bps, LSB First)
// - OLED: GPIO4(SDA), GPIO5(SCL) ※未使用可
// - テストスイッチ: GPIO8 (HIGHでテスト送信)

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define UART_RX_PIN 44
#define UART_TX_PIN 43
#define BB_TX_PIN 2
#define TEST_PIN 8

#define BAUD_UART 9600
#define BAUD_BITBANG 300
#define BIT_DURATION_US (1000000 / BAUD_BITBANG)

#define MAX_PACKET_SIZE 6

QueueHandle_t bitbangTxQueue;

void bitBangSendByte(uint8_t b) {
  noInterrupts();
  digitalWrite(BB_TX_PIN, LOW);  // Start bit
  delayMicroseconds(BIT_DURATION_US);

  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(BB_TX_PIN, (b >> i) & 0x01);
    delayMicroseconds(BIT_DURATION_US);
  }

  digitalWrite(BB_TX_PIN, HIGH); // Stop bit
  delayMicroseconds(BIT_DURATION_US);
  interrupts();
}

void bitBangSendPacket(uint8_t *packet, size_t len) {
  for (size_t i = 0; i < len; i++) {
    bitBangSendByte(packet[i]);
  }
}

void TaskUartReceive(void *pvParameters) {
  uint8_t buffer[MAX_PACKET_SIZE];
  size_t len = 0;

  while (1) {
    if (Serial2.available()) {
      buffer[len++] = Serial2.read();
      delay(3);  // 連続読み取りのための簡易ウェイト
      if (len >= MAX_PACKET_SIZE) {
        Serial.print("UART RX: ");
        for (int i = 0; i < len; i++) Serial.printf("%02X ", buffer[i]);
        Serial.println();

        xQueueSend(bitbangTxQueue, buffer, 0);
        len = 0;
      }
    }
    vTaskDelay(1);
  }
}

void TaskBitBangSend(void *pvParameters) {
  uint8_t buffer[MAX_PACKET_SIZE];
  while (1) {
    if (xQueueReceive(bitbangTxQueue, buffer, portMAX_DELAY) == pdTRUE) {
      Serial.print("BitBang TX: ");
      for (int i = 0; i < MAX_PACKET_SIZE; i++) Serial.printf("%02X ", buffer[i]);
      Serial.println();
      bitBangSendPacket(buffer, MAX_PACKET_SIZE);
    }
  }
}

void TaskTestSender(void *pvParameters) {
  const uint8_t testPacket[MAX_PACKET_SIZE] = {0x80, 0x60, 0xA0, 0x00, 0x00, 0x30};
  while (1) {
    if (digitalRead(TEST_PIN) == HIGH) {
      Serial.println("[TEST] Sending test packet...");
      xQueueSend(bitbangTxQueue, (void*)testPacket, 0);
      delay(500);
    }
    vTaskDelay(100);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(BAUD_UART, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);  // Idle High

  pinMode(TEST_PIN, INPUT_PULLDOWN);

  bitbangTxQueue = xQueueCreate(8, MAX_PACKET_SIZE);

  xTaskCreatePinnedToCore(TaskUartReceive, "UART_RX", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskBitBangSend, "BB_TX", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskTestSender,  "TEST",   2048, NULL, 1, NULL, 0);

  Serial.println("UART → BitBang Repeater Ready");
}

void loop() {
  // OLEDは無しの場合、このloopは使われない
}
