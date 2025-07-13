#include <Arduino.h>
#include "driver/uart.h"
#include "freertos/queue.h"

#define UART_NUM        UART_NUM_1
#define UART_TX_PIN     43
#define UART_RX_PIN     44
#define BB_TX_PIN       2
#define BB_RX_PIN       3

#define BAUD_UART       9600
#define BIT_DELAY_US    3333
#define PACKET_SIZE     6

QueueHandle_t txQueue;
QueueHandle_t rxQueue;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// === BitBang送信 ===
void bitBangSendByte(uint8_t b) {
  taskENTER_CRITICAL(&mux);
  digitalWrite(BB_TX_PIN, LOW);  // Start
  delayMicroseconds(BIT_DELAY_US);
  for (int i = 0; i < 8; i++) {
    digitalWrite(BB_TX_PIN, (b >> i) & 1);
    delayMicroseconds(BIT_DELAY_US);
  }
  digitalWrite(BB_TX_PIN, HIGH); // Stop
  delayMicroseconds(BIT_DELAY_US);
  taskEXIT_CRITICAL(&mux);
}

void bitBangSendPacket(const uint8_t* data) {
  Serial.print("BB TX: ");
  for (int i = 0; i < PACKET_SIZE; i++) {
    Serial.printf("%02X ", data[i]);
    bitBangSendByte(data[i]);
  }
  Serial.println();
}

// === BitBang受信 ===
bool bitBangReceivePacket(uint8_t* data) {
  for (int i = 0; i < PACKET_SIZE; i++) {
    // スタートビット待機（ブロッキング回避）
    while (digitalRead(BB_RX_PIN) == HIGH) {
      vTaskDelay(pdMS_TO_TICKS(1));  // ← CPU占有を防ぐ
    }

    delayMicroseconds(BIT_DELAY_US * 1.5);

    uint8_t b = 0;
    for (int j = 0; j < 8; j++) {
      b |= (digitalRead(BB_RX_PIN) << j);
      delayMicroseconds(BIT_DELAY_US);
    }

    delayMicroseconds(BIT_DELAY_US);  // Stop bit
    data[i] = b;

    taskYIELD();  // ← 他タスクへ一時的にCPUを譲る
  }
  return true;
}

// === タスク: UART受信 → BitBang送信 ===
void TaskUartToBitBang(void* pv) {
  uint8_t buf[PACKET_SIZE];
  while (1) {
    int len = uart_read_bytes(UART_NUM, buf, PACKET_SIZE, pdMS_TO_TICKS(100));
    if (len == PACKET_SIZE) {
      xQueueSend(txQueue, buf, 0);
      Serial.print("UART RX: ");
      for (int i = 0; i < PACKET_SIZE; i++) Serial.printf("%02X ", buf[i]);
      Serial.println();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// === タスク: BitBang受信 → UART送信 ===
void TaskBitBangToUart(void* pv) {
  uint8_t buf[PACKET_SIZE];
  while (1) {
    if (bitBangReceivePacket(buf)) {
      Serial.print("BB RX: ");
      for (int i = 0; i < PACKET_SIZE; i++) Serial.printf("%02X ", buf[i]);
      Serial.println();
      uart_write_bytes(UART_NUM, buf, PACKET_SIZE);
    }
    vTaskDelay(pdMS_TO_TICKS(5));  // ← 安全マージン
  }
}

// === セットアップ ===
void setup() {
  Serial.begin(115200);
  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);
  pinMode(BB_RX_PIN, INPUT_PULLUP);

  uart_config_t config = {
    .baud_rate = BAUD_UART,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM, &config);
  uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM, 256, 0, 0, NULL, 0);

  txQueue = xQueueCreate(4, PACKET_SIZE);
  rxQueue = xQueueCreate(4, PACKET_SIZE);

  xTaskCreatePinnedToCore(TaskUartToBitBang, "U2B", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskBitBangToUart, "B2U", 2048, NULL, 1, NULL, 0);

  Serial.println("Bridge Ready");
}

void loop() {
  delay(10);
}
