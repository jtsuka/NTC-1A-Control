// ================================================
// ESP32-S3 TC Repeater with FreeRTOS - 安定版（固定長パケット）
// - BitBang 300bps TXのみ (GPIO2)
// - UART Pi接続 (GPIO43: TX, GPIO44: RX)
// - テスト送信スイッチ: GPIO8（HIGH時に1回送信）
// ================================================

#include <Arduino.h>

#define PI_UART_TX_PIN 43
#define PI_UART_RX_PIN 44
#define TC_UART_TX_PIN 2
#define TEST_PIN 8
#define LED_PIN 21

#define PKT_LEN 6
#define BIT_DELAY_US 3333  // 約300bps相当

QueueHandle_t piToTcQueue;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

const uint8_t testPacket[PKT_LEN] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};

void bitbangSendPacket(const uint8_t* data, size_t len) {
  portENTER_CRITICAL(&mux);
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = data[i];
    digitalWrite(TC_UART_TX_PIN, LOW); delayMicroseconds(BIT_DELAY_US);
    for (int j = 0; j < 8; ++j) {
      digitalWrite(TC_UART_TX_PIN, b & 0x01); delayMicroseconds(BIT_DELAY_US);
      b >>= 1;
    }
    digitalWrite(TC_UART_TX_PIN, HIGH); delayMicroseconds(BIT_DELAY_US);
  }
  delayMicroseconds(BIT_DELAY_US * 2);  // パケット間ギャップ
  portEXIT_CRITICAL(&mux);
}

void uartToTcTask(void* pv) {
  uint8_t buf[PKT_LEN];
  for (;;) {
    if (Serial2.available() >= PKT_LEN) {
      Serial2.readBytes(buf, PKT_LEN);
      xQueueSend(piToTcQueue, buf, portMAX_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void tcSenderTask(void* pv) {
  uint8_t buf[PKT_LEN];
  for (;;) {
    if (xQueueReceive(piToTcQueue, buf, portMAX_DELAY)) {
      bitbangSendPacket(buf, PKT_LEN);
    }
  }
}

void testPacketTask(void* pv) {
  static bool wasHigh = false;
  static unsigned long lastSend = 0;
  for (;;) {
    bool isHigh = digitalRead(TEST_PIN);
    if (isHigh && (!wasHigh || millis() - lastSend > 2000)) {
      xQueueSend(piToTcQueue, (void*)testPacket, portMAX_DELAY);
      lastSend = millis();
    }
    wasHigh = isHigh;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(TEST_PIN, INPUT);
  pinMode(TC_UART_TX_PIN, OUTPUT);
  digitalWrite(TC_UART_TX_PIN, HIGH);

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);

  piToTcQueue = xQueueCreate(8, PKT_LEN);

  xTaskCreatePinnedToCore(uartToTcTask,   "UART->TC",  2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(tcSenderTask,   "TC SEND",   2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(testPacketTask, "TEST",      2048, NULL, 1, NULL, 1);
}

void loop() {}
