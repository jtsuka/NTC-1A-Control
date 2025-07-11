// ================================================
// ESP32-S3 TC Repeater with FreeRTOS (安定送信特化版)
// - BitBang 300bps (GPIO2: TX)
// - UART Pi側 (GPIO43: TX, GPIO44: RX)
// - テスト送信スイッチ GPIO8
// - FreeRTOSベース
// - OLED/受信処理なし（送信確認用）
// ================================================

#include <Arduino.h>

#define PI_UART_TX_PIN 43
#define PI_UART_RX_PIN 44
#define TC_UART_TX_PIN 2
#define TEST_PIN 8
#define LED_PIN 21

#define PKT_LEN 6
#define BIT_DELAY 3333
#define GAP_DELAY 4000

QueueHandle_t piToTcQueue;
portMUX_TYPE serialMux = portMUX_INITIALIZER_UNLOCKED;

// LSB順そのまま送信
void bitbangSendPacket(const uint8_t* data, size_t len) {
  portENTER_CRITICAL(&serialMux);
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = data[i];
    digitalWrite(TC_UART_TX_PIN, LOW); delayMicroseconds(BIT_DELAY);
    for (int j = 0; j < 8; ++j) {
      digitalWrite(TC_UART_TX_PIN, b & 0x01);
      delayMicroseconds(BIT_DELAY);
      b >>= 1;
    }
    digitalWrite(TC_UART_TX_PIN, HIGH); delayMicroseconds(BIT_DELAY);
  }
  delayMicroseconds(GAP_DELAY);
  portEXIT_CRITICAL(&serialMux);
}

void uartToTcTask(void* pv) {
  uint8_t buf[PKT_LEN];
  for (;;) {
    if (Serial2.available() >= PKT_LEN) {
      Serial2.readBytes(buf, PKT_LEN);
      xQueueSend(piToTcQueue, buf, portMAX_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
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

const uint8_t testPacket[PKT_LEN] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};
void testLoopTask(void* pv) {
  static unsigned long lastSend = 0;
  static bool wasHigh = false;
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

  piToTcQueue = xQueueCreate(4, PKT_LEN);

  xTaskCreatePinnedToCore(uartToTcTask,   "UART->TC",  2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(tcSenderTask,   "TC SEND",   2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(testLoopTask,   "TEST",      2048, NULL, 1, NULL, 1);
}

void loop() { }
