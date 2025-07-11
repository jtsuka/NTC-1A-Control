/**********************************************************************
  ESP32-S3 TC Repeater (Stable Ver)
  - UART (9600bps) GPIO43:TX, GPIO44:RX ⇔ BitBang 300bps GPIO2:TX, GPIO3:RX
  - 固定長 6バイトパケットのみを中継
  - FreeRTOS構成、キュー管理
  - テスト送信（GPIO8=HIGH）で固定パケット送出
  - OLEDやMSB/LSB切替、可変長処理は除外（安定性優先）
**********************************************************************/

#include <Arduino.h>

#define UART_TX_PIN 43
#define UART_RX_PIN 44
#define BB_TX_PIN   2
#define BB_RX_PIN   3
#define TEST_PIN    8
#define LED_PIN     21

#define BIT_DELAY_US 3333
#define PKT_LEN 6

QueueHandle_t uartToBBQueue;
QueueHandle_t bbToUartQueue;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

const uint8_t testPacket[PKT_LEN] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};

void bitbangSend(const uint8_t* data) {
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < PKT_LEN; i++) {
    uint8_t b = data[i];
    digitalWrite(BB_TX_PIN, LOW);
    delayMicroseconds(BIT_DELAY_US);
    for (int j = 0; j < 8; j++) {
      digitalWrite(BB_TX_PIN, (b >> j) & 0x01);
      delayMicroseconds(BIT_DELAY_US);
    }
    digitalWrite(BB_TX_PIN, HIGH);
    delayMicroseconds(BIT_DELAY_US);
    delayMicroseconds(800);  // inter-byte gap
  }
  delayMicroseconds(3000);
  portEXIT_CRITICAL(&mux);
}

bool bitbangReceive(uint8_t* out) {
  if (digitalRead(BB_RX_PIN) == LOW) {
    noInterrupts();
    delayMicroseconds(BIT_DELAY_US / 2);
    uint8_t b = 0;
    for (int i = 0; i < 8; i++) {
      delayMicroseconds(BIT_DELAY_US);
      b |= (digitalRead(BB_RX_PIN) << i);
    }
    interrupts();
    *out = b;
    delayMicroseconds(BIT_DELAY_US);
    return true;
  }
  return false;
}

void uartToBBTask(void* pv) {
  uint8_t buf[PKT_LEN];
  for (;;) {
    if (Serial2.available() >= PKT_LEN) {
      Serial2.readBytes(buf, PKT_LEN);
      xQueueSend(uartToBBQueue, buf, portMAX_DELAY);
    }
    vTaskDelay(1);
  }
}

void bbToUartTask(void* pv) {
  uint8_t pkt[PKT_LEN];
  for (;;) {
    if (bitbangReceive(&pkt[0])) {
      for (int i = 1; i < PKT_LEN; i++) {
        while (!bitbangReceive(&pkt[i])) {
          vTaskDelay(1);
        }
      }
      xQueueSend(bbToUartQueue, pkt, portMAX_DELAY);
    } else {
      vTaskDelay(1);
    }
  }
}

void bbSenderTask(void* pv) {
  uint8_t buf[PKT_LEN];
  for (;;) {
    if (xQueueReceive(uartToBBQueue, buf, portMAX_DELAY)) {
      bitbangSend(buf);
    }
  }
}

void uartSenderTask(void* pv) {
  uint8_t buf[PKT_LEN];
  for (;;) {
    if (xQueueReceive(bbToUartQueue, buf, portMAX_DELAY)) {
      delayMicroseconds(4000);
      Serial2.write(buf, PKT_LEN);
      Serial2.flush();
    }
  }
}

void testPacketTask(void* pv) {
  for (;;) {
    if (digitalRead(TEST_PIN) == HIGH) {
      xQueueSend(uartToBBQueue, (void*)testPacket, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  pinMode(BB_TX_PIN, OUTPUT);
  pinMode(BB_RX_PIN, INPUT_PULLUP);
  digitalWrite(BB_TX_PIN, HIGH);
  pinMode(TEST_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  uartToBBQueue = xQueueCreate(4, PKT_LEN);
  bbToUartQueue = xQueueCreate(4, PKT_LEN);

  xTaskCreatePinnedToCore(uartToBBTask,   "UART->BB", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(bbSenderTask,   "BB SEND",  2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(bbToUartTask,   "BB->UART", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(uartSenderTask, "UART SEND",2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(testPacketTask, "TEST",     2048, NULL, 1, NULL, 1);
}

void loop() {}
