// TC Repeater with BitBang Response Timeout - ESP32-S3 (修正版)
// ----------------------------------------
// - UART RX/TX (to Pi): GPIO44 / GPIO43
// - BitBang TX/RX (to TC): GPIO2 / GPIO3
// - Baud rate: UART = 9600bps, BitBang = 300bps
// - Wait up to 100ms for TC response, then reply to Pi
// - Safe with FreeRTOS & taskENTER_CRITICAL
// - 2025.07.14 3th Try Version
// ----------------------------------------

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define UART_RX_PIN        44
#define UART_TX_PIN        43
#define BITBANG_TX_PIN     2
#define BITBANG_RX_PIN     3

#define UART_BAUD_RATE     9600
#define BITBANG_DELAY_US   3340
#define RESPONSE_TIMEOUT_MS 200
#define MAX_PACKET_LEN     32
#define FIXED_PACKET_LEN   6

QueueHandle_t bitbangRxQueue;
portMUX_TYPE bitbangMux = portMUX_INITIALIZER_UNLOCKED;

void uartInit() {
  Serial1.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
}

int uartReceivePacket(uint8_t *buf) {
  int len = 0;
  unsigned long start = millis();
  while ((millis() - start) < 100 && len < MAX_PACKET_LEN) {
    if (Serial1.available()) {
      buf[len++] = Serial1.read();
    }
  }
  return len;
}

void uartSendPacket(const uint8_t *buf, int len) {
  Serial.print("[SEND Pi] ");
  for (int i = 0; i < len; i++) Serial.printf("%02X ", buf[i]);
  Serial.println();
  Serial1.write(buf, len);
  Serial1.flush();
  delay(1);
}

void bitBangSendByte(uint8_t b) {
  taskENTER_CRITICAL(&bitbangMux);
  digitalWrite(BITBANG_TX_PIN, HIGH);   // Start HIGH
  delayMicroseconds(BITBANG_DELAY_US);
  for (int i = 0; i < 8; i++) {
    digitalWrite(BITBANG_TX_PIN, !(b >> i & 1));   // データ反転
//    digitalWrite(BITBANG_TX_PIN, (b >> i) & 0x01);
    delayMicroseconds(BITBANG_DELAY_US);
  }
  digitalWrite(BITBANG_TX_PIN, LOW);    // Stop LOW
  delayMicroseconds(BITBANG_DELAY_US);
  taskEXIT_CRITICAL(&bitbangMux);
}

void bitBangSendPacket(const uint8_t *buf, int len) {
  Serial.print("[SEND TC] ");
  for (int i = 0; i < len; i++) {
    bitBangSendByte(buf[i]);
    Serial.printf("%02X ", buf[i]);
  }
  Serial.println();
}

int bitBangReceivePacket(uint8_t *buf, int maxLen) {
  int byteCount = 0;
  while (byteCount < maxLen) {
    unsigned long start = millis();
    while (digitalRead(BITBANG_RX_PIN) == LOW) {
      if ((millis() - start) > 30) return 0;  // ← 延長
    }
    delayMicroseconds(BITBANG_DELAY_US * 1.7); // サンプリング点
    uint8_t b = 0;
    for (int i = 0; i < 8; i++) {
      b |= (digitalRead(BITBANG_RX_PIN) << i); // 反転して取り込む
      delayMicroseconds(BITBANG_DELAY_US);
    }
//    delayMicroseconds(BITBANG_DELAY_US);
//    if (digitalRead(BITBANG_RX_PIN) == LOW) {
//      Serial.println("[WARN] Stop bit error.");
//      continue;
//    }
    if (byteCount == 0 && (b == 0x00 || b == 0xFF)) {
      Serial.println("[WARN] Invalid start byte.");
      return 0;
    }
    buf[byteCount++] = b;
    if (byteCount >= FIXED_PACKET_LEN) break;
  }
  return byteCount;
}

void TaskBitBangReceive(void *pvParameters) {
  uint8_t rxBuf[MAX_PACKET_LEN];
  while (1) {
    int len = bitBangReceivePacket(rxBuf, MAX_PACKET_LEN);
    if (len == FIXED_PACKET_LEN) {
      uint8_t* copyBuf = (uint8_t*)malloc(len);
      if (copyBuf != nullptr) {
        memcpy(copyBuf, rxBuf, len);
        if (xQueueSend(bitbangRxQueue, &copyBuf, 0) != pdTRUE) {
          free(copyBuf);  // キューがいっぱい → メモリ解放
          Serial.println("[ERROR] xQueueSend failed. Buffer discarded.");
        }
      } else {
        Serial.println("[ERROR] malloc failed in BitBangReceive");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void TaskUartReceive(void *pvParameters) {
  uint8_t buf[MAX_PACKET_LEN];
  while (1) {
    int len = uartReceivePacket(buf);
    if (len > 0) {
      Serial.println("[INFO] Pi -> TC へ送信開始");
      bitBangSendPacket(buf, len);

      // TCからの応答を受信（TaskBitBangReceiveが malloc したもの）
      uint8_t* echoBuf = nullptr;
      if (xQueueReceive(bitbangRxQueue, &echoBuf, pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS)) == pdTRUE) {
        uartSendPacket(echoBuf, FIXED_PACKET_LEN);
        Serial.print("[RECV TC] ");
        for (int i = 0; i < FIXED_PACKET_LEN; i++) {
          Serial.printf("%02X ", echoBuf[i]);
        }
        Serial.println();
        if ( echoBuf != nullptr ) {
          free(echoBuf);  // ← malloc されたメモリの解放
        }
      } else {
        Serial.println("[WARN] TC応答なし (timeout)");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}


void setup() {
  Serial.begin(115200);
  uartInit();

  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);
  Serial.printf("[DBG] idle-level=%d\n", digitalRead(BITBANG_RX_PIN));

  pinMode(BITBANG_TX_PIN, OUTPUT);
  digitalWrite(BITBANG_TX_PIN, HIGH);  // idle HIGH
  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);  // ★プルアップを戻す
  gpio_set_pull_mode((gpio_num_t)BITBANG_RX_PIN,
                   GPIO_PULLUP_ONLY);   // ★強制 47k → 10k 相当へ

//  pinMode(BITBANG_RX_PIN, INPUT);      // ← PULLUPを削除

  bitbangRxQueue = xQueueCreate(4, sizeof(uint8_t*));
  xTaskCreatePinnedToCore(TaskBitBangReceive, "BitBangRX", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskUartReceive, "UartRX", 4096, NULL, 1, NULL, 1);

  Serial.println("[START] TC Repeater Ready.");
  Serial.print("[DEBUG] RXB idle level = ");
  Serial.println(digitalRead(BITBANG_RX_PIN));   // 0 なら LOW、1 なら HIGH
}

void loop() {
  // loopなし
}
