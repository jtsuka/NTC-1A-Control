// TC Repeater with BitBang Response Timeout - ESP32-S3 (FreeRTOS)
// ----------------------------------------
// - UART RX/TX (to Pi): GPIO44 / GPIO43
// - BitBang TX/RX (to TC): GPIO2 / GPIO3
// - Baud rate: UART = 9600bps, BitBang = 300bps LSB-first
// - After sending BitBang, wait up to 100ms for response
// - Includes buffer isolation, packet validation, length-safe response
// - FreeRTOS-safe BitBang using taskENTER_CRITICAL
// ----------------------------------------

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define UART_RX_PIN 44
#define UART_TX_PIN 43
#define BITBANG_TX_PIN 2
#define BITBANG_RX_PIN 3

#define UART_BAUD_RATE 9600
#define BITBANG_DELAY_US 3330   // Adjusted for stable 300bps
#define RESPONSE_TIMEOUT_MS 100
#define MAX_PACKET_LEN 32
#define FIXED_PACKET_LEN 6

QueueHandle_t bitbangRxQueue;

// ---------------- UART ------------------
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
  Serial1.write(buf, len);
  Serial1.flush();
  delay(1);  // Ensure completion
}

// ------------- BitBang TX ---------------
void bitBangSendByte(uint8_t b) {
  taskENTER_CRITICAL();
  digitalWrite(BITBANG_TX_PIN, LOW); // Start bit
  delayMicroseconds(BITBANG_DELAY_US);

  for (int i = 0; i < 8; i++) {
    digitalWrite(BITBANG_TX_PIN, (b >> i) & 0x01);
    delayMicroseconds(BITBANG_DELAY_US);
  }

  digitalWrite(BITBANG_TX_PIN, HIGH); // Stop bit
  delayMicroseconds(BITBANG_DELAY_US);
  taskEXIT_CRITICAL();
}

void bitBangSendPacket(const uint8_t *buf, int len) {
  Serial.print("[SEND] ");
  for (int i = 0; i < len; i++) {
    bitBangSendByte(buf[i]);
    Serial.printf("%02X ", buf[i]);
  }
  Serial.println();
}

// ------------- BitBang RX ---------------
int bitBangReceivePacket(uint8_t *buf, int maxLen) {
  int byteCount = 0;
  while (byteCount < maxLen) {
    unsigned long start = millis();
    while (digitalRead(BITBANG_RX_PIN) == HIGH) {
      if ((millis() - start) > 10) return 0;
    }
    delayMicroseconds(BITBANG_DELAY_US * 1.6);

    uint8_t b = 0;
    for (int i = 0; i < 8; i++) {
      b |= (digitalRead(BITBANG_RX_PIN) << i);
      delayMicroseconds(BITBANG_DELAY_US);
    }

    delayMicroseconds(BITBANG_DELAY_US);
    if (digitalRead(BITBANG_RX_PIN) == LOW) {
      Serial.println("[WARN] Stop bit not detected. Skipping byte.");
      continue;
    }

    if (byteCount == 0 && (b == 0x00 || b == 0xFF)) {
      Serial.println("[WARN] Invalid start byte.");
      return 0;
    }

    buf[byteCount++] = b;

    if (byteCount >= FIXED_PACKET_LEN) break;
  }
  return byteCount;
}

// ------------- FreeRTOS Tasks ------------
void TaskBitBangReceive(void *pvParameters) {
  uint8_t rxBuf[MAX_PACKET_LEN];
  while (1) {
    int len = bitBangReceivePacket(rxBuf, MAX_PACKET_LEN);
    if (len == FIXED_PACKET_LEN) {
      uint8_t* copyBuf = (uint8_t*)malloc(len);
      if (copyBuf != nullptr) {
        memcpy(copyBuf, rxBuf, len);
        xQueueSend(bitbangRxQueue, &copyBuf, 0);
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
      Serial.println("[INFO] Packet received from Pi. Sending to TC...");
      bitBangSendPacket(buf, len);

      uint8_t* echoBuf = nullptr;
      if (xQueueReceive(bitbangRxQueue, &echoBuf, pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS)) == pdTRUE) {
        uartSendPacket(echoBuf, FIXED_PACKET_LEN);
        Serial.print("[INFO] Echoed: ");
        for (int i = 0; i < FIXED_PACKET_LEN; i++) Serial.printf("%02X ", echoBuf[i]);
        Serial.println();
        free(echoBuf);
      } else {
        Serial.println("[WARN] No response from TC within timeout.");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// -------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  uartInit();

  pinMode(BITBANG_TX_PIN, OUTPUT);
  digitalWrite(BITBANG_TX_PIN, HIGH);  // Ensure idle HIGH
  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);

  bitbangRxQueue = xQueueCreate(4, sizeof(uint8_t*));

  xTaskCreatePinnedToCore(TaskBitBangReceive, "BitBangRX", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskUartReceive, "UartRX", 4096, NULL, 1, NULL, 1);

  Serial.println("[START] TC Repeater Ready.");
}

void loop() {
  // nothing
}
