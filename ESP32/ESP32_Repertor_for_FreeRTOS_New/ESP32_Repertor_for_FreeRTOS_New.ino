// ================================================
// ESP32-S3 TC Repeater with FreeRTOS + 割り込み保護版
// - BitBang 300bps (GPIO2: TX, GPIO3: RX)
// - UART Pi (GPIO43: TX, GPIO44: RX)
// - OLED無効化（I2C: GPIO4, GPIO5）
// - テスト送信スイッチ: GPIO8（HIGH時に送信）
// - 内蔵LED: GPIO21
// ================================================

#include <Arduino.h>

// GPIO定義
#define PI_UART_TX_PIN 43
#define PI_UART_RX_PIN 44
#define TC_UART_TX_PIN 2
#define TC_UART_RX_PIN 3
#define TEST_PIN 8
#define LED_PIN 21

// FreeRTOS用キュー
QueueHandle_t piToTcQueue;
QueueHandle_t tcToPiQueue;

// シリアル排他用
portMUX_TYPE serialMux = portMUX_INITIALIZER_UNLOCKED;

// テストパケット
const uint8_t testPacket[6] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};

// シリアル出力（FreeRTOSセーフ）
void safeSerialPrint(const char* msg) {
  portENTER_CRITICAL(&serialMux);
  Serial.print(msg);
  portEXIT_CRITICAL(&serialMux);
}
void safeSerialPrintln(const char* msg) {
  portENTER_CRITICAL(&serialMux);
  Serial.println(msg);
  portEXIT_CRITICAL(&serialMux);
}

// BitBang送信（バイト間delayあり）
void bitbangSendPacket(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = data[i];
    digitalWrite(TC_UART_TX_PIN, LOW); delayMicroseconds(3333); // Start
    for (int j = 0; j < 8; ++j) {
      digitalWrite(TC_UART_TX_PIN, b & 0x01); delayMicroseconds(3333);
      b >>= 1;
    }
    digitalWrite(TC_UART_TX_PIN, HIGH); delayMicroseconds(3333); // Stop
    delayMicroseconds(3000);
  }
}

// 割り込み保護付き BitBang受信（MSBファースト受信テスト版）
bool bitbangReceiveByte(uint8_t* outByte) {
  if (digitalRead(TC_UART_RX_PIN) == LOW) {
    noInterrupts();
    delayMicroseconds(1666);  // Half-bit to center of first bit
    while (digitalRead(TC_UART_RX_PIN) == LOW); // Wait for start bit to end
    delayMicroseconds(3333);  // Center of first data bit

    uint8_t b = 0;
    for (int i = 0; i < 8; ++i) {
      b <<= 1;
      if (digitalRead(TC_UART_RX_PIN)) b |= 0x01;
      delayMicroseconds(3333);
    }
    interrupts();
    *outByte = b;
    Serial.printf("[RX byte] %02X\n", b);
    return true;
  }
  return false;
}

#if 0
// 割り込み保護付き BitBang受信
bool bitbangReceiveByte(uint8_t* outByte) {
  if (digitalRead(TC_UART_RX_PIN) == LOW) {
    noInterrupts();
    delayMicroseconds(1666);  // Half-bit
    while (digitalRead(TC_UART_RX_PIN) == LOW); // Start bit end
    delayMicroseconds(3333);  // Bit center

    uint8_t b = 0;
    for (int i = 0; i < 8; ++i) {
      b >>= 1;
      if (digitalRead(TC_UART_RX_PIN)) b |= 0x80;
      delayMicroseconds(3333);
    }
    interrupts();
    *outByte = b;
    Serial.printf("[RX byte] %02X\n", b);
    return true;
  }
  return false;
}
#endif

// UART→キュー（piToTc）
void uartToTcTask(void* pv) {
  uint8_t buf[6];
  for (;;) {
    if (Serial2.available() >= 6) {
      Serial2.readBytes(buf, 6);
      Serial.print("[UART→Q piToTc] ");
      for (int i = 0; i < 6; ++i) Serial.printf("%02X ", buf[i]);
      Serial.println();
      xQueueSend(piToTcQueue, buf, portMAX_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// キュー→BitBang送信
void tcSenderTask(void* pv) {
  uint8_t buf[6];
  for (;;) {
    if (xQueueReceive(piToTcQueue, buf, portMAX_DELAY)) {
      Serial.print("[Q piToTc→TC] ");
      for (int i = 0; i < 6; ++i) Serial.printf("%02X ", buf[i]);
      Serial.println();
      bitbangSendPacket(buf, 6);
    }
  }
}

// BitBang受信→Piキュー転送
void tcReceiverTask(void* pv) {
  uint8_t packet[6];
  size_t index = 0;
  for (;;) {
    uint8_t b;
    if (bitbangReceiveByte(&b)) {
      packet[index++] = b;
      if (index == 6) {
        Serial.print("[TC→Q tcToPi] ");
        for (int i = 0; i < 6; ++i) Serial.printf("%02X ", packet[i]);
        Serial.println();
        xQueueSend(tcToPiQueue, packet, portMAX_DELAY);
        index = 0;
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
}

// Piキュー→UART送信
void tcToUartTask(void* pv) {
  uint8_t buf[6];
  for (;;) {
    if (xQueueReceive(tcToPiQueue, buf, portMAX_DELAY)) {
      Serial.print("[Q tcToPi→UART] ");
      for (int i = 0; i < 6; ++i) Serial.printf("%02X ", buf[i]);
      Serial.println();
      Serial2.write(buf, 6);
    }
  }
}

// テスト送信（GPIO8 HIGH時に送信）
void testLoopTask(void* pv) {
  static unsigned long lastSend = 0;
  static bool wasHigh = false;
  for (;;) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    bool isHigh = (digitalRead(TEST_PIN) == HIGH);
    if (isHigh) {
      unsigned long now = millis();
      if (!wasHigh || (now - lastSend > 2000)) {
        Serial.print("[TEST→Q piToTc] ");
        for (int i = 0; i < 6; ++i) Serial.printf("%02X ", testPacket[i]);
        Serial.println();
        xQueueSend(piToTcQueue, (void*)testPacket, portMAX_DELAY);
        lastSend = now;
      }
    }
    wasHigh = isHigh;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(TEST_PIN, INPUT);
  pinMode(TC_UART_TX_PIN, OUTPUT);
  pinMode(TC_UART_RX_PIN, INPUT_PULLUP);
  digitalWrite(TC_UART_TX_PIN, HIGH); // Idle

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);

  piToTcQueue = xQueueCreate(8, 6);
  tcToPiQueue = xQueueCreate(8, 6);

  xTaskCreatePinnedToCore(uartToTcTask,   "UART->TC",  2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(tcSenderTask,   "TC SEND",   2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(tcReceiverTask, "TC RECV",   2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(tcToUartTask,   "TC->UART",  2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(testLoopTask,   "TEST",      2048, NULL, 1, NULL, 1);
}

void loop() {
  // FreeRTOS利用のため未使用
}
