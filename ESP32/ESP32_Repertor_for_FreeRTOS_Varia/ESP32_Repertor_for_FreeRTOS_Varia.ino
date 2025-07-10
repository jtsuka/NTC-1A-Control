// ================================================
// ESP32-S3 TC Repeater with FreeRTOS + 可変長パケット対応
// - BitBang 300bps (GPIO2: TX, GPIO3: RX)
// - UART Pi (GPIO43: TX, GPIO44: RX)
// - OLED無効化（I2C: GPIO4, GPIO5）
// - テスト送信スイッチ: GPIO8（HIGH時に送信）
// - 内蔵LED: GPIO21
// ================================================

#include <Arduino.h>

#define PI_UART_TX_PIN 43
#define PI_UART_RX_PIN 44
#define TC_UART_TX_PIN 2
#define TC_UART_RX_PIN 3
#define TEST_PIN 8
#define LED_PIN 21

#define MAX_PKT_SIZE 16  // 最大パケットサイズ

#define BIT_PAT        false   // LSB

QueueHandle_t piToTcQueue;
QueueHandle_t tcToPiQueue;

portMUX_TYPE serialMux = portMUX_INITIALIZER_UNLOCKED;

struct PacketInfo {
  uint8_t cmd;
  uint8_t len;
};

const PacketInfo packetTable[] = {
  {0x06, 6},
  {0x10, 6},
  {0x11, 6},
  {0x50, 6},
  {0x13, 8},
  {0x20, 8},
};

uint8_t reverseBits(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

uint8_t getExpectedLength(uint8_t cmd) {
  for (size_t i = 0; i < sizeof(packetTable) / sizeof(PacketInfo); ++i) {
    if (packetTable[i].cmd == cmd) return packetTable[i].len;
  }
  return 6; // デフォルト長
}

void bitbangSendPacket(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    uint8_t msbBit = BIT_PAT ? reverseBits(data[i]): data[i];
    uint8_t b = msbBit;
    digitalWrite(TC_UART_TX_PIN, LOW); delayMicroseconds(3333);
    for (int j = 0; j < 8; ++j) {
      digitalWrite(TC_UART_TX_PIN, b & 0x01); delayMicroseconds(3333);
      b >>= 1;
    }
    digitalWrite(TC_UART_TX_PIN, HIGH); delayMicroseconds(3333);
//    delayMicroseconds(3000);
  }
}

bool bitbangReceiveByte(uint8_t* outByte) {
  if (digitalRead(TC_UART_RX_PIN) == LOW) {
    noInterrupts();
    delayMicroseconds(1666);  // スタートビット中央へ
    while (digitalRead(TC_UART_RX_PIN) == LOW);
    delayMicroseconds(3333); // 最初のデータビットへ

    uint8_t b = 0;
    for (int i = 0; i < 8; ++i) {
      b |= (digitalRead(TC_UART_RX_PIN) << i);  // ← LSB順にそのまま受信
      delayMicroseconds(3333);
    }
    interrupts();
    // モードによって切換え
    *outByte = BIT_PAT ? reverseBits(b) : b;
    return true;
  }
  return false;
}

#if 0
bool bitbangReceiveByte(uint8_t* outByte) {
  if (digitalRead(TC_UART_RX_PIN) == LOW) {
    noInterrupts();
    delayMicroseconds(1666);
    while (digitalRead(TC_UART_RX_PIN) == LOW);
    delayMicroseconds(3333);

    uint8_t b = 0;
    for (int i = 0; i < 8; ++i) {
      b <<= 1;
      if (digitalRead(TC_UART_RX_PIN)) b |= 0x80;
      delayMicroseconds(3333);
    }
    interrupts();
    *outByte = reverseBits(b);  // LSB → MSB に変換
    return true;
  }
  return false;
}
#endif

void uartToTcTask(void* pv) {
  uint8_t buf[MAX_PKT_SIZE];
  for (;;) {
    if (Serial2.available()) {
      uint8_t cmd = Serial2.read();
      uint8_t len = getExpectedLength(cmd);
      buf[0] = cmd;
      Serial2.readBytes(buf + 1, len - 1);
      xQueueSend(piToTcQueue, buf, portMAX_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void tcSenderTask(void* pv) {
  uint8_t buf[MAX_PKT_SIZE];
  for (;;) {
    if (xQueueReceive(piToTcQueue, buf, portMAX_DELAY)) {
      uint8_t len = getExpectedLength(buf[0]);
      bitbangSendPacket(buf, len);
    }
  }
}

void tcReceiverTask(void* pv) {
  uint8_t packet[MAX_PKT_SIZE];
  for (;;) {
    uint8_t b;
    if (bitbangReceiveByte(&b)) {
      uint8_t expectedLen = getExpectedLength(b);
      packet[0] = b;
      for (uint8_t i = 1; i < expectedLen; ++i) {
        while (!bitbangReceiveByte(&packet[i])) vTaskDelay(1);
      }
      xQueueSend(tcToPiQueue, packet, portMAX_DELAY);
    } else {
      vTaskDelay(1);
    }
  }
}

void tcToUartTask(void* pv) {
  uint8_t buf[MAX_PKT_SIZE];
  for (;;) {
    if (xQueueReceive(tcToPiQueue, buf, portMAX_DELAY)) {
      uint8_t len = getExpectedLength(buf[0]);
      //  そのまま返す or 反転
      uint8_t msbBuf[MAX_PKT_SIZE];
      for (uint8_t i = 0; i < len; ++i) {
//        msbBuf[i] = BIT_PAT ? reverseBits(buf[i]) : buf[i];
        msbBuf[i] = BIT_PAT ? buf[i] : reverseBits(buf[i]) ;
      }
      Serial2.write(msbBuf, len);
      Serial2.flush();  // UART TXバッファが空になるまで待機
    }
  }
}

const uint8_t testPacket[6] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};
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
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(TEST_PIN, INPUT);
  pinMode(TC_UART_TX_PIN, OUTPUT);
  pinMode(TC_UART_RX_PIN, INPUT_PULLUP);
  digitalWrite(TC_UART_TX_PIN, HIGH);

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);

  piToTcQueue = xQueueCreate(8, MAX_PKT_SIZE);
  tcToPiQueue = xQueueCreate(8, MAX_PKT_SIZE);

  xTaskCreatePinnedToCore(uartToTcTask,   "UART->TC",  2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(tcSenderTask,   "TC SEND",   2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(tcReceiverTask, "TC RECV",   2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(tcToUartTask,   "TC->UART",  2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(testLoopTask,   "TEST",      2048, NULL, 1, NULL, 1);
}

void loop() {}
