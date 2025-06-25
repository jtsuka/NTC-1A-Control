/*********************************************************************
  XIAO ESP32S3 + Grove Shield – 非同期・同期対応 TC中継スケッチ
  - BitBang送受信（GPIO2, GPIO3）
  - UART送受信（GPIO4, GPIO5）
  - OLED表示（SSD1306 I2C）
  - FreeRTOS + Queue + TaskNotify + Mutex 構成
  - チェックサム検証
  - 特殊コマンドによる中継停止/再開
  - 2025.06.25 19:30 ベータバージョン B-1.0
*********************************************************************/

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <freertos/semphr.h>

#define BITBANG_TX_PIN 2
#define BITBANG_RX_PIN 3
#define UART_TX_PIN    4
#define UART_RX_PIN    5
#define I2C_SDA        6
#define I2C_SCL        7
#define OLED_ADDR      0x3C
#define UART_BAUD      115200
#define PACKET_SIZE    6
#define BIT_DURATION_US 3333

// 特殊コマンド
const uint8_t CMD_STOP[6]  = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};
const uint8_t CMD_RESUME[6]= {0x01, 0x06, 0x05, 0xFF, 0xFF, 0x17};

TaskHandle_t task_uart_rx, task_uart_tx;
TaskHandle_t task_bb_rx, task_bb_tx;
QueueHandle_t queue_uart_rx, queue_bb_rx;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
SemaphoreHandle_t xMutex;
bool stopFlag = false;

// Utility
bool isChecksumValid(const uint8_t* data) {
  uint8_t sum = 0;
  for (int i = 0; i < PACKET_SIZE - 1; i++) sum += data[i];
  return (sum == data[PACKET_SIZE - 1]);
}

bool isPacketEqual(const uint8_t* a, const uint8_t* b) {
  for (int i = 0; i < PACKET_SIZE; i++) if (a[i] != b[i]) return false;
  return true;
}

void showOLED(const char* label, const uint8_t* data) {
  if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.print(label);
    for (int i = 0; i < PACKET_SIZE; i++) {
      display.print(" ");
      if (data[i] < 0x10) display.print("0");
      display.print(data[i], HEX);
    }
    display.display();
    xSemaphoreGive(xMutex);
  }
}

// BitBang I/O
void sendBitBangByte(uint8_t b) {
  digitalWrite(BITBANG_TX_PIN, LOW); delayMicroseconds(BIT_DURATION_US);
  for (uint8_t j = 0; j < 8; j++) {
    digitalWrite(BITBANG_TX_PIN, (b >> j) & 0x01);
    delayMicroseconds(BIT_DURATION_US);
  }
  digitalWrite(BITBANG_TX_PIN, HIGH); delayMicroseconds(BIT_DURATION_US);
}

void sendBitBangPacket(const uint8_t* data) {
  for (int i = 0; i < PACKET_SIZE; i++) sendBitBangByte(data[i]);
}

uint8_t receiveBitBangByte() {
  uint8_t value = 0;
  while (digitalRead(BITBANG_RX_PIN) == HIGH);
  delayMicroseconds(BIT_DURATION_US + BIT_DURATION_US / 2);
  for (uint8_t i = 0; i < 8; i++) {
    value |= (digitalRead(BITBANG_RX_PIN) << i);
    delayMicroseconds(BIT_DURATION_US);
  }
  delayMicroseconds(BIT_DURATION_US);
  return value;
}

// UART -> BitBang
void task_uart_rx_func(void* pv) {
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  uint8_t buf[PACKET_SIZE];
  while (1) {
    if (Serial1.available() >= PACKET_SIZE) {
      Serial1.readBytes(buf, PACKET_SIZE);
      if (isPacketEqual(buf, CMD_STOP)) {
        stopFlag = true; showOLED("CMD:STOP", buf); continue;
      } else if (isPacketEqual(buf, CMD_RESUME)) {
        stopFlag = false; showOLED("CMD:RESUME", buf); continue;
      } else if (!isChecksumValid(buf)) {
        showOLED("UART NG", buf); continue;
      }
      xQueueSend(queue_uart_rx, buf, portMAX_DELAY);
      xTaskNotifyGive(task_bb_tx);
      showOLED("UART->", buf);
    }
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

void task_bb_tx_func(void* pv) {
  uint8_t packet[PACKET_SIZE];
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (stopFlag) continue;
    if (xQueueReceive(queue_uart_rx, packet, 0) == pdTRUE) {
      sendBitBangPacket(packet);
      showOLED("BB TX->", packet);
    }
  }
}

// BitBang -> UART
void task_bb_rx_func(void* pv) {
  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);
  uint8_t buf[PACKET_SIZE];
  while (1) {
    if (digitalRead(BITBANG_RX_PIN) == LOW) {
      for (int i = 0; i < PACKET_SIZE; i++) buf[i] = receiveBitBangByte();
      if (isPacketEqual(buf, CMD_STOP)) {
        stopFlag = true; showOLED("CMD:STOP", buf); continue;
      } else if (isPacketEqual(buf, CMD_RESUME)) {
        stopFlag = false; showOLED("CMD:RESUME", buf); continue;
      } else if (!isChecksumValid(buf)) {
        showOLED("BB NG", buf); continue;
      }
      xQueueSend(queue_bb_rx, buf, portMAX_DELAY);
      xTaskNotifyGive(task_uart_tx);
      showOLED("BB RX->", buf);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void task_uart_tx_func(void* pv) {
  uint8_t packet[PACKET_SIZE];
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (stopFlag) continue;
    if (xQueueReceive(queue_bb_rx, packet, 0) == pdTRUE) {
      Serial1.write(packet, PACKET_SIZE);
      showOLED("UART TX->", packet);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BITBANG_TX_PIN, OUTPUT); digitalWrite(BITBANG_TX_PIN, HIGH);
  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA, I2C_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay(); display.display();
  xMutex = xSemaphoreCreateMutex();
  queue_uart_rx = xQueueCreate(5, sizeof(uint8_t) * PACKET_SIZE);
  queue_bb_rx   = xQueueCreate(5, sizeof(uint8_t) * PACKET_SIZE);

  xTaskCreatePinnedToCore(task_uart_rx_func, "UART_RX", 2048, NULL, 1, &task_uart_rx, 1);
  xTaskCreatePinnedToCore(task_bb_tx_func,   "BB_TX",   2048, NULL, 1, &task_bb_tx,   0);
  xTaskCreatePinnedToCore(task_bb_rx_func,   "BB_RX",   2048, NULL, 1, &task_bb_rx,   0);
  xTaskCreatePinnedToCore(task_uart_tx_func, "UART_TX", 2048, NULL, 1, &task_uart_tx, 1);
}

// 将来の監視処理の為にVtaskDelay()をloop()に設定
void loop() {
  // 監視処理など（今は未使用）
  vTaskDelay(pdMS_TO_TICKS(100));  // 負荷を抑えつつ柔軟に対応
}
