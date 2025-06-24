/*********************************************************************
  XIAO ESP32S3 + Grove Shield – 非同期・同期対応 TC中継スケッチ
  - BitBang送受信（GPIO2, GPIO3）
  - UART送受信（GPIO4, GPIO5）
  - OLED表示（SSD1306 I2C）
  - FreeRTOS + Queue + TaskNotify + Mutex 構成
*********************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <freertos/semphr.h>

// ---------------- ピン定義 ----------------
#define BITBANG_TX_PIN 2
#define BITBANG_RX_PIN 3
#define UART_TX_PIN    4
#define UART_RX_PIN    5
#define I2C_SDA        6
#define I2C_SCL        7
#define OLED_ADDR      0x3C
#define PACKET_SIZE    6
#define BIT_DURATION_US 3333
#define UART_BAUD      115200

// ---------------- FreeRTOS ----------------
TaskHandle_t task_uart_rx, task_uart_tx, task_bb_rx, task_bb_tx;
QueueHandle_t queue_uart_rx, queue_bb_rx;
SemaphoreHandle_t oled_mutex;
bool stopFlag = false;

// ---------------- OLED ----------------
Adafruit_SSD1306 display(128, 64, &Wire, -1);

void showOLED(const char* label, const uint8_t* data, bool error = false) {
  if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
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
    if (error) display.print(" ERR");
    display.display();
    xSemaphoreGive(oled_mutex);
  }
}

// ---------------- チェックサム ----------------
bool verifyChecksum(const uint8_t* packet) {
  uint8_t sum = 0;
  for (int i = 0; i < PACKET_SIZE - 1; i++) sum += packet[i];
  return (sum == packet[PACKET_SIZE - 1]);
}

bool isStopPacket(const uint8_t* packet) {
  const uint8_t stop_cmd[6] = { 0x01, 0x06, 0x05, 0x00, 0x00, 0x0C };
  for (int i = 0; i < PACKET_SIZE; i++)
    if (packet[i] != stop_cmd[i]) return false;
  return true;
}

// ---------------- BitBang送受信 ----------------
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

// ---------------- タスク実装 ----------------
void task_uart_rx_func(void* pv) {
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  uint8_t buf[PACKET_SIZE];
  while (!stopFlag) {
    if (Serial1.available() >= PACKET_SIZE) {
      Serial1.readBytes(buf, PACKET_SIZE);
      if (isStopPacket(buf)) { stopFlag = true; showOLED("STOP CMD", buf); continue; }
      if (verifyChecksum(buf)) {
        xQueueSend(queue_uart_rx, buf, portMAX_DELAY);
        xTaskNotifyGive(task_bb_tx);
        showOLED("UART→", buf);
      } else {
        showOLED("UART CRC", buf, true);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  vTaskDelete(NULL);
}

void task_bb_tx_func(void* pv) {
  uint8_t packet[PACKET_SIZE];
  while (!stopFlag) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (xQueueReceive(queue_uart_rx, packet, 0) == pdTRUE) {
      sendBitBangPacket(packet);
      showOLED("BB TX→", packet);
    }
  }
  vTaskDelete(NULL);
}

void task_bb_rx_func(void* pv) {
  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);
  uint8_t buf[PACKET_SIZE];
  while (!stopFlag) {
    if (digitalRead(BITBANG_RX_PIN) == LOW) {
      for (int i = 0; i < PACKET_SIZE; i++) buf[i] = receiveBitBangByte();
      if (isStopPacket(buf)) { stopFlag = true; showOLED("STOP CMD", buf); continue; }
      if (verifyChecksum(buf)) {
        xQueueSend(queue_bb_rx, buf, portMAX_DELAY);
        xTaskNotifyGive(task_uart_tx);
        showOLED("BB RX→", buf);
      } else {
        showOLED("BB CRC", buf, true);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  vTaskDelete(NULL);
}

void task_uart_tx_func(void* pv) {
  uint8_t packet[PACKET_SIZE];
  while (!stopFlag) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (xQueueReceive(queue_bb_rx, packet, 0) == pdTRUE) {
      Serial1.write(packet, PACKET_SIZE);
      showOLED("UART←", packet);
    }
  }
  vTaskDelete(NULL);
}

// ---------------- 初期化 ----------------
void setup() {
  Serial.begin(115200);
  pinMode(BITBANG_TX_PIN, OUTPUT); digitalWrite(BITBANG_TX_PIN, HIGH);
  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA, I2C_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay(); display.display();
  oled_mutex = xSemaphoreCreateMutex();

  queue_uart_rx = xQueueCreate(4, PACKET_SIZE);
  queue_bb_rx   = xQueueCreate(4, PACKET_SIZE);

  xTaskCreatePinnedToCore(task_uart_rx_func, "UART_RX", 2048, NULL, 1, &task_uart_rx, 1);
  xTaskCreatePinnedToCore(task_bb_tx_func,   "BB_TX",   2048, NULL, 1, &task_bb_tx,   0);
  xTaskCreatePinnedToCore(task_bb_rx_func,   "BB_RX",   2048, NULL, 1, &task_bb_rx,   0);
  xTaskCreatePinnedToCore(task_uart_tx_func, "UART_TX", 2048, NULL, 1, &task_uart_tx, 1);
}

void loop() {
  if (stopFlag) {
    vTaskDelay(pdMS_TO_TICKS(1000)); // 無限待機
  }
}
