/*********************************************************************
  XIAO ESP32S3 + Grove Shield 完全版 – TC中継スケッチ（相互変換モデル）
  - BitBang送受信（GPIO2, GPIO3）
  - UART送受信（GPIO4, GPIO5）
  - OLED表示（SSD1306 I2C）
  - FreeRTOS + Queue + TaskNotify 構成
*********************************************************************/

#include <Wire.h>
#include <Adafruit_SSD1306.h>

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

TaskHandle_t task_uart_rx, task_uart_tx;
TaskHandle_t task_bb_rx, task_bb_tx;
QueueHandle_t queue_uart_rx, queue_bb_rx;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

void showOLED(const char* label, const uint8_t* data) {
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
}

// ---------------- BitBang送受信 ----------------
void sendBitBangByte(uint8_t b) {
  portENTER_CRITICAL();
  digitalWrite(BITBANG_TX_PIN, LOW); delayMicroseconds(BIT_DURATION_US);
  for (uint8_t j = 0; j < 8; j++) {
    digitalWrite(BITBANG_TX_PIN, (b >> j) & 0x01);
    delayMicroseconds(BIT_DURATION_US);
  }
  digitalWrite(BITBANG_TX_PIN, HIGH); delayMicroseconds(BIT_DURATION_US);
  portEXIT_CRITICAL();
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

// ---------------- UART受信 → BitBang送信 ----------------
void task_uart_rx_func(void* pv) {
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  uint8_t buf[PACKET_SIZE];
  while (1) {
    if (Serial1.available() >= PACKET_SIZE) {
      Serial1.readBytes(buf, PACKET_SIZE);
      xQueueSend(queue_uart_rx, buf, portMAX_DELAY);
      xTaskNotifyGive(task_bb_tx);
      showOLED("UART->", buf);
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void task_bb_tx_func(void* pv) {
  uint8_t packet[PACKET_SIZE];
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (xQueueReceive(queue_uart_rx, packet, 0) == pdTRUE) {
      sendBitBangPacket(packet);
      showOLED("BB TX->", packet);
    }
  }
}

// ---------------- BitBang受信 → UART送信 ----------------
void task_bb_rx_func(void* pv) {
  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);
  uint8_t buf[PACKET_SIZE];
  while (1) {
    if (digitalRead(BITBANG_RX_PIN) == LOW) {
      portENTER_CRITICAL();
      for (int i = 0; i < PACKET_SIZE; i++) buf[i] = receiveBitBangByte();
      portEXIT_CRITICAL();
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

  queue_uart_rx = xQueueCreate(5, sizeof(uint8_t) * PACKET_SIZE);
  queue_bb_rx   = xQueueCreate(5, sizeof(uint8_t) * PACKET_SIZE);

  xTaskCreatePinnedToCore(task_uart_rx_func, "UART_RX", 2048, NULL, 1, &task_uart_rx, 1);
  xTaskCreatePinnedToCore(task_bb_tx_func,   "BB_TX",   2048, NULL, 1, &task_bb_tx,   0);
  xTaskCreatePinnedToCore(task_bb_rx_func,   "BB_RX",   2048, NULL, 1, &task_bb_rx,   0);
  xTaskCreatePinnedToCore(task_uart_tx_func, "UART_TX", 2048, NULL, 1, &task_uart_tx, 1);
}

void loop() {}
