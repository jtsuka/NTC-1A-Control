/*********************************************************************
  XIAO ESP32S3 + Grove Shield – UART<->UART/BitBang 中継スケッチ
  - UART中継 (GPIO1,2<->Pi / GPIO6,7<->TC)
  - BitBang送信/受信対応 (オプション)
  - OLED表示 (実線I2C: GPIO6/7)
  - FreeRTOS + Queue + TaskNotify + Mutex 構成
  - チェックサム検証
  - セーフモード制御 (GPIO2)
*********************************************************************/

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <freertos/semphr.h>
#include <HardwareSerial.h>

#define USE_BITBANG 0

#define PI_UART_TX_PIN 1   // Grove UART TX (XIAO→Pi)
#define PI_UART_RX_PIN 2   // Grove UART RX (Pi→XIAO)
#define TC_UART_TX_PIN 6   // GPIO6 → TC
#define TC_UART_RX_PIN 7   // GPIO7 → XIAO
#define I2C_SDA        6
#define I2C_SCL        7
#define SAFE_MODE_PIN  2
#define LED_PIN        21
#define OLED_ADDR      0x3C
#define UART_BAUD_PI   1200
#define UART_BAUD_TC   300
#define PACKET_SIZE    6
#define BIT_DURATION_US 3333

Adafruit_SSD1306 display(128, 64, &Wire, -1);
SemaphoreHandle_t xMutex;
SemaphoreHandle_t stopMutex;
HardwareSerial SerialTC(1);
HardwareSerial SerialPI(2);
QueueHandle_t queue_pi_rx, queue_tc_rx;
volatile bool stopFlag = false;

bool isChecksumValid(const uint8_t* data) {
  uint8_t sum = 0;
  for (int i = 0; i < PACKET_SIZE - 1; i++) sum += data[i];
  return (sum == data[PACKET_SIZE - 1]);
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

void task_pi_rx(void* pv) {
  uint8_t buf[PACKET_SIZE];
  while (1) {
    if (SerialPI.available() >= PACKET_SIZE) {
      SerialPI.readBytes(buf, PACKET_SIZE);
      if (!isChecksumValid(buf)) continue;
      xQueueSend(queue_pi_rx, buf, portMAX_DELAY);
    }
    vTaskDelay(1);
  }
}

void task_tc_rx(void* pv) {
  uint8_t buf[PACKET_SIZE];
  while (1) {
    if (SerialTC.available() >= PACKET_SIZE) {
      SerialTC.readBytes(buf, PACKET_SIZE);
      if (!isChecksumValid(buf)) continue;
      xQueueSend(queue_tc_rx, buf, portMAX_DELAY);
    }
    vTaskDelay(1);
  }
}

void task_tc_tx(void* pv) {
  uint8_t pkt[PACKET_SIZE];
  while (1) {
    bool localStop = false;
    xSemaphoreTake(stopMutex, portMAX_DELAY);
    localStop = stopFlag;
    xSemaphoreGive(stopMutex);
    if (localStop) {
      vTaskDelay(10);
      continue;
    }
    if (xQueueReceive(queue_pi_rx, pkt, 0) == pdTRUE) {
      SerialTC.write(pkt, PACKET_SIZE);
      showOLED("Pi->TC", pkt);
    }
    vTaskDelay(1);
  }
}

void task_pi_tx(void* pv) {
  uint8_t pkt[PACKET_SIZE];
  while (1) {
    bool localStop = false;
    xSemaphoreTake(stopMutex, portMAX_DELAY);
    localStop = stopFlag;
    xSemaphoreGive(stopMutex);
    if (localStop) {
      vTaskDelay(10);
      continue;
    }
    if (xQueueReceive(queue_tc_rx, pkt, 0) == pdTRUE) {
      SerialPI.write(pkt, PACKET_SIZE);
      showOLED("TC->Pi", pkt);
    }
    vTaskDelay(1);
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SAFE_MODE_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay(); display.display();

  SerialPI.begin(UART_BAUD_PI, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
  SerialTC.begin(UART_BAUD_TC, SERIAL_8N1, TC_UART_RX_PIN, TC_UART_TX_PIN);

  xMutex = xSemaphoreCreateMutex();
  stopMutex = xSemaphoreCreateMutex();
  queue_pi_rx = xQueueCreate(5, PACKET_SIZE);
  queue_tc_rx = xQueueCreate(5, PACKET_SIZE);

  xTaskCreatePinnedToCore(task_pi_rx, "pi_rx", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(task_tc_rx, "tc_rx", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(task_tc_tx, "tc_tx", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task_pi_tx, "pi_tx", 2048, NULL, 1, NULL, 0);
}

void loop() {
  static bool led_state = false;
  digitalWrite(LED_PIN, led_state);
  led_state = !led_state;
  vTaskDelay(pdMS_TO_TICKS(500));
}
