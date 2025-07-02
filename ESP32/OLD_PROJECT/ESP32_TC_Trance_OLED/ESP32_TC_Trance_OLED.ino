/*********************************************************************
  XIAO ESP32S3 + Grove Shield – UART<->UART/BitBang中継スケッチ（OLED付き）
  - UART中継 (J1=GPIO44/43: Pi 、J4=GPIO0/1: TC)
  - FreeRTOS + Queue + TaskNotify + Mutex 構成
  - チェックサム検証 + OLED表示
  - セーフモード制御 (GPIO2 = D2)
  - 2025.06.28 OLEDログ復活版・安定動作確認済み
*********************************************************************/

#include <freertos/semphr.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define PI_UART_TX_PIN 43
#define PI_UART_RX_PIN 44
#define TC_UART_TX_PIN 1
#define TC_UART_RX_PIN 0
#define SAFE_MODE_PIN   2
#define LED_PIN        21

#define UART_BAUD_PI   9600
#define UART_BAUD_TC    300
#define PACKET_SIZE       6

HardwareSerial SerialTC(1);
HardwareSerial SerialPI(2);

SemaphoreHandle_t xStopFlagMutex;
QueueHandle_t queue_pi_rx, queue_tc_rx;
volatile bool stopFlag = false;

// ===== チェックサム検証 =====
bool isChecksumValid(const uint8_t* data) {
  uint8_t sum = 0;
  for (int i = 0; i < PACKET_SIZE - 1; i++) sum += data[i];
  return (sum == data[PACKET_SIZE - 1]);
}

void showPacket(const char* label, const uint8_t* data, int row) {
  display.setCursor(0, row * 8);
  display.printf("%s", label);
  for (int i = 0; i < PACKET_SIZE; i++) {
    display.printf(" %02X", data[i]);
  }
  display.display();
}

void printHex(const char* label, const uint8_t* data) {
  Serial.print(label);
  for (int i = 0; i < PACKET_SIZE; i++) {
    Serial.print(" ");
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
  }
  Serial.println();
}

void task_pi_rx(void* pv) {
  uint8_t buf[PACKET_SIZE];
  static bool first = true;
  if (first) {
    Serial.println("[START] task_pi_rx launched");
    first = false;
  }

  while (1) {
    if (SerialPI.available() >= PACKET_SIZE) {
      SerialPI.readBytes(buf, PACKET_SIZE);
      if (!isChecksumValid(buf)) continue;
      printHex("[PI->Relay]", buf);
      showPacket("PI->Relay:", buf, 0);
      xQueueSend(queue_pi_rx, buf, portMAX_DELAY);
    }
    vTaskDelay(1);
  }
}

void task_tc_rx(void* pv) {
  uint8_t buf[PACKET_SIZE];
  static bool first = true;
  if (first) {
    Serial.println("[START] task_tc_rx launched");
    first = false;
  }

  while (1) {
    if (SerialTC.available() >= PACKET_SIZE) {
      SerialTC.readBytes(buf, PACKET_SIZE);
      if (!isChecksumValid(buf)) continue;
      printHex("[TC->Relay]", buf);
      showPacket("TC->Relay:", buf, 2);
      xQueueSend(queue_tc_rx, buf, portMAX_DELAY);
    }
    vTaskDelay(1);
  }
}

void task_tc_tx(void* pv) {
  uint8_t pkt[PACKET_SIZE];
  static bool first = true;
  if (first) {
    Serial.println("[START] task_tc_tx launched");
    first = false;
  }

  while (1) {
    bool flag;
    xSemaphoreTake(xStopFlagMutex, portMAX_DELAY);
    flag = stopFlag;
    xSemaphoreGive(xStopFlagMutex);
    if (flag || !digitalRead(SAFE_MODE_PIN)) {
      vTaskDelay(10);
      continue;
    }
    if (xQueueReceive(queue_pi_rx, pkt, 0) == pdTRUE) {
      SerialTC.write(pkt, PACKET_SIZE);
      printHex("[Relay->TC]", pkt);
      showPacket("Relay->TC:", pkt, 1);
    }
    vTaskDelay(1);
  }
}

void task_pi_tx(void* pv) {
  uint8_t pkt[PACKET_SIZE];
  static bool first = true;
  if (first) {
    Serial.println("[START] task_pi_tx launched");
    first = false;
  }

  while (1) {
    bool flag;
    xSemaphoreTake(xStopFlagMutex, portMAX_DELAY);
    flag = stopFlag;
    xSemaphoreGive(xStopFlagMutex);
    if (flag || !digitalRead(SAFE_MODE_PIN)) {
      vTaskDelay(10);
      continue;
    }
    if (xQueueReceive(queue_tc_rx, pkt, 0) == pdTRUE) {
      SerialPI.write(pkt, PACKET_SIZE);
      printHex("[Relay->PI]", pkt);
      showPacket("Relay->PI:", pkt, 3);
    }
    vTaskDelay(1);
  }
}

void waitForSerial() {
  while (!Serial) {
    delay(10);
  }
}

void setup() {
  BaseType_t TaskResult;
  pinMode(LED_PIN, OUTPUT);
  pinMode(SAFE_MODE_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  waitForSerial();
  Serial.println("Serial ready");

  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("UART Relay w/OLED");
  display.display();

  SerialPI.begin(UART_BAUD_PI, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
  SerialTC.begin(UART_BAUD_TC, SERIAL_8N1, TC_UART_RX_PIN, TC_UART_TX_PIN);

  xStopFlagMutex = xSemaphoreCreateMutex();
  queue_pi_rx = xQueueCreate(5, PACKET_SIZE);
  queue_tc_rx = xQueueCreate(5, PACKET_SIZE);

  TaskResult = xTaskCreatePinnedToCore(task_pi_rx, "pi_rx", 2048, NULL, 1, NULL, 1);
  TaskResult = xTaskCreatePinnedToCore(task_tc_rx, "tc_rx", 2048, NULL, 1, NULL, 1);
  TaskResult = xTaskCreatePinnedToCore(task_tc_tx, "tc_tx", 2048, NULL, 1, NULL, 0);
  TaskResult = xTaskCreatePinnedToCore(task_pi_tx, "pi_tx", 2048, NULL, 1, NULL, 0);

  Serial.println("=== UART Relay Ready (OLED) ===");
}

void loop() {
  static bool led_state = false;
  digitalWrite(LED_PIN, led_state);
  led_state = !led_state;
  vTaskDelay(pdMS_TO_TICKS(500));
}
