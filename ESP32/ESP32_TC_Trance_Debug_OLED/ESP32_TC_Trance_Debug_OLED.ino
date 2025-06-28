// OLEDログ表示クラス付き スケッチ統合版
// XIAO ESP32S3 + Grove Shield + OLED + UART中継（FreeRTOS）
/*********************************************************************
  XIAO ESP32S3 + Grove Shield – UART<->UART/BitBang中継スケッチ（OLED付き）
  - XIAO ESP32S3 + Grove Shield + OLED + UART中継（FreeRTOS）
  - UART中継 (J1=GPIO44/43: Pi 、J4=GPIO0/1: TC)
  - FreeRTOS + Queue + TaskNotify + Mutex 構成
  - チェックサム検証 + OLED表示
  - セーフモード制御 (GPIO2 = D2)
  - 2025.06.28 OLEDログ表示クラス付き スケッチ統合版
*********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <freertos/semphr.h>
#include <HardwareSerial.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// OLEDログクラス（排他制御付き）
class OLEDLogger {
private:
  SemaphoreHandle_t mutex;
  String lines[4];
public:
  OLEDLogger() {
    mutex = xSemaphoreCreateMutex();
  }
  void begin() {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("OLED Logger Ready");
    display.display();
  }
  void logLine(int line, const String& msg) {
    if (line < 0 || line >= 4) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    lines[line] = msg;
    refresh();
    xSemaphoreGive(mutex);
  }
  void refresh() {
    display.clearDisplay();
    for (int i = 0; i < 4; ++i) {
      display.setCursor(0, i * 16);
      display.print(lines[i]);
    }
    display.display();
  }
};

// ==== ピン定義・UART設定 ====
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
OLEDLogger oled;

// Debug for Test Packet
const uint8_t test_packet_pi[PACKET_SIZE] = { 0x01, 0x06, 0x05, 0x00, 0x00, 0x0C };
const uint8_t test_packet_tc[PACKET_SIZE] = { 0x02, 0x06, 0x07, 0x00, 0x00, 0x0F };

SemaphoreHandle_t xStopFlagMutex;
QueueHandle_t queue_pi_rx, queue_tc_rx;
volatile bool stopFlag = false;

// 擬似タスク
void task_fake_pi_tx(void* pv) {
  while (1) {
    bool flag;
    xSemaphoreTake(xStopFlagMutex, portMAX_DELAY);
    flag = stopFlag;
    xSemaphoreGive(xStopFlagMutex);
    if (!flag && digitalRead(SAFE_MODE_PIN)) {
      xQueueSend(queue_pi_rx, test_packet_pi, portMAX_DELAY);
      oled.logLine(0, "[TEST->TC] test_packet_pi");
    }
    vTaskDelay(pdMS_TO_TICKS(5000));  // 5秒間隔
  }
}

void task_fake_tc_tx(void* pv) {
  while (1) {
    bool flag;
    xSemaphoreTake(xStopFlagMutex, portMAX_DELAY);
    flag = stopFlag;
    xSemaphoreGive(xStopFlagMutex);
    if (!flag && digitalRead(SAFE_MODE_PIN)) {
      xQueueSend(queue_tc_rx, test_packet_tc, portMAX_DELAY);
      oled.logLine(2, "[TEST->PI] test_packet_tc");
    }
    vTaskDelay(pdMS_TO_TICKS(5000));  // 5秒間隔
  }
}


// ===== チェックサム検証 =====
bool isChecksumValid(const uint8_t* data) {
  uint8_t sum = 0;
  for (int i = 0; i < PACKET_SIZE - 1; i++) sum += data[i];
  return (sum == data[PACKET_SIZE - 1]);
}

void printHex(String& result, const uint8_t* data) {
  result = "";
  for (int i = 0; i < PACKET_SIZE; i++) {
    if (data[i] < 0x10) result += "0";
    result += String(data[i], HEX);
    if (i < PACKET_SIZE - 1) result += " ";
  }
}

void task_pi_rx(void* pv) {
  uint8_t buf[PACKET_SIZE];
  while (1) {
    if (SerialPI.available() >= PACKET_SIZE) {
      SerialPI.readBytes(buf, PACKET_SIZE);
      if (!isChecksumValid(buf)) continue;
      xQueueSend(queue_pi_rx, buf, portMAX_DELAY);
      String msg;
      printHex(msg, buf);
      oled.logLine(0, "[PI->Relay] " + msg);
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
      String msg;
      printHex(msg, buf);
      oled.logLine(2, "[TC->Relay] " + msg);
    }
    vTaskDelay(1);
  }
}

void task_tc_tx(void* pv) {
  uint8_t pkt[PACKET_SIZE];
  while (1) {
    xSemaphoreTake(xStopFlagMutex, portMAX_DELAY);
    bool flag = stopFlag;
    xSemaphoreGive(xStopFlagMutex);
    if (flag || !digitalRead(SAFE_MODE_PIN)) {
      vTaskDelay(10);
      continue;
    }
    if (xQueueReceive(queue_pi_rx, pkt, 0) == pdTRUE) {
      SerialTC.write(pkt, PACKET_SIZE);
      String msg;
      printHex(msg, pkt);
      oled.logLine(1, "[Relay->TC] " + msg);
    }
    vTaskDelay(1);
  }
}

void task_pi_tx(void* pv) {
  uint8_t pkt[PACKET_SIZE];
  while (1) {
    xSemaphoreTake(xStopFlagMutex, portMAX_DELAY);
    bool flag = stopFlag;
    xSemaphoreGive(xStopFlagMutex);
    if (flag || !digitalRead(SAFE_MODE_PIN)) {
      vTaskDelay(10);
      continue;
    }
    if (xQueueReceive(queue_tc_rx, pkt, 0) == pdTRUE) {
      SerialPI.write(pkt, PACKET_SIZE);
      String msg;
      printHex(msg, pkt);
      oled.logLine(3, "[Relay->PI] " + msg);
    }
    vTaskDelay(1);
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SAFE_MODE_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial) delay(10);
  oled.begin();
  Serial.println("Serial ready");

  SerialPI.begin(UART_BAUD_PI, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
  SerialTC.begin(UART_BAUD_TC, SERIAL_8N1, TC_UART_RX_PIN, TC_UART_TX_PIN);

  xStopFlagMutex = xSemaphoreCreateMutex();
  queue_pi_rx = xQueueCreate(5, PACKET_SIZE);
  queue_tc_rx = xQueueCreate(5, PACKET_SIZE);

  xTaskCreatePinnedToCore(task_pi_rx, "pi_rx", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(task_tc_rx, "tc_rx", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(task_tc_tx, "tc_tx", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task_pi_tx, "pi_tx", 2048, NULL, 1, NULL, 0);
  Serial.println("=== UART Relay with OLED Ready ===");
  // Test用擬似タスク
#if 0
  xTaskCreatePinnedToCore(task_fake_pi_tx, "fake_pi_tx", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task_fake_tc_tx, "fake_tc_tx", 2048, NULL, 1, NULL, 0);
#endif
}

void loop() {
  static bool led_state = false;
  digitalWrite(LED_PIN, led_state);
  led_state = !led_state;
  vTaskDelay(pdMS_TO_TICKS(500));
}
