/*********************************************************************
  XIAO ESP32S3 + Grove Shield – UART<->UART/BitBang中継スケッチ
  - UART中継 (J1=GPIO44/43: Pi 、J4=GPIO0/1: TC)
  - OLED表示（SSD1306 I2C: Grove J2 = GPIO5(SDA)/GPIO6(SCL)）
  - FreeRTOS + Queue + TaskNotify + Mutex 構成
  - チェックサム検証
  - セーフモード制御 (GPIO2 = D2)
  - 2025.06.26 OLED対応完全版
*********************************************************************/

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <freertos/semphr.h>
#include <HardwareSerial.h>

// Groveポート対応GPIO定義
#define PI_UART_TX_PIN 43  // Grove J1: Pi TX
#define PI_UART_RX_PIN 44  // Grove J1: Pi RX
#define TC_UART_TX_PIN 1   // Grove J4: TC TX
#define TC_UART_RX_PIN 0   // Grove J4: TC RX
#define OLED_SDA        5  // Grove J2: OLED SDA
#define OLED_SCL        6  // Grove J2: OLED SCL
#define SAFE_MODE_PIN   2  // GPIO2 = セーフモード
#define LED_PIN        21  // 内蔵LED

#define UART_BAUD_PI   1200
#define UART_BAUD_TC    300
#define PACKET_SIZE       6

Adafruit_SSD1306 display(128, 64, &Wire, -1);
HardwareSerial SerialTC(1);
HardwareSerial SerialPI(2);

SemaphoreHandle_t xMutex, xStopFlagMutex;
QueueHandle_t queue_pi_rx, queue_tc_rx;
volatile bool stopFlag = false;

// ===== チェックサム検証（最終バイト＝加算チェック） =====
bool isChecksumValid(const uint8_t* data) {
  uint8_t sum = 0;
  for (int i = 0; i < PACKET_SIZE - 1; i++) sum += data[i];
  return (sum == data[PACKET_SIZE - 1]);
}

// ===== OLED表示関数（ラベル + 6バイト表示） =====
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

// ===== Pi→中継機 受信スレッド（UART2） =====
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

// ===== TC→中継機 受信スレッド（UART1） =====
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

// ===== Pi→TC 送信スレッド（中継） =====
void task_tc_tx(void* pv) {
  uint8_t pkt[PACKET_SIZE];
  while (1) {
    bool flag = false;
    xSemaphoreTake(xStopFlagMutex, portMAX_DELAY);
    flag = stopFlag;
    xSemaphoreGive(xStopFlagMutex);
    if (flag || !digitalRead(SAFE_MODE_PIN)) {
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

// ===== TC→Pi 送信スレッド（中継） =====
void task_pi_tx(void* pv) {
  uint8_t pkt[PACKET_SIZE];
  while (1) {
    bool flag = false;
    xSemaphoreTake(xStopFlagMutex, portMAX_DELAY);
    flag = stopFlag;
    xSemaphoreGive(xStopFlagMutex);
    if (flag || !digitalRead(SAFE_MODE_PIN)) {
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

// ===== 初期化処理 =====
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SAFE_MODE_PIN, INPUT_PULLUP);

  Wire.begin(OLED_SDA, OLED_SCL);  // OLED I2Cピンを明示
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (true); // OLED初期化失敗
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println("XIAO ESP32S3 UART Relay");
  display.println("Pi <-> TC + OLED OK");
  display.display();
  delay(1500);

  SerialPI.begin(UART_BAUD_PI, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
  SerialTC.begin(UART_BAUD_TC, SERIAL_8N1, TC_UART_RX_PIN, TC_UART_TX_PIN);

  xMutex = xSemaphoreCreateMutex();
  xStopFlagMutex = xSemaphoreCreateMutex();
  queue_pi_rx = xQueueCreate(5, PACKET_SIZE);
  queue_tc_rx = xQueueCreate(5, PACKET_SIZE);

  xTaskCreatePinnedToCore(task_pi_rx, "pi_rx", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(task_tc_rx, "tc_rx", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(task_tc_tx, "tc_tx", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task_pi_tx, "pi_tx", 2048, NULL, 1, NULL, 0);
}

// ===== LED点滅（動作確認用） =====
void loop() {
  static bool led_state = false;
  digitalWrite(LED_PIN, led_state);
  led_state = !led_state;
  vTaskDelay(pdMS_TO_TICKS(500));
}
