/*********************************************************************
  XIAO ESP32S3 + Grove Shield – UART<->UART/BitBang中継スケッチ（OLEDなし）
  - UART中継 (J1=GPIO44/43: Pi 、J4=GPIO0/1: TC)
  - FreeRTOS + Queue + TaskNotify + Mutex 構成
  - チェックサム検証
  - セーフモード制御 (GPIO2 = D2)
  - 2025.06.26 OLED除去版・CoreDump対策
  - 2025.06.27 Debug シリアルログ
*********************************************************************/

#include <freertos/semphr.h>
#include <HardwareSerial.h>

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

// ===== チェックサム検証（最終バイト＝加算チェック） =====
bool isChecksumValid(const uint8_t* data) {
  uint8_t sum = 0;
  for (int i = 0; i < PACKET_SIZE - 1; i++) sum += data[i];
  return (sum == data[PACKET_SIZE - 1]);
}

// ===== HEX配列表示ユーティリティ =====
void printHex(const char* label, const uint8_t* data) {
  Serial.print(label);
  for (int i = 0; i < PACKET_SIZE; i++) {
    Serial.print(" ");
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
  }
  Serial.println();
}

// ===== Pi→中継機 受信スレッド（UART2） =====
void task_pi_rx(void* pv) {
  uint8_t buf[PACKET_SIZE];
  static bool first = true;
    if (first) {
    Serial.println("[START] task_pi_rx launched");
    Serial.flush(); 
    first = false;
  }

  while (1) {
    // スレッドチェック
    if (first) {
      Serial.println("[START] task_pi_rx launched");
      first = false;
    }

    if (SerialPI.available() >= PACKET_SIZE) {
      SerialPI.readBytes(buf, PACKET_SIZE);
      if (!isChecksumValid(buf)) continue;
      printHex("[PI->Relay]", buf);
      xQueueSend(queue_pi_rx, buf, portMAX_DELAY);
    }
    vTaskDelay(1);
  }
}

// ===== TC→中継機 受信スレッド（UART1） =====
void task_tc_rx(void* pv) {
  uint8_t buf[PACKET_SIZE];
  static bool first = true;

  while (1) {
    // スレッドチェック
    if (first) {
      Serial.println("[START] task_tc_rx launched");
      Serial.flush(); 
      first = false;
    }
    if (SerialTC.available() >= PACKET_SIZE) {
      SerialTC.readBytes(buf, PACKET_SIZE);
      if (!isChecksumValid(buf)) continue;
      printHex("[TC->Relay]", buf);
      xQueueSend(queue_tc_rx, buf, portMAX_DELAY);
    }
    vTaskDelay(1);
  }
}

// ===== Pi→TC 送信スレッド（中継） =====
void task_tc_tx(void* pv) {
  uint8_t pkt[PACKET_SIZE];
  static bool first = true;

  while (1) {
    // スレッドチェック
    if (first) {
      Serial.println("[START] task_tc_tx launched");
      Serial.flush(); 
      first = false;
    }
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
      printHex("[Relay->TC]", pkt);
    }
    vTaskDelay(1);
  }
}

// ===== TC→Pi 送信スレッド（中継） =====
void task_pi_tx(void* pv) {
  uint8_t pkt[PACKET_SIZE];

  static bool first = true;
  if (first) {
    Serial.println("[START] task_pi_tx launched");
    Serial.flush(); 
    first = false;
  }
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
      printHex("[Relay->PI]", pkt);
    }
    vTaskDelay(1);
  }
}

// === デバックシリアルPC接続完了待ち ===
void waitForSerial() {
  while (!Serial) {
    delay(10);
  }
}

// ===== 初期化処理 =====
void setup() {
  BaseType_t TaskResult;

  pinMode(LED_PIN, OUTPUT);
  pinMode(SAFE_MODE_PIN, INPUT_PULLUP);

  Serial.begin(115200); // USBシリアル（ログ用）
  waitForSerial();  // ← ここでPCの接続待ち
  Serial.println("Serial ready");

  SerialPI.begin(UART_BAUD_PI, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
  SerialTC.begin(UART_BAUD_TC, SERIAL_8N1, TC_UART_RX_PIN, TC_UART_TX_PIN);

  xStopFlagMutex = xSemaphoreCreateMutex();
  queue_pi_rx = xQueueCreate(5, PACKET_SIZE);
  queue_tc_rx = xQueueCreate(5, PACKET_SIZE);

  TaskResult = xTaskCreatePinnedToCore(task_pi_rx, "pi_rx", 2048, NULL, 1, NULL, 1);
  if (TaskResult != pdPASS) Serial.println("[ERR] task_pi_rx failed");
  TaskResult = xTaskCreatePinnedToCore(task_tc_rx, "tc_rx", 2048, NULL, 1, NULL, 1);
  if (TaskResult != pdPASS) Serial.println("[ERR] task_tc_rx failed");
  TaskResult = xTaskCreatePinnedToCore(task_tc_tx, "tc_tx", 2048, NULL, 1, NULL, 0);
  if (TaskResult != pdPASS) Serial.println("[ERR] task_tc_tx failed");
  TaskResult = xTaskCreatePinnedToCore(task_pi_tx, "pi_tx", 2048, NULL, 1, NULL, 0);
  if (TaskResult != pdPASS) Serial.println("[ERR] task_pi_tx failed");

  Serial.println("=== UART Relay Ready ===");
  Serial.flush(); 
}

// ===== LED点滅（動作確認用） =====
void loop() {
  static bool led_state = false;
  digitalWrite(LED_PIN, led_state);
  led_state = !led_state;
  vTaskDelay(pdMS_TO_TICKS(500));
}
