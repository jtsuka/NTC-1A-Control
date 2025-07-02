/*********************************************************************
  XIAO ESP32S3 + Grove Shield – UART<->UART/BitBang中継スケッチ (Mutex保護版)
  - UART中継 (GPIO6,7<->TC / GPIO1,2<->Pi)
  - BitBang送信/受信対応 (オプション)
  - OLED表示（SSD1306 I2C）
  - FreeRTOS + Queue + TaskNotify + Mutex + CriticalSection
  - チェックサム検証
  - セーフモード制御 (GPIO2)
  - 2025.06.26 Mutex保護版
*********************************************************************/

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <freertos/semphr.h>
#include <HardwareSerial.h>

#define USE_BITBANG 0
#define PI_UART_TX_PIN 43
#define PI_UART_RX_PIN 44
#define TC_UART_TX_PIN 1
#define TC_UART_RX_PIN 0
#define LED_PIN 21
#define I2C_SDA 5
#define I2C_SCL 6
#define SAFE_MODE_PIN 2
#define OLED_ADDR 0x3C
#define UART_BAUD_PI 1200
#define UART_BAUD_TC 300
#define PACKET_SIZE 6
#define BIT_DURATION_US 3333

Adafruit_SSD1306 display(128, 64, &Wire, -1);
SemaphoreHandle_t xMutex;
portMUX_TYPE flagMux = portMUX_INITIALIZER_UNLOCKED;
HardwareSerial SerialTC(1);
HardwareSerial SerialPI(2);
QueueHandle_t queue_pi_rx, queue_tc_rx;
bool stopFlag = false;

bool getStopFlag() {
  portENTER_CRITICAL(&flagMux);
  bool val = stopFlag;
  portEXIT_CRITICAL(&flagMux);
  return val;
}

void setStopFlag(bool val) {
  portENTER_CRITICAL(&flagMux);
  stopFlag = val;
  portEXIT_CRITICAL(&flagMux);
}

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

#if 0
void task_pi_rx(void* pv) {
  uint8_t buf[PACKET_SIZE];
  while (1) {
    if (SerialPI.available() >= PACKET_SIZE) {
      SerialPI.readBytes(buf, PACKET_SIZE);
      Serial.print("Pi RX: ");
      for (int i = 0; i < PACKET_SIZE; i++) {
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      vTaskDelay(1);
    }
  }
}
#endif

void task_pi_rx(void* pv) {
  uint8_t buf[PACKET_SIZE];
  while (1) {
    if (SerialPI.available() >= PACKET_SIZE) {
      SerialPI.readBytes(buf, PACKET_SIZE);
      Serial.print("Pi RX: ");
      for (int i = 0; i < PACKET_SIZE; i++) {
        Serial.print(buf[i], HEX); Serial.print(" ");
      }
      Serial.println();

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
      Serial.print("TC RX: ");
      for (int i = 0; i < PACKET_SIZE; i++) {
        Serial.print(buf[i], HEX); Serial.print(" ");
      }
      Serial.println();

      if (!isChecksumValid(buf)) continue;
      xQueueSend(queue_tc_rx, buf, portMAX_DELAY);
    }
    vTaskDelay(1);
  }
}

#if 0
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
#endif

void task_tc_tx(void* pv) {
  uint8_t pkt[PACKET_SIZE];
  while (1) {
    if (!getStopFlag()) {
      if (xQueueReceive(queue_pi_rx, pkt, 0) == pdTRUE) {
        Serial.print("[TC_TX] Received from Pi: ");
        for (int i = 0; i < PACKET_SIZE; i++) {
          if (pkt[i] < 0x10) Serial.print("0");
          Serial.print(pkt[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

#if USE_BITBANG
        // BitBang送信未実装
#else
        Serial.println("[TC_TX] Sending to TC...");
        SerialTC.write(pkt, PACKET_SIZE);
#endif

        showOLED("Pi->TC", pkt);
      }
    }
    vTaskDelay(1);
  }
}

void task_pi_tx(void* pv) {
  uint8_t pkt[PACKET_SIZE];
  while (1) {
    if (!getStopFlag()) {
      if (xQueueReceive(queue_tc_rx, pkt, 0) == pdTRUE) {
        SerialPI.write(pkt, PACKET_SIZE);
        showOLED("TC->Pi", pkt);
      }
    }
    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);  // USB接続が安定するまで待つ（特にMac/Linuxで有効）
  Serial.println("Ready!");

  pinMode(LED_PIN, OUTPUT);
  pinMode(SAFE_MODE_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA, I2C_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay(); display.display();

  SerialPI.begin(UART_BAUD_PI, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
  SerialTC.begin(UART_BAUD_TC, SERIAL_8N1, TC_UART_RX_PIN, TC_UART_TX_PIN);

  xMutex = xSemaphoreCreateMutex();
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
