// ==============================
// TC Emulator with BitBang Parser (ESP32-S3)
// - 300bps BitBang RX/TX
// - FreeRTOS構成 / OLED初期化のみ
// - コマンドIDに応じて受信長を切替
// ==============================

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define TC_UART_TX_PIN 2
#define TC_UART_RX_PIN 3
#define LED_PIN        21
#define TEST_PIN        8
#define OLED_SDA_PIN    4
#define OLED_SCL_PIN    5
#define BAUD_RATE      300
#define BIT_DURATION_US (1000000 / BAUD_RATE)

// ========== OLED ==========
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ========== コマンド定義テーブル ==========
typedef struct {
  uint8_t cmd_id;
  uint8_t payload_size;
} CommandDef;

static const CommandDef cmd_table[] = {
  {0x01, 5}, {0x02, 0}, {0x03, 1},
  {0x04, 2}, {0x05, 0}, {0x06, 1}, {0x07, 1},
};
const size_t CMD_TABLE_LEN = sizeof(cmd_table) / sizeof(CommandDef);

QueueHandle_t sendQueue;

const uint8_t testPacket[6] = { 0x01, 0x06, 0x05, 0x00, 0x00, 0x0C };

void logPacket(const char* label, const uint8_t* data, size_t len) {
  Serial.print(label);
  for (size_t i = 0; i < len; ++i) Serial.printf(" %02X", data[i]);
  Serial.println();
}

int lookupPayloadSize(uint8_t cmd_id) {
  for (size_t i = 0; i < CMD_TABLE_LEN; ++i) {
    if (cmd_table[i].cmd_id == cmd_id) return cmd_table[i].payload_size;
  }
  return -1;
}

void bitBangSendByte(uint8_t b) {
  digitalWrite(TC_UART_TX_PIN, LOW); delayMicroseconds(BIT_DURATION_US);
  for (int i = 0; i < 8; ++i) {
    digitalWrite(TC_UART_TX_PIN, b & 0x01);
    delayMicroseconds(BIT_DURATION_US);
    b >>= 1;
  }
  digitalWrite(TC_UART_TX_PIN, HIGH); delayMicroseconds(BIT_DURATION_US);
}

void sendPacket(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) bitBangSendByte(data[i]);
}

bool receiveByte(uint8_t* outByte) {
  while (digitalRead(TC_UART_RX_PIN) == HIGH) vTaskDelay(1);
  delayMicroseconds(BIT_DURATION_US + BIT_DURATION_US / 2);

  uint8_t b = 0;
  for (int i = 0; i < 8; ++i) {
    b |= (digitalRead(TC_UART_RX_PIN) << i);
    delayMicroseconds(BIT_DURATION_US);
  }
  delayMicroseconds(BIT_DURATION_US);
  *outByte = b;
  return true;
}

void TaskBitBangReceive(void* pvParameters) {
  for (;;) {
    if (digitalRead(TEST_PIN) == HIGH) {
      logPacket("[TEST]", testPacket, 6);
      xQueueSend(sendQueue, testPacket, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    uint8_t cmd;
    if (!receiveByte(&cmd)) continue;
    int plen = lookupPayloadSize(cmd);
    if (plen < 0 || plen > 10) continue;

    uint8_t packet[11];
    packet[0] = cmd;
    for (int i = 0; i < plen; ++i) {
      if (!receiveByte(&packet[i + 1])) return;
    }
    logPacket("[RECV]", packet, plen + 1);
    xQueueSend(sendQueue, packet, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void TaskBitBangSend(void* pvParameters) {
  uint8_t packet[11];
  for (;;) {
    if (xQueueReceive(sendQueue, &packet, portMAX_DELAY) == pdTRUE) {
      logPacket("[SEND]", packet, packet[1] ? packet[1] + 1 : 1);
      sendPacket(packet, packet[1] ? packet[1] + 1 : 1);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void TaskLED(void* pvParameters) {
  for (;;) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void initOLED() {
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("TC Emulator Ready");
  display.display();
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  pinMode(TC_UART_TX_PIN, OUTPUT);
  pinMode(TC_UART_RX_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TEST_PIN, INPUT_PULLUP);
  digitalWrite(TC_UART_TX_PIN, HIGH);

  initOLED();

  sendQueue = xQueueCreate(4, sizeof(uint8_t[11]));
  xTaskCreatePinnedToCore(TaskBitBangReceive, "Receive", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskBitBangSend, "Send", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskLED, "LED", 1024, NULL, 1, NULL, 1);
}

void loop() {}
