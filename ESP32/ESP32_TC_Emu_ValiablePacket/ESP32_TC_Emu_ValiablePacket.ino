// ==============================
// TC Emulator with FreeRTOS-safe Logging (ESP32-S3)
// - 300bps BitBang RX/TX
// - FreeRTOS構成 / OLED初期化のみ
// - 可変長パケット対応
// - Serial出力にportMUX排他制御を追加
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
#define BIT_PAT         true    // LSB
#define BIT_DURATION_US (1000000 / BAUD_RATE)

#define MAX_PAYLOAD_SIZE 10
#define MAX_PACKET_SIZE  11  // cmd + payload(max10)

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ========== FreeRTOS Serial Mutex ==========
portMUX_TYPE serialMux = portMUX_INITIALIZER_UNLOCKED;

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

// ========== 受信構造体 ==========
typedef struct {
  uint8_t cmd_id;
  uint8_t payload[MAX_PAYLOAD_SIZE];
  uint8_t length;
} CommandPacket;

QueueHandle_t cmdQueue;

// ビット反転（MSB → LSB変換）
uint8_t reverseBits(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

// ========== ログ出力 ==========
void logPacket(const char* label, const uint8_t* data, size_t len) {
  char buffer[128];
  int idx = snprintf(buffer, sizeof(buffer), "%s", label);

  for (size_t i = 0; i < len && idx < sizeof(buffer) - 4; ++i) {
    int written = snprintf(&buffer[idx], sizeof(buffer) - idx, " %02X", data[i]);
    if (written < 0 || written >= (int)(sizeof(buffer) - idx)) break;
    idx += written;
  }
#if 0
  for (size_t i = 0; i < len && idx < sizeof(buffer) - 4; ++i) {
    idx += snprintf(&buffer[idx], sizeof(buffer) - idx, " %02X", data[i]);
  }
#endif
  portENTER_CRITICAL(&serialMux);
  Serial.println(buffer);
  portEXIT_CRITICAL(&serialMux);
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

// MSB/LSB対応 送信関数
void sendPacket(const uint8_t* data, size_t len, bool lsbMode) {
  portENTER_CRITICAL(&serialMux);
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = lsbMode ? reverseBits(data[i]) : data[i];
    bitBangSendByte(b);
  }
  portEXIT_CRITICAL(&serialMux);
  delayMicroseconds(3000);  // パケット後の分離タイミング
}

#if 0
void sendPacket(const uint8_t* data, size_t len) {
  portENTER_CRITICAL(&serialMux);
  for (size_t i = 0; i < len; ++i) bitBangSendByte(data[i]);
  portEXIT_CRITICAL(&serialMux);
  delayMicroseconds(3000);  // パケット後の分離タイミング
}
#endif

#if 0
void sendPacket(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) bitBangSendByte(data[i]);
}
#endif

bool receiveByte(uint8_t* outByte) {
  while (digitalRead(TC_UART_RX_PIN) == HIGH) vTaskDelay(1);
//  delayMicroseconds(BIT_DURATION_US + BIT_DURATION_US / 2);
  delayMicroseconds(BIT_DURATION_US * 1.5);  // ← 中央補正をしっかり

  uint8_t b = 0;
  for (int i = 0; i < 8; ++i) {
    b |= (digitalRead(TC_UART_RX_PIN) << i);
    delayMicroseconds(BIT_DURATION_US);
  }
  delayMicroseconds(BIT_DURATION_US);

  *outByte = BIT_PAT ? reverseBits(b) : b;  // ← この行を修正
  return true;
}

void handleCommand(const CommandPacket& pkt, uint8_t* response, uint8_t* respLen) {
  switch (pkt.cmd_id) {
    case 0x01:
    case 0x02:
    case 0x03:
    case 0x04:
    case 0x05:
      // pkt.length + 1 は cmd_id + payload長
      response[0] = pkt.cmd_id;
      for (uint8_t i = 0; i < pkt.length; ++i) {
        response[1 + i] = pkt.payload[i];
      }
      *respLen = pkt.length + 1;
      break;

    default:
      *respLen = 0;
      break;
  }
}

#if 0
void handleCommand(const CommandPacket& pkt, uint8_t* response, uint8_t* respLen) {
  switch (pkt.cmd_id) {
    case 0x01:
      response[0] = 0x10;
      response[1] = 0x00;
      response[2] = 0x05;
      response[3] = 0xA0;
      response[4] = 0x00;
      response[5] = 0x7F;
      *respLen = 6;
      break;
    default:
      *respLen = 0;
      break;
  }
}
#endif

void TaskBitBangReceive(void* pvParameters) {
  for (;;) {

    if (digitalRead(TEST_PIN) == HIGH) {
      CommandPacket pkt = {0x01, {0x06, 0x05, 0x00, 0x00, 0x0C}, 5};
      xQueueSend(cmdQueue, &pkt, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    uint8_t cmd;
    if (!receiveByte(&cmd)) continue;
    int plen = lookupPayloadSize(cmd);
    if (plen < 0 || plen > MAX_PAYLOAD_SIZE) continue;

    CommandPacket pkt;
    pkt.cmd_id = cmd;
    pkt.length = plen;
    for (int i = 0; i < plen; ++i) {
      if (!receiveByte(&pkt.payload[i])) return;
    }

    logPacket("[RECV]", &pkt.cmd_id, 1);
    logPacket("[RECV-DATA]", pkt.payload, pkt.length);
    xQueueSend(cmdQueue, &pkt, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void TaskBitBangSend(void* pvParameters) {
  CommandPacket pkt;
  uint8_t response[MAX_PACKET_SIZE];
  uint8_t respLen = 0;

  for (;;) {
    if (xQueueReceive(cmdQueue, &pkt, portMAX_DELAY) == pdTRUE) {
      handleCommand(pkt, response, &respLen);
      if (respLen > 0) {
        logPacket("[SEND]", response, respLen);
        sendPacket(response, respLen, BIT_PAT);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
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
  delay(1000);

  cmdQueue = xQueueCreate(8, sizeof(CommandPacket));
  xTaskCreatePinnedToCore(TaskBitBangReceive, "Receive", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskBitBangSend, "Send", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskLED, "LED", 1024, NULL, 1, NULL, 1);
}

void loop() {
//  display.display();   // 1フレームの描画だけここで呼ぶ
  delay(10);           // 無駄なCPU消費を避ける
}
