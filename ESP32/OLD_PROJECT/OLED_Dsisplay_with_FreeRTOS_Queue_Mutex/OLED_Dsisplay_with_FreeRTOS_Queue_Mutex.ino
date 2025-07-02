/**********************************************************************
  OLED CRC Display with FreeRTOS Queue + Mutex
  - ESP32 / XIAO ESP32S3 + SSD1306 I2C OLED
  - 受信パケット + CRCチェック結果を表示
**********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED定義
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// OLED描画データ構造体
typedef struct {
  char line1[32];
  char line2[32];
  char status[16];
} OledMessage;

QueueHandle_t oledQueue;
SemaphoreHandle_t oledMutex;

// ---------------------- CRC計算 ------------------------
uint8_t calcCRC(uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
  }
  return crc;
}

// ---------------------- 描画専用タスク ------------------------
void oledTask(void *param) {
  OledMessage msg;
  while (1) {
    if (xQueueReceive(oledQueue, &msg, portMAX_DELAY) == pdTRUE) {
      if (xSemaphoreTake(oledMutex, portMAX_DELAY) == pdTRUE) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.println(msg.line1);
        display.println(msg.line2);
        display.println(msg.status);
        display.display();
        xSemaphoreGive(oledMutex);
      }
    }
  }
}

// ---------------------- OLED描画リクエスト関数 ------------------------
void requestOledDraw(const char *line1, const char *line2, const char *status) {
  OledMessage msg;
  strncpy(msg.line1, line1, sizeof(msg.line1));
  strncpy(msg.line2, line2, sizeof(msg.line2));
  strncpy(msg.status, status, sizeof(msg.status));
  xQueueSend(oledQueue, &msg, 0);
}

// ---------------------- テスト用受信シミュレーションタスク ------------------------
void simulateRxTask(void *param) {
  const uint8_t packet[6] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C}; // 最後はCRC想定
  char line1[32], line2[32], status[16];

  while (1) {
    // バイナリを文字列化
    sprintf(line1, "RX Packet:");
    sprintf(line2, "%02X %02X %02X %02X %02X %02X",
            packet[0], packet[1], packet[2], packet[3], packet[4], packet[5]);

    uint8_t calc = calcCRC((uint8_t *)packet, 5);
    if (calc == packet[5]) {
      sprintf(status, "CRC OK");
    } else {
      sprintf(status, "CRC NG (%02X)", calc);
    }

    requestOledDraw(line1, line2, status);
    vTaskDelay(pdMS_TO_TICKS(1500)); // 1.5秒ごとに更新
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed!");
    while (1);
  }
  display.clearDisplay();
  display.display();

  oledQueue = xQueueCreate(5, sizeof(OledMessage));
  oledMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(oledTask, "OledTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(simulateRxTask, "RxTask", 4096, NULL, 1, NULL, 1);  // 仮想受信用
}

void loop() {
  // 未使用。タスクで制御。
}
