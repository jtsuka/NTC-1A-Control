#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "SafeOLED.h"

// OLED定義
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 rawDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SafeOLED oled(&rawDisplay);

typedef struct {
  char line1[32];
  char line2[32];
  char status[16];
} OledMessage;

QueueHandle_t oledQueue;

// OLED描画専用タスク
void oledTask(void* param) {
  OledMessage msg;
  while (1) {
    if (xQueueReceive(oledQueue, &msg, portMAX_DELAY) == pdTRUE) {
      oled.drawText(msg.line1, msg.line2, msg.status);
    }
  }
}

// 描画リクエスト用関数
void requestOledDraw(const char* l1, const char* l2, const char* st) {
  OledMessage msg;
  strncpy(msg.line1, l1, sizeof(msg.line1));
  strncpy(msg.line2, l2, sizeof(msg.line2));
  strncpy(msg.status, st, sizeof(msg.status));
  xQueueSend(oledQueue, &msg, 0);
}

// テスト用：1.5秒ごとに表示更新
void testTask(void* param) {
  int counter = 0;
  char line1[32], line2[32], status[16];
  while (1) {
    snprintf(line1, sizeof(line1), "Hello OLED");
    snprintf(line2, sizeof(line2), "Counter: %d", counter++);
    snprintf(status, sizeof(status), "Status: OK");
    requestOledDraw(line1, line2, status);
    vTaskDelay(pdMS_TO_TICKS(1500));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  oled.begin();

  oledQueue = xQueueCreate(5, sizeof(OledMessage));

  xTaskCreatePinnedToCore(oledTask, "OledTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(testTask, "TestTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // 未使用
}
