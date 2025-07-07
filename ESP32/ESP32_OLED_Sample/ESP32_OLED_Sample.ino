// OLED Adafruit_SSD1306 + FreeRTOS 対応（安定版）
// for ESP32-S3 + Grove Shield（I2C: SDA=4, SCL=5）
// 生成日: 2025-07-07

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ==== Grove Shield I2Cピン定義 ====
#define I2C_SDA 4
#define I2C_SCL 5

// ==== グローバルポインタ（初期はnull） ====
Adafruit_SSD1306* display = nullptr;
class OLEDLogger;
OLEDLogger* oled = nullptr;

// ==== OLED出力キュー構造体 ====
typedef struct {
  String line1;
  String line2;
  bool isTop;
} OledMessage;

QueueHandle_t oledQueue;

// ==== OLEDLoggerクラス定義 ====
class OLEDLogger {
private:
  Adafruit_SSD1306* disp;
  SemaphoreHandle_t mutex;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

public:
  OLEDLogger(Adafruit_SSD1306* d) : disp(d) {
    mutex = xSemaphoreCreateMutex();
  }

  bool begin() {
    if (!disp->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      return false;
    }
    disp->clearDisplay();
    taskENTER_CRITICAL(&mux);
    display->display();
    taskEXIT_CRITICAL(&mux);
    return true;
  }

  void updateTop(const String& line1, const String& line2) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
      disp->fillRect(0, 0, 128, 32, BLACK);
      disp->setTextSize(1);
      disp->setTextColor(WHITE);
      disp->setCursor(0, 0);
      disp->println(line1);
      disp->println(line2);
      taskENTER_CRITICAL(&mux);
      display->display();
      taskEXIT_CRITICAL(&mux);
      xSemaphoreGive(mutex);
    }
  }

  void updateBottom(const String& line1, const String& line2) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
      disp->fillRect(0, 32, 128, 32, BLACK);
      disp->setTextSize(1);
      disp->setTextColor(WHITE);
      disp->setCursor(0, 32);
      disp->println(line1);
      disp->println(line2);
      taskENTER_CRITICAL(&mux);
      display->display();
      taskEXIT_CRITICAL(&mux);
      xSemaphoreGive(mutex);
    }
  }
};

// ==== OLED描画専用タスク ====
void oledTask(void* pv) {
  OledMessage msg;
  for (;;) {
    if (xQueueReceive(oledQueue, &msg, portMAX_DELAY)) {
      if (oled) {
        if (msg.isTop) oled->updateTop(msg.line1, msg.line2);
        else oled->updateBottom(msg.line1, msg.line2);
      }
    }
  }
}

// ==== OLED描画要求ユーティリティ ====
void sendToOLEDQueue(const String& l1, const String& l2, bool isTop) {
  OledMessage msg = { l1, l2, isTop };
  xQueueSend(oledQueue, &msg, 0);
}

// ==== SETUP ====
void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);  // 最初にWireを初期化
  Serial.begin(115200);
  delay(100);

  // OLEDインスタンス生成とクラス初期化
  display = new Adafruit_SSD1306(128, 64, &Wire, -1);
  oled = new OLEDLogger(display);

  if (!oled->begin()) {
    Serial.println("OLED init failed");
    while (1);
  }

  oledQueue = xQueueCreate(8, sizeof(OledMessage));
  xTaskCreatePinnedToCore(oledTask, "OLEDTask", 4096, NULL, 1, NULL, 1);

  sendToOLEDQueue("ESP32 OLED TEST", "Logger Ready", true);
  sendToOLEDQueue("FreeRTOS Task OK", "", false);
}

// ==== LOOP ====
void loop() {
  static uint32_t last = 0;
  static bool toggle = false;

  if (millis() - last > 3000) {
    if (toggle) {
      sendToOLEDQueue("Mode: REPEATER", "GPIO8 = HIGH", true);
      sendToOLEDQueue("Recv: 01 06", "ACK OK", false);
    } else {
      sendToOLEDQueue("Mode: EMULATOR", "GPIO8 = LOW", true);
      sendToOLEDQueue("Recv: 02 05", "ACK 0C", false);
    }
    toggle = !toggle;
    last = millis();
  }
}
