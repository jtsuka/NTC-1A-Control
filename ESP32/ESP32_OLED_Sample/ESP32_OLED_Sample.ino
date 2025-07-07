// OLED Adafruit_SSD1306 FreeRTOS対応(マルチスレッド)
// for ESP32S3
// Initial 2025.07.07

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ==== OLEDLogger クラス定義 ====
class OLEDLogger {
private:
  Adafruit_SSD1306* display;
  SemaphoreHandle_t mutex;

public:
  OLEDLogger(Adafruit_SSD1306* disp) : display(disp) {
    mutex = xSemaphoreCreateMutex();
  }

  bool begin() {
    if (!display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      return false;
    }
    display->clearDisplay();
    display->display();
    return true;
  }

  void updateTop(const String& line1, const String& line2) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
      display->fillRect(0, 0, 128, 32, BLACK);
      display->setTextSize(1);
      display->setTextColor(WHITE);
      display->setCursor(0, 0);
      display->println(line1);
      display->println(line2);
      display->display();
      xSemaphoreGive(mutex);
    }
  }

  void updateBottom(const String& line1, const String& line2) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
      display->fillRect(0, 32, 128, 32, BLACK);
      display->setTextSize(1);
      display->setTextColor(WHITE);
      display->setCursor(0, 32);
      display->println(line1);
      display->println(line2);
      display->display();
      xSemaphoreGive(mutex);
    }
  }
};

// ==== OLED通信ピン（Grove Shield）====
#define I2C_SDA 4
#define I2C_SCL 5

// ==== グローバルポインタ ====
Adafruit_SSD1306* display = nullptr;
OLEDLogger* oled = nullptr;

QueueHandle_t oledQueue;

// ==== OLEDログメッセージ構造体 ====
typedef struct {
  String line1;
  String line2;
  bool isTop;
} OledMessage;

// ==== OLEDタスク ====
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

// ==== OLEDキュー送信ユーティリティ ====
void sendToOLEDQueue(const String& l1, const String& l2, bool isTop) {
  OledMessage msg = { l1, l2, isTop };
  xQueueSend(oledQueue, &msg, 0);
}

// ==== SETUP ====
void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);  // 必ず最初
  Serial.begin(115200);
  delay(100);

  display = new Adafruit_SSD1306(128, 64, &Wire, -1);
  oled = new OLEDLogger(display);

  if (!oled->begin()) {
    Serial.println("OLED init failed");
    while (1); // stop here
  }

  oledQueue = xQueueCreate(8, sizeof(OledMessage));
  xTaskCreatePinnedToCore(oledTask, "OLEDTask", 4096, NULL, 1, NULL, 1);

  sendToOLEDQueue("ESP32 OLED TEST", "Logger Ready", true);
  sendToOLEDQueue("Task Started", "", false);
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
