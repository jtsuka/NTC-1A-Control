#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

// OLEDピン設定（Grove Shield for XIAO ESP32S3）
#define I2C_SDA 4
#define I2C_SCL 5

// SSD1306インスタンスは外で作る（staticに1個だけ）
Adafruit_SSD1306 display(128, 64, &Wire, -1);
OLEDLogger oled(&display);  // クラスに渡す

QueueHandle_t oledQueue;

typedef struct {
  String line1;
  String line2;
  bool isTop;  // true = 上段, false = 下段
} OledMessage;

void oledTask(void* pv) {
  OledMessage msg;
  for (;;) {
    if (xQueueReceive(oledQueue, &msg, portMAX_DELAY)) {
      if (msg.isTop) {
        oled.updateTop(msg.line1, msg.line2);
      } else {
        oled.updateBottom(msg.line1, msg.line2);
      }
    }
  }
}

void sendToOLEDQueue(const String& l1, const String& l2, bool isTop) {
  OledMessage msg = { l1, l2, isTop };
  xQueueSend(oledQueue, &msg, 0);
}

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);  // 最初にWireを初期化
  Serial.begin(115200);
  delay(100);

  if (!oled.begin()) {
    Serial.println("OLED init failed");
    while (1);  // フリーズ
  }

  oledQueue = xQueueCreate(8, sizeof(OledMessage));
  xTaskCreatePinnedToCore(oledTask, "OLEDTask", 4096, NULL, 1, NULL, 1);

  sendToOLEDQueue("ESP32 OLED TEST", "Logger Ready", true);
  sendToOLEDQueue("Task Started", "", false);
}

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

