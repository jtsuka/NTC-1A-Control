// FreeRTOS対応 OLED表示スケッチ（ESP32-S3 Grove I2C）
// display.display() は loop() 内でのみ実行

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define I2C_SDA 4
#define I2C_SCL 5

Adafruit_SSD1306 display(128, 64, &Wire, -1);

typedef struct {
  String line1;
  String line2;
  bool isTop;
} OledMessage;

QueueHandle_t oledQueue;
bool shouldUpdateDisplay = false;

class OLEDLogger {
public:
  void init() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      Serial.println("!!! OLED初期化失敗 !!!");
      while (1);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("OLED Ready");
    display.display();
  }

  void drawTop(const String& line1, const String& line2) {
    display.fillRect(0, 0, 128, 32, BLACK);
    display.setCursor(0, 0);
    display.println(line1);
    display.println(line2);
    shouldUpdateDisplay = true;
  }

  void drawBottom(const String& line1, const String& line2) {
    display.fillRect(0, 32, 128, 32, BLACK);
    display.setCursor(0, 32);
    display.println(line1);
    display.println(line2);
    shouldUpdateDisplay = true;
  }
};

OLEDLogger oled;

void oledTask(void* pv) {
  OledMessage msg;
  for (;;) {
    if (xQueueReceive(oledQueue, &msg, portMAX_DELAY)) {
      if (msg.isTop) oled.drawTop(msg.line1, msg.line2);
      else oled.drawBottom(msg.line1, msg.line2);
    }
  }
}

void sendToOLEDQueue(const String& l1, const String& l2, bool isTop) {
  OledMessage msg = { l1, l2, isTop };
  xQueueSend(oledQueue, &msg, 0);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Wire.begin(I2C_SDA, I2C_SCL);
  oled.init();

  oledQueue = xQueueCreate(8, sizeof(OledMessage));
  xTaskCreatePinnedToCore(oledTask, "OLEDTask", 4096, NULL, 1, NULL, 1);

  sendToOLEDQueue("ESP32 OLED", "Logger Ready", true);
  sendToOLEDQueue("FreeRTOS Task OK", "", false);
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

  // display() は loop() 内でのみ呼び出す
  if (shouldUpdateDisplay) {
    display.display();
    shouldUpdateDisplay = false;
  }
}
