#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// ==== OLEDLogger クラス定義 ====

class OLEDLogger {
public:
  OLEDLogger(TwoWire* wire) : _wire(wire), display(128, 64, wire, -1) {
    mutex = xSemaphoreCreateMutex();
  }

  bool begin() {
    _wire->begin(4, 5, 400000);  // SDA, SCL, Freq
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      return false;
    }
    display.clearDisplay();
    display.display();
    return true;
  }

  void updateTop(const String& l1, const String& l2) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.fillRect(0, 0, 128, 32, SSD1306_BLACK);
      display.setCursor(0, 0);
      display.println(l1);
      display.setCursor(0, 16);
      display.println(l2);
      display.display();
      xSemaphoreGive(mutex);
    }
  }

  void updateBottom(const String& l1, const String& l2) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.fillRect(0, 32, 128, 32, SSD1306_BLACK);
      display.setCursor(0, 32);
      display.println(l1);
      display.setCursor(0, 48);
      display.println(l2);
      display.display();
      xSemaphoreGive(mutex);
    }
  }

private:
  TwoWire* _wire;
  Adafruit_SSD1306 display;
  SemaphoreHandle_t mutex;
};

// ==== 構造体とグローバル定義 ====

typedef struct {
  String line1;
  String line2;
  bool isTop;
} OledMessage;

TwoWire I2CBus = TwoWire(0);
OLEDLogger oled(&I2CBus);
QueueHandle_t oledQueue;

// ==== タスク ====

void oledTask(void* pv) {
  OledMessage msg;
  while (true) {
    if (xQueueReceive(oledQueue, &msg, portMAX_DELAY) == pdTRUE) {
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

// ==== セットアップ ====

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!oled.begin()) {
    Serial.println("OLED init failed");
    while (1);
  }

  oledQueue = xQueueCreate(8, sizeof(OledMessage));
  xTaskCreatePinnedToCore(oledTask, "OLEDTask", 4096, NULL, 1, NULL, 1);

  sendToOLEDQueue("OLEDLogger Ready", "FreeRTOS OK", true);
  sendToOLEDQueue("System Starting...", "", false);
}

// ==== メインループ ====

void loop() {
  static uint32_t last = 0;
  static bool toggle = false;

  if (millis() - last > 4000) {
    if (toggle) {
      sendToOLEDQueue("EMULATOR MODE", "Echo Ready", true);
      sendToOLEDQueue("Recv: 02 05", "ACK 0C", false);
    } else {
      sendToOLEDQueue("REPEATER MODE", "GPIO8 = HIGH", true);
      sendToOLEDQueue("Recv: 01 06", "ACK OK", false);
    }
    toggle = !toggle;
    last = millis();
  }
}
