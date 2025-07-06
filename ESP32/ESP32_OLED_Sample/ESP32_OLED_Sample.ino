#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==== OLEDLoggerクラス定義（クラス化・インスタンス1つのみ） ====

class OLEDLogger {
private:
  Adafruit_SSD1306 display;
  SemaphoreHandle_t mutex;

public:
  OLEDLogger() : display(128, 64, &Wire, -1) {
    mutex = xSemaphoreCreateMutex();
  }

  bool begin() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      return false;
    }
    display.clearDisplay();
    display.display();
    return true;
  }

  void updateTop(const String& line1, const String& line2) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
      display.fillRect(0, 0, 128, 32, BLACK);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.println(line1);
      display.println(line2);
      display.display();
      xSemaphoreGive(mutex);
    }
  }

  void updateBottom(const String& line1, const String& line2) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
      display.fillRect(0, 32, 128, 32, BLACK);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 32);
      display.println(line1);
      display.println(line2);
      display.display();
      xSemaphoreGive(mutex);
    }
  }
};

// ==== 定義と初期化 ====

#define I2C_SDA 4
#define I2C_SCL 5

OLEDLogger oled;
QueueHandle_t oledQueue;

typedef struct {
  String line1;
  String line2;
  bool isTop;  // true=上段, false=下段
} OledMessage;

// ==== OLED描画タスク ====
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

// ==== キュー送信ユーティリティ ====
void sendToOLEDQueue(const String& l1, const String& l2, bool isTop) {
  OledMessage msg = { l1, l2, isTop };
  xQueueSend(oledQueue, &msg, 0);  // 非ブロッキング
}

// ==== セットアップ ====
void setup() {
  Wire.begin(I2C_SDA, I2C_SCL, 400000);  // Grove Shield対応
  Serial.begin(115200);
  delay(200);

  if (!oled.begin()) {
    Serial.println("OLED init failed");
    while (1);
  }

  oledQueue = xQueueCreate(8, sizeof(OledMessage));
  xTaskCreatePinnedToCore(oledTask, "OLEDTask", 4096, NULL, 1, NULL, 1);

  sendToOLEDQueue("ESP32 OLED TEST", "Logger Ready", true);
  sendToOLEDQueue("Loop Starts...", "", false);
}

// ==== メインループ ====
void loop() {
  static uint32_t last = 0;
  static bool toggle = false;

  if (millis() - last > 4000) {
    if (toggle) {
      sendToOLEDQueue("REPEATER MODE", "GPIO8 = HIGH", true);
      sendToOLEDQueue("Recv: 01 06", "ACK OK", false);
    } else {
      sendToOLEDQueue("EMULATOR MODE", "Echo Ready", true);
      sendToOLEDQueue("Recv: 02 05", "ACK 0C", false);
    }
    toggle = !toggle;
    last = millis();
  }
}
