// ESP32-S3 + Grove OLED 安定動作スケッチ
// 描画バッファはFreeRTOSタスクで更新、display()はloop()で呼び出し
// 2025.07.07 12:00 ver

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2Cピン設定（Grove Shield）
#define I2C_SDA 4
#define I2C_SCL 5

// OLED表示構造体
typedef struct {
  String line1;
  String line2;
  bool isTop;
} OledMessage;

// グローバルインスタンスとフラグ
Adafruit_SSD1306 display(128, 64, &Wire, -1);
QueueHandle_t oledQueue;
bool shouldUpdateDisplay = false;

// 描画バッファ更新関数
void updateOLED(const String& l1, const String& l2, bool top) {
  if (top) {
    display.fillRect(0, 0, 128, 32, BLACK);
    display.setCursor(0, 0);
  } else {
    display.fillRect(0, 32, 128, 32, BLACK);
    display.setCursor(0, 32);
  }
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println(l1);
  display.println(l2);
  shouldUpdateDisplay = true;
}

// OLED描画タスク（バッファだけ更新）
void oledTask(void* pv) {
  OledMessage msg;
  for (;;) {
    if (xQueueReceive(oledQueue, &msg, portMAX_DELAY)) {
      updateOLED(msg.line1, msg.line2, msg.isTop);
    }
  }
}

// OLEDキュー送信用ユーティリティ
void sendToOLEDQueue(const String& l1, const String& l2, bool isTop) {
  OledMessage msg = { l1, l2, isTop };
  xQueueSend(oledQueue, &msg, 0);
}

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  delay(100);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (1);
  }
  display.clearDisplay();

  oledQueue = xQueueCreate(8, sizeof(OledMessage));
  xTaskCreatePinnedToCore(oledTask, "OLEDTask", 4096, NULL, 1, NULL, 1);

  sendToOLEDQueue("ESP32 OLED", "Logger Ready", true);
  sendToOLEDQueue("FreeRTOS OK", "", false);
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

  // I2C display()はloop内でのみ実行
  if (shouldUpdateDisplay) {
    display.display();
    shouldUpdateDisplay = false;
  }
}
