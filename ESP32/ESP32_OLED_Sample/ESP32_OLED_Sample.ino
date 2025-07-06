#include <Arduino.h>
#include "OLEDLogger.h"

// ==== 表示メッセージ構造体とキュー定義 ====

typedef struct {
  String line1;
  String line2;
  bool isTop;  // true = 上段, false = 下段
} OledMessage;

QueueHandle_t oledQueue;
OLEDLogger oled;  // インスタンス生成

// ==== OLED描画タスク ====
void oledTask(void *pv) {
  OledMessage msg;
  for (;;) {
    if (xQueueReceive(oledQueue, &msg, portMAX_DELAY) == pdTRUE) {
      if (msg.isTop) {
        oled.updateTop(msg.line1, msg.line2);
      } else {
        oled.updateBottom(msg.line1, msg.line2);
      }
    }
  }
}

// ==== キュー送信用ユーティリティ関数 ====
void sendToOLEDQueue(const String& l1, const String& l2, bool isTop) {
  OledMessage msg = { l1, l2, isTop };
  xQueueSend(oledQueue, &msg, 0);  // 即時送信（失敗時は無視）
}

// ==== セットアップ ====
void setup() {
  Serial.begin(115200);

  oled.begin();  // OLED初期化
  oledQueue = xQueueCreate(8, sizeof(OledMessage));
  xTaskCreatePinnedToCore(oledTask, "OLEDTask", 4096, NULL, 1, NULL, 1);

  sendToOLEDQueue("ESP32 OLED TEST", "Logger Ready", true);   // 上段に表示
  sendToOLEDQueue("Loop Starts...", "", false);               // 下段に表示
}

// ==== メインループ：5秒おきに表示切替 ====
void loop() {
  static uint32_t last = 0;
  static bool toggle = false;

  if (millis() - last > 5000) {
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
