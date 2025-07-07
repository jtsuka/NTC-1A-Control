// ESP32-S3 + FreeRTOS + U8g2 OLED表示安定版
// Grove I2C (SDA=4, SCL=5)
// for u8g2 版
// 生成日: 2025-07-07

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ==== OLED: U8g2 ハードウェアI2C ==== 
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ==== Grove I2Cピン定義（GPIO4=SDA, GPIO5=SCL） ====
#define I2C_SDA 4
#define I2C_SCL 5

// ==== OLED表示メッセージ構造体 ====
typedef struct {
  String line1;
  String line2;
  bool isTop;
} OledMessage;

QueueHandle_t oledQueue;

// ==== OLED描画バッファ更新タスク ====
void oledTask(void* pv) {
  OledMessage msg;
  for (;;) {
    if (xQueueReceive(oledQueue, &msg, portMAX_DELAY)) {
      u8g2.clearBuffer();

      if (msg.isTop) {
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 12, msg.line1.c_str());
        u8g2.drawStr(0, 24, msg.line2.c_str());
      } else {
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 44, msg.line1.c_str());
        u8g2.drawStr(0, 56, msg.line2.c_str());
      }
      // バッファ送信は loop() に任せる
    }
  }
}

// ==== キュー送信ユーティリティ ====
void sendToOLEDQueue(const String& l1, const String& l2, bool isTop) {
  OledMessage msg = { l1, l2, isTop };
  xQueueSend(oledQueue, &msg, 0);
}

// ==== SETUP ====
void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  delay(100);

  u8g2.begin();

  oledQueue = xQueueCreate(8, sizeof(OledMessage));
  xTaskCreatePinnedToCore(oledTask, "OLEDTask", 4096, NULL, 1, NULL, 1);

  sendToOLEDQueue("U8g2 OLED READY", "ESP32-S3 OK", true);
  sendToOLEDQueue("Task started", "Queue active", false);
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

  // OLED描画更新はここでまとめて処理（FreeRTOS非干渉）
  u8g2.sendBuffer();
}
