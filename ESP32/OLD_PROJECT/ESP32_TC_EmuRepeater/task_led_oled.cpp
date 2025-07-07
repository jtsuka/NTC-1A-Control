#include "task_led_oled.h"
#include "config.h"

ModeType currentMode;

void startLEDOLEDTask() {
  xTaskCreatePinnedToCore([](void*) {
    while (true) {
      int sw = digitalRead(TEST_PIN);
      if (sw == LOW) {
        digitalWrite(LED_PIN, LOW);
      } else {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // 点滅
      }
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }, "LEDTask", 2048, nullptr, 1, nullptr, 1);
}
