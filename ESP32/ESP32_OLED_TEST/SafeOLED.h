#ifndef SAFE_OLED_H
#define SAFE_OLED_H

#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <freertos/semphr.h>

class SafeOLED {
private:
  Adafruit_SSD1306* display;
  SemaphoreHandle_t mutex;

public:
  SafeOLED(Adafruit_SSD1306* disp) {
    display = disp;
    mutex = xSemaphoreCreateMutex();
  }

  void begin() {
    display->begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display->clearDisplay();
    display->display();
  }

  void drawText(const char* line1, const char* line2, const char* status) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
      display->clearDisplay();
      display->setCursor(0, 0);
      display->setTextSize(1);
      display->setTextColor(SSD1306_WHITE);
      display->println(line1);
      display->println(line2);
      display->println(status);
      display->display();
      xSemaphoreGive(mutex);
    }
  }
};

#endif
