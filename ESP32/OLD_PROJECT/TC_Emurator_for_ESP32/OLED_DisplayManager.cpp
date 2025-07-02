#include "OLED_DisplayManager.h"

OLED_DisplayManager::OLED_DisplayManager(uint8_t width, uint8_t height, TwoWire* wire, int8_t reset_pin)
: display(width, height, wire, reset_pin) {
  mutex = xSemaphoreCreateMutex();
}

void OLED_DisplayManager::begin(uint8_t address) {
  display.begin(SSD1306_SWITCHCAPVCC, address);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
}

void OLED_DisplayManager::clearAll() {
  if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    display.clearDisplay();
    display.display();
    xSemaphoreGive(mutex);
  }
}

void OLED_DisplayManager::printHexLine(const char* label, const uint8_t* data, uint8_t row) {
  if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    display.setCursor(0, row * 8);
    display.printf("%s:", label);
    for (int i = 0; i < 6; i++) {
      display.printf(" %02X", data[i]);
    }
    display.display();
    xSemaphoreGive(mutex);
  }
}

void OLED_DisplayManager::printMessage(const char* msg, uint8_t row) {
  if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    display.setCursor(0, row * 8);
    display.print(msg);
    display.display();
    xSemaphoreGive(mutex);
  }
}
