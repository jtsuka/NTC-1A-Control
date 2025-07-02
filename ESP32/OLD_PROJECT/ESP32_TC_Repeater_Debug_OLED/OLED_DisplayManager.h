#ifndef OLED_DISPLAY_MANAGER_H
#define OLED_DISPLAY_MANAGER_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>
#include "freertos/semphr.h"

class OLED_DisplayManager {
public:
  OLED_DisplayManager(uint8_t width, uint8_t height, TwoWire* wire = &Wire, int8_t reset_pin = -1);
  void begin(uint8_t address = 0x3C);
  void printHexLine(const char* label, const uint8_t* data, uint8_t row);
  void printMessage(const char* msg, uint8_t row);
  void clearAll();

private:
  Adafruit_SSD1306 display;
  SemaphoreHandle_t mutex;
};

#endif
