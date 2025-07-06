#ifndef OLED_LOG_H
#define OLED_LOG_H

#include <Adafruit_SSD1306.h>
#include "config.h"

void initOLED();
void logLineTop(const String& msg);
void logLineBottom(const String& msg);

#endif
