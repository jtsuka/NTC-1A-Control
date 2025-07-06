#include "oled_log.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void initOLED() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("OLED Ready");
  display.display();
}

void logLineTop(const String& msg) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(msg);
  display.display();
}

void logLineBottom(const String& msg) {
  display.setCursor(0, 16);
  display.print(msg);
  display.display();
}
