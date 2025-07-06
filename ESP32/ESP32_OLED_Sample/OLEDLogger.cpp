#include "OLEDLogger.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define I2C_SDA_PIN    4
#define I2C_SCL_PIN    5

#define LINE_Y(n) (n * 16)

// OLED I2C bus
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

OLEDLogger::OLEDLogger() {
  oledMutex = xSemaphoreCreateMutex();
}

void OLEDLogger::begin() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // 標準インスタンスで初期化
  Wire.setClock(400000);                // 必要に応じてクロック変更
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  display.clearDisplay();
  display.display();
}

void OLEDLogger::drawLine(int y, const String& text) {
  display.setCursor(0, y);
  display.println(text);
}

void OLEDLogger::clearLines(int yStart, int lines) {
  display.fillRect(0, yStart, SCREEN_WIDTH, 16 * lines, SSD1306_BLACK);
}

void OLEDLogger::updateTop(const String& l1, const String& l2) {
  if (xSemaphoreTake(oledMutex, portMAX_DELAY) == pdTRUE) {
    topLine1 = l1;
    topLine2 = l2;
    clearLines(0, 2);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    drawLine(0, topLine1);
    drawLine(16, topLine2);
    display.display();
    xSemaphoreGive(oledMutex);
  }
}

void OLEDLogger::updateBottom(const String& l1, const String& l2) {
  if (xSemaphoreTake(oledMutex, portMAX_DELAY) == pdTRUE) {
    bottomLine1 = l1;
    bottomLine2 = l2;
    clearLines(32, 2);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    drawLine(32, bottomLine1);
    drawLine(48, bottomLine2);
    display.display();
    xSemaphoreGive(oledMutex);
  }
}

void OLEDLogger::clearAll() {
  if (xSemaphoreTake(oledMutex, portMAX_DELAY) == pdTRUE) {
    display.clearDisplay();
    display.display();
    xSemaphoreGive(oledMutex);
  }
}

void OLEDLogger::redrawAll() {
  if (xSemaphoreTake(oledMutex, portMAX_DELAY) == pdTRUE) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    drawLine(0, topLine1);
    drawLine(16, topLine2);
    drawLine(32, bottomLine1);
    drawLine(48, bottomLine2);
    display.display();
    xSemaphoreGive(oledMutex);
  }
}
