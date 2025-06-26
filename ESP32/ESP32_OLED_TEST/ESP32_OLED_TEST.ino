#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SDA_PIN        7  // Grove J2のSDA（黄）
#define SCL_PIN        6  // Grove J2のSCL（白）
#define OLED_ADDR   0x3C  // 通常のI2Cアドレス

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("OLED Init...");

  // OLED初期化チェック（WDT回避対策）
  bool oled_ok = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oled_ok) {
    Serial.println("OLED init failed! Continuing without OLED.");
    return;  // setup終了（loop() は動作し続ける）
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED OK");
  display.display();
}

void loop() {
  // WDTタイムアウト防止のため delay を使う
  delay(1000);
}
