#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Grove Shield for Seeed XIAO ESP32S3 の I2Cピン
#define I2C_SDA 4
#define I2C_SCL 5

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== I2C OLED 初期化チェック ===");

  // 念のためWireを明示的に終了・再開
  Wire.end();
  delay(50);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  // 通常の100kHz
  delay(200);             // OLEDのI2C安定化待ち

  // I2Cアドレススキャン（範囲指定）
  Serial.println("[I2Cスキャン開始]");
  for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2Cデバイス検出: 0x");
      Serial.println(addr, HEX);
    }
  }
  Serial.println("[スキャン完了]");

  // OLEDの初期化
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("!!! OLED初期化失敗 !!!");
    while (1);  // ハングしてエラー報告待ち
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("OLED 0x3C Detected");
  display.println("Init Successful!");
  display.display();
  Serial.println("OLED初期化 成功！画面に表示しています。");
}

void loop() {
  // 何もしない
}
