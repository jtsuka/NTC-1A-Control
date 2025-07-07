// ESP32_TC_EmuRepeater_Integrated.ino
// 統合版スケッチ - EMULATOR/REPEATER 切替対応

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// OLED 定義
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// GPIO 定義
#define TEST_PIN     8
#define LED_PIN      10

// MACアドレスによるモード切替
#define EMULATOR_MAC "D8:3B:DA:74:82:78"  // 新ESP32S3
String localMAC;

enum Mode { MODE_REPEATER, MODE_EMULATOR };
Mode currentMode;

void logLineTop(const String& msg) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.print(msg);
  display.display();
}

void logLineBottom(const String& msg) {
  display.setCursor(0, 16);
  display.print(msg);
  display.display();
}

// ===== REPEATERタスク（仮）
void startRepeaterTasks() {
  logLineBottom("Repeater Ready");
}

// ===== EMULATORタスク（仮）
void startEmulatorTasks() {
  logLineBottom("Emulator Ready");
}

// ===== LED/OLED制御タスク
void startLEDOLEDTask() {
  xTaskCreatePinnedToCore([](void*){
    while (true) {
      int sw = digitalRead(TEST_PIN);
      if (sw == LOW) {
        digitalWrite(LED_PIN, LOW);
      } else {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      }
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }, "LEDTask", 2048, nullptr, 1, nullptr, 1);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  localMAC = WiFi.macAddress();

  pinMode(TEST_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  logLineTop("Booting...");
  logLineBottom(localMAC);

  if (localMAC == EMULATOR_MAC) {
    currentMode = MODE_EMULATOR;
    logLineTop("Mode: EMULATOR");
    startEmulatorTasks();
  } else {
    currentMode = MODE_REPEATER;
    logLineTop("Mode: REPEATER");
    startRepeaterTasks();
  }

  startLEDOLEDTask();
}

void loop() {
  delay(100);
}
