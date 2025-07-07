// メインスケッチ - モード検出とタスク起動

#include "config.h"
#include "oled_log.h"
#include "task_bridge.h"
#include "task_emulator.h"
#include "task_led_oled.h"

void setup() {
  Serial.begin(115200);
  delay(200);
  WiFi.mode(WIFI_MODE_NULL);  // MAC取得のみ
  String mac = WiFi.macAddress();

  pinMode(TEST_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  initOLED();
  logLineTop("Booting...");
  logLineBottom(mac);

  currentMode = (mac == EMULATOR_MAC) ? MODE_EMULATOR : MODE_REPEATER;
  if (currentMode == MODE_EMULATOR) {
    logLineTop("Mode: EMULATOR");
    startEmulatorTasks();
  } else {
    logLineTop("Mode: REPEATER");
    startRepeaterTasks();
  }

  startLEDOLEDTask();  // LED/OLED制御開始
}

void loop() {
  delay(100);
}
