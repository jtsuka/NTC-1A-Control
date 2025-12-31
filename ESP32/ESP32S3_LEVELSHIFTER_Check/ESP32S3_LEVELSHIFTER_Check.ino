#include <Arduino.h>

// ========================================================
// 1. チャッピー決定版ピン割り当て
// ========================================================
#define P_DIR1 4   // D3 (GPIO 4)  : 200ms 周期
#define P_OE1  5   // D4 (GPIO 5)  : 400ms 周期
#define P_DIR2 7   // D8 (GPIO 7)  : 600ms 周期
#define P_OE2  8   // D9 (GPIO 8)  : 800ms 周期
#define P_TXA  43  // D6 (GPIO 43) : 1000ms (1.0秒) 周期
#define P_RXA  44  // D7 (GPIO 44) : 1200ms (1.2秒) 周期
#define P_PI_RX 1  // D0 (GPIO 1)  : 1400ms (1.4秒) 周期
#define P_PI_TX 2  // D1 (GPIO 2)  : 1600ms (1.6秒) 周期

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 全てのピンを「出力」として設定（確認用）
  pinMode(P_DIR1, OUTPUT);
  pinMode(P_OE1,  OUTPUT);
  pinMode(P_DIR2, OUTPUT);
  pinMode(P_OE2,  OUTPUT);
  pinMode(P_TXA,  OUTPUT);
  pinMode(P_RXA,  OUTPUT);
  pinMode(P_PI_RX, OUTPUT);
  pinMode(P_PI_TX, OUTPUT);

  Serial.println("\n--- [FINAL PHYSICAL CHECK] Fingerprint Start ---");
  Serial.println("Measure periods on PulseView to identify pins:");
}

// ピンと周期を指定してパタパタさせる関数
void fingerPrint(int pin, uint32_t ms) {
  static uint32_t last[50] = {0}; 
  uint32_t now = millis();
  if (now - last[pin % 50] >= ms) {
    last[pin % 50] = now;
    digitalWrite(pin, !digitalRead(pin));
  }
}

void loop() {
  // 全てのピンに固有のリズム（指紋）を与える
  fingerPrint(P_DIR1, 200);
  fingerPrint(P_OE1,  400);
  fingerPrint(P_DIR2, 600);
  fingerPrint(P_OE2,  800);
  fingerPrint(P_TXA,  1000);
  fingerPrint(P_RXA,  1200);
  fingerPrint(P_PI_RX, 1400);
  fingerPrint(P_PI_TX, 1600);
}