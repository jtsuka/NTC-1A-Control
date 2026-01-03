#include <Arduino.h>

/*
  =========================================================
  [FINAL PHYSICAL CHECK] Fingerprint Sketch (RX also OUTPUT)
  ---------------------------------------------------------
  目的：
    ・ユニバーサル基板の配線/ピン取り違いを PulseView の「周期」で確定
    ・DIR/OE/TX/RX すべてに固有周期を割り当てて迷わないようにする

  重要：
    ※ RXA(GPIO44) と PI_RX(GPIO1) を OUTPUT でトグルします。
    ※ 外部回路（Pi/TC/レベルシフタ等）を必ず外した状態で実行してください。

  決定版ピン（固定）：
    OE1  = GPIO5
    DIR1 = GPIO6
    OE2  = GPIO7
    DIR2 = GPIO8
    TXA  = GPIO43
    RXA  = GPIO44  ★今回OUTPUTでトグル
    PI_TX= GPIO2
    PI_RX= GPIO1   ★今回OUTPUTでトグル

  PulseViewで見える「周期」指紋：
    DIR1(GPIO6)  : 200ms
    OE1 (GPIO5)  : 400ms
    DIR2(GPIO8)  : 600ms
    OE2 (GPIO7)  : 800ms
    TXA (GPIO43) : 1000ms
    RXA (GPIO44) : 1200ms
    PI_RX(GPIO1) : 1400ms
    PI_TX(GPIO2) : 1600ms
  =========================================================
*/

// ========================================================
// 1) 決定版ピン割り当て（固定）
// ========================================================
#define P_OE1    5
#define P_DIR1   6
#define P_OE2    7
#define P_DIR2   8
#define P_TXA   43
#define P_RXA   44
#define P_PI_RX  1
#define P_PI_TX  2

// ========================================================
// 2) 指紋トグル関数（周期ごとに反転）
// ========================================================
void toggleEvery(uint32_t &last, int pin, uint32_t period_ms) {
  uint32_t now = millis();
  if (now - last >= period_ms) {
    last = now;
    digitalWrite(pin, !digitalRead(pin));
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // すべて OUTPUT（指紋対象）
  pinMode(P_DIR1,  OUTPUT);
  pinMode(P_OE1,   OUTPUT);
  pinMode(P_DIR2,  OUTPUT);
  pinMode(P_OE2,   OUTPUT);
  pinMode(P_TXA,   OUTPUT);
  pinMode(P_RXA,   OUTPUT);   // ★RXも出力
  pinMode(P_PI_RX, OUTPUT);   // ★RXも出力
  pinMode(P_PI_TX, OUTPUT);

  // 初期値（LOWスタート）
  digitalWrite(P_DIR1,  LOW);
  digitalWrite(P_OE1,   LOW);
  digitalWrite(P_DIR2,  LOW);
  digitalWrite(P_OE2,   LOW);
  digitalWrite(P_TXA,   LOW);
  digitalWrite(P_RXA,   LOW);
  digitalWrite(P_PI_RX, LOW);
  digitalWrite(P_PI_TX, LOW);

  Serial.println();
  Serial.println("--- [FINAL PHYSICAL CHECK] Fingerprint Start (RX OUTPUT) ---");
  Serial.println("Check periods on PulseView:");
  Serial.println("DIR1=200, OE1=400, DIR2=600, OE2=800, TXA=1000, RXA=1200, PI_RX=1400, PI_TX=1600 (ms)");
}

void loop() {
  static uint32_t t_dir1 = 0;
  static uint32_t t_oe1  = 0;
  static uint32_t t_dir2 = 0;
  static uint32_t t_oe2  = 0;
  static uint32_t t_txa  = 0;
  static uint32_t t_rxa  = 0;
  static uint32_t t_pirx = 0;
  static uint32_t t_pitx = 0;

  toggleEvery(t_dir1, P_DIR1,  200);   // DIR1(GPIO6)  : 200ms
  toggleEvery(t_oe1,  P_OE1,   400);   // OE1 (GPIO5)  : 400ms
  toggleEvery(t_dir2, P_DIR2,  600);   // DIR2(GPIO8)  : 600ms
  toggleEvery(t_oe2,  P_OE2,   800);   // OE2 (GPIO7)  : 800ms
  toggleEvery(t_txa,  P_TXA,  1000);   // TXA (GPIO43) : 1000ms
  toggleEvery(t_rxa,  P_RXA,  1200);   // RXA (GPIO44) : 1200ms
  toggleEvery(t_pirx, P_PI_RX,1400);   // PI_RX(GPIO1) : 1400ms
  toggleEvery(t_pitx, P_PI_TX,1600);   // PI_TX(GPIO2) : 1600ms
}
