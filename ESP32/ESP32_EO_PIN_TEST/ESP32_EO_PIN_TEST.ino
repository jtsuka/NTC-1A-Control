// ---------- GPIO5 Stand-Alone Test (ESP32-S3) ----------
// 1 Hz で LOW→HIGH を交互出力し、シリアルにも状態を表示します。
//   ・LED で確認: GPIO5 ↔ LED ↔ 330 Ω ↔ GND
//   ・テスター/ロジアナ: GPIO5 と GND 間の電圧 or 波形を見る
// --------------------------------------------------------

const int TEST_PIN = 5;      // GPIO番号を変えたい場合はここを変更
const unsigned long PERIOD_MS = 1000;   // トグル周期 (1 Hz)

void setup() {
  Serial.begin(115200);
  delay(200);                    // シリアル安定待ち

  pinMode(TEST_PIN, OUTPUT);     // プッシュプル出力
  digitalWrite(TEST_PIN, LOW);   // 初期状態＝LOW

  Serial.println("=== GPIO5 Stand-Alone Toggle Test ===");
}

void loop() {
  static bool level = false;     // false=LOW, true=HIGH
  static unsigned long t0 = 0;

  unsigned long now = millis();
  if (now - t0 >= PERIOD_MS) {
    level = !level;                          // トグル
    digitalWrite(TEST_PIN, level ? HIGH : LOW);
    Serial.printf("GPIO5 is now: %s\n", level ? "HIGH" : "LOW");
    t0 = now;
  }
}
