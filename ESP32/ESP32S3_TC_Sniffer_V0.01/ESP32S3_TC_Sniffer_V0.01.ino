// TC106 OC line pulse checker (ESP32-S3)
// TP1 -> GPIOxx, with 4.7k~10k pull-up to 3.3V
constexpr int PIN_TC = 4;   // ←あなたの配線GPIOに変更

volatile uint32_t fall_cnt = 0;

void IRAM_ATTR isr_fall() {
  fall_cnt++;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TC, INPUT);                 // 外付けプルアップ前提
  attachInterrupt(digitalPinToInterrupt(PIN_TC), isr_fall, FALLING);
}

void loop() {
  static uint32_t prev = 0;
  uint32_t now = fall_cnt;
  if (now != prev) {
    Serial.printf("FALLING edges: %lu\n", (unsigned long)now);
    prev = now;
  }
  delay(200);
}
