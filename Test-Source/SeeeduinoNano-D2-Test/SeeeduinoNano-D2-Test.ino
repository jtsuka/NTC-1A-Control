//
// Pin D2 の信号テスト用 200ms毎にHIGH LOW
// 2025.6.13

#define TEST_PIN 2

void setup() {
  pinMode(TEST_PIN, OUTPUT);
}

void loop() {
  digitalWrite(TEST_PIN, HIGH);
  delay(200);  // 200ms HIGH
  digitalWrite(TEST_PIN, LOW);
  delay(200);  // 200ms LOW
}
