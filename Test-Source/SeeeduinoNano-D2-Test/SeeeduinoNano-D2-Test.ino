//
// 200m 毎に Pin2の信号をHIGH->LOWを繰り返す（信号テスト用)
//
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
