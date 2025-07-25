#define PIN_OE_A 5      // 1OE  (J5 白, GPIO5)
#define PIN_DIR_A 6     // 1DIR (J5 黄, GPIO6)
#define PIN_OE_B 7      // 2OE  (J7 白, GPIO7)
#define PIN_DIR_B 8     // 2DIR (J7 黄, GPIO8)

void setup() {
  pinMode(PIN_OE_A, OUTPUT);
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_OE_B, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);
}

void loop() {
  digitalWrite(PIN_OE_A, HIGH); delay(300);
  digitalWrite(PIN_OE_A, LOW ); delay(300);

  digitalWrite(PIN_DIR_A, HIGH); delay(300);
  digitalWrite(PIN_DIR_A, LOW ); delay(300);

  digitalWrite(PIN_OE_B, HIGH); delay(300);
  digitalWrite(PIN_OE_B, LOW ); delay(300);

  digitalWrite(PIN_DIR_B, HIGH); delay(300);
  digitalWrite(PIN_DIR_B, LOW ); delay(300);
}
