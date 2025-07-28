// ======  LVC8T245  DIR / OE Blink Tester  ======
// Grove J5  : GPIO5 = 1OE (白)   / GPIO6 = 1DIR (黄)
// Grove J7  : GPIO9 = 2OE (白)   / GPIO10 = 2DIR (黄)

#define PIN_OE_A 5      // 1OE  (J5-白)
#define PIN_DIR_A 6     // 1DIR (J5-黄)
#define PIN_OE_B 9      // 2OE  (J7-白)
#define PIN_DIR_B 10    // 2DIR (J7-黄)

void setup() {
  pinMode(PIN_OE_A,  OUTPUT);
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_OE_B,  OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);
  digitalWrite(PIN_OE_A, HIGH);
  digitalWrite(PIN_DIR_A, HIGH);
  digitalWrite(PIN_OE_B, HIGH);
  digitalWrite(PIN_DIR_B, HIGH);
}

void loop() {
  // --- フェーズ 0: 1OE ブリンク -----------------
//  digitalWrite(PIN_OE_A, HIGH);  delay(500);
//  digitalWrite(PIN_OE_A, LOW );  delay(500);

  // --- フェーズ 1: 1DIR ブリンク -----------------
//  digitalWrite(PIN_DIR_A, HIGH); delay(500);
//  digitalWrite(PIN_DIR_A, LOW ); delay(500);

  // --- フェーズ 2: 2OE ブリンク -----------------
//  digitalWrite(PIN_OE_B, HIGH);  delay(500);
//  digitalWrite(PIN_OE_B, LOW );  delay(500);

  // --- フェーズ 3: 2DIR ブリンク -----------------
//  digitalWrite(PIN_DIR_B, HIGH); delay(500);
//  digitalWrite(PIN_DIR_B, LOW ); delay(500);
}
