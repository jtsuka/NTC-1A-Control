/* LVC16T245 DIR/OE セルフチェック関数一式（ESP32-S3 / Arduino） */

struct LvcPins {
  int OE1 = 5;   // J4-Pin2(白) → GPIO5
  int DIR1 = 6;  // J4-Pin1(黄) → GPIO6
  int OE2 = 7;   // J7-Pin1(黄) → GPIO7
  int DIR2 = 8;  // J7-Pin2(白) → GPIO8
};

/* 外部プルアップ/プルダウン有無の簡易プローブ（任意呼び出し） */
void lvc_probe_pullups(const LvcPins& p, Stream &out = Serial) {
  int pins[4] = {p.OE1, p.DIR1, p.OE2, p.DIR2};
  const char* names[4] = {"1OE","1DIR","2OE","2DIR"};
  for (int i=0;i<4;i++) { pinMode(pins[i], INPUT); }
  delay(5);
  out.println("[LVC] Probe external pulls (INPUT):");
  for (int i=0;i<4;i++) {
    int v = digitalRead(pins[i]);
    out.printf("  %-4s : %s\n", names[i], v ? "HIGH(外部PUの可能性)" : "LOW(外部PD/なし)");
  }
}

/* 既定レベルを出力して読み戻し、合否判定。true=PASS */
bool lvc_selfcheck_apply_and_verify(const LvcPins& p, Stream &out = Serial) {
  // 既定値をドライブ
  pinMode(p.OE1, OUTPUT);  digitalWrite(p.OE1, LOW);   // 有効
  pinMode(p.DIR1, OUTPUT); digitalWrite(p.DIR1, HIGH); // A→B（ESP→TC）
  pinMode(p.OE2, OUTPUT);  digitalWrite(p.OE2, LOW);   // 有効
  pinMode(p.DIR2, OUTPUT); digitalWrite(p.DIR2, LOW);  // B→A（TC→ESP）
  delay(5);

  // 読み戻し
  int vOE1 = digitalRead(p.OE1);
  int vDIR1= digitalRead(p.DIR1);
  int vOE2 = digitalRead(p.OE2);
  int vDIR2= digitalRead(p.DIR2);

  // 表示
  out.println("[LVC] Drive & verify (OUTPUT→readback):");
  out.printf("  1OE = %s (期待:LOW)\n",  vOE1 ? "HIGH" : "LOW");
  out.printf("  1DIR= %s (期待:HIGH)\n", vDIR1? "HIGH" : "LOW");
  out.printf("  2OE = %s (期待:LOW)\n",  vOE2 ? "HIGH" : "LOW");
  out.printf("  2DIR= %s (期待:LOW)\n",  vDIR2 ? "HIGH" : "LOW");

  bool pass = (vOE1==LOW) && (vDIR1==HIGH) && (vOE2==LOW) && (vDIR2==LOW);
  out.printf("[LVC] RESULT: %s\n", pass ? "PASS" : "NG");
  if (!pass) {
    out.println("      -> 配線の入れ替わり / Groveポート取り違え / 外部プル確認を。");
  }
  return pass;
}

/* 使い方例 */
LvcPins lvc;  // 既定: 5,6,7,8

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== LVC16T245 DIR/OE Self-Check ===");
  // 任意：外部プルの有無を先に観察（I2Cピン(J4)はHIGHになりやすい）
  lvc_probe_pullups(lvc);
  // 既定レベルを設定し、読み戻しで合否判定
  lvc_selfcheck_apply_and_verify(lvc);
}

void loop() { /* 何もしない */ }
