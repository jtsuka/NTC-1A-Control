// ---- 300 bps のパルスが入っている GPIO を探す ----
int last[48] = {0};

void setup() {
  Serial.begin(115200);
  // 0〜47 まで全ピンをプルアップ入力に
  for (int g=0; g<=47; g++){
    pinMode(g, INPUT_PULLUP);
    last[g] = digitalRead(g);
  }
}

void loop() {
  for (int g=0; g<=47; g++){
    int v = digitalRead(g);
    if (v != last[g]){          // 0→1 / 1→0 変化があったピンを表示
       Serial.printf("Edge on GPIO%d\n", g);
       last[g] = v;
    }
  }
  delayMicroseconds(200);       // 0.2 ms 毎にスキャン（300 bps の約1/5）
}
