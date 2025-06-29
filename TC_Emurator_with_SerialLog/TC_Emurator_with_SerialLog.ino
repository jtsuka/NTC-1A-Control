/**********************************************************************
  TC Emulator – OLED無効・詳細デバッグ付き (300bps BitBang受信)

  ▸ Serialモニタで受信プロセスを逐一ログ出力
  ▸ BB_RX_PINの状態、スタート検出、各バイト受信タイミングを記録
  ▸ 割り込みの影響も検出しやすく
**********************************************************************/

#define BB_RX_PIN 3
#define BB_TX_PIN 2
#define BB_BAUD 300
#define BIT_DELAY ((1000000UL / BB_BAUD) + 26)   // 約3333μs + α
#define HALF_DELAY (BIT_DELAY / 2 + 20)          // 調整用
#define BYTE_GAP_TIME 800

uint8_t recv_buf[6];

void setup() {
  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);
  pinMode(BB_RX_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  delay(300);

  Serial.println("=== TC Emulator (DEBUG MODE) ===");
}

void loop() {
  Serial.print(".");  // 動作確認用

  if (receive_packet()) {
    Serial.println("\n[OK] Packet received:");
    Serial.print("RAW: ");
    print_packet(recv_buf);

    delayMicroseconds(BIT_DELAY * 4);  // Stopビットの終了待ち
    send_packet(recv_buf);

    Serial.print("ECHO: ");
    print_packet(recv_buf);
  } else {
    Serial.print("x");  // 失敗時の表示
    delay(100);         // 表示間隔
  }
}

// ====== 詳細ログ付き BitBang受信 ======
bool receive_packet() {
  unsigned long start = millis();
  bool success = true;

  noInterrupts();
  Serial.println("\n[RECV] Start packet receive");

  for (int i = 0; i < 6; i++) {
    bool detected = false;
    Serial.print("[WAIT] Byte "); Serial.print(i); Serial.println(" waiting for start bit...");

    for (int retry = 0; retry < 3; retry++) {
      while (digitalRead(BB_RX_PIN) == HIGH) {
        if (millis() - start > 1000) {
          Serial.println("[TIMEOUT] No start bit detected");
          success = false;
          goto end;
        }
      }

      delayMicroseconds(HALF_DELAY);
      if (digitalRead(BB_RX_PIN) == LOW) {
        detected = true;
        Serial.print("[OK] Start bit detected (byte ");
        Serial.print(i); Serial.println(")");
        break;
      }
      delayMicroseconds(HALF_DELAY);
    }
    
    interrupts();  // ★復帰

    if (!detected) {
      Serial.print("[FAIL] Failed to detect start bit at byte ");
      Serial.println(i);
      success = false;
      goto end;
    }

    uint8_t b = 0;
    for (int bit = 0; bit < 8; bit++) {
      delayMicroseconds(BIT_DELAY - 2);
      b |= (digitalRead(BB_RX_PIN) << bit);
      delayMicroseconds(30);
    }

    delayMicroseconds(BIT_DELAY + 100);
    recv_buf[i] = b;

    Serial.print("[BYTE] "); Serial.print(i); Serial.print(" = 0x");
    if (b < 0x10) Serial.print("0");
    Serial.println(b, HEX);
  }

end:
  interrupts();
  return success;
}

// ====== BitBang送信 ======
void send_packet(uint8_t *buf) {
  noInterrupts();
  for (int i = 0; i < 6; i++) {
    send_bitbang_byte(buf[i]);
    digitalWrite(BB_TX_PIN, HIGH);
    delayMicroseconds(5);
    delayMicroseconds(BYTE_GAP_TIME);
  }
  delayMicroseconds(BIT_DELAY * 2 + 300);
  interrupts();
}

void send_bitbang_byte(uint8_t b){
  noInterrupts();
  digitalWrite(BB_TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY);

  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(BB_TX_PIN, (b >> i) & 0x01);
    delayMicroseconds(BIT_DELAY);
  }

  digitalWrite(BB_TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY * 2 + 300);
  delayMicroseconds(200);
  interrupts();
}

// ====== 表示関数 ======
void print_packet(uint8_t* buf) {
  for (int i = 0; i < 6; i++) {
    if (buf[i] < 0x10) Serial.print("0");
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
