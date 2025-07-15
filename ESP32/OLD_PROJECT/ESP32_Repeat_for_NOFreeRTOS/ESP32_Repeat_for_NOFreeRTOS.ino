/**********************************************************************
  ESP32-S3 BitBang TX 300bps テストスケッチ
  GPIO2 = BitBang TX
  GPIO3 = (未使用)
  - 固定長パケット [0x01, 0x06, 0x05, 0x00, 0x00, 0x0C] を送信
  - 起動後 2秒後に一度だけ送信
**********************************************************************/

#define BB_TX_PIN 2
#define BB_BAUD 300
#define BIT_DELAY (1000000UL / BB_BAUD)

uint8_t testPacket[6] = { 0x01, 0x06, 0x05, 0x00, 0x00, 0x0C };

void setup() {
  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);  // Idle状態
  delay(2000);  // 2秒待ってから送信開始
  send_fixed_packet(testPacket);
}

void loop() {
  // 何もしない（1回だけ送信）
}

void send_fixed_packet(uint8_t *buf) {
  for (int i = 0; i < 6; i++) {
    send_bitbang_byte(buf[i]);
    delayMicroseconds(800);  // バイト間ギャップ
  }
}

void send_bitbang_byte(uint8_t b) {
  // スタートビット
  digitalWrite(BB_TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY);

  // データビット（LSBファースト）
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(BB_TX_PIN, (b >> i) & 0x01);
    delayMicroseconds(BIT_DELAY);
  }

  // ストップビット
  digitalWrite(BB_TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY);
}
