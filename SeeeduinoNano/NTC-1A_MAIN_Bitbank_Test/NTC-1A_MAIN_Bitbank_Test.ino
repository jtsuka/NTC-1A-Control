/**********************************************************************
  NTC-1A 固定パケット送信機 – Seeeduino Nano
  TCへ 300bps ビットバンギング送信（REPEAT_SEND定義で切替）
  TX:D2  RX:D3 （BB通信）
**********************************************************************/

#define REPEAT_SEND 1  // ← 1で1秒ごと送信、0で起動時に1回だけ送信

#define BB_TX_PIN 2
#define BIT_DELAY 3333
#define STOPBIT_GAP_US 300
#define BYTE_GAP_US 1500

const uint8_t test_packet[6] = { 0x01, 0x06, 0x05, 0x00, 0x00, 0x0C };

void write_bitbang_byte(uint8_t b) {
  digitalWrite(BB_TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY);

  for (uint8_t i = 0; i < 8; i++) {
    bool bit_val = (b >> i) & 1;
    digitalWrite(BB_TX_PIN, bit_val);
    delayMicroseconds(BIT_DELAY);
  }

  digitalWrite(BB_TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY + STOPBIT_GAP_US);
}

void bitbangWritePacket(const uint8_t* data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    write_bitbang_byte(data[i]);
    delayMicroseconds(BYTE_GAP_US);
  }

  // Stopビット混入防止
//  digitalWrite(BB_TX_PIN, LOW);
//  delayMicroseconds(1000);
//  digitalWrite(BB_TX_PIN, HIGH);
}

void setup() {
  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);  // アイドルHIGH
}

void loop() {
#if REPEAT_SEND
  bitbangWritePacket(test_packet, 6);
  delay(1000);  // 毎秒送信
#else
  static bool sent = false;
  if (!sent) {
    bitbangWritePacket(test_packet, 6);
    sent = true;
  }
  delay(1000);  // 念のため遅延
#endif
}
