/**********************************************************************
  NTC-1A 固定パケット送信機 – Seeeduino Nano
  TCへ 300bps ビットバンギング送信を繰り返し実施（検証用）
  TX:D2  RX:D3 （BB通信）
**********************************************************************/

#define BB_TX_PIN 2
#define BIT_DELAY 3333
#define STOPBIT_GAP_US 300
#define BYTE_GAP_US 1500

const uint8_t test_packet[6] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C}; // CRC付き固定パケット

void write_bitbang_byte(uint8_t b) {
  // Start bit
  digitalWrite(BB_TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY);

  // データビット（8ビット）
  for (uint8_t i = 0; i < 8; i++) {
    bool bit_val = (b >> i) & 1;
    digitalWrite(BB_TX_PIN, bit_val);
    delayMicroseconds(BIT_DELAY);
  }

  // Stop bit
  digitalWrite(BB_TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY + STOPBIT_GAP_US);
}

void bitbangWritePacket(const uint8_t* data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    write_bitbang_byte(data[i]);
    delayMicroseconds(BYTE_GAP_US);
  }
}

void setup() {
  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);  // アイドル時はHIGH
}

void loop() {
  bitbangWritePacket(test_packet, 6);
  delay(500);  // 500msごとに送信
}
