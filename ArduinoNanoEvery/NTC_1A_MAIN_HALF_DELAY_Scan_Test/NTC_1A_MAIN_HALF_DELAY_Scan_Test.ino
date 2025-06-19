/**********************************************************************
  NTC-1A TC Emulator – HALF_DELAY Scan版（Serial出力）
  Arduino Nano Every (ATmega4809)
  RX: D3 (ビットバンギング受信)
  通信速度：300bps、HALF_DELAYを1400～1900μsでスキャン
  OLEDなし／シリアルモニタ出力
**********************************************************************/

#define RX_PIN 3
#define PACKET_LEN 6
#define TRIALS 10
#define MIN_DELAY 1400
#define MAX_DELAY 1900
#define STEP 50

uint8_t packet[PACKET_LEN];

bool receive_packet(uint16_t half_delay) {
  while (digitalRead(RX_PIN) == HIGH);  // Start bit待ち
  delayMicroseconds(half_delay);

  for (uint8_t i = 0; i < PACKET_LEN; i++) {
    uint8_t b = 0;
    for (uint8_t bit = 0; bit < 8; bit++) {
      delayMicroseconds(half_delay * 2);
      b |= (digitalRead(RX_PIN) << bit);
    }
    packet[i] = b;
    delayMicroseconds(half_delay * 2);  // Stop bit
  }

  // チェックサム検証（単純加算）
  uint8_t sum = 0;
  for (uint8_t i = 0; i < PACKET_LEN - 1; i++) sum += packet[i];
  return (sum & 0xFF) == packet[PACKET_LEN - 1];
}

void setup() {
  pinMode(RX_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial);  // USB接続待機（特にNano Everyで有効）
  Serial.println("HALF_DELAY Scan Start");
}

void loop() {
  Serial.println("======================================");
  Serial.println(" DELAY(us) |  Success/Trials");
  Serial.println("--------------------------------------");

  for (uint16_t delay_us = MIN_DELAY; delay_us <= MAX_DELAY; delay_us += STEP) {
    uint8_t success_count = 0;

    for (uint8_t t = 0; t < TRIALS; t++) {
      if (receive_packet(delay_us)) {
        success_count++;
      }
      delay(10);  // 次のパケット待ち
    }

    Serial.print("  ");
    Serial.print(delay_us);
    Serial.print("      |     ");
    Serial.print(success_count);
    Serial.print("/");
    Serial.println(TRIALS);
  }

  Serial.println("Scan Complete. Reset to retry.");
  while (true);  // 無限ループで終了
}
