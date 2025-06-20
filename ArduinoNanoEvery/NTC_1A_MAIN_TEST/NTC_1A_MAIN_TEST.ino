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
#define MIN_DELAY 1900
#define MAX_DELAY 2200
#define STEP 10
#define LSB_FIRST 0  // ← 0 にすると MSBファースト読みになる

uint8_t packet[PACKET_LEN];

bool receive_packet(uint16_t half_delay) {
  while (digitalRead(RX_PIN) == HIGH);  // Start bit待ち
  delayMicroseconds(half_delay);

  for (uint8_t i = 0; i < PACKET_LEN; i++) {
    uint8_t b = 0;
    Serial.print("Byte ");
    Serial.print(i);
    Serial.print(" bits: ");

    for (uint8_t bit = 0; bit < 8; bit++) {
      delayMicroseconds(half_delay * 2);
      int val = digitalRead(RX_PIN);
#if LSB_FIRST
      b |= (val << bit);     // LSBファースト
#else
      b <<= 1;
      b |= val;              // MSBファースト
#endif
      Serial.print(val);     // 各ビット出力（0または1）
    }

    Serial.println();        // 改行（次のByteへ）

    packet[i] = b;
    delayMicroseconds(half_delay * 2);  // Stop bit
  }

  // CRCチェック（加算チェックサム）
  uint8_t sum = 0;
  for (uint8_t i = 0; i < PACKET_LEN - 1; i++) {
    sum += packet[i];
  }
  bool valid = ((sum & 0xFF) == packet[PACKET_LEN - 1]);

  Serial.print("CRC ");
  Serial.print(valid ? "OK" : "FAIL");
  Serial.print(" (Expected: ");
  Serial.print(sum & 0xFF, HEX);
  Serial.print(", Got: ");
  Serial.print(packet[PACKET_LEN - 1], HEX);
  Serial.println(")");

  return valid;
}

void setup() {
  pinMode(RX_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial);  // USB接続待機（特にNano Everyで有効）
  Serial.println("HALF_DELAY Scan Start");
}

void loop() {
#if 1
Serial.print("Pin D3 = ");
Serial.println(digitalRead(RX_PIN));
delay(100);
#else
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
#endif
}
