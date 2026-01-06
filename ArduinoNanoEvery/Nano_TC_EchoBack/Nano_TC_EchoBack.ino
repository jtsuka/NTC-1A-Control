/*
  Nano Every Echo-back (6 bytes packet)
  Link: Serial1 (pins D0=RX1, D1=TX1)
  Debug: Serial (USB)

  - Receives 6 bytes on Serial1 at 300bps
  - When 6 bytes collected, sends same 6 bytes back on Serial1
*/

#include <Arduino.h>

static const uint32_t LINK_BAUD = 300;
static const uint8_t  PKT_LEN  = 6;

// 300bpsで6byteは概ね200ms程度かかるので、少し余裕を持たせる
static const uint16_t RX_TIMEOUT_MS = 450;

uint8_t  rxbuf[PKT_LEN];
uint8_t  idx = 0;
uint32_t last_rx_ms = 0;

static void dumpHex(const uint8_t* p, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    if (p[i] < 0x10) Serial.print('0');
    Serial.print(p[i], HEX);
    if (i != n - 1) Serial.print(' ');
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println(F("Nano Every: Echo-back 6 bytes on Serial1"));

  Serial1.begin(LINK_BAUD); // D0/D1
  idx = 0;
  last_rx_ms = millis();
}

void loop() {
  // タイムアウトで受信中断扱い
  if (idx != 0 && (millis() - last_rx_ms) > RX_TIMEOUT_MS) {
    Serial.print(F("[TIMEOUT] partial="));
    Serial.println(idx);
    idx = 0;
  }

  while (Serial1.available() > 0) {
    uint8_t b = (uint8_t)Serial1.read();
    last_rx_ms = millis();

    rxbuf[idx++] = b;

    if (idx >= PKT_LEN) {
      // 受信パケット表示
      Serial.print(F("[RX] "));
      dumpHex(rxbuf, PKT_LEN);
      Serial.println();

      // そのままエコーバック
      Serial1.write(rxbuf, PKT_LEN);
      Serial1.flush(); // 送信完了待ち（必要なら）

      Serial.print(F("[TX] "));
      dumpHex(rxbuf, PKT_LEN);
      Serial.println();

      idx = 0;
    }
  }
}
