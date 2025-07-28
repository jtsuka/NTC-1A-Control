/*********************************************************************
  TC Repeater – FULL Version (ESP32-S3)
  ---------------------------------------------------------------
  • Pi  ←→  ESP32  ←→  TC   （6-byte fixed packet）
  • Pi <--9600 bps--> Serial1 (UART1)     GPIO43:TX / GPIO44:RX
  • TC <-- 300 bps--> Serial2 (UART2)     GPIO2 :TX / GPIO3 :RX
  • Bit-order conversion (rev8) both directions
  • SN74LVC16T245 DIR/OE pins fixed
    └ OE1=GPIO5 LOW, DIR1=GPIO6 HIGH, OE2=GPIO7 LOW, DIR2=GPIO8 LOW
*********************************************************************/

#include <Arduino.h>

// ---------- UART & Packet ---------- //
#define PACKET_LEN     6

// Pi side (9600 bps)
#define UART_PI_TX_PIN 43
#define UART_PI_RX_PIN 44
#define UART_PI_BAUD   9600
HardwareSerial SerialPi(1);   // UART1

// TC side (300 bps)
#define UART_TC_TX_PIN 2
#define UART_TC_RX_PIN 3
#define UART_TC_BAUD   300
HardwareSerial SerialTC(2);   // UART2

// ---------- LVC16T245 Control ---------- //
#define PIN_OE1  5   // TX側 OE   (LOW=enable)
#define PIN_DIR1 6   // TX側 DIR  (HIGH: A→B)  ESP32→TC
#define PIN_OE2  7   // RX側 OE   (LOW=enable)
#define PIN_DIR2 8   // RX側 DIR  (LOW : B→A)  TC→ESP32

// ---------- rev8 helper ---------- //
static inline uint8_t rev8(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

// ---------- Buffers ---------- //
uint8_t bufPi[PACKET_LEN];   int idxPi = 0;   // Pi → TC
uint8_t bufTC[PACKET_LEN];   int idxTC = 0;   // TC → Pi

void setup() {
  Serial.begin(115200);               // debug console

  // -- UART init --
  SerialPi.begin(UART_PI_BAUD, SERIAL_8N1, UART_PI_RX_PIN, UART_PI_TX_PIN);
  SerialTC.begin(UART_TC_BAUD, SERIAL_8N1, UART_TC_RX_PIN, UART_TC_TX_PIN);

  // -- LVC16T245 fixed control --
  pinMode(PIN_OE1, OUTPUT); digitalWrite(PIN_OE1, LOW);   // enable
  pinMode(PIN_DIR1, OUTPUT); digitalWrite(PIN_DIR1, HIGH); // A→B
  pinMode(PIN_OE2, OUTPUT); digitalWrite(PIN_OE2, LOW);   // enable
  pinMode(PIN_DIR2, OUTPUT); digitalWrite(PIN_DIR2, LOW);  // B→A

  Serial.println("=== TC Repeater FULL v1.0  ===");
}

void loop() {

  /******************* ① Pi → ESP32 → TC *******************/
  while (SerialPi.available()) {
    uint8_t b = SerialPi.read();

    /* ①‐A スタート同期  :  Pi→TC の先頭は rev8 前で 0x01 */
//    if (idxPi == 0 && b != 0x01) {
    // 修正後（正）  Piから来る先頭 0x01 を受け入れる
    if (idxPi == 0 && b != 0x01) {
      /* 先頭バイトが 0x01 以外なら捨てる（D5 18 7F などを除去） */
      continue;
    }

    bufPi[idxPi++] = b;

    if (idxPi == PACKET_LEN) {
      /* ①‐B rev8 → TCへ送信（300 bps） */
      for (int i = 0; i < PACKET_LEN; ++i) bufPi[i] = rev8(bufPi[i]);
      SerialTC.write(bufPi, PACKET_LEN);
      SerialTC.flush();       // 送信完了待ちで FIFO 残しゼロ

      /* -- 任意: デバッグ表示 --
      Serial.print("[Pi→TC] ");
      for (int i = 0; i < PACKET_LEN; ++i) Serial.printf("%02X ", bufPi[i]);
      Serial.println();
      */

      idxPi = 0;              // バッファリセット
    }
  }


  /******************* ② TC → ESP32 → Pi *******************/
  while (SerialTC.available()) {
    uint8_t b = SerialTC.read();

    /* ②‐A スタート同期 : TC→Pi の先頭は 0x80 */
    if (idxTC == 0 && b != 0x80) {
      continue;               // ゴミ（チャタリング由来 0x7F 等）を無視
    }

    bufTC[idxTC++] = b;

    if (idxTC == PACKET_LEN) {
      /* ②‐B rev8 → Piへ送信（9600 bps） */
      for (int i = 0; i < PACKET_LEN; ++i) bufTC[i] = rev8(bufTC[i]);
      SerialPi.write(bufTC, PACKET_LEN);
      SerialPi.flush();

      /* -- 任意: デバッグ表示 --
      Serial.print("[TC→Pi] ");
      for (int i = 0; i < PACKET_LEN; ++i) Serial.printf("%02X ", bufTC[i]);
      Serial.println();
      */

      idxTC = 0;
    }
  }

  delay(1);   // tiny yield
}


#if 0
void loop() {
  // -------------------------------------------------
  // ① Pi → ESP32 → TC  (9600 → 300, rev8 before send)
  // -------------------------------------------------
  while (SerialPi.available()) {
    bufPi[idxPi++] = SerialPi.read();
    if (idxPi >= PACKET_LEN) {
      // debug
      Serial.print("[Pi→TC raw] ");
      for (int i = 0; i < PACKET_LEN; ++i) { Serial.printf("%02X ", bufPi[i]); }
      Serial.println();

      // bit-reverse & send to TC
      for (int i = 0; i < PACKET_LEN; ++i) bufPi[i] = rev8(bufPi[i]);
      SerialTC.write(bufPi, PACKET_LEN);

      Serial.print("[Pi→TC send] ");
      for (int i = 0; i < PACKET_LEN; ++i) { Serial.printf("%02X ", bufPi[i]); }
      Serial.println();

      idxPi = 0;
    }
  }

  // -------------------------------------------------
  // ② TC → ESP32 → Pi  (300 → 9600, rev8 before send)
  // -------------------------------------------------
  while (SerialTC.available()) {
    bufTC[idxTC++] = SerialTC.read();
    if (idxTC >= PACKET_LEN) {
      // debug
      Serial.print("[TC→Pi raw] ");
      for (int i = 0; i < PACKET_LEN; ++i) { Serial.printf("%02X ", bufTC[i]); }
      Serial.println();

      // bit-reverse & send to Pi
      for (int i = 0; i < PACKET_LEN; ++i) bufTC[i] = rev8(bufTC[i]);
      SerialPi.write(bufTC, PACKET_LEN);

      Serial.print("[TC→Pi send] ");
      for (int i = 0; i < PACKET_LEN; ++i) { Serial.printf("%02X ", bufTC[i]); }
      Serial.println();

      idxTC = 0;
    }
  }

  // small yield to keep WiFi/BLE happy if future expansion
  delay(1);
}
#endif
