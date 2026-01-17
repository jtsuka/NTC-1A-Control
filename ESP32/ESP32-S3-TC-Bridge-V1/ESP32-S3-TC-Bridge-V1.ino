/**
 * ESP32-S3 TC Bridge Master v1.3.1 (修正版)
 * --------------------------------------------------
 * Pi側: GPIO 1(TX), 2(RX) / 9600bps
 * Nano側: GPIO 43(TX), 44(RX) / 300bps
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <tc_packet_fixed.hpp> 

// --- ピンアサイン（本番配線・非SWAP） ---
static constexpr int PIN_PI_TX = 1;   // ESP32 TX -> Pi RX
static constexpr int PIN_PI_RX = 2;   // Pi TX -> ESP32 RX
static constexpr int PIN_TC_TX = 43;  // ESP32 TX -> Nano D10(RX)
static constexpr int PIN_TC_RX = 44;  // Nano D11(TX) -> ESP32 RX

// --- レベルシフタ (SN74LVC16T245) 制御 ---
// Bank1: ESP(A) <-> Nano側(B)
static constexpr int PIN_LSHIFT_OE1  = 6;  // OE1 (LOWで有効)
static constexpr int PIN_LSHIFT_DIR1 = 5;  // DIR1 (HIGHで ESP->Nano方向)

// Bank2: ESP(A) <-> Pi側(B)
static constexpr int PIN_LSHIFT_OE2  = 7;  // OE2 (LOWで有効)
static constexpr int PIN_LSHIFT_DIR2 = 8;  // DIR2 (LOWで Pi->ESP方向)

static constexpr uint32_t PI_BAUD   = 9600;
static constexpr uint32_t NANO_BAUD = 300;
static constexpr uint32_t NANO_RX_TIMEOUT_MS = 600;

HardwareSerial SerialPi(1);
HardwareSerial SerialNano(2);

/**
 * タイムアウト付きで固定6バイトを受信する
 */
static bool readFixed6(Stream& s, tc::Fixed6& out, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  uint8_t idx = 0;
  while (idx < tc::Fixed6::LEN) {
    if (s.available() > 0) {
      out.b[idx++] = (uint8_t)s.read();
      t0 = millis();
    } else {
      if ((millis() - t0) > timeoutMs) return false;
      yield(); 
    }
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  
  // 安全起動シーケンス
  pinMode(PIN_LSHIFT_OE1, OUTPUT);  digitalWrite(PIN_LSHIFT_OE1, HIGH);
  pinMode(PIN_LSHIFT_OE2, OUTPUT);  digitalWrite(PIN_LSHIFT_OE2, HIGH);
  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH);
  pinMode(PIN_LSHIFT_DIR2, OUTPUT); digitalWrite(PIN_LSHIFT_DIR2, LOW);

  SerialPi.begin(PI_BAUD, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);
  SerialNano.begin(NANO_BAUD, SERIAL_8N1, PIN_TC_RX, PIN_TC_TX);
  
  while (SerialPi.available()) SerialPi.read();
  while (SerialNano.available()) SerialNano.read();

  delay(200);
  digitalWrite(PIN_LSHIFT_OE1, LOW); 
  digitalWrite(PIN_LSHIFT_OE2, LOW); 

  Serial.println(F("--- [ESP32 Bridge v1.3.1: Ready] ---"));
}

void loop() {
  tc::Fixed6 pktFromPi{};
  
  if (readFixed6(SerialPi, pktFromPi, 100)) {
    while (SerialNano.available() > 0) (void)SerialNano.read();

    SerialNano.write(pktFromPi.b, tc::Fixed6::LEN);

    tc::Fixed6 respFromNano{};
    if (readFixed6(SerialNano, respFromNano, NANO_RX_TIMEOUT_MS)) {
      SerialPi.write(respFromNano.b, tc::Fixed6::LEN);
    } else {
      tc::Fixed6 z{}; 
      for(int i=0; i<6; i++) z.b[i] = 0;
      SerialPi.write(z.b, tc::Fixed6::LEN);
    }
  }
}