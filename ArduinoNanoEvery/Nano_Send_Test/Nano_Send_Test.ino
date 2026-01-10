#include <Arduino.h>

static const uint32_t BAUD_USB = 115200;
static const uint32_t BAUD_TC  = 300;

static const uint8_t  PACKET_LEN = 6;

// 300bps: 1byte≒33ms。バイト間の途切れを捨てる時間として余裕を見て設定
static const uint16_t RX_GAP_TIMEOUT_MS = 150;

// LED 5回点滅（非ブロッキング）
static const uint8_t  BLINK_COUNT = 5;
static const uint16_t BLINK_ON_MS  = 50;
static const uint16_t BLINK_OFF_MS = 50;

uint8_t  buf[PACKET_LEN];
uint8_t  idx = 0;
uint32_t t_last_rx = 0;
uint32_t pkt_count = 0;

// 点滅ステート
bool     blinking = false;
uint8_t  blink_phase = 0;   // 0..(BLINK_COUNT*2-1)  even=ON, odd=OFF
uint32_t t_blink = 0;

static void printHex6(const uint8_t *p) {
  for (int i = 0; i < 6; i++) {
    if (p[i] < 0x10) Serial.print('0');
    Serial.print(p[i], HEX);
    Serial.print(i == 5 ? "" : " ");
  }
}

static void startBlink5() {
  blinking = true;
  blink_phase = 0;
  t_blink = millis();
  digitalWrite(LED_BUILTIN, HIGH); // phase0 = ON
}

static void serviceBlink() {
  if (!blinking) return;

  const uint32_t now = millis();
  const bool isOn = (blink_phase % 2 == 0);
  const uint16_t dur = isOn ? BLINK_ON_MS : BLINK_OFF_MS;

  if (now - t_blink >= dur) {
    t_blink = now;
    blink_phase++;

    if (blink_phase >= (BLINK_COUNT * 2)) {
      // 完了
      blinking = false;
      digitalWrite(LED_BUILTIN, LOW);
      return;
    }

    // 次フェーズへ
    digitalWrite(LED_BUILTIN, (blink_phase % 2 == 0) ? HIGH : LOW);
  }
}

void setup() {
  Serial.begin(BAUD_USB);
  while (!Serial) {}

  Serial1.begin(BAUD_TC);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("=== Nano Every 300bps RX6 + Echo + LED(5x) Test (non-blocking) ===");
  Serial.println("LED blinks 5x ONLY when 6 bytes received.");
}

void loop() {
  // 点滅処理（受信を止めない）
  serviceBlink();

  // バイト間ギャップでpartial破棄
  if (idx > 0 && (millis() - t_last_rx) > RX_GAP_TIMEOUT_MS) {
    Serial.print("[TIMEOUT] partial=");
    Serial.println(idx);
    idx = 0;
  }

  // 受信処理
  while (Serial1.available() > 0) {
    uint8_t b = (uint8_t)Serial1.read();
    t_last_rx = millis();

    buf[idx++] = b;

    if (idx >= PACKET_LEN) {
      pkt_count++;

      // 受信成立の視認（5回点滅開始）
      startBlink5();

      Serial.print("[RX#");
      Serial.print(pkt_count);
      Serial.print("] ");
      printHex6(buf);
      Serial.println();

      // オウム返し
      Serial1.write(buf, PACKET_LEN);
      Serial1.flush();

      Serial.print("[TX#");
      Serial.print(pkt_count);
      Serial.print("] ");
      printHex6(buf);
      Serial.println();

      idx = 0;
    }
  }
}
