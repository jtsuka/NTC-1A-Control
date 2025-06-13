// Arduino Nano Every  – TC Emulator  (8‑bit checksum, OLED packet display, RX‑LED)
// Pins: D2 = TX  (bit‑bang out 300 bps)
//       D3 = RX  (bit‑bang in  300 bps)
//       LED_PIN = 13  … 受信開始を点灯
// I2C OLED 128×64 (SSD1306) at 0x3C
// ------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- Pin / Timing ----------
#define BB_TX_PIN 2   // out to Seeeduino RX (D3)
#define BB_RX_PIN 3   // in  from Seeeduino TX (D2)
#define LED_PIN   13  // 内蔵 LED 受信スタート指示
#define BB_BAUD   300
#define BIT_DELAY   (1000000UL / BB_BAUD)
#define HALF_DELAY  (BIT_DELAY / 2)

// ---------- OLED ----------
#define OLED_W 128
#define OLED_H 64
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);

uint8_t packet[6];

// プロトタイプ -------------------------------------------------
bool read_byte_bitbang(uint8_t &b, uint16_t timeout_ms = 1500);
void write_byte_bitbang(uint8_t b);
bool receive_packet();
void send_packet(uint8_t *buf);
void show_packet(const char *label, uint8_t *buf);

// =============================================================
void setup() {
  pinMode(BB_TX_PIN, OUTPUT);  digitalWrite(BB_TX_PIN, HIGH);   // idle = HIGH
  pinMode(BB_RX_PIN, INPUT_PULLUP);
  pinMode(LED_PIN,   OUTPUT);  digitalWrite(LED_PIN, LOW);

  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (true); // OLED init fail
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("TC Emulator 8‑bit"));
  display.display();
}

// =============================================================
void loop() {
  if (receive_packet()) {
    show_packet("RECV", packet);
    send_packet(packet);
    show_packet("SEND", packet);
  }
}

// =============================================================
// 6‑byte packet reception (8‑bit checksum)
bool receive_packet() {
  for (uint8_t i = 0; i < 6; i++) {
    if (!read_byte_bitbang(packet[i])) return false;
  }
  uint8_t sum = packet[0] + packet[1] + packet[2] + packet[3] + packet[4];
  return ((sum & 0xFF) == packet[5]);
}

// -------------------------------------------------------------
bool read_byte_bitbang(uint8_t &b, uint16_t timeout_ms) {
  unsigned long t0 = millis();
  while (digitalRead(BB_RX_PIN) == HIGH) {
    if (millis() - t0 > timeout_ms) return false; // timeout wait start
  }
  digitalWrite(LED_PIN, HIGH);                     // ★ start bit検出 → LED ON

  delayMicroseconds(HALF_DELAY);
  if (digitalRead(BB_RX_PIN) != LOW) {            // ノイズ検知
    digitalWrite(LED_PIN, LOW);
    return false;
  }
  delayMicroseconds(HALF_DELAY);

  b = 0;
  for (uint8_t bit = 0; bit < 8; bit++) {
    delayMicroseconds(BIT_DELAY);
    if (digitalRead(BB_RX_PIN)) b |= (1 << bit);
  }
  delayMicroseconds(BIT_DELAY);                   // stop bit
  digitalWrite(LED_PIN, LOW);                     // ★ byte完了 → LED OFF
  return true;
}

// -------------------------------------------------------------
void send_packet(uint8_t *buf) {
  for (uint8_t i = 0; i < 6; i++) write_byte_bitbang(buf[i]);
}

void write_byte_bitbang(uint8_t b) {
  digitalWrite(BB_TX_PIN, LOW);                    // start
  delayMicroseconds(BIT_DELAY);
  for (uint8_t bit = 0; bit < 8; bit++) {
    digitalWrite(BB_TX_PIN, (b >> bit) & 1);
    delayMicroseconds(BIT_DELAY);
  }
  digitalWrite(BB_TX_PIN, HIGH);                   // stop
  delayMicroseconds(BIT_DELAY);
}

// -------------------------------------------------------------
void show_packet(const char *label, uint8_t *buf) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(label);
  display.println(F(" pkt"));
  for (uint8_t i = 0; i < 6; i++) {
    display.print(F("0x")); if (buf[i] < 0x10) display.print('0');
    display.print(buf[i], HEX); display.print(' ');
    if (i == 2) display.println();
  }
  display.display();

  Serial.print('['); Serial.print(label); Serial.print("] ");
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(buf[i], HEX); Serial.print(' ');
  }
  Serial.println();
}
