/**********************************************************************
  TC-Emulator 300 bps  (Arduino Nano Every)
  D2 : TX  (bit-bang out)     → Seeeduino Nano の D3
  D3 : RX  (bit-bang in)      ← Seeeduino Nano の D2
  OLED : SSD1306 128×64  I2C  (アドレス 0x3C)

  パケット 6byte  [CH CMD VAL 00 00 CHK]   CHK = (Σ 先頭5byte) & 0xFF
**********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- 定数 ----------
#define BB_TX_PIN   2          // D2 : TX
#define BB_RX_PIN   3          // D3 : RX
#define BB_BAUD     300
#define BIT_DELAY   (1000000UL / BB_BAUD)
#define HALF_DELAY  (BIT_DELAY / 2)

#define OLED_W 128
#define OLED_H 64
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);

uint8_t packet[6];             // 受信／送信共用バッファ

// ---------- プロトタイプ ----------
bool read_byte_bitbang(uint8_t &b, uint16_t timeout_ms = 1500);
void write_byte_bitbang(uint8_t b);
bool receive_packet(void);
void send_packet(uint8_t *buf);
void show_packet(const char *label, uint8_t *buf);

// ==================================================================
void setup()
{
  pinMode(BB_TX_PIN, OUTPUT);   digitalWrite(BB_TX_PIN, HIGH); // idle = HIGH
  pinMode(BB_RX_PIN, INPUT_PULLUP);

  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (true); // OLED init fail
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("TC Emulator 8-bit"));
  display.display();
}

// ==================================================================
void loop()
{
  if (receive_packet()) {
    show_packet("RECV", packet);   // 受信を表示
    send_packet(packet);           // そのままエコーバック
    show_packet("SEND", packet);   // 送信を表示
  }
}

//===================================================================
// 6byte パケット受信 （8bit CHK 検証）
bool receive_packet()
{
  for (uint8_t i = 0; i < 6; i++) {
    if (!read_byte_bitbang(packet[i])) return false;
  }
  uint8_t sum = packet[0] + packet[1] + packet[2] + packet[3] + packet[4];
  if ((sum & 0xFF) != packet[5]) return false;   // CHK 不一致
  return true;
}

//===================================================================
// 1byte 300bps 受信
bool read_byte_bitbang(uint8_t &b, uint16_t timeout_ms)
{
  unsigned long t0 = millis();
  while (digitalRead(BB_RX_PIN) == HIGH) {       // start待ち
    if (millis() - t0 > timeout_ms) return false;
  }
  delayMicroseconds(HALF_DELAY);
  if (digitalRead(BB_RX_PIN) != LOW) return false; // invalid start
  delayMicroseconds(HALF_DELAY);

  b = 0;
  for (uint8_t bit = 0; bit < 8; bit++) {
    delayMicroseconds(BIT_DELAY);
    if (digitalRead(BB_RX_PIN)) b |= (1 << bit);
  }
  delayMicroseconds(BIT_DELAY);                  // stop bit
  return true;
}

//===================================================================
// 6byte 送信
void send_packet(uint8_t *buf)
{
  for (uint8_t i = 0; i < 6; i++) write_byte_bitbang(buf[i]);
}

// 1byte 300bps 送信
void write_byte_bitbang(uint8_t b)
{
  digitalWrite(BB_TX_PIN, LOW);                  // start
  delayMicroseconds(BIT_DELAY);
  for (uint8_t bit = 0; bit < 8; bit++) {
    digitalWrite(BB_TX_PIN, (b >> bit) & 1);
    delayMicroseconds(BIT_DELAY);
  }
  digitalWrite(BB_TX_PIN, HIGH);                 // stop
  delayMicroseconds(BIT_DELAY);
}

//===================================================================
// OLED & USB シリアルにパケット表示
void show_packet(const char *label, uint8_t *buf)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(label);
  display.println(F(" pkt"));
  for (uint8_t i = 0; i < 6; i++) {
    display.print(F("0x"));
    if (buf[i] < 0x10) display.print('0');
    display.print(buf[i], HEX);
    display.print(' ');
    if (i == 2) display.println();
  }
  display.display();

  Serial.print('['); Serial.print(label); Serial.print("] ");
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(buf[i], HEX); Serial.print(' ');
  }
  Serial.println();
}
