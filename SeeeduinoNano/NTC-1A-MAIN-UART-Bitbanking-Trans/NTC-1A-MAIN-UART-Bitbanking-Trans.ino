/**********************************************************************
  NTC-1A MAIN – Seeeduino Nano
  UART 9600 bps  ⇆  Bit-Bang 300 bps (D2:TX → TC,  D3:RX ← TC)
  8-bit checksum / OLED 128×64 DEBUG
  2025-06-14  Serial.flush() & println 表示版
**********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* -------- pins & timing -------- */
#define BB_TX_PIN 2          // to TC  (bit-bang TX)
#define BB_RX_PIN 3          // from TC(bit-bang RX)
#define BB_BAUD   300
#define BIT_DELAY   (1000000UL / BB_BAUD)
#define HALF_DELAY  (BIT_DELAY / 2)

/* -------- OLED  -------- */
#define OLED_W 128
#define OLED_H 64
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);

/* -------- buffers -------- */
uint8_t uart_out[6];   // Pi → TC
uint8_t bb_in  [6];    // TC → Pi

/* ---------- helpers ---------- */
void write_bitbang_byte(uint8_t b)
{
  digitalWrite(BB_TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(BB_TX_PIN, (b >> i) & 1);
    delayMicroseconds(BIT_DELAY);
  }
  digitalWrite(BB_TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY);
}

bool read_bitbang_byte(uint8_t &b, uint16_t to_ms = 1500)
{
  unsigned long t0 = millis();
  while (digitalRead(BB_RX_PIN) == HIGH) {
    if (millis() - t0 > to_ms) return false;
  }
  delayMicroseconds(HALF_DELAY);
  if (digitalRead(BB_RX_PIN) != LOW) return false;
  delayMicroseconds(HALF_DELAY);

  b = 0;
  for (uint8_t i = 0; i < 8; i++) {
    delayMicroseconds(BIT_DELAY);
    if (digitalRead(BB_RX_PIN)) b |= 1 << i;
  }
  delayMicroseconds(BIT_DELAY);  // stop
  return true;
}

void show_packets()
{
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  // Pi → TC
  oled.setCursor(0, 0);
  oled.println(F("UART -> TC"));
  for (uint8_t i = 0; i < 6; i++) {
    if (i == 3) oled.println();
    oled.print(F("0x"));
    if (uart_out[i] < 0x10) oled.print('0');
    oled.print(uart_out[i], HEX);
    oled.print(' ');
  }

  // TC → Pi
  oled.setCursor(0, 32);
  oled.println(F("TC -> UART"));
  for (uint8_t i = 0; i < 6; i++) {
    if (i == 3) oled.println();
    oled.print(F("0x"));
    if (bb_in[i] < 0x10) oled.print('0');
    oled.print(bb_in[i], HEX);
    oled.print(' ');
  }
  oled.display();
}

/* ---------- setup ---------- */
void setup()
{
  pinMode(BB_TX_PIN, OUTPUT);  digitalWrite(BB_TX_PIN, HIGH);
  pinMode(BB_RX_PIN, INPUT_PULLUP);
  Serial.begin(9600);

  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println(F("Main Relay Ready"));
  oled.display();
}

/* ---------- main loop ---------- */
void loop()
{
  /* --- Pi → TC --- */
  if (Serial.available() >= 6) {
    for (uint8_t i = 0; i < 6; i++) uart_out[i] = Serial.read();

    uint8_t sum = uart_out[0] + uart_out[1] + uart_out[2] +
                  uart_out[3] + uart_out[4];
    if ((sum & 0xFF) == uart_out[5]) {          // checksum OK
      for (uint8_t i = 0; i < 6; i++)
        write_bitbang_byte(uart_out[i]);
    }
  }

  /* --- TC → Pi --- */
  if (digitalRead(BB_RX_PIN) == LOW) {          // start detected
    if (read_bitbang_byte(bb_in[0])) {
      for (uint8_t i = 1; i < 6; i++) {
        if (!read_bitbang_byte(bb_in[i])) return;   // abort on error
      }
      /* Pi へ返送 & flush */
      Serial.write(bb_in, 6);
      Serial.flush();                           // ★ バッファ即排出
      show_packets();
    }
  }
}