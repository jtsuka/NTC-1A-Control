/**********************************************************************
  TC Emulator - Arduino Nano Every
  RX:D3  TX:D2   OLED 128x64
  Bit-banging 8-bit UART 300bps emulation with OLED packet monitor
**********************************************************************/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define BB_RX_PIN 3
#define BB_TX_PIN 2
#define LED_PIN    4
#define BB_BAUD    300
#define BIT_DELAY  (1000000UL / BB_BAUD)
#define HALF_DELAY (BIT_DELAY / 2)
#define OLED_W 128
#define OLED_H 64

Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);

uint8_t recv_buf[6];

void setup() {
  pinMode(BB_TX_PIN, OUTPUT); digitalWrite(BB_TX_PIN, HIGH);
  pinMode(BB_RX_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
  Serial.begin(9600);

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (1);
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println(F("TC Emulator 8-bit"));
  oled.display();
  delay(1000);
}

void loop() {
  if (receive_packet()) {
    show_packet("RAW pkt", recv_buf);
    send_packet(recv_buf);
  }
}

bool receive_packet() {
  for (uint8_t i = 0; i < 6; i++) {
    if (!read_byte_bitbang(recv_buf[i], 1500)) return false;
  }
  return true;
}

void send_packet(uint8_t *buf) {
  for (uint8_t i = 0; i < 6; i++) {
    write_byte_bitbang(buf[i]);
  }
}

bool read_byte_bitbang(uint8_t &b, uint16_t timeout_ms) {
  unsigned long t0 = millis();
  while (digitalRead(BB_RX_PIN) == HIGH) {
    if (millis() - t0 > timeout_ms) return false;
  }
  digitalWrite(LED_PIN, HIGH);

  delayMicroseconds(HALF_DELAY);
  if (digitalRead(BB_RX_PIN) != LOW) {
    digitalWrite(LED_PIN, LOW);
    return false;
  }
  delayMicroseconds(HALF_DELAY);

  b = 0;
  for (uint8_t i = 0; i < 8; i++) {
    delayMicroseconds(BIT_DELAY);
    b |= (digitalRead(BB_RX_PIN) << i);
  }

  delayMicroseconds(BIT_DELAY / 2);
  if (digitalRead(BB_RX_PIN) == LOW) {
    digitalWrite(LED_PIN, LOW);
    return false;  // stop bit should be HIGH
  }

  digitalWrite(LED_PIN, LOW);
  return true;
}

void write_byte_bitbang(uint8_t b) {
  digitalWrite(BB_TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(BB_TX_PIN, (b >> i) & 0x01);
    delayMicroseconds(BIT_DELAY);
  }
  digitalWrite(BB_TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY);
}

void show_packet(const char *label, uint8_t *buf) {
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println(label);
  for (uint8_t i = 0; i < 6; i++) {
    oled.print("0x");
    if (buf[i] < 0x10) oled.print('0');
    oled.print(buf[i], HEX);
    if (i % 3 == 2) oled.println(); else oled.print(' ');
  }
  oled.display();
}