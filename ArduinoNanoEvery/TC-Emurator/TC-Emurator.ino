/**********************************************************************
  NTC-1A TC Emulator – Arduino Nano Every  (Bit‑Bang 300 bps ⇆ UART)
  RX:D3  TX:D2   OLED 128×64
  ▸ 上段 = 受信したRAWパケット
  ▸ 下段 = チェックサムOK時にエコーバック送信
  LED_BUILTIN = 受信中点灯
**********************************************************************/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ======== 設定 ======== */
#define BB_RX_PIN 3
#define BB_TX_PIN 2
#define LED_PIN   LED_BUILTIN
#define OLED_ADDR 0x3C
#define OLED_W    128
#define OLED_H    64
#define BB_BAUD   300
#define BIT_DELAY     (1000000UL / BB_BAUD)
#define HALF_DELAY    (BIT_DELAY / 2)

Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);
uint8_t packet[6];

/* ======== ビットバンギング受信 ======== */
bool read_byte_bitbang(uint8_t &b, uint16_t timeout_ms) {
  unsigned long t0 = millis();
  while (digitalRead(BB_RX_PIN) == HIGH) {
    if (millis() - t0 > timeout_ms) return false;
  }
  digitalWrite(LED_PIN, HIGH);  // 受信中点灯

  delayMicroseconds(HALF_DELAY);
  if (digitalRead(BB_RX_PIN) != LOW) {
    digitalWrite(LED_PIN, LOW);
    return false;
  }
  delayMicroseconds(BIT_DELAY + HALF_DELAY);  // ビット0中央へ

  b = 0;
  for (uint8_t i = 0; i < 8; i++) {
    b |= (digitalRead(BB_RX_PIN) << i);
    delayMicroseconds(BIT_DELAY);
  }
  digitalWrite(LED_PIN, LOW);  // 受信完了
  return true;
}

/* ======== ビットバンギング送信 ======== */
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

/* ======== OLEDパケット表示 ======== */
void draw_packets() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0); oled.println(F("RAW pkt"));
  for (uint8_t i = 0; i < 6; i++) {
    oled.print(F("0x"));
    if (packet[i] < 0x10) oled.print("0");
    oled.print(packet[i], HEX);
    oled.print(" ");
    if (i == 2 || i == 5) oled.println();
  }
  oled.display();
}

/* ======== 初期化 ======== */
void setup() {
  pinMode(BB_RX_PIN, INPUT_PULLUP);
  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println(F("TC Emulator 8-bit"));
  oled.display();
  delay(1000);
}

/* ======== メインループ ======== */
void loop() {
  if (digitalRead(BB_RX_PIN) == LOW) {
    bool ok = true;
    for (uint8_t i = 0; i < 6; i++) {
      if (!read_byte_bitbang(packet[i], 1000)) {
        ok = false;
        break;
      }
    }
    if (!ok) return;
    draw_packets();
    uint8_t sum = 0;
    for (int i = 0; i < 5; i++) sum += packet[i];
    if ((sum & 0xFF) == packet[5]) {
      for (int i = 0; i < 6; i++) write_byte_bitbang(packet[i]);
    }
  }
}