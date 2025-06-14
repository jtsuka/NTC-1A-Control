/**********************************************************************
  TC Emulator – Arduino Nano Every
  ◉ 受信: ビットバンギング (D3ピン / 330bps / 8bit固定)
  ◉ 送信: ビットバンギング (D2ピン / 330bps / 8bit固定)
  ◉ OLED表示: I2C (128x64) に送受信パケットを表示
  ◉ 通信プロトコル:
     - 固定長 6バイトパケット
     - チェックサム: 最後の1バイト = 先頭5バイトの合計の下位8bit
  ◉ 動作概要:
     - Seeeduino Nano（メイン）から受信 → OLED表示
     - そのまま応答として返送 → Pi側に回送
     - 不正なパケットは無視し "Checksum NG" 表示
  ◉ LED: 受信時に一瞬点灯

  使用ピン:
    - D2: BB_TX (TCからPiへの返信)
    - D3: BB_RX (PiからTCへの受信)
    - I2C: A4(SDA), A5(SCL) (OLED表示)
    - LED_BUILTIN: 受信確認用点灯

  2025-06 Updated: ビット受信の中央値補正 + OLED強化表示
**********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define BB_RX_PIN 3
#define BB_TX_PIN 2
#define BB_BAUD 330
#define BIT_DELAY (1000000UL / BB_BAUD)
#define HALF_DELAY (BIT_DELAY / 2)
#define ADJUST_RX_US 20  // ★ 受信タイミング微調整

uint8_t packet[6];

void setup() {
  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);
  pinMode(BB_RX_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("TC Emulator 8-bit"));
  display.display();
  delay(1000);
}

void loop() {
  if (receive_packet()) {
    digitalWrite(LED_BUILTIN, HIGH);
    display_packet("UART -> TC:", packet);
    send_packet(packet);
    display_packet("TC -> UART:", packet);
    Serial.write(packet, 6);
    Serial.flush();
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }
}

bool receive_packet() {
  for (int i = 0; i < 6; i++) {
    if (!read_bitbang_byte(packet[i])) {
      show_error("Recv Timeout");
      return false;
    }
  }
  uint8_t sum = 0;
  for (int i = 0; i < 5; i++) sum += packet[i];
  if ((sum & 0xFF) != packet[5]) {
    show_error("Checksum NG!");
    return false;
  }
  return true;
}

void send_packet(uint8_t *buf) {
  for (int i = 0; i < 6; i++) {
    write_bitbang_byte(buf[i]);
  }
}

bool read_bitbang_byte(uint8_t &b) {
  unsigned long start = millis();
  while (digitalRead(BB_RX_PIN) == HIGH) {
    if (millis() - start > 1000) return false;
  }

  delayMicroseconds(HALF_DELAY + ADJUST_RX_US);  // ★微調整
  if (digitalRead(BB_RX_PIN) != LOW) return false;
  delayMicroseconds(HALF_DELAY);

  b = 0;
  for (int i = 0; i < 8; i++) {
    delayMicroseconds(BIT_DELAY);
    if (digitalRead(BB_RX_PIN)) b |= (1 << i);
  }

  delayMicroseconds(BIT_DELAY);  // Stop bit
  return true;
}

void write_bitbang_byte(uint8_t b) {
  digitalWrite(BB_TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY);
  for (int i = 0; i < 8; i++) {
    digitalWrite(BB_TX_PIN, (b >> i) & 1);
    delayMicroseconds(BIT_DELAY);
  }
  digitalWrite(BB_TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY);
}

void display_packet(const char *label, uint8_t *buf) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(label);
  for (int i = 0; i < 6; i++) {
    display.print("0x");
    if (buf[i] < 0x10) display.print("0");
    display.print(buf[i], HEX);
    display.print(" ");
    if (i == 2 || i == 5) display.println();
  }
  display.display();
}

void show_error(const char *msg) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("ERROR:");
  display.println(msg);
  display.display();
}