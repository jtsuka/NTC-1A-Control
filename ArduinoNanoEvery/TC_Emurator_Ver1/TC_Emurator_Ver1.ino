/**********************************************************************
  NTC-1A TC Emulator (Arduino Nano Every, Bit-Bang UART 300bps)
  - RX: D3, TX: D2
  - OLED 128x64 I2C (Grove)
  - 可変長パケット対応（6バイト〜最大16バイト）
  - 受信後はエコーバック送信
  - OLED 表示は 上段: 受信, 下段: 返信

  更新日: 2025-07-10
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
#define BB_BAUD 300
#define BIT_DELAY ((1000000UL / BB_BAUD) + 26)
#define HALF_DELAY (BIT_DELAY / 2 + 20)
#define BYTE_GAP_TIME 800
#define MAX_PKT_SIZE 16

uint8_t recv_buf[MAX_PKT_SIZE];
uint8_t recv_len = 0;

void setup() {
  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);
  pinMode(BB_RX_PIN, INPUT_PULLUP);
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (true); // OLED init failed
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("TC Emulator Ready"));
  display.display();
  delay(1000);
}

void loop() {
  if (receive_packet()) {
    display_packet("RECV", recv_buf);
    delayMicroseconds(BIT_DELAY * 4);  // Stop bit含めた余白
    send_packet(recv_buf);
    display_packet("ECHO", recv_buf);
    delayMicroseconds(500);  // 連続受信回避
  }
}

uint8_t getPacketLength(uint8_t cmd) {
  switch (cmd) {
    case 0x06:
    case 0x10:
    case 0x11:
    case 0x50:
      return 6;
    case 0x13:
    case 0x20:
      return 8;
    default:
      return 6;
  }
}

bool waitForStartBit(unsigned long start) {
  while (digitalRead(BB_RX_PIN) == HIGH) {
    if (millis() - start > 1000) return false;
  }
  delayMicroseconds(HALF_DELAY);
  return (digitalRead(BB_RX_PIN) == LOW);
}

uint8_t receiveByte() {
  uint8_t b = 0;
  for (int bit = 0; bit < 8; bit++) {
    delayMicroseconds(BIT_DELAY - 2);
    b |= (digitalRead(BB_RX_PIN) << bit);
    delayMicroseconds(30);
  }
  delayMicroseconds(BIT_DELAY + 100);
  return b;
}

bool receive_packet() {
  unsigned long start = millis();
  bool success = true;
  noInterrupts();

  if (!waitForStartBit(start)) {
    interrupts(); return false;
  }
  recv_buf[0] = receiveByte();
  recv_len = getPacketLength(recv_buf[0]);

  for (int i = 1; i < recv_len; i++) {
    if (!waitForStartBit(start)) {
      success = false;
      break;
    }
    recv_buf[i] = receiveByte();
  }

  interrupts();
  return success;
}

void send_packet(uint8_t *buf) {
  noInterrupts();
  for (int i = 0; i < recv_len; i++) {
    send_bitbang_byte(buf[i]);
    digitalWrite(BB_TX_PIN, HIGH);
    delayMicroseconds(5);
    delayMicroseconds(BYTE_GAP_TIME);
  }
  delayMicroseconds(BIT_DELAY * 2 + 300);
  interrupts();
}

void send_bitbang_byte(uint8_t b) {
  noInterrupts();
  digitalWrite(BB_TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(BB_TX_PIN, (b >> i) & 0x01);
    delayMicroseconds(BIT_DELAY);
  }
  digitalWrite(BB_TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY * 2 + 300);
  delayMicroseconds(200);
  interrupts();
}

void display_packet(const char *label, uint8_t *buf) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(label);
  display.println(" Packet");
  for (int i = 0; i < recv_len; i++) {
    display.print("0x");
    if (buf[i] < 0x10) display.print("0");
    display.print(buf[i], HEX);
    display.print(" ");
    if (i % 3 == 2) display.println();
  }
  display.display();
}
