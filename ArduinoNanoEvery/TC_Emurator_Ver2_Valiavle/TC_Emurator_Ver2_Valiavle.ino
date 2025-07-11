/**********************************************************************
  NTC-1A TC Emulator – Arduino Nano Every (Bit-Bang UART 300bps)
  RX:D3  TX:D2   OLED 128×64 表示付き
  - 可変長パケット対応（CMD + LEN 形式）
  - LSBファースト、エコーバック対応
  - GUIと連携しやすい構成

  更新日: 2025-07-11
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

#define MAX_PKT_LEN 16
uint8_t recv_buf[MAX_PKT_LEN];
uint8_t recv_len = 0;

void setup() {
  pinMode(BB_TX_PIN, OUTPUT); digitalWrite(BB_TX_PIN, HIGH);
  pinMode(BB_RX_PIN, INPUT_PULLUP);
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("TC Emulator VarLen"));
  display.display();
  delay(1000);
}

void loop() {
  Serial.println("loop"); 
  if (receive_packet()) {
    display_packet("RECV", recv_buf, recv_len);
    delayMicroseconds(BIT_DELAY * 4);  // パケット間待ち
    send_packet(recv_buf, recv_len);
    display_packet("SEND", recv_buf, recv_len);
    delayMicroseconds(500);  // 次の受信への休止
  } else {
    Serial.println("receive failed");  // ③ 失敗時表示
  }
}

bool receive_packet() {
  unsigned long start = millis();
  bool success = true;

  memset(recv_buf, 0, sizeof(recv_buf));  // ★ゼロ初期化

  noInterrupts();

  for (int i = 0; i < 2; i++) {  // CMD + LEN 先読み
    bool detected = false;
    for (int retry = 0; retry < 3; retry++) {
      while (digitalRead(BB_RX_PIN) == HIGH) {
        if (millis() - start > 1000) {
          success = false;
          goto end;
        }
      }
      delayMicroseconds(HALF_DELAY);
      if (digitalRead(BB_RX_PIN) == LOW) {
        detected = true; break;
      }
      delayMicroseconds(HALF_DELAY);
    }
    if (!detected) {
      success = false;
      goto end;
    }

    uint8_t b = 0;
    for (int bit = 0; bit < 8; bit++) {
      delayMicroseconds(BIT_DELAY - 2);
      b |= (digitalRead(BB_RX_PIN) << bit);
      delayMicroseconds(30);
    }
    delayMicroseconds(BIT_DELAY + 100);
    recv_buf[i] = b;
  }

  recv_len = recv_buf[1];  // LEN値

// ★ここでLENバイト制限を入れる（2以上、MAX以下）
  if (recv_len < 2 || recv_len > MAX_PKT_LEN) {
    success = false;
    goto end;
  }

  if (recv_len > MAX_PKT_LEN) recv_len = MAX_PKT_LEN;

  // CMD受信後すぐに確認
  Serial.print("CMD="); Serial.println(recv_buf[0], HEX);
  Serial.print("LEN="); Serial.println(recv_buf[1]);  // ← これ重要

  for (int i = 2; i < recv_len; i++) {
    uint8_t b = 0;
    for (int bit = 0; bit < 8; bit++) {
      delayMicroseconds(BIT_DELAY - 2);
      b |= (digitalRead(BB_RX_PIN) << bit);
      delayMicroseconds(30);
    }
    delayMicroseconds(BIT_DELAY + 100);
    recv_buf[i] = b;
  }

end:
  interrupts();
  Serial.println("receive OK");
  return success;
}

void send_packet(uint8_t *buf, uint8_t len) {
  noInterrupts();
  for (int i = 0; i < len; i++) {
    send_bitbang_byte(buf[i]);
    digitalWrite(BB_TX_PIN, HIGH);
    delayMicroseconds(5);
    delayMicroseconds(BYTE_GAP_TIME);
  }
  delayMicroseconds(BIT_DELAY * 2 + 300);
  interrupts();
  Serial.print("SEND len="); Serial.println(len);
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
  Serial.println(b, HEX);
}

void display_packet(const char *label, uint8_t *buf, uint8_t len) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(label); display.println(" Packet:");
  for (uint8_t i = 0; i < len; i++) {
    display.print("0x");
    if (buf[i] < 0x10) display.print("0");
    display.print(buf[i], HEX); display.print(" ");
    if ((i + 1) % 6 == 0) display.println();
  }
  display.display();
}
