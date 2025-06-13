// Seeeduino Nano用（メイン側）
// UART -> TCへD2でビットバンキング送信、D3で受信、OLEDでパケット表示

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define BB_TX_PIN 2
#define BB_RX_PIN 3
#define BB_BAUD 300
#define BIT_DELAY (1000000UL / BB_BAUD)
#define HALF_DELAY (BIT_DELAY / 2)

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint8_t send_buf[6];
uint8_t recv_buf[6];

void setup() {
  Serial.begin(9600);
  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);
  pinMode(BB_RX_PIN, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (1);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Main: UART to TC");
  display.display();
}

void loop() {
  if (Serial.available() >= 6) {
    for (int i = 0; i < 6; i++) send_buf[i] = Serial.read();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("TX -> TC");
    printPacket(send_buf, 1);

    for (int i = 0; i < 6; i++) send_bitbang_byte(send_buf[i]);

    delay(10);  // 応答待ち
    bool ok = true;
    for (int i = 0; i < 6; i++) {
      if (!receive_bitbang_byte(recv_buf[i], 1500)) {
        ok = false;
        break;
      }
    }

    if (ok) {
      Serial.write(recv_buf, 6);
      display.setCursor(0, 32);
      display.println("<- RX from TC");
      printPacket(recv_buf, 5);
    } else {
      display.setCursor(0, 32);
      display.println("RX Timeout");
    }

    display.display();
  }
}

void send_bitbang_byte(uint8_t b) {
  digitalWrite(BB_TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY);
  for (int i = 0; i < 8; i++) {
    digitalWrite(BB_TX_PIN, (b >> i) & 0x01);
    delayMicroseconds(BIT_DELAY);
  }
  digitalWrite(BB_TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY);
}

bool receive_bitbang_byte(uint8_t &b, unsigned long timeout_ms) {
  unsigned long start = millis();
  while (digitalRead(BB_RX_PIN) == HIGH) {
    if (millis() - start > timeout_ms) return false;
  }

  delayMicroseconds(HALF_DELAY);
  if (digitalRead(BB_RX_PIN) != LOW) return false;
  delayMicroseconds(HALF_DELAY);

  b = 0;
  for (int i = 0; i < 8; i++) {
    delayMicroseconds(BIT_DELAY);
    b |= (digitalRead(BB_RX_PIN) << i);
  }

  delayMicroseconds(BIT_DELAY);  // Stop bit
  return true;
}

void printPacket(uint8_t *data, int row_start) {
  for (int i = 0; i < 6; i++) {
    display.setCursor((i % 3) * 42, row_start * 8 + (i / 3) * 8);
    display.print("0x");
    if (data[i] < 0x10) display.print("0");
    display.print(data[i], HEX);
  }
}
