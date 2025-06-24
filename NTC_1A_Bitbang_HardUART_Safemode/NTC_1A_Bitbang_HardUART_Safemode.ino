/**********************************************************************
  NTC-1A MAIN - Seeeduino Nano  (UART + BitBang送受信 + OLED + セーフモード制御)
  TX:D2  RX:D3   OLED 128×64 (I2C A4/A5)  SAFE_MODE_PIN = D10

  - ハードウェアUARTは Serial を使用（Serial1 は未対応）
  - PiからのUART受信（通常パケット/セーフモードコマンド）
  - TCとのBitBang通信（330bps）
  - セーフモード：D10ピンを一時的に HIGH/LOW に出力（INPUT_PULLUPに戻す）
**********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define BITBANG_TX_PIN 2
#define BITBANG_RX_PIN 3
#define SAFE_MODE_PIN 10
#define DEBUG_LED_PIN 6

#define OLED_RESET -1
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

#define PACKET_SIZE 6
#define BAUD_RATE 115200
#define BIT_DURATION_US (1000000 / 330)

uint8_t uart_buffer[PACKET_SIZE];
uint8_t uart_index = 0;
bool packet_ready = false;

const uint8_t CMD_SAFE_MODE = 0xF0;  // セーフモードコマンド

void setup() {
  pinMode(BITBANG_TX_PIN, OUTPUT);
  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);
  pinMode(SAFE_MODE_PIN, INPUT_PULLUP);
  pinMode(DEBUG_LED_PIN, OUTPUT);

  Serial.begin(BAUD_RATE);

  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("UART + BitBang + SAFE"));
  display.display();
}

void loop() {
  handle_uart_receive();

  if (packet_ready) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("RX from UART:"));
    for (uint8_t i = 0; i < PACKET_SIZE; i++) {
      display.print(uart_buffer[i], HEX); display.print(" ");
    }
    display.display();

    // TCへBitBang送信
    bitbang_send_packet(uart_buffer);

    packet_ready = false;
    uart_index = 0;
  }

  // BitBang受信（必要に応じて）
  if (bitbang_detect_start_bit()) {
    uint8_t recv_buf[PACKET_SIZE];
    if (bitbang_receive_packet(recv_buf)) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(F("RECV from TC:"));
      for (uint8_t i = 0; i < PACKET_SIZE; i++) {
        display.print(recv_buf[i], HEX); display.print(" ");
      }
      display.display();
    }
  }
}

void handle_uart_receive() {
  while (Serial.available()) {
    uint8_t b = Serial.read();

    if (b == CMD_SAFE_MODE) {
      // セーフモード制御コマンド（次のバイトがモード）
      while (!Serial.available());
      uint8_t mode = Serial.read();

      digitalWrite(DEBUG_LED_PIN, HIGH);
      pinMode(SAFE_MODE_PIN, OUTPUT);
      digitalWrite(SAFE_MODE_PIN, mode ? HIGH : LOW);
      delay(10);  // 信号保持時間
      pinMode(SAFE_MODE_PIN, INPUT_PULLUP);
      digitalWrite(DEBUG_LED_PIN, LOW);
      continue;
    }

    // 通常パケット受信処理
    if (uart_index < PACKET_SIZE) {
      uart_buffer[uart_index++] = b;
      if (uart_index == PACKET_SIZE) {
        packet_ready = true;
      }
    } else {
      uart_index = 0;
    }
  }
}

// ------------------ BitBang送受信 ------------------

void bitbang_send_packet(uint8_t *data) {
  for (uint8_t i = 0; i < PACKET_SIZE; i++) {
    bitbang_send_byte(data[i]);
  }
}

void bitbang_send_byte(uint8_t b) {
  digitalWrite(BITBANG_TX_PIN, LOW);  // Start bit
  delayMicroseconds(BIT_DURATION_US);
  for (int i = 0; i < 8; i++) {
    digitalWrite(BITBANG_TX_PIN, (b >> i) & 0x01);
    delayMicroseconds(BIT_DURATION_US);
  }
  digitalWrite(BITBANG_TX_PIN, HIGH);  // Stop bit
  delayMicroseconds(BIT_DURATION_US);
}

bool bitbang_detect_start_bit(uint16_t timeout_ms = 1000) {
  uint32_t start = millis();
  while (digitalRead(BITBANG_RX_PIN) == HIGH) {
    if (millis() - start > timeout_ms) return false;
  }
  return true;
}

bool bitbang_receive_packet(uint8_t *buf) {
  for (int i = 0; i < PACKET_SIZE; i++) {
    buf[i] = bitbang_receive_byte();
  }
  return true;
}

uint8_t bitbang_receive_byte() {
  while (digitalRead(BITBANG_RX_PIN) == HIGH);
  noInterrupts();
  delayMicroseconds(BIT_DURATION_US + BIT_DURATION_US / 2);  // 中央へ

  uint8_t value = 0;
  for (int i = 0; i < 8; i++) {
    value |= (digitalRead(BITBANG_RX_PIN) << i);
    delayMicroseconds(BIT_DURATION_US);
  }

  delayMicroseconds(BIT_DURATION_US);  // Stop bit
  interrupts();
  return value;
}
