/*
  BitBang_UART_OLED_NonBlocking.ino
  Seeeduino Nano (ATmega328P) 用
  - ハードウェアUARTは Serial を使用（Serial1 は未対応）
  - ビットバンキング通信（D2: TX, D3: RX）
  - OLED表示（I2C: A4/A5）
  - 擬似スレッドによるUART受信とBitBang送信の分離
  - MSB/LSB切り替えは #define で切替
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET -1
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

#define BITBANG_TX_PIN 2
#define BITBANG_RX_PIN 3
#define DEBUG_LED_PIN 6

#define UART_BAUD 1200
#define PACKET_SIZE 6

#define USE_MSB_FIRST  // ← 切り替えはここ（MSB or LSB）

uint8_t uart_buffer[PACKET_SIZE];
uint8_t uart_index = 0;
bool packet_ready = false;

uint8_t reply_packet[PACKET_SIZE] = {0x11, 0x22, 0x33, 0x00, 0x00, 0x66};

void setup() {
  pinMode(BITBANG_TX_PIN, OUTPUT);
  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);
  pinMode(DEBUG_LED_PIN, OUTPUT);
  digitalWrite(DEBUG_LED_PIN, LOW);

  Serial.begin(UART_BAUD);  // Seeeduino Nano: Serial = HW UART

  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("BitBang+UART Ready"));
  display.display();
}

// ==== スタートビット検出（LOW待ち）関数 ====
bool bitbang_detect_start_bit(uint16_t timeout_ms = 1000) {
  uint32_t start_time = millis();
  while (digitalRead(BITBANG_RX_PIN) == HIGH) {
    if (millis() - start_time > timeout_ms) {
      return false;  // タイムアウト
    }
  }
  // LOW検出（スタートビット）成功
  return true;
}


void loop() {
  handle_uart_receive();

  if (packet_ready) {
    digitalWrite(DEBUG_LED_PIN, HIGH);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("RX Packet:"));
    for (uint8_t i = 0; i < PACKET_SIZE; i++) {
      display.print(uart_buffer[i], HEX); display.print(" ");
    }
    display.display();
    delay(200);
    bitbang_send_packet(reply_packet);
    digitalWrite(DEBUG_LED_PIN, LOW);
    uart_index = 0;
    packet_ready = false;
  }

  
  if (bitbang_detect_start_bit()) {
    uint8_t recv_buf[PACKET_SIZE];
    if (bitbang_receive_packet(recv_buf)) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(F("RECV:"));
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

void bitbang_send_packet(uint8_t *data) {
  digitalWrite(DEBUG_LED_PIN, HIGH); delay(100); digitalWrite(DEBUG_LED_PIN, LOW);
  for (uint8_t i = 0; i < PACKET_SIZE; i++) {
    bitbang_send_byte(data[i]);
  }
}

void bitbang_send_byte(uint8_t b) {
  digitalWrite(BITBANG_TX_PIN, LOW);
  delayMicroseconds(bit_duration());
#ifdef USE_MSB_FIRST
  for (int i = 7; i >= 0; i--) {
    digitalWrite(BITBANG_TX_PIN, (b >> i) & 0x01);
    delayMicroseconds(bit_duration());
  }
#else
  for (int i = 0; i < 8; i++) {
    digitalWrite(BITBANG_TX_PIN, (b >> i) & 0x01);
    delayMicroseconds(bit_duration());
  }
#endif
  digitalWrite(BITBANG_TX_PIN, HIGH);
  delayMicroseconds(bit_duration());
}

// ==== BitBang 6バイトパケット受信 ====
bool bitbang_receive_packet(uint8_t *buffer) {
  for (int i = 0; i < PACKET_SIZE; i++) {
    buffer[i] = bitbang_receive_byte();
  }
  return true;  // チェックサム検証などがあればここに追加
}

// ==== BitBang 1バイト受信（ビット順切り替え対応） ====
uint8_t bitbang_receive_byte() {
  uint8_t received = 0;

  // スタートビット待ち（LOWを検出）
  while (digitalRead(BITBANG_RX_PIN) == HIGH);

  // 受信中断防止：割り込み無効
  noInterrupts();

  delayMicroseconds(bit_duration() + bit_duration()/2);  // 1.5ビット分待つ

#ifdef USE_MSB_FIRST
  for (int i = 7; i >= 0; i--) {
    received |= (digitalRead(BITBANG_RX_PIN) << i);
    delayMicroseconds(bit_duration());
  }
#else
  for (int i = 0; i < 8; i++) {
    received |= (digitalRead(BITBANG_RX_PIN) << i);
    delayMicroseconds(bit_duration());
  }
#endif

  // ストップビットを読み飛ばす
  delayMicroseconds(bit_duration());

  // 割り込み再有効化
  interrupts();

  return received;
}


int bit_duration() {
  return 1000000 / 300;  // 300bps に対応
}
