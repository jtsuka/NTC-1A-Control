/**********************************************************************
  BitBang + UART + OLED + Pseudo Threading
  - Seeeduino Nano用（ATmega328P）
  - D2 = BitBang TX, D3 = BitBang RX
  - D6 = デバッグLED
  - UART = Serial (ピン0: RX, 1: TX)
  - OLED = I2C (0x3C)
**********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==================== 設定 ====================
#define OLED_ENABLED     1
#define OLED_ADDR        0x3C
#define OLED_WIDTH       128
#define OLED_HEIGHT      64
#define DEBUG_PIN        6

// UART
#define UART_BAUD        1200
#define MAX_QUEUE        4
#define PACKET_SIZE      6

// BitBang通信
#define BB_TX_PIN        2
#define BB_RX_PIN        3
#define BB_BAUD          300
#define BIT_DELAY_US     (1000000UL / BB_BAUD)
#define HALF_DELAY_US    (BIT_DELAY_US / 2)
#define BYTE_GAP_US      1500

// ビット順切り替え（再コンパイルで変更）
#define LSB_FIRST        1

// ==================== グローバル ====================
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

uint8_t uart_rx_queue[MAX_QUEUE][PACKET_SIZE];
volatile int uart_rx_head = 0;
volatile int uart_rx_tail = 0;

uint8_t current_packet[PACKET_SIZE];
uint8_t reply_packet[PACKET_SIZE];
unsigned long last_exec_time = 0;

enum State { IDLE, SENDING, WAIT_REPLY };
State state = IDLE;

// ==================== キュー処理 ====================
bool enqueue_uart_packet(uint8_t *data) {
  int next = (uart_rx_head + 1) % MAX_QUEUE;
  if (next == uart_rx_tail) return false;
  memcpy(uart_rx_queue[uart_rx_head], data, PACKET_SIZE);
  uart_rx_head = next;
  return true;
}

bool dequeue_uart_packet(uint8_t *data) {
  if (uart_rx_head == uart_rx_tail) return false;
  memcpy(data, uart_rx_queue[uart_rx_tail], PACKET_SIZE);
  uart_rx_tail = (uart_rx_tail + 1) % MAX_QUEUE;
  return true;
}

// ==================== BitBang送信 ====================
void write_bitbang_byte(uint8_t b) {
  digitalWrite(BB_TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY_US);

#if LSB_FIRST
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(BB_TX_PIN, (b >> i) & 1);
    delayMicroseconds(BIT_DELAY_US);
  }
#else
  for (int8_t i = 7; i >= 0; i--) {
    digitalWrite(BB_TX_PIN, (b >> i) & 1);
    delayMicroseconds(BIT_DELAY_US);
  }
#endif

  digitalWrite(BB_TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY_US);
}

void bitbangWrite(uint8_t *data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    write_bitbang_byte(data[i]);
    delayMicroseconds(BYTE_GAP_US);
  }
}

// ==================== BitBang受信 ====================
bool read_bitbang_byte(uint8_t &b, uint16_t timeout_ms = 2000) {
  unsigned long t0 = millis();
  while (digitalRead(BB_RX_PIN) == HIGH) {
    if (millis() - t0 > timeout_ms) return false;
  }

  delayMicroseconds(HALF_DELAY_US);
  if (digitalRead(BB_RX_PIN) != LOW) return false;
  delayMicroseconds(HALF_DELAY_US);

  b = 0;
#if LSB_FIRST
  for (uint8_t i = 0; i < 8; i++) {
    delayMicroseconds(BIT_DELAY_US);
    if (digitalRead(BB_RX_PIN)) b |= (1 << i);
  }
#else
  for (int8_t i = 7; i >= 0; i--) {
    delayMicroseconds(BIT_DELAY_US);
    if (digitalRead(BB_RX_PIN)) b |= (1 << i);
  }
#endif

  delayMicroseconds(BIT_DELAY_US); // STOP
  return true;
}

bool bitbangRead(uint8_t *buf, uint8_t len, uint16_t timeout_ms = 2000) {
  for (uint8_t i = 0; i < len; i++) {
    if (!read_bitbang_byte(buf[i], timeout_ms)) return false;
  }
  return true;
}

// ==================== OLED ====================
void init_oled() {
#if OLED_ENABLED
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("BitBang UART Ready");
  display.display();
#endif
}

void show_packet(const char *label, const uint8_t *pkt) {
#if OLED_ENABLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(label);
  for (uint8_t i = 0; i < PACKET_SIZE; i++) {
    display.print(" ");
    if (pkt[i] < 0x10) display.print('0');
    display.print(pkt[i], HEX);
  }
  display.display();
#endif
}

// ==================== UART受信 ====================
void handle_uart_receive() {
  static uint8_t buf[PACKET_SIZE];
  static uint8_t idx = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();
    buf[idx++] = b;

    if (idx == PACKET_SIZE) {
      enqueue_uart_packet(buf);
      idx = 0;
    }
  }
}

// ==================== セットアップ ====================
void setup() {
  pinMode(DEBUG_PIN, OUTPUT);
  pinMode(BB_TX_PIN, OUTPUT); digitalWrite(BB_TX_PIN, HIGH);
  pinMode(BB_RX_PIN, INPUT_PULLUP);
//  Serial.begin(115200);       // USB debug
  Serial.begin(UART_BAUD);   // Hardware UART
  init_oled();
  delay(200);
}

// ==================== メインループ ====================
void loop() {
  handle_uart_receive();

  switch (state) {
    case IDLE:
      if (dequeue_uart_packet(current_packet)) {
        show_packet("SEND:", current_packet);
        bitbangWrite(current_packet, PACKET_SIZE);
        last_exec_time = millis();
        state = WAIT_REPLY;
      }
      break;

    case WAIT_REPLY:
      if (millis() - last_exec_time > 50) {
        if (bitbangRead(reply_packet, PACKET_SIZE)) {
          show_packet("RECV:", reply_packet);
          Serial.write(reply_packet, PACKET_SIZE);
        } else {
//          Serial.println("[TIMEOUT]");
        }
        state = IDLE;
      }
      break;
  }
}
