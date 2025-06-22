/**********************************************************************
  NTC-1A MAIN (擬似スレッド + UART受信キュー付き)
  Seeeduino Nano - BitBang(D2,D3) <-> TC / UART(HW) <-> Pi
**********************************************************************/

#include <Arduino.h>

// === 定数定義 ===
#define BB_TX_PIN 2
#define BB_RX_PIN 3
#define DEBUG_PIN 6
#define UART_BAUD 1200
#define BB_BAUD 300
#define BIT_DELAY   (1000000UL / BB_BAUD)
#define HALF_DELAY  ((BIT_DELAY / 2) + 60)
#define BYTE_GAP_US 1500

// === UARTキュー定義 ===
#define UART_QUEUE_LEN 4
uint8_t uart_queue[UART_QUEUE_LEN][6];
volatile uint8_t uart_qhead = 0, uart_qtail = 0;
volatile uint8_t uart_qidx[UART_QUEUE_LEN] = {0};

// === 通信状態 ===
bool busy = false;
unsigned long last_start_time = 0;
uint8_t current_pkt[6], reply_pkt[6];

void enqueue_uart_byte(uint8_t b) {
  int next = (uart_qhead + 1) % UART_QUEUE_LEN;
  if (next == uart_qtail) return; // overflow

  uart_queue[uart_qhead][uart_qidx[uart_qhead]++] = b;
  if (uart_qidx[uart_qhead] == 6) {
    uart_qidx[uart_qhead] = 0;
    uart_qhead = next;
  }
}

void write_bitbang_byte(uint8_t b){
  digitalWrite(BB_TX_PIN, LOW); delayMicroseconds(BIT_DELAY);
  for(uint8_t i = 0; i < 8; i++){
    digitalWrite(BB_TX_PIN, (b >> i) & 1);
    delayMicroseconds(BIT_DELAY);
  }
  digitalWrite(BB_TX_PIN, HIGH); delayMicroseconds(BIT_DELAY);
  delayMicroseconds(300);
}

bool read_bitbang_byte(uint8_t &b, uint16_t to_ms=1500){
  unsigned long t0 = millis();
  while (digitalRead(BB_RX_PIN) == HIGH) {
    if (millis() - t0 > to_ms) return false;
  }
  delayMicroseconds(HALF_DELAY);
  b = 0;
  for(uint8_t i = 0; i < 8; i++) {
    delayMicroseconds(BIT_DELAY);
    if (digitalRead(BB_RX_PIN)) b |= (1 << i);
  }
  delayMicroseconds(BIT_DELAY);
  return true;
}

void bitbangWrite(uint8_t* data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    write_bitbang_byte(data[i]);
    delayMicroseconds(BYTE_GAP_US);
  }
}

bool bitbangRead(uint8_t* data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (!read_bitbang_byte(data[i])) return false;
  }
  return true;
}

void setup() {
  pinMode(BB_TX_PIN, OUTPUT); digitalWrite(BB_TX_PIN, HIGH);
  pinMode(BB_RX_PIN, INPUT_PULLUP);
  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(DEBUG_PIN, LOW);
  Serial.begin(UART_BAUD);
}

void loop() {
  // UART受信 → バイトごとにキューに貯める
  while (Serial.available()) {
    enqueue_uart_byte(Serial.read());
  }

  // 応答中でなければ次のパケット処理へ
  if (!busy && uart_qtail != uart_qhead) {
    memcpy(current_pkt, uart_queue[uart_qtail], 6);
    uart_qtail = (uart_qtail + 1) % UART_QUEUE_LEN;
    bitbangWrite(current_pkt, 6);
    last_start_time = millis();
    busy = true;
  }

  // 応答待ち処理
  if (busy && millis() - last_start_time > 10) {
    if (bitbangRead(reply_pkt, 6)) {
      Serial.write(reply_pkt, 6);
    }
    busy = false;
  }
}
