/**********************************************************************
  NTC-1A MAIN – Seeeduino Nano  (SoftwareSerial: D4,D5 ⇆ Pi UART / Bit-Bang: D2,D3 ⇆ TC)
  TX:D2  RX:D3   OLED 128×64
  ▸ 上半分 = Pi→TC 送信パケット
  ▸ 下半分 = TC→Pi 受信パケット
**********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
//#include <AltSoftSerial.h>
//AltSoftSerial mySerial;  // D4=RX, D5=TX が AltSoftSerial に対応


// ======== SoftwareSerial for Pi接続 (D4=RX, D5=TX) ========
#define PI_RX_PIN 4
#define PI_TX_PIN 5
SoftwareSerial mySerial(PI_RX_PIN, PI_TX_PIN);  // SoftwareSerial: Piと通信

// ======== DEBUGマクロ定義（Pi混線防止のため無効推奨）========
#define DEBUG_ENABLED 0

#if DEBUG_ENABLED
  #define DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

// ----- SpoolBuffer -----
#define MAX_QUEUE 4
uint8_t rx_queue[MAX_QUEUE][6];  // 受信専用バッファ
uint8_t tx_queue[MAX_QUEUE][6];  // 送信専用バッファ

volatile int rx_head = 0, rx_tail = 0;
volatile int tx_head = 0, tx_tail = 0;

int rx_queue_idx[MAX_QUEUE] = {0};  // 受信キューのバイトカウンタ

// UART Buffer
uint8_t last_reply[6];   // TCからの直近の応答
bool show_pending = false;
enum RelayState { IDLE, WAITING_REPLY };
RelayState state = IDLE;
uint8_t current_pkt[6], reply_pkt[6];
unsigned long t_start = 0;

// Bit-bang UART pins (to TC)
#define BB_TX_PIN 2
#define BB_RX_PIN 3
#define BB_BAUD   300
#define BIT_DELAY   (1000000UL / BB_BAUD)
#define HALF_DELAY  ((BIT_DELAY / 2) + 40)
#define BYTE_GAP_US 1500
#define STOPBIT_GAP_US 300
#define BB_SEND_DELAY 2

// UART bps for Rasppary Pi
#define UART_BPS 300

// OLED
#define OLED_W 128
#define OLED_H 64
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);
#define OLED_ENABLED 0  // ← OLED描画を止めたいときは 0 に

// 表示フラグ
bool oled_needs_refresh = false;
uint8_t uart_out[6], bb_in[6];

// =====================================================
//  受信バッファもキューに変更
void checkReceive() {
  while (mySerial.available()) {
    int next = (rx_head + 1) % MAX_QUEUE;
    if (next == rx_tail) return;

    uint8_t b = mySerial.read();
    rx_queue[rx_head][rx_queue_idx[rx_head]++] = b;

    // checkReceive() 修正案：
    if (rx_queue_idx[rx_head] == 6) {
     bool ok = enqueue_tx_packet(rx_queue[rx_head]);  // まず tx_queue へ
      if (ok) {
        rx_queue_idx[rx_head] = 0;
        rx_head = next;
      }
    // else → tx_queueが詰まっていたら破棄される（または再試行ロジックを入れる）
    }
  }
}

bool enqueue_tx_packet(const uint8_t *pkt) {
  int next = (tx_head + 1) % MAX_QUEUE;
  if (next == tx_tail) return false;  // バッファ満杯
  memcpy(tx_queue[tx_head], pkt, 6);
  tx_head = next;
  return true;
}

void draw_header() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0,0);  oled.println(F("UART -> TC"));
  oled.setCursor(0,32); oled.println(F("TC -> UART"));
  oled.display();
}

void print_hex_row(uint8_t y, const uint8_t *buf) {
  oled.setCursor(0,y);
  for(uint8_t i=0;i<6;i++){
    oled.print(F("0x")); if(buf[i]<0x10) oled.print('0');
    oled.print(buf[i],HEX); oled.print(' ');
  }
}

void show_uart_tx(const uint8_t *buf) {
#if OLED_ENABLED
  oled.fillRect(0, 8, OLED_W, 16, BLACK);
  print_hex_row(8, buf);
  oled.display();
#endif
}

void show_tc_rx(const uint8_t *buf) {
#if OLED_ENABLED
  oled.fillRect(0, 40, OLED_W, 16, BLACK);
  print_hex_row(40, buf);
  oled.display();
#endif
}

// ---------- bit-bang helpers ----------
void write_bitbang_byte(uint8_t b){
  digitalWrite(BB_TX_PIN, LOW); delayMicroseconds(BIT_DELAY);
  for(uint8_t i = 0; i < 8; i++){
    digitalWrite(BB_TX_PIN, (b >> i) & 1);
    delayMicroseconds(BIT_DELAY);
  }
  digitalWrite(BB_TX_PIN, HIGH); delayMicroseconds(BIT_DELAY);
  delayMicroseconds(STOPBIT_GAP_US);
}

bool read_bitbang_byte(uint8_t &b, uint16_t to_ms=1500){
  unsigned long t0 = millis();
  while (digitalRead(BB_RX_PIN) == HIGH) {
    if (millis() - t0 > to_ms) return false;
  }
  delayMicroseconds(HALF_DELAY);
  if (digitalRead(BB_RX_PIN) != LOW) return false;
  delayMicroseconds(HALF_DELAY);

  b = 0;
  for(uint8_t i = 0; i < 8; i++) {
    delayMicroseconds(BIT_DELAY);
    if (digitalRead(BB_RX_PIN)) b |= (1 << i);
  }

  delayMicroseconds(BIT_DELAY);
  delayMicroseconds(300);

  DEBUG_PRINT("BYTE = 0x");
  if (b < 0x10) DEBUG_PRINT('0');
  DEBUG_PRINT(b, HEX);
  DEBUG_PRINT(" [");
  for (int i = 7; i >= 0; i--) DEBUG_PRINT((b >> i) & 1);
  DEBUG_PRINTLN("]");

  return true;
}

void bitbangWrite(uint8_t* data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    write_bitbang_byte(data[i]);
    delayMicroseconds(BYTE_GAP_US);
  }
  delayMicroseconds(1000);
}

bool bitbangRead(uint8_t* data, uint8_t len, uint16_t timeout_ms = 2000) {
  unsigned long t0 = millis();
  while (digitalRead(BB_RX_PIN) == HIGH) {
    if (millis() - t0 > timeout_ms) return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    if (!read_bitbang_byte(data[i])) return false;
  }

  return true;
}

// ---------- setup ----------
void setup(){
  pinMode(BB_TX_PIN,OUTPUT); digitalWrite(BB_TX_PIN,HIGH);
  pinMode(BB_RX_PIN,INPUT_PULLUP);
  pinMode(PI_RX_PIN, INPUT_PULLUP);  // ← D4 = RX にプルアップ追加
  mySerial.begin(UART_BPS);  // SoftwareSerial for Pi
#if DEBUG_ENABLED
  Serial.begin(115200);  // USB Serial for Debug
#endif
#if OLED_ENABLED
  oled.begin(SSD1306_SWITCHCAPVCC,0x3C);
  draw_header();
#endif
}

// ---------- main loop ----------
void loop() {
  // IDEL 時だけ、レシーブチェック
  if (state == IDLE) {
    checkReceive();
  }

  switch (state) {
    case IDLE:
      if (rx_tail != rx_head) {
        memcpy(current_pkt, rx_queue[rx_tail], 6);
        rx_tail = (rx_tail + 1) % MAX_QUEUE;
        show_uart_tx(current_pkt);
        bitbangWrite(current_pkt, 6);
        delay(BB_SEND_DELAY);   // 2ms間隔を挿入（delayMicroseconds(500) → delay(2) に）
        t_start = millis();
        state = WAITING_REPLY;
      }
      break;
    case WAITING_REPLY:
      if (bitbangRead(reply_pkt, 6)) {
        noInterrupts();
        mySerial.write(reply_pkt, 6);
        mySerial.flush();
        interrupts();
        delay(10);
        // for OLED Delay
        memcpy(last_reply, reply_pkt, 6);
        oled_needs_refresh = true;
        state = IDLE;
      } else if (millis() - t_start > 2000) {
        state = IDLE;
      }
      break;
  }

  if (oled_needs_refresh) {
    show_tc_rx(last_reply);
    oled_needs_refresh = false;
  }
}
