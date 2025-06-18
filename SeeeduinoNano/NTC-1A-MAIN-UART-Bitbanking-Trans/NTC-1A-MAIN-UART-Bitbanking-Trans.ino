/**********************************************************************
  NTC-1A MAIN – Seeeduino Nano  (UART 9600 bps  ⇆  Bit-Bang 300 bps)
  TX:D2  RX:D3   OLED 128×64
  ▸ 上半分 = Pi→TC 送信パケット
  ▸ 下半分 = TC→Pi 受信パケット
  Serial.flush() あり・領域別描画でチラつきを最小化
**********************************************************************/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ----- SpoolBuffer ---- */
#define MAX_QUEUE 4
uint8_t queue[MAX_QUEUE][6];
volatile int q_head = 0;
volatile int q_tail = 0;

/* UART Buffer*/
uint8_t last_reply[6];   // TCからの直近の応答バッファ
bool show_pending = false; // OLED描画待ちフラグ
enum RelayState { IDLE, SENDING, WAITING_REPLY };
RelayState state = IDLE;
uint8_t current_pkt[6], reply_pkt[6];
unsigned long t_start = 0;

/* -------- pins & timing -------- */
#define BB_TX_PIN 2
#define BB_RX_PIN 3
#define BB_BAUD   300
#define BIT_DELAY   (1000000UL / BB_BAUD)
#define HALF_DELAY  ((BIT_DELAY / 2) + 20)
//#define BYTE_GAP_US 800
#define BYTE_GAP_US 1500
#define STOPBIT_GAP_US 500

/* -------- OLED -------- */
#define OLED_W 128
#define OLED_H 64
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);

/* -------- buffers -------- */
uint8_t uart_out[6];   // Pi → TC
uint8_t bb_in  [6];    // TC → Pi

/* flags */
bool drawFlag = false;


/* ===================================================== */

// Pi→TCキュー受信
void checkReceive() {
  static uint8_t buf[6];
  static int idx = 0;

  while (Serial.available()) {
    buf[idx++] = Serial.read();
    if (idx == 6) {
      int next = (q_head + 1) % MAX_QUEUE;
      if (next != q_tail) { // queue not full
        memcpy(queue[q_head], buf, 6);
        q_head = next;
      }
      idx = 0;
    }
  }
}

void draw_header()
{
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0,0);  oled.println(F("UART -> TC"));
  oled.setCursor(0,32); oled.println(F("TC -> UART"));
  oled.display();
}

void print_hex_row(uint8_t y, const uint8_t *buf)
{
  oled.setCursor(0,y);
  for(uint8_t i=0;i<6;i++){
    oled.print(F("0x")); if(buf[i]<0x10) oled.print('0');
    oled.print(buf[i],HEX); oled.print(' ');
  }
}

void show_uart_tx(const uint8_t *buf)
{
  /* 上段(8~16px)だけをクリア */
  oled.fillRect(0, 8, OLED_W, 16, BLACK);
  print_hex_row(8, buf);
  oled.display();
}

void show_tc_rx(const uint8_t *buf)
{
  /* 下段(40~48px)だけをクリア */
  oled.fillRect(0, 40, OLED_W, 16, BLACK);
  print_hex_row(40, buf);
  oled.display();
}

/* ---------- bit-bang helpers ---------- */
void write_bitbang_byte(uint8_t b){
  digitalWrite(BB_TX_PIN, LOW); delayMicroseconds(BIT_DELAY);  // Start
  for(uint8_t i = 0; i < 8; i++){
    digitalWrite(BB_TX_PIN, (b >> i) & 1);
    delayMicroseconds(BIT_DELAY);
  }
  digitalWrite(BB_TX_PIN, HIGH); delayMicroseconds(BIT_DELAY);  // Stop
  delayMicroseconds(STOPBIT_GAP_US);  // ← ★ 追加: バイト間ギャップ
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

  delayMicroseconds(BIT_DELAY);  // Stop bit
  delayMicroseconds(100);        // ← ★ Stopビット後の安定化待ち
  return true;
}


void bitbangWrite(uint8_t* data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    write_bitbang_byte(data[i]);
    delayMicroseconds(BYTE_GAP_US);  // ← ★ここを追加（1ビット10msなので最低100〜300μs推奨）
  }
}

bool bitbangRead(uint8_t* data, uint8_t len, uint16_t timeout_ms = 2000) {
  unsigned long t0 = millis();

  // ★ 応答開始待ち
  while (digitalRead(BB_RX_PIN) == HIGH) {
    if (millis() - t0 > timeout_ms) return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    if (!read_bitbang_byte(data[i])) return false;
  }

  return true;
}

/* ---------- setup ---------- */
void setup(){
  pinMode(BB_TX_PIN,OUTPUT); digitalWrite(BB_TX_PIN,HIGH);
  pinMode(BB_RX_PIN,INPUT_PULLUP);
  Serial.begin(9600);
  oled.begin(SSD1306_SWITCHCAPVCC,0x3C);
  draw_header();
}

/* ---------- main loop ---------- */

void loop() {
  checkReceive();

  switch (state) {
    case IDLE:
      if (q_tail != q_head) {
        memcpy(current_pkt, queue[q_tail], 6);
        q_tail = (q_tail + 1) % MAX_QUEUE;
        show_uart_tx(current_pkt);
        bitbangWrite(current_pkt, 6);
        delayMicroseconds(500);  // ← ★ これが無いと次のリードタイミングが間に合わない
        t_start = millis();
        state = WAITING_REPLY;
      }
      break;

    case WAITING_REPLY:
      if (bitbangRead(reply_pkt, 6)) {
#if 0
        Serial.write(reply_pkt, 6);
#else
        for (int i = 0; i < 6; i++) {
          Serial.write(reply_pkt[i]);
          delayMicroseconds(300);  // ← 300～1000μsで調整候補
        }
#endif
        Serial.flush();  // ← ★ 明示的にバッファ送信完了までブロック
#if 0
        show_tc_rx(reply_pkt);
#else
      memcpy(last_reply, reply_pkt, 6);  // ← 応答内容を保存
      drawFlag = true;                   // ← 描画フラグON
#endif
        state = IDLE;
      } else if (millis() - t_start > 2000) {
        state = IDLE;
      }
      break;
  }
  // OLEDは落ち着いたところで表示
    if (drawFlag) {
    show_tc_rx(last_reply);  // ← 表示処理は通信完了後に実行
    drawFlag = false;
  }
}
