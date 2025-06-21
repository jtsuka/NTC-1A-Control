/**********************************************************************
  NTC-1A MAIN - Seeeduino Nano  (SoftwareSerial: D4,D5 <-> Pi UART / Bit-Bang: D2,D3 <-> TC)
  TX:D2  RX:D3   OLED 128×64
  > 上半分 = Pi→TC 送信パケット
  > 下半分 = TC→Pi 受信パケット
**********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <SoftwareSerial.h>
// 変更後
//#include <NeoSWSerial.h>

#include <AltSoftSerial.h>
AltSoftSerial mySerial;  // D8=RX, D9=TX が AltSoftSerial に対応


// ======== SoftwareSerial for Pi接続 (D4=RX, D5=TX) ========
#define PI_RX_PIN 4
#define PI_TX_PIN 5
//SoftwareSerial mySerial(PI_RX_PIN, PI_TX_PIN);  // SoftwareSerial: Piと通信
//NeoSWSerial mySerial(PI_RX_PIN, PI_TX_PIN);  // D4/D5

// ======== DEBUGマクロ定義（Pi混線防止のため無効推奨）========
#define DEBUG_ENABLED 1

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
#define HALF_DELAY  ((BIT_DELAY / 2) + 60)
#define BYTE_GAP_US 1500
#define STOPBIT_GAP_US 300
#define BB_SEND_DELAY 2

// UART bps for Rasppary Pi
#define UART_BPS 1200

// OLED
#define OLED_W 128
#define OLED_H 64
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);
#define OLED_ENABLED 0  // ← OLED描画を止めたいときは 0 に

// for Debug D6 pin
#define DEBUG_PIN 6


// 表示フラグ
bool oled_needs_refresh = false;
uint8_t uart_out[6], bb_in[6];

// =====================================================
//  受信バッファもキューに変更
#if 1
void checkReceive() {
  static byte idx_dbg = 0;

  if (mySerial.available() == 0) {
    Serial.println(F("[DEBUG] Piからの受信なし"));
    return;
  }

  while (mySerial.available()) {
    uint8_t b = mySerial.read();
    Serial.print(F("[DEBUG] Pi->TC RX[")); Serial.print(idx_dbg++); Serial.print(F("] = 0x"));
    if (b < 0x10) Serial.print('0');
    Serial.println(b, HEX);

    int next = (rx_head + 1) % MAX_QUEUE;
    if (next == rx_tail) {
      Serial.println(F("[WARN] RX Queue FULL!"));
      return;
    }

    rx_queue[rx_head][rx_queue_idx[rx_head]++] = b;

    if (rx_queue_idx[rx_head] == 6) {
      bool ok = enqueue_tx_packet(rx_queue[rx_head]);
      if (ok) {
        Serial.println(F("[INFO] RXバッファ6バイト→TXキューに登録"));
        rx_queue_idx[rx_head] = 0;
        rx_head = next;
      } else {
        Serial.println(F("[ERROR] TX Queue FULL! パケット破棄"));
      }
    }
  }
}
#else
void checkReceive() {
  // for Debug
  // checkReceive() の先頭あたりに追加
  static byte idx_dbg = 0;

  while (mySerial.available()) {
    // for Debug
    uint8_t b = mySerial.read();
    Serial.print("RX["); Serial.print(idx_dbg++); Serial.print("]=");
    Serial.println(b, HEX);
    // for Debug

    int next = (rx_head + 1) % MAX_QUEUE;
    if (next == rx_tail) return;

//    for Debug
//    uint8_t b = mySerial.read();
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
#endif

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

  // 同期パルス開始
  digitalWrite(DEBUG_PIN, HIGH);

  delayMicroseconds(HALF_DELAY);
  if (digitalRead(BB_RX_PIN) != LOW){
    digitalWrite(DEBUG_PIN, LOW);  // 中断時もLOWに戻す
    return false;
  }
  delayMicroseconds(HALF_DELAY);

  b = 0;
  for(uint8_t i = 0; i < 8; i++) {
    delayMicroseconds(BIT_DELAY);
    if (digitalRead(BB_RX_PIN)) b |= (1 << i);
  }

  delayMicroseconds(BIT_DELAY); // STOP bit
  delayMicroseconds(300);
  
   // 同期パルス終了
  digitalWrite(DEBUG_PIN, LOW);

  DEBUG_PRINT("BYTE = 0x");
  if (b < 0x10) DEBUG_PRINT('0');
  DEBUG_PRINT(b, HEX);
  DEBUG_PRINT(" [");
  for (int i = 7; i >= 0; i--) DEBUG_PRINT((b >> i) & 1);
  DEBUG_PRINTLN("]");

  return true;
}

void bitbangWrite(uint8_t* data, uint8_t len) {

  digitalWrite(DEBUG_PIN, HIGH);   // ★ 送信開始フラグを立てる

  for (uint8_t i = 0; i < len; i++) {
    write_bitbang_byte(data[i]);
    delayMicroseconds(BYTE_GAP_US);
  }
  delayMicroseconds(1000);

  digitalWrite(DEBUG_PIN, LOW);    // ★ 送信完了フラグを下げる
}

bool bitbangRead(uint8_t* data, uint8_t len, uint16_t timeout_ms = 2000) {
  unsigned long t0 = millis();
  while (digitalRead(BB_RX_PIN) == HIGH) {
    if (millis() - t0 > timeout_ms) return false;
  }

  noInterrupts();  // 割り込み禁止（受信直前）
  for (uint8_t i = 0; i < len; i++) {
    if (!read_bitbang_byte(data[i])) return false;
  }
  interrupts();    // 受信終了後に再度許可

  return true;
}

// ---------- setup ----------
void setup(){
//  Serial.begin(115200);     // ★ USBシリアルを必ず初期化する

  // for Debug D6 pin 初期化
  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(DEBUG_PIN, LOW);  // 初期状態はLOW
//  pinMode(BB_TX_PIN,OUTPUT); digitalWrite(BB_TX_PIN,HIGH);
//  pinMode(BB_RX_PIN,INPUT_PULLUP);
//  pinMode(PI_RX_PIN, INPUT_PULLUP);  // ← D4 = RX にプルアップ追加
//  mySerial.begin(UART_BPS);  // SoftwareSerial for Pi
  mySerial.begin(UART_BPS);  // AltSoftSerial for Pi
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
#if 0
    case IDLE:
      if (tx_tail != tx_head) {
        Serial.println(F("[INFO] TXキューからパケット送信準備"));
        memcpy(current_pkt, tx_queue[tx_tail], 6);
          tx_tail = (tx_tail + 1) % MAX_QUEUE;

        Serial.print(F("[SEND] "));
        for (int i = 0; i < 6; i++) {
          Serial.print(current_pkt[i], HEX); Serial.print(' ');
         }
        Serial.println();

        mySerial.end();
        bitbangWrite(current_pkt, 6);
        mySerial.begin(UART_BPS);

        t_start = millis();
        state = WAITING_REPLY;
      } else {
        Serial.println(F("[INFO] tx_queue empty（送信保留中）"));
      }
      break;
#else
    case IDLE:
      if (tx_tail != tx_head) {
        Serial.println(F("[INFO] Sending packet from tx_queue"));
        memcpy(current_pkt, tx_queue[tx_tail], 6);  // ✅ tx_queue に修正
        tx_tail = (tx_tail + 1) % MAX_QUEUE;
        show_uart_tx(current_pkt);
        /* --- 対策A：NeoSWSerial の割り込みを一時停止 --- */
        mySerial.end();                 // ❶ 割り込み源を断つ
     
        bitbangWrite(current_pkt, 6);

        mySerial.begin(UART_BPS);       // ❷ 1200 bps 割り込みを復活

        delay(BB_SEND_DELAY);
        t_start = millis();
        state = WAITING_REPLY;
      } else {
        Serial.println(F("[INFO] tx_queue empty"));
      }
      break;
#endif
    case WAITING_REPLY:
     // SoftwareSerialを一時停止して割り込み源を除去
      mySerial.end();
      noInterrupts();
      bool success = bitbangRead(reply_pkt, 6);
      interrupts();
      mySerial.begin(UART_BPS);  // 割り込み再開

      if (success) {
        mySerial.write(reply_pkt, 6);
        mySerial.flush();
        delay(10);  // OLED表示などへの考慮
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
