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
uint8_t queue[MAX_QUEUE][6];
volatile int q_head = 0;
volatile int q_tail = 0;

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

// OLED
#define OLED_W 128
#define OLED_H 64
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);

// 表示フラグ
bool drawFlag = false;
uint8_t uart_out[6], bb_in[6];

// =====================================================

void checkReceive() {
  static uint8_t buf[6];
  static int idx = 0;

  while (mySerial.available()) {
    buf[idx++] = mySerial.read();
    if (idx == 6) {
      int next = (q_head + 1) % MAX_QUEUE;
      if (next != q_tail) {
        memcpy(queue[q_head], buf, 6);
        q_head = next;
      }
      idx = 0;
    }
  }
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
  oled.fillRect(0, 8, OLED_W, 16, BLACK);
  print_hex_row(8, buf);
  oled.display();
}

void show_tc_rx(const uint8_t *buf) {
  oled.fillRect(0, 40, OLED_W, 16, BLACK);
  print_hex_row(40, buf);
  oled.display();
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
  mySerial.begin(9600);  // SoftwareSerial for Pi
#if DEBUG_ENABLED
  Serial.begin(115200);  // USB Serial for Debug
#endif
  oled.begin(SSD1306_SWITCHCAPVCC,0x3C);
  draw_header();
}

// ---------- main loop ----------
void loop() {
  checkReceive();

  switch (state) {
    case IDLE:
      if (q_tail != q_head) {
        memcpy(current_pkt, queue[q_tail], 6);
        q_tail = (q_tail + 1) % MAX_QUEUE;
        show_uart_tx(current_pkt);
        bitbangWrite(current_pkt, 6);
        delayMicroseconds(500);
        t_start = millis();
        state = WAITING_REPLY;
      }
      break;

    case WAITING_REPLY:
      if (bitbangRead(reply_pkt, 6)) {
        mySerial.write(reply_pkt, 6);
        mySerial.flush();
        delay(10);
        // for OLED Delay
        memcpy(last_reply, reply_pkt, 6);
        drawFlag = true;
        state = IDLE;
      } else if (millis() - t_start > 2000) {
        state = IDLE;
      }
      break;
  }

  if (drawFlag) {
    show_tc_rx(last_reply);
    drawFlag = false;
  }
}
