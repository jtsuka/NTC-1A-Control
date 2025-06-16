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


/* -------- pins & timing -------- */
#define BB_TX_PIN 2
#define BB_RX_PIN 3
#define BB_BAUD   300
#define BIT_DELAY   (1000000UL / BB_BAUD)
#define HALF_DELAY  (BIT_DELAY / 2)

/* -------- OLED -------- */
#define OLED_W 128
#define OLED_H 64
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);

/* -------- buffers -------- */
uint8_t uart_out[6];   // Pi → TC
uint8_t bb_in  [6];    // TC → Pi

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
  digitalWrite(BB_TX_PIN,LOW); delayMicroseconds(BIT_DELAY);
  for(uint8_t i=0;i<8;i++){ digitalWrite(BB_TX_PIN,(b>>i)&1); delayMicroseconds(BIT_DELAY);}  
  digitalWrite(BB_TX_PIN,HIGH); delayMicroseconds(BIT_DELAY);
}

bool read_bitbang_byte(uint8_t &b, uint16_t to_ms=1500){
  unsigned long t0=millis(); while(digitalRead(BB_RX_PIN)==HIGH){ if(millis()-t0>to_ms) return false; }
  delayMicroseconds(HALF_DELAY); if(digitalRead(BB_RX_PIN)!=LOW) return false; delayMicroseconds(HALF_DELAY);
  b=0; for(uint8_t i=0;i<8;i++){ delayMicroseconds(BIT_DELAY); if(digitalRead(BB_RX_PIN)) b|=1<<i; }
  delayMicroseconds(BIT_DELAY); return true;  // stop bit
}

void bitbangWrite(uint8_t* data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    write_bitbang_byte(data[i]);
  }
}

bool bitbangRead(uint8_t* data, uint8_t len, uint16_t timeout_ms = 2000) {
  unsigned long start = millis();
  for (uint8_t i = 0; i < len; i++) {
     unsigned long wait = millis();
    while (digitalRead(BB_RX_PIN) == HIGH) {
      if (millis() - start > timeout_ms) return false;
    }
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
void loop(){

 checkReceive(); // キューバッファーのチェック
  if (q_tail != q_head) {
    // キューにデータがある
    uint8_t* pkt = queue[q_tail];
    q_tail = (q_tail + 1) % MAX_QUEUE;

    // OLED表示（必要であれば）
    show_uart_tx(pkt);

    // TCにbitbang送信
    bitbangWrite(pkt, 6);

    // 応答受信（300bps用タイミングで）
    uint8_t reply[6];
    if (bitbangRead(reply, 6)) {
      Serial.write(reply, 6); // PCへ返す
        memcpy(last_reply, reply, 6);  // グローバルに保存
        show_pending = true;
    } else {
      Serial.println("[TIMEOUT]");
    }
  }
  
  // loop()後半で描画
  if (show_pending) {
    show_tc_rx(last_reply);
    show_pending = false;
  }
}

#if 0
  /* Pi→TC */
  if(Serial.available()>=6){
    for(uint8_t i=0;i<6;i++) uart_out[i]=Serial.read();
    uint8_t sum=(uart_out[0]+uart_out[1]+uart_out[2]+uart_out[3]+uart_out[4]) & 0xFF;
    if(sum==uart_out[5]){
      for(uint8_t i=0;i<6;i++) write_bitbang_byte(uart_out[i]);
      show_uart_tx(uart_out);                 // 上段更新
    }
  }
  /* TC→Pi */
  if(digitalRead(BB_RX_PIN)==LOW){
    if(read_bitbang_byte(bb_in[0])){
      for(uint8_t i=1;i<6;i++){ if(!read_bitbang_byte(bb_in[i])) return; }
      Serial.write(bb_in,6); Serial.flush();
      show_tc_rx(bb_in);                     // 下段更新
    }
  }
#endif
}
