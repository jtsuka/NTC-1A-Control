// Seeeduino Nano  – MAIN side  (UART in ↔ Bit‑bang out 300 bps)  8‑bit checksum + OLED debug
// 2025‑06‑xx  v1.0
// ------------------------------------------------------------
// Pins  :  D2 = TX  (bit‑bang to TC)
//          D3 = RX  (bit‑bang from TC)
//          I2C OLED SSD1306 128×64 at 0x3C
// UART   : USB Serial 9600 bps (Python GUI → Nano)
// Packet : 6‑byte  [CH CMD VAL 00 00 CHK]  ,  CHK = (Σ first5) & 0xFF
//-------------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define BB_TX_PIN 2
#define BB_RX_PIN 3
#define BB_BAUD   300
#define BIT_DELAY  (1000000UL / BB_BAUD)
#define HALF_DELAY (BIT_DELAY / 2)

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint8_t uart_buf[6];   // from Python GUI
uint8_t tc_buf[6];     // response from TC

//-------------------------------------------------------------------
void setup() {
  pinMode(BB_TX_PIN, OUTPUT);  digitalWrite(BB_TX_PIN, HIGH);
  pinMode(BB_RX_PIN, INPUT_PULLUP);
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("MAIN 8‑bit ready"));
  display.display();
}

//-------------------------------------------------------------------
void loop() {
  // ---- UART 6‑byte reception ----
  if (Serial.available() >= 6) {
    for (uint8_t i=0;i<6;i++) uart_buf[i] = Serial.read();

    // 8‑bit checksum verify
    uint8_t sum = uart_buf[0]+uart_buf[1]+uart_buf[2]+uart_buf[3]+uart_buf[4];
    if ((sum & 0xFF) != uart_buf[5]) {
      showMsg("CHK ERR");
      return;
    }

    // TX packet to TC -------------------------------------------------
    showPacket("TX->TC", uart_buf);
    for (uint8_t i=0;i<6;i++) send_bitbang_byte(uart_buf[i]);

    // RX packet from TC ----------------------------------------------
    bool ok = true;
    for (uint8_t i=0;i<6;i++) {
      if (!receive_bitbang_byte(tc_buf[i],1500)) { ok=false; break; }
    }
    if (ok) {
      Serial.write(tc_buf,6); Serial.flush();
      showPacket("<-TC", tc_buf);
    } else {
      showMsg("RX Timeout");
    }
  }
}

//-------------------------------------------------------------------
void send_bitbang_byte(uint8_t b){
  digitalWrite(BB_TX_PIN, LOW);            // start bit
  delayMicroseconds(BIT_DELAY);
  for(uint8_t i=0;i<8;i++){
    digitalWrite(BB_TX_PIN, (b>>i)&1);
    delayMicroseconds(BIT_DELAY);
  }
  digitalWrite(BB_TX_PIN, HIGH);           // stop bit
  delayMicroseconds(BIT_DELAY);
}

bool receive_bitbang_byte(uint8_t &b, uint16_t timeout_ms){
  unsigned long t0=millis();
  while(digitalRead(BB_RX_PIN)==HIGH){ if(millis()-t0>timeout_ms) return false; }
  delayMicroseconds(HALF_DELAY);
  if(digitalRead(BB_RX_PIN)!=LOW) return false; // invalid start
  delayMicroseconds(HALF_DELAY);

  b=0;
  for(uint8_t i=0;i<8;i++){
    delayMicroseconds(BIT_DELAY);
    if(digitalRead(BB_RX_PIN)) b |= (1<<i);
  }
  delayMicroseconds(BIT_DELAY); // stop bit
  return true;
}

//-------------------------------------------------------------------
void showPacket(const char *label, uint8_t *buf){
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(label);
  display.println();
  for(uint8_t i=0;i<6;i++){
    display.print(F("0x")); if(buf[i]<0x10) display.print('0');
    display.print(buf[i],HEX); display.print(' ');
    if(i==2) display.println();
  }
  display.display();
  // USB debug
  Serial.print('[');Serial.print(label);Serial.print("] ");
  for(uint8_t i=0;i<6;i++){ Serial.print(buf[i],HEX); Serial.print(' ');} Serial.println();
}

void showMsg(const char* msg){
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(msg);
  display.display();
  Serial.println(msg);
}
