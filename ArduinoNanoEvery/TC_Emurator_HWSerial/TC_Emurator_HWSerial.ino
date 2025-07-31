/**********************************************************************
  TC Emulator – Arduino Nano Every / Seeeduino Nano
  - SoftwareSerial (TX: D2, RX: D3)
  - 300 bps / 8 N 1
  - 可変長パケット (6 / 8 / 12 B)  + 7-bit 加算チェックサム
**********************************************************************/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>

/* === 設定 === */
#define BB_TX_PIN   2          // D2 → ESP32 / TC
#define BB_RX_PIN   3          // D3 ← ESP32 / TC
#define UART_BAUD   300
#define OLED_ADDR   0x3C
#define MAX_LEN     12         // 最大 12 B
#define USE_LSB     false      // true: LSB-first (rev8) で送受信

/* === OLED === */
Adafruit_SSD1306 display(128, 64, &Wire);

/* === SoftwareSerial === */
SoftwareSerial tcSerial(BB_RX_PIN, BB_TX_PIN);   // RX, TX

/* === util ---------------------------------------------------- */
inline uint8_t rev8(uint8_t v){
  v = (v >> 4) | (v << 4);
  v = ((v & 0xCC) >> 2) | ((v & 0x33) << 2);
  v = ((v & 0xAA) >> 1) | ((v & 0x55) << 1);
  return v;
}
inline uint8_t checksum7(const uint8_t *d, uint8_t n){
  uint8_t s = 0; for(uint8_t i=0;i<n;++i) s += d[i]; return s & 0x7F;
}

/* === globals ------------------------------------------------- */
uint8_t ring[32];     // シンプル 32 B リング
uint8_t wrIdx = 0;

/* === helpers ------------------------------------------------- */
void showPacket(const char *tag, const uint8_t *buf, uint8_t len, bool err){
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(tag);
  if(err) display.print(" ERR!");
  display.print(" ("); display.print(len); display.println("B)");
  for(uint8_t i=0;i<len;++i){
    char s[4]; sprintf(s,"%02X ",buf[i]);
    display.print(s);
  }
  display.display();

  Serial.print(tag);
  if(err) Serial.print(" ERR");
  Serial.print(" [");
  for(uint8_t i=0;i<len;++i){
    Serial.printf("%02X ",buf[i]);
  }
  Serial.println("]");
}

void setup() {
  Serial.begin(115200);
  tcSerial.begin(UART_BAUD);

  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)){
    while(true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("TC Emulator Ready"));
  display.display();
}

void loop() {
  /* --- 受信リングに格納 --- */
  while(tcSerial.available()){
    uint8_t b = tcSerial.read();
    if(USE_LSB) b = rev8(b);+    // 反転 = true で TX/RX ともアイドル LOW に
+    SerialTC.begin(BAUD_TC, SERIAL_8N1,
+                   UART_TC_RX, UART_TC_TX,
+                   /*invert = */ true);
    ring[wrIdx++] = b;
    wrIdx &= 0x1F;                    // 32 で巻き戻し
  }

  /
