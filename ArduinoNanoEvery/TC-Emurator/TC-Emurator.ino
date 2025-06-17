/**********************************************************************
  NTC-1A TC Emulator – Arduino Nano Every (Bit-Bang UART 300bps)
  RX:D3  TX:D2   OLED 128×64 表示付き

  ▸ Pi → TC ←→ Pi の双方向エコーバック機能
  ▸ 上半分：受信パケット表示（RAW）
  ▸ 下半分：返信パケット表示（Echo）

  更新日: 2025-06-17
  生成時刻: 2025-06-17 19:30 JST
**********************************************************************/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define BB_RX_PIN 3
#define BB_TX_PIN 2
#define BB_BAUD 300
#define BIT_DELAY ((1000000UL / BB_BAUD) + 26)  // 3333 + α
#define HALF_DELAY (BIT_DELAY / 2 + 20)   // ← 前より +25くらいセンターを後ろへ
#define BYTE_GAP_TIME 800
// 最速は PORTD 直接操作 (例: TX_PIN = D2 → PD2)
//#define TX_HIGH()  PORTD |=  _BV(2)
//#define TX_LOW()   PORTD &= ~_BV(2)

uint8_t recv_buf[6];

void setup() {
  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);
  pinMode(BB_RX_PIN, INPUT_PULLUP);
  Serial.begin(9600);

  // if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
  //   while (true); // OLED初期化失敗
  // }
  // display.clearDisplay();
  // display.setTextSize(1);
  // display.setTextColor(SSD1306_WHITE);
  // display.setCursor(0, 0);
  // display.println(F("TC Emulator 8-bit"));
  // display.display();
  delay(1000);
}

void loop() {
  if (receive_packet()) {
    display_packet("RAW", recv_buf);
    send_packet(recv_buf);
    delayMicroseconds(100);  // ← すぐ次のパケット受信に入らないように
  }
}

bool receive_packet() {
  unsigned long start = millis();

  for (int i = 0; i < 6; i++) {
    bool detected = false;

    for (int retry = 0; retry < 3; retry++) {
      while (digitalRead(BB_RX_PIN) == HIGH) {
        if (millis() - start > 1000) return false;
      }

      delayMicroseconds(HALF_DELAY);
      if (digitalRead(BB_RX_PIN) == LOW) {
        detected = true;
        break;
      }
      delayMicroseconds(HALF_DELAY);
    }

    if (!detected) return false;

    uint8_t b = 0;
  
    noInterrupts();  // ★ 受信中は割り込みを無効化
    for (int bit = 0; bit < 8; bit++) {
      delayMicroseconds(BIT_DELAY - 2);   // <-タイミング調整
      b |= (digitalRead(BB_RX_PIN) << bit);
      delayMicroseconds(30);  // ★安定化対策（誤検出防止）
    }
    interrupts();    // ★ 受信完了後に割り込み再開

    delayMicroseconds(BIT_DELAY + 100);  // Stop bit
    recv_buf[i] = b;
  }

  return true;
}

/***** 2. send_packet() のギャップ位置を変更 *****/
void send_packet(uint8_t *buf) {
 noInterrupts();  // ★割り込み停止

  for (int i = 0; i < 6; i++) {
    send_bitbang_byte(buf[i]);
    digitalWrite(BB_TX_PIN, HIGH);  // 明示的に戻す
    delayMicroseconds(5);
    delayMicroseconds(BYTE_GAP_TIME);
  }
  delayMicroseconds(BIT_DELAY * 2 + 300);  // ← ★最終送信後にも余白を確保
  interrupts();    // ★割り込み再開
}

void send_bitbang_byte(uint8_t b){
  noInterrupts();  // ← ★送信中の割り込み禁止（追加）

  digitalWrite(BB_TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY);

  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(BB_TX_PIN, (b >> i) & 0x01);
    delayMicroseconds(BIT_DELAY);
  }

  digitalWrite(BB_TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY * 2 + 300);  // ← Stop + 余白（2倍から調整）
  delayMicroseconds(200);           // ← ★安定化用の余白

  interrupts();  // ← ★送信後に割り込み再開（追加）
}

void display_packet(const char *label, uint8_t *buf) {
//  display.clearDisplay();
//  display.setCursor(0, 0);
//  display.print(label);
//  display.println(" Packet");
//
//  for (int i = 0; i < 6; i++) {
//    display.print("0x");
//    if (buf[i] < 0x10) display.print("0");
//    display.print(buf[i], HEX);
//    display.print(" ");
//    if (i == 2 || i == 5) display.println();
//  }
//  display.display();
}