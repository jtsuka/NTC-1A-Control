// Seeeduino Nano: スプール付き中継スケッチ（OLED表示あり）
// - ラズパイ側と SoftwareSerial で通信（D4=RX, D5=TX）
// - TC側とは bitbang TX/RX（例: D7:TX, D6:RX）
// - スプールで1パケットずつ処理＋OLED表示
// 2025.06.16 19:30

#include <SoftwareSerial.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define OLED_ADDR 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

#define RX_PIN 4   // ラズパイ → Seeeduino（SoftwareSerial）
#define TX_PIN 5   // Seeeduino → ラズパイ（SoftwareSerial）
#define TC_TX 7    // Seeeduino → TC
#define TC_RX 6    // TC → Seeeduino

SoftwareSerial raspi(RX_PIN, TX_PIN);

#define PACKET_SIZE 6
uint8_t packetBuf[PACKET_SIZE];
bool waitingResponse = false;
unsigned long timeoutStart = 0;

void setup() {
  pinMode(TC_TX, OUTPUT);
  pinMode(TC_RX, INPUT);
  raspi.begin(9600);
  Serial.begin(115200); // デバッグ用

  // OLED初期化
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (1); // OLED失敗時停止
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("TC Relay Ready");
  display.display();
}

void loop() {
  if (!waitingResponse && raspi.available() >= PACKET_SIZE) {
    for (int i = 0; i < PACKET_SIZE; i++) {
      packetBuf[i] = raspi.read();
    }

    // OLED表示
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("RX:");
    for (int i = 0; i < PACKET_SIZE; i++) {
      display.printf(" %02X", packetBuf[i]);
    }
    display.display();

    // TCへbitbang送信
    bitbangWrite(packetBuf, PACKET_SIZE);
    waitingResponse = true;
    timeoutStart = millis();
  }

  if (waitingResponse) {
    if (millis() - timeoutStart > 2000) {
      // タイムアウト処理
      waitingResponse = false;
      return;
    }

    // 受信チェック
    if (bitbangAvailable()) {
      uint8_t resp[PACKET_SIZE];
      if (bitbangRead(resp, PACKET_SIZE)) {
        // 受信成功 → OLED表示＆raspiに返信
        display.setCursor(0, 20);
        display.print("TX:");
        for (int i = 0; i < PACKET_SIZE; i++) {
          display.printf(" %02X", resp[i]);
          raspi.write(resp[i]);
        }
        display.display();
        waitingResponse = false;
      }
    }
  }
}

// BitBang送信（例: 330bps, 1 Start, 8 Data, 1 Stop）
void bitbangWrite(uint8_t* data, int len) {
  for (int i = 0; i < len; i++) {
    uint8_t b = data[i];
    // スタートビット
    digitalWrite(TC_TX, LOW);
    delayMicroseconds(3030);
    for (int j = 0; j < 8; j++) {
      digitalWrite(TC_TX, (b >> j) & 1);
      delayMicroseconds(3030);
    }
    // ストップビット
    digitalWrite(TC_TX, HIGH);
    delayMicroseconds(3030);
  }
}

// BitBang受信準備確認
bool bitbangAvailable() {
  return digitalRead(TC_RX) == LOW;
}

// BitBang受信（簡易）
bool bitbangRead(uint8_t* buf, int len) {
  for (int i = 0; i < len; i++) {
    // スタートビット待ち
    while (digitalRead(TC_RX) == HIGH);
    delayMicroseconds(1515); // 中央に合わせてサンプリング

    uint8_t val = 0;
    for (int j = 0; j < 8; j++) {
      val |= (digitalRead(TC_RX) << j);
      delayMicroseconds(3030);
    }
    buf[i] = val;
    delayMicroseconds(3030); // ストップビット
  }
  return true;
}
