// ============================
// Seeeduino Nano 用（メイン送信側）
// D2: TX, D3: RX（ビットバンギング）
// ============================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define BB_TX_PIN 2
#define BB_RX_PIN 3
#define MODE_PIN 6  // D6でテストモード切替
#define BB_BAUD 300
#define BIT_DELAY (1000000UL / BB_BAUD)
#define HALF_DELAY (BIT_DELAY / 2)
#define MAX_RETRIES 3
#define RETRY_DELAY_MS 300

uint8_t uart_buf[6];
uint8_t recv_buf[6];

void setup() {
  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);
  pinMode(BB_RX_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MODE_PIN, INPUT_PULLUP);  // テストモードスイッチ
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (true);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Booting...");
  display.display();
  delay(500);
}

void loop() {
  bool test_mode = digitalRead(MODE_PIN) == LOW;

  if (test_mode) {
    // テストモード：定期的にダミーパケットを送信
    static unsigned long lastSend = 0;
    if (millis() - lastSend > 2000) {
      uint8_t testPacket[6] = {0x01, 0x02, 0x03, 0x00, 0x00, 0x06};
      Serial.write(testPacket, 6);
      Serial.flush();
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("[TEST MODE]");
      printPacket(testPacket, 1);
      lastSend = millis();
    }
    return;
  }

  if (Serial.available() >= 6) {
    for (int i = 0; i < 6; i++) uart_buf[i] = Serial.read();

    uint8_t sum = 0;
    for (int i = 0; i < 5; i++) sum += uart_buf[i];
    if ((sum & 0x7F) != uart_buf[5]) {
      showMessage("Checksum ERR");
      return;
    }

    digitalWrite(LED_BUILTIN, HIGH);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("->TC:");
    printPacket(uart_buf, 1);

    for (int i = 0; i < 6; i++) send_bitbang_byte(uart_buf[i]);

    bool success = false;
    for (int retry = 0; retry < MAX_RETRIES && !success; retry++) {
      delay(RETRY_DELAY_MS);
      bool ok = true;
      for (int i = 0; i < 6; i++) {
        if (!receive_bitbang_byte(recv_buf[i], 1500)) {
          ok = false;
          break;
        }
      }

      if (ok) {
        Serial.write(recv_buf, 6);
        Serial.flush();
        display.setCursor(0, 24);
        display.println("<-TC:");
        printPacket(recv_buf, 3);
        success = true;
      }
    }

    if (!success) showMessage("Recv Timeout");
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void showMessage(const char* msg) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(msg);
  display.display();
}

void printPacket(uint8_t *data, int row_start) {
  for (int i = 0; i < 6; i++) {
    display.setCursor((i % 3) * 42, row_start * 8 + (i / 3) * 8);
    display.print("0x");
    if (data[i] < 0x10) display.print("0");
    display.print(data[i], HEX);
  }
  display.display();
}

void send_bitbang_byte(uint8_t b) {
  digitalWrite(BB_TX_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(BB_TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY);
  for (int i = 0; i < 8; i++) {
    digitalWrite(BB_TX_PIN, (b >> i) & 1);
    delayMicroseconds(BIT_DELAY);
  }
  digitalWrite(BB_TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY);
}

bool receive_bitbang_byte(uint8_t &b, unsigned long timeout_ms) {
  unsigned long start = millis();
  while (digitalRead(BB_RX_PIN) == HIGH) {
    if (millis() - start > timeout_ms) return false;
  }

  delayMicroseconds(HALF_DELAY);
  if (digitalRead(BB_RX_PIN) != LOW) return false;
  delayMicroseconds(HALF_DELAY);

  b = 0;
  for (int i = 0; i < 8; i++) {
    delayMicroseconds(BIT_DELAY);
    b |= (digitalRead(BB_RX_PIN) << i);
  }

  delayMicroseconds(BIT_DELAY);  // Stop bit
  return true;
}
