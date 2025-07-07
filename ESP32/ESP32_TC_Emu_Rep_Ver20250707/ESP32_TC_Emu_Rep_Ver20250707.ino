// =========================================
// TC Emulator & Repeater Unified Sketch
// for XIAO ESP32S3 + Grove Shield
// Version: 2025-07-06
// Author: ChatGPT with user collaboration
// =========================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
extern "C" {
  #include "esp_system.h"   // esp_read_mac
  #include "esp_wifi_types.h" // ESP_MAC_WIFI_STA
}


// === OLED ===
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C
#define OLED_SDA      4
#define OLED_SCL      5
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// === Pin Definitions ===
#define UART_TX_PIN   8   // Pi へ送信
#define UART_RX_PIN   9   // Pi から受信
#define BB_TX_PIN     2   // TC へ送信 (BitBang)
#define BB_RX_PIN     3   // TC から受信 (BitBang)
#define TEST_PIN      8   // トグルスイッチ（HIGH=テスト/スリープ）
#define LED_PIN       10  // オンボードLED

// === Communication ===
#define UART_BAUDRATE 115200
#define BIT_DELAY_US  3333   // 300bps = 約3333us/bit
#define PACKET_SIZE   6

// === MACアドレスでモード判定 ===
const uint8_t MAC_REPEATER[6]  = {0x8C, 0xBF, 0xEA, 0x8E, 0x57, 0xF4};
const uint8_t MAC_EMULATOR[6]  = {0xD8, 0x3B, 0xDA, 0x74, 0x82, 0x78};
enum Mode { UNKNOWN, REPEATER, EMULATOR };
Mode currentMode = UNKNOWN;

// === State Flags ===
bool testMode = false;

// === Utility ===
void printHexPacket(uint8_t *data, const char *label) {
  char buf[32];
  sprintf(buf, "%s: %02X %02X %02X %02X %02X %02X", label,
          data[0], data[1], data[2], data[3], data[4], data[5]);
  Serial.println(buf);
  display.setCursor(0, label[0] == 'T' ? 0 : 10);
  display.print(buf);
  display.display();
}

void setLEDState() {
  if (testMode) {
    digitalWrite(LED_PIN, millis() % 1000 < 500 ? HIGH : LOW); // 点滅
  } else {
    digitalWrite(LED_PIN, currentMode == EMULATOR ? LOW : HIGH); // 点灯 or 消灯
  }
}

bool isMAC(const uint8_t *mac1, const uint8_t *mac2) {
  for (int i = 0; i < 6; i++) if (mac1[i] != mac2[i]) return false;
  return true;
}

void detectMode() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  if (isMAC(mac, MAC_REPEATER)) {
    currentMode = REPEATER;
  } else if (isMAC(mac, MAC_EMULATOR)) {
    currentMode = EMULATOR;
  } else {
    currentMode = UNKNOWN;
  }
}

void showStatusOLED(const char *msg) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(msg);
  display.display();
}

// === BitBang送信 ===
void sendBitBangPacket(uint8_t *data) {
  for (int i = 0; i < PACKET_SIZE; i++) {
    uint8_t b = data[i];
    // Start bit
    digitalWrite(BB_TX_PIN, LOW);
    delayMicroseconds(BIT_DELAY_US);
    for (int j = 0; j < 8; j++) {
      digitalWrite(BB_TX_PIN, (b >> j) & 0x01);
      delayMicroseconds(BIT_DELAY_US);
    }
    // Stop bit
    digitalWrite(BB_TX_PIN, HIGH);
    delayMicroseconds(BIT_DELAY_US);
  }
}

// === テストパケット ===
uint8_t testPacket[6] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};

// === SETUP ===
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(TEST_PIN, INPUT);
  pinMode(BB_TX_PIN, OUTPUT);
  digitalWrite(BB_TX_PIN, HIGH);

  Serial.begin(UART_BAUDRATE);
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  detectMode();

  showStatusOLED(currentMode == REPEATER ? "Mode: REPEATER" :
                 currentMode == EMULATOR ? "Mode: EMULATOR" : "Mode: UNKNOWN");
}

// === LOOP ===
void loop() {
  testMode = digitalRead(TEST_PIN) == HIGH;
  setLEDState();

  if (currentMode == REPEATER) {
    if (testMode) {
      sendBitBangPacket(testPacket);
      printHexPacket(testPacket, "TX TEST");
      delay(1000);
    } else {
      // リピーター動作未実装（必要に応じて追加）
    }
  } else if (currentMode == EMULATOR) {
    if (testMode) {
      showStatusOLED("TC Emulator Sleep");
      delay(500);
    } else {
      // 実パケット受信＆エコーバック処理未実装（今後追加）
    }
  }
  delay(10);
}
