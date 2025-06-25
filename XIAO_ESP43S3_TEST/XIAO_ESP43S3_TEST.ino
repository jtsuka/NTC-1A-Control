/*
  ESP32S3 UART受信確認スケッチ（パケット表示付き）
  - UART2(GPIO44=RX, GPIO43=TX) を使用
  - OLEDに受信パケットを表示
  - I2C: SDA=GPIO5, SCL=GPIO6
  - ボーレート: 1200bps（必要に応じて変更）
*/

#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define UART_RX_PIN 44
#define UART_TX_PIN 43
#define UART_BAUD    1200
#define PACKET_SIZE  6
#define I2C_SDA      5
#define I2C_SCL      6
#define OLED_ADDR    0x3C

Adafruit_SSD1306 display(128, 64, &Wire, -1);
HardwareSerial SerialPI(2);  // UART2

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("UART RX Debug");
  display.display();

  Serial.begin(115200);
  SerialPI.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
}

void loop() {
  static uint8_t buf[PACKET_SIZE];

  if (SerialPI.available() >= PACKET_SIZE) {
    SerialPI.readBytes(buf, PACKET_SIZE);

    // OLED表示
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("RX: ");
    for (int i = 0; i < PACKET_SIZE; i++) {
      if (buf[i] < 0x10) display.print("0");
      display.print(buf[i], HEX);
      display.print(" ");
    }
    display.display();

    // シリアル出力
    Serial.print("[RX] ");
    for (int i = 0; i < PACKET_SIZE; i++) {
      Serial.printf("%02X ", buf[i]);
    }
    Serial.println();
  }
} 
