/*********************************************************************
  XIAO ESP32S3 UART<->UART 中継スケッチ（BitBang切替可）
  - TC側: UART1 (TX:D6=GPIO6, RX:D7=GPIO7)
  - Pi側: UART2 (TX:D1=GPIO1, RX:D2=GPIO2)
  - OLED表示: I2C (SDA:D4=GPIO4, SCL:D5=GPIO5)
  - USE_BITBANG を有効化でBitBang通信に切替
  - チェックサム対応 + FreeRTOS Task構成
*********************************************************************/

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <freertos/semphr.h>

#define PACKET_SIZE     6
#define UART_BAUD       115200
#define OLED_ADDR       0x3C
#define BIT_DURATION_US 3333

// ------- 切替定義 -------
#define USE_BITBANG 0  // 1: BitBang, 0: UART

// ------- GPIO定義 -------
#if USE_BITBANG
#define BB_TX_PIN 6  // D6
#define BB_RX_PIN 7  // D7
#else
#define TC_TX_PIN 6  // D6
#define TC_RX_PIN 7  // D7
#define PI_TX_PIN 1  // D1
#define PI_RX_PIN 2  // D2
#endif
#define OLED_SDA 4
#define OLED_SCL 5

// ------- OLED・FreeRTOS -------
Adafruit_SSD1306 display(128, 64, &Wire, -1);
SemaphoreHandle_t xMutex;

// ------- Serial定義 -------
#if !USE_BITBANG
HardwareSerial SerialTC(1);
HardwareSerial SerialPI(2);
#endif

// ------- ユーティリティ -------
bool isChecksumValid(const uint8_t* d) {
  uint8_t s = 0; for (int i = 0; i < PACKET_SIZE - 1; i++) s += d[i];
  return (s == d[5]);
}

void showOLED(const char* label, const uint8_t* data) {
  if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.print(label);
    for (int i = 0; i < PACKET_SIZE; i++) {
      display.print(" ");
      if (data[i] < 0x10) display.print("0");
      display.print(data[i], HEX);
    }
    display.display();
    xSemaphoreGive(xMutex);
  }
}

#if USE_BITBANG
void sendBitBangByte(uint8_t b) {
  digitalWrite(BB_TX_PIN, LOW); delayMicroseconds(BIT_DURATION_US);
  for (uint8_t j = 0; j < 8; j++) {
    digitalWrite(BB_TX_PIN, (b >> j) & 1);
    delayMicroseconds(BIT_DURATION_US);
  }
  digitalWrite(BB_TX_PIN, HIGH); delayMicroseconds(BIT_DURATION_US);
}

void sendBitBangPacket(uint8_t* buf) {
  for (int i = 0; i < PACKET_SIZE; i++) sendBitBangByte(buf[i]);
}

uint8_t recvBitBangByte() {
  uint8_t val = 0;
  while (digitalRead(BB_RX_PIN) == HIGH);
  delayMicroseconds(BIT_DURATION_US + BIT_DURATION_US / 2);
  for (int i = 0; i < 8; i++) {
    val |= (digitalRead(BB_RX_PIN) << i);
    delayMicroseconds(BIT_DURATION_US);
  }
  delayMicroseconds(BIT_DURATION_US);
  return val;
}
#endif

// ------- Task定義 -------
void task_forward_tc_to_pi(void* pv) {
  uint8_t buf[PACKET_SIZE];
  while (1) {
#if USE_BITBANG
    if (digitalRead(BB_RX_PIN) == LOW) {
      for (int i = 0; i < PACKET_SIZE; i++) buf[i] = recvBitBangByte();
#else
    if (SerialTC.available() >= PACKET_SIZE) {
      SerialTC.readBytes(buf, PACKET_SIZE);
#endif
      if (!isChecksumValid(buf)) continue;
#if USE_BITBANG
      SerialPI.write(buf, PACKET_SIZE);
#else
      SerialPI.write(buf, PACKET_SIZE);
#endif
      showOLED("TC -> Pi", buf);
    }
    vTaskDelay(1);
  }
}

void task_forward_pi_to_tc(void* pv) {
  uint8_t buf[PACKET_SIZE];
  while (1) {
    if (SerialPI.available() >= PACKET_SIZE) {
      SerialPI.readBytes(buf, PACKET_SIZE);
      if (!isChecksumValid(buf)) continue;
#if USE_BITBANG
      sendBitBangPacket(buf);
#else
      SerialTC.write(buf, PACKET_SIZE);
#endif
      showOLED("Pi -> TC", buf);
    }
    vTaskDelay(1);
  }
}

// ------- 初期化 -------
void setup() {
  Serial.begin(115200);
#if USE_BITBANG
  pinMode(BB_TX_PIN, OUTPUT); digitalWrite(BB_TX_PIN, HIGH);
  pinMode(BB_RX_PIN, INPUT_PULLUP);
#else
  SerialTC.begin(UART_BAUD, SERIAL_8N1, TC_RX_PIN, TC_TX_PIN);
  SerialPI.begin(UART_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
#endif
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay(); display.display();
  xMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(task_forward_tc_to_pi, "TC2PI", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task_forward_pi_to_tc, "PI2TC", 2048, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(100);
}
