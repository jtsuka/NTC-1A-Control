/**********************************************************************
  NTC-1A TC Emulator – ESP32-S3版 (Bit-Bang UART 300bps)
  TX: GPIO43  RX: GPIO44  OLED(I2C) 128×64
  - BitBangで受信→そのままエコーバック
  - OLEDにRECV/SENDを16進表示
  - GPIO8がLOWでスリープモード表示＆停止
  - 本体LED(GPIO48)が常時点滅（動作インジケータ）
  - FreeRTOS + スレッドセーフOLED対応
  - 更新日: 2025-07-01
**********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define BB_RX 44
#define BB_TX 43
#define SLEEP_MODE_PIN 8
#define LED_PIN 48

#define PACKET_SIZE 6
#define BAUD_RATE 300
#define BIT_DELAY_US (1000000UL / BAUD_RATE)
#define HALF_DELAY_US (BIT_DELAY_US / 2 + 20)
#define BYTE_GAP_US 800

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
SemaphoreHandle_t oledMutex;

uint8_t recv_buf[PACKET_SIZE];

// ===== OLED表示関数 =====
void displayHexLine(const char* label, const uint8_t* data, uint8_t row) {
  if (xSemaphoreTake(oledMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    display.setCursor(0, row * 8);
    display.printf("%s:", label);
    for (int i = 0; i < PACKET_SIZE; i++) {
      display.printf(" %02X", data[i]);
    }
    display.display();
    xSemaphoreGive(oledMutex);
  }
}

// ===== BitBang受信 =====
bool receive_packet(uint8_t* buf) {
  unsigned long t_start = millis();
  for (int i = 0; i < PACKET_SIZE; i++) {
    while (digitalRead(BB_RX) == HIGH) {
      if (millis() - t_start > 1000) return false;
    }
    delayMicroseconds(HALF_DELAY_US);
    if (digitalRead(BB_RX) != LOW) return false;

    uint8_t val = 0;
    for (int b = 0; b < 8; b++) {
      delayMicroseconds(BIT_DELAY_US - 2);
      val |= (digitalRead(BB_RX) << b);
      delayMicroseconds(30);
    }

    buf[i] = val;
    delayMicroseconds(BIT_DELAY_US + 100);
  }
  return true;
}

// ===== BitBang送信 =====
void send_packet(const uint8_t* buf) {
  for (int i = 0; i < PACKET_SIZE; i++) {
    send_byte(buf[i]);
    digitalWrite(BB_TX, HIGH);
    delayMicroseconds(BYTE_GAP_US);
  }
}

void send_byte(uint8_t val) {
  noInterrupts();
  digitalWrite(BB_TX, LOW);
  delayMicroseconds(BIT_DELAY_US);
  for (int i = 0; i < 8; i++) {
    digitalWrite(BB_TX, (val >> i) & 0x01);
    delayMicroseconds(BIT_DELAY_US);
  }
  digitalWrite(BB_TX, HIGH);
  delayMicroseconds(BIT_DELAY_US * 2);
  interrupts();
}

// ===== メイン通信タスク =====
void task_main(void* arg) {
  while (true) {
    if (receive_packet(recv_buf)) {
      displayHexLine("RECV", recv_buf, 0);
      delayMicroseconds(BIT_DELAY_US * 4);
      send_packet(recv_buf);
      displayHexLine("SEND", recv_buf, 2);
    } else {
      delay(10);
    }
  }
}

// ===== LED点滅タスク =====
void task_led(void* arg) {
  while (true) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ===== 初期化 =====
void setup() {
  pinMode(BB_RX, INPUT_PULLUP);
  pinMode(BB_TX, OUTPUT);
  digitalWrite(BB_TX, HIGH);

  pinMode(SLEEP_MODE_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);
  Wire.begin();
  oledMutex = xSemaphoreCreateMutex();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("TC Emu Ready");
  display.display();

  xTaskCreatePinnedToCore(task_main, "TCEmuMain", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(task_led,  "LEDTask",   1024, NULL, 1, NULL, 0);
}

// ===== スリープモード監視 =====
void loop() {
  if (digitalRead(SLEEP_MODE_PIN) == LOW) {
    if (xSemaphoreTake(oledMutex, pdMS_TO_TICKS(100))) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("== SLEEP MODE ==");
      display.display();
      xSemaphoreGive(oledMutex);
    }
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  delay(100);
}
