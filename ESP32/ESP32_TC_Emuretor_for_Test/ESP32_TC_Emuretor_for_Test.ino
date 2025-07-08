// ==============================
// TC Emulator for ESP32-S3
// 300bps BitBang通信 / OLED表示 / FreeRTOS構成
// ==============================

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// ======= ピン定義 =======
#define PI_UART_TX_PIN 43   // 未使用（予約）
#define PI_UART_RX_PIN 44   // 未使用（予約）
#define TC_UART_TX_PIN 2    // BitBang送信
#define TC_UART_RX_PIN 3    // BitBang受信
#define TEST_PIN       8    // テスト用トグルスイッチ
#define LED_PIN       21    // 動作インジケータLED
#define OLED_SDA_PIN   4
#define OLED_SCL_PIN   5

// ======= 通信設定 =======
#define BAUD_RATE       300
#define BIT_DURATION_US (1000000 / BAUD_RATE)

// ======= OLED定義 =======
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ======= グローバル変数 =======
uint8_t recvBuffer[6];
QueueHandle_t sendQueue;

// ======= テストパケット =======
const uint8_t testPacket[6] = { 0x01, 0x06, 0x05, 0x00, 0x00, 0x0C };

// ==============================
// OLED初期化と表示（setup時のみ）
// ==============================
void initOLED() {
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // OLEDエラー
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("TC Emulator Ready");
  display.display();
  delay(1000);
}

// ==============================
// ビットバンギング送信 (300bps)
// ==============================
void bitBangSendByte(uint8_t b) {
  digitalWrite(TC_UART_TX_PIN, LOW); // Start bit
  delayMicroseconds(BIT_DURATION_US + BIT_DURATION_US / 2);
  for (int i = 0; i < 8; i++) {
    digitalWrite(TC_UART_TX_PIN, (b >> i) & 0x01); // LSBファースト
    delayMicroseconds(BIT_DURATION_US);
  }
  digitalWrite(TC_UART_TX_PIN, HIGH); // Stop bit
  delayMicroseconds(BIT_DURATION_US);
}

void sendPacket(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    bitBangSendByte(data[i]);
  }
}

// ==============================
// ビットバンギング受信（6バイト）
// ==============================
bool bitBangReceivePacket(uint8_t* buffer, size_t len) {
  for (size_t i = 0; i < len; i++) {
    // スタートビット待ち
    while (digitalRead(TC_UART_RX_PIN) == HIGH) {
      vTaskDelay(1);
    }
    delayMicroseconds(BIT_DURATION_US + BIT_DURATION_US / 2); // 中央に合わせる

    uint8_t byte = 0;
    for (int bit = 0; bit < 8; bit++) {
      byte |= (digitalRead(TC_UART_RX_PIN) << bit);
      delayMicroseconds(BIT_DURATION_US);
    }

    // ストップビット（無視）
    delayMicroseconds(BIT_DURATION_US);

    buffer[i] = byte;
  }
  return true;
}

// ==============================
// Task: BitBang受信 → 送信キューへ
// ==============================
void TaskBitBangReceive(void* pvParameters) {
  for (;;) {
    if (digitalRead(TEST_PIN) == HIGH) {
      uint8_t tempBuf[6];
      memcpy(tempBuf, &testPacket, 6);
      xQueueSend(sendQueue, &tempBuf, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(2000));
    } else {
      if (bitBangReceivePacket(recvBuffer, 6)) {
        uint8_t tempBuf[6];
        memcpy(tempBuf, recvBuffer, 6);
        xQueueSend(sendQueue, &tempBuf, portMAX_DELAY);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==============================
// Task: BitBang送信（エコーバック）
// ==============================
void TaskBitBangSend(void* pvParameters) {
  uint8_t packet[6];
  for (;;) {
    if (xQueueReceive(sendQueue, &packet, portMAX_DELAY) == pdTRUE) {
      sendPacket(packet, 6);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==============================
// Task: LED点滅
// ==============================
void TaskLED(void* pvParameters) {
  for (;;) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ==============================
// setup()
// ==============================
void setup() {
  pinMode(TC_UART_TX_PIN, OUTPUT);
  pinMode(TC_UART_RX_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TEST_PIN, INPUT_PULLUP);
  digitalWrite(TC_UART_TX_PIN, HIGH); // idle high

  initOLED();

  sendQueue = xQueueCreate(4, sizeof(recvBuffer));
  xTaskCreatePinnedToCore(TaskBitBangReceive, "Receive", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskBitBangSend, "Send", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskLED, "LED", 1024, NULL, 1, NULL, 1);
}

// ==============================
// loop()
// ==============================
void loop() {
  // FreeRTOSに完全移譲
}
