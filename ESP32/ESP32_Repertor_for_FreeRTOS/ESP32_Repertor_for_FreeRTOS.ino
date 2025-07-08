// ================================================
// ESP32-S3 TC Repeater with FreeRTOS (改良版)
// - BitBang 300bps (GPIO2: TX, GPIO3: RX)
// - UART Pi (GPIO43: TX, GPIO44: RX)
// - OLED (GPIO4: SDA, GPIO5: SCL)
// - テスト送信スイッチ: GPIO8（HIGH時に模擬パケット送信）
// - 内蔵LED: GPIO21
// ================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// GPIO定義
#define PI_UART_TX_PIN 43
#define PI_UART_RX_PIN 44
#define TC_UART_TX_PIN 2
#define TC_UART_RX_PIN 3
#define TEST_PIN 8
#define LED_PIN 21

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// FreeRTOS用キュー
QueueHandle_t piToTcQueue;
QueueHandle_t tcToPiQueue;

// シリアル排他用
portMUX_TYPE serialMux = portMUX_INITIALIZER_UNLOCKED;

// テストパケット
const uint8_t testPacket[6] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};

// シリアル出力（FreeRTOSセーフ）
void safeSerialPrint(const char* msg) {
  portENTER_CRITICAL(&serialMux);
  Serial.print(msg);
  portEXIT_CRITICAL(&serialMux);
}

void safeSerialPrintln(const char* msg) {
  portENTER_CRITICAL(&serialMux);
  Serial.println(msg);
  portEXIT_CRITICAL(&serialMux);
}

// OLED表示
void showModeOLED(const char* line1, const char* line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  display.setCursor(0, 16);
  display.println(line2);
  display.display();
}

// BitBang送信（改良：バイト間に delay）
void bitbangSendPacket(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = data[i];
    digitalWrite(TC_UART_TX_PIN, LOW); delayMicroseconds(3333); // Start bit
    for (int j = 0; j < 8; ++j) {
      digitalWrite(TC_UART_TX_PIN, b & 0x01); delayMicroseconds(3333);
      b >>= 1;
    }
    digitalWrite(TC_UART_TX_PIN, HIGH); delayMicroseconds(3333); // Stop bit
    delayMicroseconds(3000); // ← 追加: バイト間
  }
}

// BitBang受信
bool bitbangReceiveByte(uint8_t* outByte) {
  if (digitalRead(TC_UART_RX_PIN) == LOW) {
    delayMicroseconds(1666); // Half-bit
    while (digitalRead(TC_UART_RX_PIN) == LOW); // Wait start bit end
    delayMicroseconds(3333); // Move to bit center

    uint8_t b = 0;
    for (int i = 0; i < 8; ++i) {
      b >>= 1;
      if (digitalRead(TC_UART_RX_PIN)) b |= 0x80;
      delayMicroseconds(3333);
    }
    *outByte = b;
    return true;
  }
  return false;
}

// UART→TC送信キュー
void uartToTcTask(void* pv) {
  uint8_t buf[6];
  for (;;) {
    if (Serial2.available() >= 6) {
      Serial2.readBytes(buf, 6);
      if (xQueueSend(piToTcQueue, buf, portMAX_DELAY) != pdPASS) {
        safeSerialPrintln("Queue piToTc FAILED!");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// TC→UART送信キュー
void tcToUartTask(void* pv) {
  uint8_t buf[6];
  for (;;) {
    if (xQueueReceive(tcToPiQueue, buf, portMAX_DELAY)) {
      Serial2.write(buf, 6);
    }
  }
}

// BitBang受信→Pi転送
void tcReceiverTask(void* pv) {
  uint8_t packet[6];
  size_t index = 0;
  for (;;) {
    uint8_t b;
    if (bitbangReceiveByte(&b)) {
      packet[index++] = b;
      if (index == 6) {
        if (xQueueSend(tcToPiQueue, packet, portMAX_DELAY) != pdPASS) {
          safeSerialPrintln("Queue tcToPi FAILED!");
        }
        index = 0;
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
}

// Pi→BitBang送信
void tcSenderTask(void* pv) {
  uint8_t buf[6];
  for (;;) {
    if (xQueueReceive(piToTcQueue, buf, portMAX_DELAY)) {
      bitbangSendPacket(buf, 6);
    }
  }
}

// LED点滅＋テスト送信
void testLoopTask(void* pv) {
  static unsigned long lastSendTime = 0;
  static bool wasHigh = false;

  for (;;) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    bool isHigh = (digitalRead(TEST_PIN) == HIGH);
    if (isHigh) {
      unsigned long now = millis();
      if (!wasHigh || (now - lastSendTime >= 2000)) {
        if (xQueueSend(piToTcQueue, (void*)testPacket, portMAX_DELAY) != pdPASS) {
          safeSerialPrintln("Test Packet send FAILED!");
        }
        lastSendTime = now;
      }
    }
    wasHigh = isHigh;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(TEST_PIN, INPUT);
  pinMode(TC_UART_TX_PIN, OUTPUT);
  pinMode(TC_UART_RX_PIN, INPUT_PULLUP);
  digitalWrite(TC_UART_TX_PIN, HIGH); // Idle

  Wire.begin(4, 5); delay(100);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.begin(115200);
    Serial.println("OLED display init failed!");
    while (1) delay(1000);
  }
  showModeOLED("TC Repeater", "FreeRTOS Ready");

  Serial.begin(115200); // Debug
  Serial2.begin(9600, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);

  piToTcQueue = xQueueCreate(8, 6);
  tcToPiQueue = xQueueCreate(8, 6);

  xTaskCreatePinnedToCore(uartToTcTask,   "UART->TC",  2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(tcSenderTask,   "TC SEND",   2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(tcReceiverTask, "TC RECV",   2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(tcToUartTask,   "TC->UART",  2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(testLoopTask,   "Test Loop", 2048, NULL, 1, NULL, 1);
}

void loop() {
  // FreeRTOS構成のため空
}
