// ================================================
// ESP32-S3 TC Repeater with FreeRTOS
// - BitBang 300bps (GPIO2: TX, GPIO3: RX)
// - UART Pi (GPIO43: TX, GPIO44: RX)
// - OLED (GPIO4: SDA, GPIO5: SCL) ※setup()のみ表示
// - テスト送信スイッチ: GPIO8（HIGH時に模擬パケット送信）
// - 内蔵LED: GPIO21
// - FreeRTOSベース、Queue2本、Task4本
// ================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define PI_UART_TX_PIN 43
#define PI_UART_RX_PIN 44
#define TC_UART_TX_PIN 2
#define TC_UART_RX_PIN 3
#define TEST_PIN 8
#define LED_PIN 21

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

QueueHandle_t piToTcQueue;
QueueHandle_t tcToPiQueue;

const uint8_t testPacket[6] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};

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

void bitbangSendPacket(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = data[i];
    digitalWrite(TC_UART_TX_PIN, LOW); delayMicroseconds(3333); // Start bit
    for (int j = 0; j < 8; ++j) {
      digitalWrite(TC_UART_TX_PIN, b & 0x01); delayMicroseconds(3333);
      b >>= 1;
    }
    digitalWrite(TC_UART_TX_PIN, HIGH); delayMicroseconds(3333); // Stop bit
  }
}

bool bitbangReceiveByte(uint8_t* outByte) {
  if (digitalRead(TC_UART_RX_PIN) == LOW) {
    delayMicroseconds(1666); // Half-bit to center of start bit
    while (digitalRead(TC_UART_RX_PIN) == LOW) ; // wait until start bit ends
    delayMicroseconds(3333); // move to first bit

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

void uartToTcTask(void* pv) {
  uint8_t buf[6];
  for (;;) {
    if (Serial2.available() >= 6) {
      Serial2.readBytes(buf, 6);
      xQueueSend(piToTcQueue, buf, portMAX_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void tcToUartTask(void* pv) {
  uint8_t buf[6];
  for (;;) {
    if (xQueueReceive(tcToPiQueue, buf, portMAX_DELAY)) {
      Serial2.write(buf, 6);
    }
  }
}

void tcReceiverTask(void* pv) {
  uint8_t packet[6];
  size_t index = 0;
  for (;;) {
    uint8_t b;
    if (bitbangReceiveByte(&b)) {
      packet[index++] = b;
      if (index == 6) {
        xQueueSend(tcToPiQueue, packet, portMAX_DELAY);
        index = 0;
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
}

void tcSenderTask(void* pv) {
  uint8_t buf[6];
  for (;;) {
    if (xQueueReceive(piToTcQueue, buf, portMAX_DELAY)) {
      bitbangSendPacket(buf, 6);
    }
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(TEST_PIN, INPUT);
  pinMode(TC_UART_TX_PIN, OUTPUT);
  pinMode(TC_UART_RX_PIN, INPUT_PULLUP);
  digitalWrite(TC_UART_TX_PIN, HIGH);

  Wire.begin(4, 5);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  showModeOLED("TC Emulator", "with FreeRTOS");

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);

  piToTcQueue = xQueueCreate(8, 6);
  tcToPiQueue = xQueueCreate(8, 6);

  xTaskCreatePinnedToCore(uartToTcTask, "UART->TC", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(tcSenderTask,   "TC SEND", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(tcReceiverTask, "TC RECV", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(tcToUartTask,   "TC->UART", 2048, NULL, 1, NULL, 1);
}

void loop() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  if (digitalRead(TEST_PIN) == HIGH) {
    xQueueSend(piToTcQueue, (void*)testPacket, portMAX_DELAY);
  }

  vTaskDelay(pdMS_TO_TICKS(500));
}
