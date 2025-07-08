// ================================================
// ESP32-S3 TC Repeater (FreeRTOS, 可変パケット対応)
// - BitBang 300bps (GPIO2/3), UART Pi (GPIO43/44)
// - OLED表示：GPIO4(SDA), GPIO5(SCL)
// - テスト送信モード：GPIO8=HIGH
// - リピーターモード：GPIO8=LOW（UART⇔TC双方向）
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
#define MAX_PACKET_SIZE 8

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// キュー
QueueHandle_t piToTcQueue;
QueueHandle_t tcToPiQueue;

// テスト用パケット（CMD=0x01, payload=5B）
const uint8_t testPacket[6] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};

// ----------------------------------
// コマンド定義（CMDごとにペイロードサイズ）
// ----------------------------------
typedef struct {
  uint8_t cmd_id;
  uint8_t payload_size;
} CommandDef;

static const CommandDef cmd_table[] = {
  {0x01, 5}, {0x02, 0}, {0x03, 1},
  {0x04, 2}, {0x05, 0}, {0x06, 1}, {0x07, 1},
};
#define CMD_TABLE_LEN (sizeof(cmd_table) / sizeof(CommandDef))

int getPayloadSizeByCommandID(uint8_t cmd_id) {
  for (int i = 0; i < CMD_TABLE_LEN; ++i) {
    if (cmd_table[i].cmd_id == cmd_id) return cmd_table[i].payload_size;
  }
  return -1; // 未定義
}

// ----------------------------------
// BitBang送信（300bps）
// ----------------------------------
void bitbangSendPacket(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = data[i];
    digitalWrite(TC_UART_TX_PIN, LOW); delayMicroseconds(3333); // Start
    for (int j = 0; j < 8; ++j) {
      digitalWrite(TC_UART_TX_PIN, b & 0x01); delayMicroseconds(3333);
      b >>= 1;
    }
    digitalWrite(TC_UART_TX_PIN, HIGH); delayMicroseconds(3333); // Stop
  }
}

bool bitbangReceiveByte(uint8_t* outByte) {
  if (digitalRead(TC_UART_RX_PIN) == LOW) {
    delayMicroseconds(1666);
    while (digitalRead(TC_UART_RX_PIN) == LOW);
    delayMicroseconds(3333);
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

bool bitbangReceivePacket(uint8_t* packet, int* packetLen) {
  uint8_t cmd;
  if (!bitbangReceiveByte(&cmd)) return false;

  int len = getPayloadSizeByCommandID(cmd);
  if (len < 0 || len + 1 > MAX_PACKET_SIZE) return false;

  packet[0] = cmd;
  for (int i = 0; i < len; ++i) {
    while (!bitbangReceiveByte(&packet[i + 1]));
  }
  *packetLen = len + 1;
  return true;
}

bool uartReceivePacket(uint8_t* packet, int* packetLen) {
  if (Serial2.available() > 0) {
    uint8_t cmd = Serial2.read();
    int len = getPayloadSizeByCommandID(cmd);
    if (len < 0 || len + 1 > MAX_PACKET_SIZE) return false;
    packet[0] = cmd;
    while (Serial2.available() < len) delay(1); // 待機
    Serial2.readBytes(&packet[1], len);
    *packetLen = len + 1;
    return true;
  }
  return false;
}

// ----------------------------------
// FreeRTOSタスク
// ----------------------------------
void uartToTcTask(void* pv) {
  uint8_t buf[MAX_PACKET_SIZE];
  int len = 0;
  for (;;) {
    if (digitalRead(TEST_PIN) == LOW) {
      if (uartReceivePacket(buf, &len)) {
        xQueueSend(piToTcQueue, buf, portMAX_DELAY);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void tcToUartTask(void* pv) {
  uint8_t buf[MAX_PACKET_SIZE];
  for (;;) {
    if (digitalRead(TEST_PIN) == LOW) {
      if (xQueueReceive(tcToPiQueue, buf, portMAX_DELAY)) {
        int len = getPayloadSizeByCommandID(buf[0]);
        if (len >= 0) Serial2.write(buf, len + 1);
      }
    }
  }
}

void tcReceiverTask(void* pv) {
  uint8_t buf[MAX_PACKET_SIZE];
  int len = 0;
  for (;;) {
    if (digitalRead(TEST_PIN) == LOW) {
      if (bitbangReceivePacket(buf, &len)) {
        xQueueSend(tcToPiQueue, buf, portMAX_DELAY);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void tcSenderTask(void* pv) {
  uint8_t buf[MAX_PACKET_SIZE];
  for (;;) {
    if (xQueueReceive(piToTcQueue, buf, portMAX_DELAY)) {
      int len = getPayloadSizeByCommandID(buf[0]);
      if (len >= 0) bitbangSendPacket(buf, len + 1);
    }
  }
}

// ----------------------------------
// 初期化
// ----------------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(TEST_PIN, INPUT);
  pinMode(TC_UART_TX_PIN, OUTPUT);
  pinMode(TC_UART_RX_PIN, INPUT_PULLUP);
  digitalWrite(TC_UART_TX_PIN, HIGH);

  Wire.begin(4, 5);
  delay(100);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.begin(115200);
    Serial.println("OLED failed");
    while (1) delay(1000);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("TC Repeater");
  display.setCursor(0, 16);
  display.println("FreeRTOS + CMD Table");
  display.display();

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);

  piToTcQueue = xQueueCreate(8, MAX_PACKET_SIZE);
  tcToPiQueue = xQueueCreate(8, MAX_PACKET_SIZE);

  xTaskCreatePinnedToCore(uartToTcTask, "UART->TC", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(tcSenderTask, "TC SEND", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(tcReceiverTask, "TC RECV", 2048, NULL, 1, NULL, 1