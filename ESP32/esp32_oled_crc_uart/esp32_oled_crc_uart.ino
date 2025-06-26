#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "SafeOLED.h"

// ===== OLED 初期定義 =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 rawDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SafeOLED oled(&rawDisplay);

// ===== データ構造 =====
typedef struct {
  char line1[32];
  char line2[32];
  char status[16];
} OledMessage;

typedef struct {
  uint8_t data[6];
} TxPacket;

// ===== FreeRTOS キューとミューテックス =====
QueueHandle_t oledQueue;
QueueHandle_t txQueue;

// ===== UART 定義（例：GPIO0=TX, GPIO1=RX） =====
#define TX_UART Serial1

// ===== CRCチェック =====
uint8_t calcCRC(uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
  }
  return crc;
}

// ===== OLED描画タスク =====
void oledTask(void *param) {
  OledMessage msg;
  while (1) {
    if (xQueueReceive(oledQueue, &msg, portMAX_DELAY) == pdTRUE) {
      oled.drawText(msg.line1, msg.line2, msg.status);
    }
  }
}

// ===== UART送信タスク（Pi向け） =====
void txTask(void *param) {
  TxPacket pkt;
  while (1) {
    if (xQueueReceive(txQueue, &pkt, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < 6; i++) {
        TX_UART.write(pkt.data[i]);
      }
      TX_UART.flush();
    }
  }
}

// ===== OLED表示リクエスト関数 =====
void requestOledDraw(const char* line1, const char* line2, const char* status) {
  OledMessage msg;
  strncpy(msg.line1, line1, sizeof(msg.line1));
  strncpy(msg.line2, line2, sizeof(msg.line2));
  strncpy(msg.status, status, sizeof(msg.status));
  xQueueSend(oledQueue, &msg, 0);
}

// ===== UART送信リクエスト関数 =====
void sendPacketToPi(const uint8_t* data) {
  TxPacket pkt;
  memcpy(pkt.data, data, 6);
  xQueueSend(txQueue, &pkt, 0);
}

// ===== テスト用パケット受信シミュレーション =====
void simulateRxTask(void *param) {
  const uint8_t packet[6] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};  // CRC=0x0C
  char line1[32], line2[32], status[16];

  while (1) {
    sprintf(line1, "RX Packet:");
    sprintf(line2, "%02X %02X %02X %02X %02X %02X",
            packet[0], packet[1], packet[2], packet[3], packet[4], packet[5]);

    uint8_t crc = calcCRC((uint8_t *)packet, 5);
    if (crc == packet[5]) {
      sprintf(status, "CRC OK");
      sendPacketToPi(packet);
    } else {
      sprintf(status, "CRC NG (%02X)", crc);
    }

    requestOledDraw(line1, line2, status);
    vTaskDelay(pdMS_TO_TICKS(1500));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  oled.begin();  // OLED初期化＋クリア

  TX_UART.begin(9600, SERIAL_8N1, 1, 0);  // GPIO1=RX, GPIO0=TX（適宜変更）

  oledQueue = xQueueCreate(5, sizeof(OledMessage));
  txQueue = xQueueCreate(5, sizeof(TxPacket));

  xTaskCreatePinnedToCore(oledTask, "OledTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(txTask,   "TxTask",   4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(simulateRxTask, "RxTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // 未使用（すべてFreeRTOSタスクで処理）
}
