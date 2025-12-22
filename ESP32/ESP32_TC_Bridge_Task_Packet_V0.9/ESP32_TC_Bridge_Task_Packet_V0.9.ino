/**
 * ESP32_TC_Bridge_Integrated.ino
 * - Core 0: High-priority Communication (9600 <-> 300bps)
 * - Core 1: Low-priority UI & Logging (OLED / Serial)
 */

#include "tc_packet.hpp"
#include <HardwareSerial.h>
#include <Adafruit_SSD1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

using namespace tc;

/* ------------- ピン / 通信設定 ----------------*/
#define UART_PI_TX 43   // Piへ
#define UART_PI_RX 44   // Piから
#define UART_TC_TX 2    // TC(Nano)へ
#define UART_TC_RX 3    // TC(Nano)から
#define BAUD_PI 9600
#define BAUD_TC 300

HardwareSerial SerialPi(1);
HardwareSerial SerialTC(2);

/* レベルシフタ SN74LVC16T245 制御ピン */
#define PIN_OE1 5       // TX 出力有効 (LOWで有効)
#define PIN_DIR1 6      // TX 方向 (HIGH: ESP32 -> TC)
#define PIN_OE2 7       // RX 出力有効 (LOWで有効)
#define PIN_DIR2 8      // RX 方向 (LOW: TC -> ESP32)

/* ------------- OLED / UI 構造体 ---------------*/
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(128, 64, &Wire);

struct DisplayData {
    char title[16];
    uint8_t cmd;
    uint8_t len;
    bool isError;
};

/* ------------- RTOS オブジェクト --------------*/
static QueueHandle_t qPi2Tc;   // Pi受信バイトのキュー
static QueueHandle_t qTc2Pi;   // TC受信バイトのキュー
static QueueHandle_t qDisplay; // OLED表示用の構造体キュー

// ビット反転(rev8)の使用フラグ (TCの仕様に合わせて変更可能)
const bool USE_REV8 = false; 

// --- 描画タスク (Core 1) ---
void taskDisplay(void* pv) {
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("OLED Init Failed");
        vTaskDelete(NULL);
    }
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("TC Bridge Ready");
    display.display();

    DisplayData d;
    while(true) {
        // 表示要求が来るまで待機
        if(xQueueReceive(qDisplay, &d, portMAX_DELAY)) {
            display.clearDisplay();
            display.setCursor(0,0);
            display.println(d.title);
            display.printf("CMD: %d\n", d.cmd);
            display.printf("LEN: %d bytes\n", d.len);
            display.println(d.isError ? "Status: ERR" : "Status: OK");
            display.display();
        }
    }
}

// --- PiからTCへ (Core 0) ---
void taskPi2Tc(void* pv) {
    uint8_t ring[RING_SIZE];
    uint8_t head = 0;
    while(true) {
        uint8_t b;
        if(xQueueReceive(qPi2Tc, &b, portMAX_DELAY)) {
            ring[head] = b;
            head = (head + 1) % RING_SIZE; // インデックスをリセットせず回す

            // パケット解析 (6, 8, 12バイトのいずれかに合致するか)
            if(auto p = PacketFactory::tryParse(ring, head)) {
                // TCへ転送 (300bps)
                uint8_t out[FRAME_MAX];
                p->toBytes(out, USE_REV8);
                SerialTC.write(out, p->len);
                SerialTC.flush();

                // OLEDへ通知
                DisplayData d = {"Pi -> TC", p->cmd(), p->len, false};
                xQueueSend(qDisplay, &d, 0);
            }
        }
    }
}

// --- TCからPiへ (Core 0) ---
void taskTc2Pi(void* pv) {
    uint8_t ring[RING_SIZE];
    uint8_t head = 0;
    while(true) {
        uint8_t b;
        if(xQueueReceive(qTc2Pi, &b, portMAX_DELAY)) {
            ring[head] = b;
            head = (head + 1) % RING_SIZE;

            // TCからの応答もパケットとして解析 (通常6バイト)
            if(auto p = PacketFactory::tryParse(ring, head)) {
                // Piへ転送 (9600bps)
                SerialPi.write(p->buf, p->len);
                SerialPi.flush();

                // OLEDへ通知
                DisplayData d = {"TC -> Pi", p->cmd(), p->len, false};
                xQueueSend(qDisplay, &d, 0);
            }
        }
    }
}

/* ------------- ISR / Callbacks ----------------*/
void IRAM_ATTR onPiRx() {
    while(SerialPi.available()) {
        uint8_t b = SerialPi.read();
        xQueueSendFromISR(qPi2Tc, &b, NULL);
    }
}

void IRAM_ATTR onTcRx() {
    while(SerialTC.available()) {
        uint8_t b = SerialTC.read();
        xQueueSendFromISR(qTc2Pi, &b, NULL);
    }
}

void setup() {
    Serial.begin(115200);

    // レベルシフタ初期設定
    pinMode(PIN_OE1, OUTPUT); digitalWrite(PIN_OE1, LOW);  // TX Enable
    pinMode(PIN_DIR1, OUTPUT); digitalWrite(PIN_DIR1, HIGH); // ESP -> TC
    pinMode(PIN_OE2, OUTPUT); digitalWrite(PIN_OE2, LOW);  // RX Enable
    pinMode(PIN_DIR2, OUTPUT); digitalWrite(PIN_DIR2, LOW);  // TC -> ESP
    
    // UART初期化
    SerialPi.begin(BAUD_PI, SERIAL_8N1, UART_PI_RX, UART_PI_TX);
    SerialTC.begin(BAUD_TC, SERIAL_8N1, UART_TC_RX, UART_TC_TX);
    
    // 割り込み設定 (高速受信対応)
    SerialPi.onReceive(onPiRx);
    SerialTC.onReceive(onTcRx);

    // キュー作成
    qPi2Tc = xQueueCreate(128, sizeof(uint8_t));
    qTc2Pi = xQueueCreate(128, sizeof(uint8_t));
    qDisplay = xQueueCreate(10, sizeof(DisplayData));

    // タスクのコア割り当て
    // 通信タスクを Core 0 に (優先度高)
    xTaskCreatePinnedToCore(taskPi2Tc, "Pi2TC", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(taskTc2Pi, "TC2Pi", 4096, NULL, 5, NULL, 0);
    
    // UIタスクを Core 1 に (優先度低)
    xTaskCreatePinnedToCore(taskDisplay, "Disp", 4096, NULL, 1, NULL, 1);

    Serial.println("System Initialized");
}

void loop() {
    vTaskDelay(portMAX_DELAY); // loopタスクは休止
}