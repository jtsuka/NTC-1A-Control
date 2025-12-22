#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_SSD1306.h>
#include "tc_packet.hpp"

/* --- ピン設定 --- */
#define PIN_TC_TX 2   // (白)TX ESP32 -> TC
#define PIN_TC_RX 3   // (黄)RX TC -> ESP32
#define UART_PI_TX 43 // Piへ
#define UART_PI_RX 44 // Piから
#define PIN_OE1 5     // LVC16T245 OE
#define PIN_DIR1 6    // LVC16T245 DIR (HIGH: ESP->TC)
#define PIN_OE2 7     // LVC16T245 OE
#define PIN_DIR2 8    // LVC16T245 DIR (LOW: TC->ESP)

/* --- 通信パラメータ --- */
#define TC_BAUD 300
#define BIT_US (1000000 / TC_BAUD) // 3333us
#define INVERT_LOGIC true          // 実機 tx_300_2 準拠

HardwareSerial SerialPi(1);
Adafruit_SSD1306 display(128, 64, &Wire);

/* --- RTOS オブジェクト --- */
static QueueHandle_t qPiToTc;
static QueueHandle_t qOLED;
struct DisplayMsg { char line[20]; uint8_t val; };

/* --- 9bit BitBang 送信 (Start + 8Data + 1Cmd + Stop) --- */
void send9bitFrame(uint8_t data, bool isCommand) {
    auto writeLevel = [](bool level) {
        bool out = INVERT_LOGIC ? !level : level; // 反転ロジック
        digitalWrite(PIN_TC_TX, out ? HIGH : LOW);
        delayMicroseconds(BIT_US);
    };

    writeLevel(false); // Start (0)
    for (int i = 0; i < 8; i++) {
        writeLevel((data >> i) & 0x01); // 8 Data
    }
    writeLevel(isCommand); // 9th bit: Command Flag
    writeLevel(true);      // Stop (1)
}

/* --- Core 1: TC通信 (最優先) --- */
void taskTC(void* pv) {
    tc::Packet p;
    while (true) {
        if (xQueueReceive(qPiToTc, &p, portMAX_DELAY)) {
            // パケットの最後の1バイトだけ Command=1 にする
            for (int i = 0; i < p.len - 1; i++) send9bitFrame(p.buf[i], false);
            send9bitFrame(p.buf[p.len - 1], true);
            
            DisplayMsg m; strcpy(m.line, "Pi -> TC"); m.val = p.cmd();
            xQueueSend(qOLED, &m, 0);
        }
    }
}

/* --- Core 0: Pi通信 & OLED --- */
void taskPiAndUI(void* pv) {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay(); display.display();
    
    uint8_t ring[tc::RING_SIZE];
    uint8_t head = 0;
    DisplayMsg m;

    while (true) {
        while (SerialPi.available()) {
            ring[head] = SerialPi.read();
            head = (head + 1) % tc::RING_SIZE;
            if (auto p = tc::PacketFactory::tryParse(ring, head)) {
                xQueueSend(qPiToTc, p, 0);
            }
        }
        if (xQueueReceive(qOLED, &m, 0) == pdTRUE) {
            display.clearDisplay();
            display.setCursor(0,0);
            display.println(m.line);
            display.printf("CMD ID: %d", m.val);
            display.display(); // 通信とは別コアなので重くても影響なし
        }
        vTaskDelay(1);
    }
}

void setup() {
    Serial.begin(115200);
    SerialPi.begin(9600, SERIAL_8N1, UART_PI_RX, UART_PI_TX);
    
    pinMode(PIN_OE1, OUTPUT); digitalWrite(PIN_OE1, LOW);
    pinMode(PIN_DIR1, OUTPUT); digitalWrite(PIN_DIR1, HIGH);
    pinMode(PIN_OE2, OUTPUT); digitalWrite(PIN_OE2, LOW);
    pinMode(PIN_DIR2, OUTPUT); digitalWrite(PIN_DIR2, LOW);

    pinMode(PIN_TC_TX, OUTPUT);
    digitalWrite(PIN_TC_TX, INVERT_LOGIC ? LOW : HIGH); // Idle電位

    qPiToTc = xQueueCreate(5, sizeof(tc::Packet));
    qOLED = xQueueCreate(10, sizeof(DisplayMsg));

    xTaskCreatePinnedToCore(taskPiAndUI, "PiUI", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(taskTC, "TC", 4096, NULL, 5, NULL, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }