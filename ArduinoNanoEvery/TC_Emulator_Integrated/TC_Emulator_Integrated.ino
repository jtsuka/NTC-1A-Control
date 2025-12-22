/**
 * TC_Emulator_Integrated.ino
 * - Arduino Nano Every / Seeeduino Nano 用
 * - ESP32 Bridge からの指令 (300bps) を受信・解析
 * - ステータス応答 (6byte) を返送
 */

#include "tc_packet.hpp"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

using namespace tc;

/* ------------- 通信設定 ----------------*/
// Nano Everyの場合、Hardware Serial1 (RX:0, TX:1) を使用
#define SerialTC Serial1 
#define BAUD_TC 300

/* ------------- OLED設定 ----------------*/
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(128, 64, &Wire);

/* ------------- グローバル変数 ----------------*/
uint8_t ring[RING_SIZE];
uint8_t head = 0;
const bool USE_REV8 = false; // ESP32側の設定と合わせる

// シミュレーション用のダミーデータ
uint16_t dummy_tension = 45;  // 4.5g相当
uint32_t dummy_length = 1000; // 100.0m相当

/* ------------- 応答パケットの送信 ----------------*/
/**
 * TCユニットの応答をシミュレート (6バイト固定)
 * [Byte0:Status, Byte1-2:dummy, Byte3:TensionL, Byte4:TensionH, Byte5:Checksum]
 */
void sendResponse(uint8_t cmd_id) {
    uint8_t resp[6];
    resp[0] = 0x11; // Dummy Status byte
    resp[1] = 0x00;
    resp[2] = 0x00;
    resp[3] = dummy_tension & 0xFF;
    resp[4] = (dummy_tension >> 8) & 0xFF;
    resp[5] = checksum7(resp, 5); // 7-bit加算チェックサム

    // 300bpsで送信
    for(int i=0; i<6; i++) {
        uint8_t out = USE_REV8 ? rev8(resp[i]) : resp[i];
        SerialTC.write(out);
    }
    SerialTC.flush();
    
    Serial.print("Replied to CMD: ");
    Serial.println(cmd_id);
}

void setup() {
    Serial.begin(115200);   // デバッグログ用 (USB)
    SerialTC.begin(BAUD_TC); // ESP32との接続用

    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("OLED failed");
    }
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("TC Emulator: READY");
    display.display();
}

void loop() {
    while (SerialTC.available()) {
        uint8_t b = SerialTC.read();
        ring[head] = b;
        head = (head + 1) % RING_SIZE;

        // パケット解析 (PacketFactory を利用)
        if (auto p = PacketFactory::tryParse(ring, head)) {
            const char* cmdName = "UNK";
            switch(p->cmd()) {
                case CMD_RESET:    cmdName = "RESET"; break;
                case CMD_SENS_ADJ: cmdName = "SENS_ADJ"; break;
                case CMD_SEND:     cmdName = "SEND"; break;
            }
            
            // OLEDに受信情報を表示
            display.clearDisplay();
            display.setCursor(0,0);
            display.printf("RECV: %s\n", cmdName);
            display.printf("LEN : %d bytes\n", p->len);
            display.printf("DATA: %02X %02X...\n", p->buf[0], p->buf[1]);
            display.display();
            
            // 応答を返す
            sendResponse(p->cmd());
        }
    }
}