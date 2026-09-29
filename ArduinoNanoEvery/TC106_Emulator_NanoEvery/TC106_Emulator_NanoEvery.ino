/**
 * @file TC106_Emulator_NanoEvery.ino
 * @brief Arduino Nano Every用 TC106 信号エミュレーター
 * オリジナルコードの300bpsソフトウェアUARTロジックを再現。
 */

#include <Arduino.h>

// =================================================================
// 1. ピンアサイン・定数定義
// =================================================================
#define PORT_RX 2  // オリジナルのRB4に相当
#define PORT_TX 3  // オリジナルのRA4に相当
#define LED_RED 4
#define LED_GRN 5

// オリジナルコードのタイマー設定(50us周期)に基づく通信間隔
// rx_timer = 65 -> 65 * 50us = 3.25ms (約300bps)
// tx_timer = 32 -> 32 * 50us = 1.6ms (サンプリング周期)
const uint32_t BIT_TIME_US = 3300; 

// =================================================================
// 2. 状態管理変数 (オリジナル FW 準拠)
// =================================================================
volatile uint8_t rx_buffer[6];
volatile uint8_t rx_bytecntr = 0;
uint8_t setlen[3] = {0, 0, 0};
uint8_t settens = 0;

// =================================================================
// 3. 通信ユーティリティ
// =================================================================

/**
 * @brief ソフトウェアUARTによる1バイト送信 (300bps相当)
 * オリジナルの tx_300 ロジックをエミュレート
 */
void sendByteTC106(uint8_t data, bool isCommand) {
    // スタートビット (LOW)
    digitalWrite(PORT_TX, LOW);
    delayMicroseconds(BIT_TIME_US);

    // データ8ビット (LSB First)
    for (int i = 0; i < 8; i++) {
        digitalWrite(PORT_TX, (data >> i) & 0x01);
        delayMicroseconds(BIT_TIME_US);
    }

    // コマンドビット (ストップビットの直前、オリジナルコードの仕様)
    digitalWrite(PORT_TX, isCommand ? HIGH : LOW);
    delayMicroseconds(BIT_TIME_US);

    // ストップビット (HIGH)
    digitalWrite(PORT_TX, HIGH);
    delayMicroseconds(BIT_TIME_US);
}

/**
 * @brief オリジナルのレスポンス送信シーケンス (tx_300_1/2/3相当)
 */
void sendResponse(uint8_t val1, uint8_t val2) {
    sendByteTC106(val1, false); // 1バイト目
    sendByteTC106(val2, false); // 2バイト目
    sendByteTC106(0x00, true);  // 3バイト目 (コマンドフラグ付)
}

// =================================================================
// 4. 初期設定
// =================================================================
void setup() {
    Serial.begin(115200); // デバッグ用
    pinMode(PORT_RX, INPUT_PULLUP);
    pinMode(PORT_TX, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GRN, OUTPUT);

    digitalWrite(PORT_TX, HIGH); // アイドル状態はHIGH
    Serial.println("TC106 Emulator Started (300bps Mode)");
}

// =================================================================
// 5. メインループ
// =================================================================
void loop() {
    // 信号の立ち下がりを検知 (スタートビット)
    if (digitalRead(PORT_RX) == LOW) {
        uint32_t tStart = micros();
        uint8_t receivedVal = 0;

        // データのサンプリング (オリジナルコードの rx_timer/rx_bitcntr 挙動)
        delayMicroseconds(BIT_TIME_US + (BIT_TIME_US / 2)); // スタートビットを飛ばし、1ビット目中央へ

        for (int i = 0; i < 8; i++) {
            if (digitalRead(PORT_RX) == HIGH) {
                receivedVal |= (1 << i);
            }
            delayMicroseconds(BIT_TIME_US);
        }

        // コマンドビットの判定 (10ビット目)
        bool isCommand = (digitalRead(PORT_RX) == HIGH);
        
        if (isCommand) {
            // 受信データの処理 (rxreset 等のロジック)
            processCommand(receivedVal);
        } else {
            // データバッファに格納
            if (rx_bytecntr < 6) {
                rx_buffer[rx_bytecntr++] = receivedVal;
            }
        }
        
        // 次の受信まで少し待機
        delay(10);
    }
}

/**
 * @brief 受信したコマンドに応じた擬似レスポンス
 */
void processCommand(uint8_t cmd) {
    Serial.print("Received Command Type: ");
    Serial.println(cmd);

    switch (cmd) {
        case 1: // Reset Command
            if (rx_bytecntr >= 5) {
                // チェックサム検証 (オリジナル: c = sum(buffer[0..3]) & 0x7F)
                uint8_t checksum = (rx_buffer[0] + rx_buffer[1] + rx_buffer[2] + rx_buffer[3]) & 0x7F;
                if (checksum == rx_buffer[4]) {
                    Serial.println("Reset: Checksum OK. Storing parameters.");
                    setlen[0] = rx_buffer[0];
                    setlen[1] = rx_buffer[1];
                    setlen[2] = rx_buffer[2];
                    settens = rx_buffer[3];
                    
                    // 成功レスポンスを返信 (ダミー値)
                    sendResponse(0x01, 0x55); 
                } else {
                    Serial.println("Reset: Checksum Error.");
                }
            }
            break;

        case 2: // Adj Command
            Serial.println("Sensor Adjustment Sequence...");
            sendResponse(0x02, 0xAA);
            break;

        case 3: // Send Data
            Serial.println("Data Request Received.");
            sendResponse(settens, 0xFF);
            break;
    }
    rx_bytecntr = 0; // カウンタリセット
}