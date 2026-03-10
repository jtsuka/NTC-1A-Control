/**
 * @file SoftwareUartDecoder.ino
 * @brief XIAO ESP32S3用 ソフトウェアUARTデコーダー
 * * 特定の通信速度（デフォルト300bps）を想定し、信号のスタートビットを検知して
 * 後続の8ビットデータとストップビットをサンプリング・解析します。
 */

#include <Arduino.h>

// =================================================================
// 1. 設定・定数定義
// =================================================================
namespace Config {
    static const int PIN_SNIFF = 44;
    static const uint32_t BAUD_MONITOR = 115200;
    
    // 仮説に基づいたビット時間 (マイクロ秒)
    // 300bps = 1秒間に300ビット = 1,000,000us / 300 ≒ 3333.3us
    static const uint32_t BIT_INTERVAL_US = 3333; 
}

// =================================================================
// 2. ユーティリティ関数
// =================================================================
/**
 * @brief 指定したビットインデックスの信号レベルをサンプリングする
 * @param t0 スタートビットの立ち下がりエッジ時刻 (us)
 * @param bitIndex ビット位置 (0=スタート, 1~8=データ, 9=ストップ)
 * @return サンプリングされた信号レベル (HIGH/LOW)
 */
uint8_t sampleBitAt(uint32_t t0, int bitIndex) {
    // ビットの境界ではなく、ノイズの影響を受けにくい「中央付近」を狙う
    // 計算式: スタートエッジから (bitIndex + 0.5) * 1ビット期間
    uint32_t sampleTime = t0 + (Config::BIT_INTERVAL_US / 2) + (bitIndex * Config::BIT_INTERVAL_US);
    
    // 指定時刻になるまで待機（スピンロック）
    while ((int32_t)(micros() - sampleTime) < 0) {
        // CPUを専有して精密に待機
    }
    
    return digitalRead(Config::PIN_SNIFF);
}

// =================================================================
// 3. 初期設定
// =================================================================
void setup() {
    Serial.begin(Config::BAUD_MONITOR);
    while (!Serial && millis() < 2000);

    pinMode(Config::PIN_SNIFF, INPUT);

    Serial.println("=== Phase4: 300bps HYPOTHESIS CHECK START ===");
    Serial.printf("Configured Bit Time: %u us\n", Config::BIT_INTERVAL_US);
}

// =================================================================
// 4. メインループ
// =================================================================
void loop() {
    static int prevLevel = HIGH;
    int currentLevel = digitalRead(Config::PIN_SNIFF);

    // 立ち下がりエッジ（スタートビットの開始）を検出
    if (prevLevel == HIGH && currentLevel == LOW) {
        uint32_t tStart = micros();

        // 1. スタートビットの正当性確認
        // ビット期間の中央でもう一度読み、依然としてLOWであることを確認する（ノイズ対策）
        delayMicroseconds(Config::BIT_INTERVAL_US / 2);
        if (digitalRead(Config::PIN_SNIFF) != LOW) {
            prevLevel = LOW; // 状態更新
            return; // 短いノイズであれば無視
        }

        // 2. データ8ビット取得（LSB first: 最下位ビットから送られると想定）
        uint8_t receivedByte = 0;
        for (int i = 0; i < 8; i++) {
            // i=0 の時、sampleBitAt(tStart, 1) を呼ぶ（1ビット分後ろを読む）
            uint8_t b = sampleBitAt(tStart, i + 1);
            if (b == HIGH) {
                receivedByte |= (1 << i);
            }
        }

        // 3. ストップビット確認
        uint8_t stopBit = sampleBitAt(tStart, 9);

        // 4. 結果出力
        Serial.print("DEC: ");
        Serial.print(receivedByte);
        Serial.print(" | HEX: 0x");
        if (receivedByte < 16) Serial.print("0");
        Serial.print(receivedByte, HEX);
        
        // ストップビットがHIGHなら正常なUART形式
        Serial.print(" | StopBit: ");
        Serial.println(stopBit == HIGH ? "OK(H)" : "ERR(L)");

        // 連続する信号の場合、少し待機してバッファ詰まりを避ける
        // ※解析対象のフレーム間隔に合わせて調整
        delay(5);
    }

    prevLevel = currentLevel;
}