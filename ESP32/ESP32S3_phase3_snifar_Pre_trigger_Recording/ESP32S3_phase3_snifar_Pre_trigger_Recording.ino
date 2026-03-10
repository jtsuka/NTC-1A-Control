/**
 * @file TriggerLogger.ino
 * @brief XIAO ESP32S3用 トリガ・スナップショット・ロガー
 * * 常時リングバッファにエッジを記録し、特定のパルス幅（トリガ条件）を
 * 検出した瞬間に、バッファに溜まっている過去256件のログを一挙に出力します。
 */

#include <Arduino.h>

// =================================================================
// 1. 設定・定数定義
// =================================================================
namespace Config {
    static const int PIN_SNIFF = 44;
    static const uint32_t BAUD_RATE = 115200;
    static const uint32_t MIN_EDGE_US = 50; // チャタリングフィルタ

    // トリガ条件：この範囲のパルス幅を見つけたらダンプする
    static const uint32_t TRG_UART_MIN = 2000;
    static const uint32_t TRG_UART_MAX = 5000;
    static const uint32_t TRG_LONG_MIN = 20000;

    // ログバッファサイズ（2のべき乗にすると計算が速くなります）
    static const size_t LOG_SIZE = 256;
}

// =================================================================
// 2. データ構造と共有変数
// =================================================================
struct EdgeLog {
    uint32_t timestamp_us;
    uint32_t delta_us;
    uint8_t  level;
};

// リングバッファ関連
volatile EdgeLog ringBuffer[Config::LOG_SIZE];
volatile uint16_t writeIndex = 0;

// トリガ制御関連
volatile uint32_t lastEdgeUs = 0;
volatile bool triggerRequested = false;
volatile uint32_t triggeredDeltaUs = 0;

// =================================================================
// 3. 割り込みサービスルーチン (ISR)
// =================================================================
void IRAM_ATTR handleEdgeInterrupt() {
    const uint32_t now = micros();
    const uint32_t dt = now - lastEdgeUs;

    // ノイズフィルタ
    if (dt < Config::MIN_EDGE_US) return;

    const uint8_t lv = digitalRead(Config::PIN_SNIFF);

    // バッファに記録
    ringBuffer[writeIndex].timestamp_us = now;
    ringBuffer[writeIndex].delta_us     = dt;
    ringBuffer[writeIndex].level        = lv;
    
    // 書き込みインデックスの更新
    writeIndex = (writeIndex + 1) % Config::LOG_SIZE;

    // トリガ判定
    // 1) UARTライクなパルス幅 (2ms～5ms) 
    // 2) 20ms以上の長いパルス
    if ((dt >= Config::TRG_UART_MIN && dt <= Config::TRG_UART_MAX) || 
        dt >= Config::TRG_LONG_MIN) {
        if (!triggerRequested) { // メインループが処理中なら多重トリガを避ける
            triggerRequested = true;
            triggeredDeltaUs = dt;
        }
    }

    lastEdgeUs = now;
}

// =================================================================
// 4. データ出力（ダンプ）処理
// =================================================================
void dumpRecentLogs() {
    uint16_t currentWriteIdx;
    uint32_t dt;

    // スナップショット時点の状態を固定
    noInterrupts();
    currentWriteIdx = writeIndex;
    dt = triggeredDeltaUs;
    triggerRequested = false;
    interrupts();

    Serial.println("\n===== TRIGGER DUMP START =====");
    Serial.printf("Reason: Trigger pulse detected (dt = %u us)\n", dt);
    Serial.println("index,abs_time_us,level,delta_us");

    // 書き込み位置(currentWriteIdx)から、古い順(i=0)に出力する
    for (size_t i = 0; i < Config::LOG_SIZE; i++) {
        uint16_t idx = (currentWriteIdx + i) % Config::LOG_SIZE;

        // ISRが動いているため、各要素へのアクセスもアトミックに保護
        noInterrupts();
        EdgeLog e = ringBuffer[idx];
        interrupts();

        // 未記録の領域はスキップ
        if (e.timestamp_us == 0) continue;

        Serial.print(i);
        Serial.print(",");
        Serial.print(e.timestamp_us);
        Serial.print(",");
        Serial.print(e.level ? "H" : "L");
        Serial.print(",");
        Serial.println(e.delta_us);
    }

    Serial.println("===== TRIGGER DUMP END =====");
}

// =================================================================
// 5. 初期設定とメインループ
// =================================================================
void setup() {
    Serial.begin(Config::BAUD_RATE);
    while (!Serial && millis() < 2000);

    pinMode(Config::PIN_SNIFF, INPUT);
    
    // バッファの初期化
    for (size_t i = 0; i < Config::LOG_SIZE; i++) {
        ringBuffer[i] = {0, 0, 0};
    }

    lastEdgeUs = micros();
    attachInterrupt(digitalPinToInterrupt(Config::PIN_SNIFF), handleEdgeInterrupt, CHANGE);

    Serial.println("=== Phase3: TRIGGER LOGGER START ===");
}

void loop() {
    bool doDump = false;

    // フラグチェック
    noInterrupts();
    doDump = triggerRequested;
    interrupts();

    if (doDump) {
        dumpRecentLogs();
        // 連続してダンプが走り、バッファがシリアル送信で溢れるのを防止
        delay(500);
    }
}