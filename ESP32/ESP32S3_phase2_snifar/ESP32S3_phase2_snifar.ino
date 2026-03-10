/**
 * @file PulseSummary.ino
 * @brief XIAO ESP32S3用 信号パルス幅統計スケッチ
 * * 信号のHIGH/LOWそれぞれの期間を計測し、一定時間ごとに
 * 最小・最大・平均および特定のパルス幅の出現頻度をレポートします。
 */

#include <Arduino.h>

// =================================================================
// 1. 設定・定数定義
// =================================================================
namespace Config {
    static const int PIN_SNIFF = 44;
    static const uint32_t BAUD_RATE = 115200;
    static const uint32_t REPORT_INTERVAL_MS = 3000; // 統計を出す間隔
    
    // フィルタ・分類しきい値 (マイクロ秒)
    static const uint32_t MIN_EDGE_US = 50;   // チャタリング除去
    static const uint32_t THRES_SHORT = 1000; // 1ms未満をカウント
    static const uint32_t UART_MIN = 2000;    // 2ms
    static const uint32_t UART_MAX = 5000;    // 5ms
    static const uint32_t THRES_LONG = 20000; // 20ms以上
}

// =================================================================
// 2. 統計データ管理用構造体
// =================================================================
struct PulseStats {
    volatile uint32_t count = 0;
    volatile uint32_t minVal = 0xFFFFFFFF;
    volatile uint32_t maxVal = 0;
    volatile uint64_t sumVal = 0;

    void reset() {
        count = 0;
        minVal = 0xFFFFFFFF;
        maxVal = 0;
        sumVal = 0;
    }
};

// グローバル変数
volatile uint32_t lastEdgeUs = 0;
volatile uint8_t  lastLevel = LOW;

PulseStats highStats;
PulseStats lowStats;

volatile uint32_t countShort = 0;
volatile uint32_t countUartLike = 0;
volatile uint32_t countLong = 0;

// =================================================================
// 3. 割り込みサービスルーチン (ISR)
// =================================================================
void IRAM_ATTR onEdge() {
    const uint32_t now = micros();
    const uint32_t dt = now - lastEdgeUs;

    if (dt < Config::MIN_EDGE_US) return;

    // 現在のレベルを取得（エッジ直後のレベル）
    const uint8_t currentLevel = digitalRead(Config::PIN_SNIFF);

    // 「直前のレベル」が継続していた時間(dt)を統計に加算
    // 立ち上がりエッジなら、直前はLOW。立ち下がりなら、直前はHIGH。
    PulseStats* target = (lastLevel == HIGH) ? &highStats : &lowStats;
    
    target->count++;
    if (dt < target->minVal) target->minVal = dt;
    if (dt > target->maxVal) target->maxVal = dt;
    target->sumVal += dt;

    // パルス幅による分類
    if (dt < Config::THRES_SHORT) countShort++;
    if (dt >= Config::UART_MIN && dt <= Config::UART_MAX) countUartLike++;
    if (dt >= Config::THRES_LONG) countLong++;

    lastEdgeUs = now;
    lastLevel = currentLevel;
}

// =================================================================
// 4. 初期設定
// =================================================================
void setup() {
    Serial.begin(Config::BAUD_RATE);
    while (!Serial && millis() < 2000);

    pinMode(Config::PIN_SNIFF, INPUT);
    lastLevel = digitalRead(Config::PIN_SNIFF);
    lastEdgeUs = micros();

    attachInterrupt(digitalPinToInterrupt(Config::PIN_SNIFF), onEdge, CHANGE);

    Serial.println("=== Phase2: PULSE SUMMARY START ===");
    Serial.println("Reporting every " + String(Config::REPORT_INTERVAL_MS) + "ms...");
}

// =================================================================
// 5. メインループ
// =================================================================
void loop() {
    static uint32_t lastReportMs = 0;

    if (millis() - lastReportMs >= Config::REPORT_INTERVAL_MS) {
        // --- データのスナップショット取得とリセット ---
        noInterrupts();

        // HIGH統計のコピー
        uint32_t hCnt = highStats.count;
        uint32_t hMin = highStats.minVal;
        uint32_t hMax = highStats.maxVal;
        uint64_t hSum = highStats.sumVal;
        highStats.reset();

        // LOW統計のコピー
        uint32_t lCnt = lowStats.count;
        uint32_t lMin = lowStats.minVal;
        uint32_t lMax = lowStats.maxVal;
        uint64_t lSum = lowStats.sumVal;
        lowStats.reset();

        // 分類カウントのコピー
        uint32_t sCnt = countShort;
        uint32_t uCnt = countUartLike;
        uint32_t lgCnt = countLong;
        countShort = countUartLike = countLong = 0;

        interrupts();

        // --- レポート出力 ---
        Serial.println("\n--- PERIODIC SUMMARY ---");

        auto printReport = [](const char* label, uint32_t cnt, uint32_t mn, uint32_t mx, uint64_t sm) {
            Serial.print(label);
            Serial.print(": count="); Serial.print(cnt);
            if (cnt > 0) {
                Serial.print(" | min="); Serial.print(mn);
                Serial.print("us | max="); Serial.print(mx);
                Serial.print("us | avg="); Serial.print((uint32_t)(sm / cnt));
                Serial.println("us");
            } else {
                Serial.println(" | No data");
            }
        };

        printReport("HIGH", hCnt, hMin, hMax, hSum);
        printReport("LOW ", lCnt, lMin, lMax, lSum);

        Serial.printf("Filter: <1ms:%u, 2-5ms:%u, >20ms:%u\n", sCnt, uCnt, lgCnt);

        lastReportMs = millis();
    }
}