/**
 * @file EdgeLogger.ino
 * @brief XIAO ESP32S3用 信号エッジ解析スケッチ
 * * 指定したピンのHIGH/LOWの変化（エッジ）をマイクロ秒単位で記録し、
 * シリアルモニタへCSV形式で出力します。
 */

#include <Arduino.h>

// =================================================================
// 1. 各種設定・定数定義
// =================================================================
namespace Config {
    // 観測対象のピン (XIAO ESP32S3のピン番号)
    static const int PIN_SNIFF = 44; 

    // シリアル通信速度
    static const uint32_t BAUD_RATE = 115200;

    // チャタリング・ノイズ除去用しきい値 (マイクロ秒)
    // これ未満の短いパルスは無視されます
    static const uint32_t MIN_EDGE_INTERVAL_US = 50;

    // ログ保存用のリングバッファサイズ
    static const size_t LOG_BUFFER_SIZE = 512;
}

// =================================================================
// 2. データ構造とグローバル変数
// =================================================================
struct EdgeLog {
    uint32_t timestamp_us; // エッジ発生時刻
    uint32_t delta_us;     // 前回エッジからの経過時間
    uint8_t  level;        // エッジ後の信号レベル (1:H, 0:L)
};

// リングバッファ用変数 (割り込み内で操作するため volatile 指定)
volatile EdgeLog logBuffer[Config::LOG_BUFFER_SIZE];
volatile uint16_t writeIndex = 0;
volatile uint16_t readIndex  = 0;
volatile uint32_t lastEdgeTimeUs = 0;

// =================================================================
// 3. 割り込みサービスルーチン (ISR)
// =================================================================
/**
 * @brief ピンレベル変化時に実行される割り込みハンドラ
 * IRAM_ATTR を付与して高速な実行（内部RAM配置）を保証します
 */
void IRAM_ATTR handleEdgeInterrupt() {
    const uint32_t now = micros();
    const uint32_t dt = now - lastEdgeTimeUs;

    // ノイズフィルタ: 短すぎる変化は無視
    if (dt < Config::MIN_EDGE_INTERVAL_US) {
        return;
    }

    // バッファの次書き込み位置を計算
    uint16_t nextWriteIndex = (writeIndex + 1) % Config::LOG_BUFFER_SIZE;

    // バッファフルチェック: 読み取りが追いつかない場合は最新データを捨てる
    if (nextWriteIndex == readIndex) {
        return;
    }

    // ログをバッファに記録
    logBuffer[writeIndex].timestamp_us = now;
    logBuffer[writeIndex].delta_us     = dt;
    logBuffer[writeIndex].level        = digitalRead(Config::PIN_SNIFF);

    writeIndex = nextWriteIndex;
    lastEdgeTimeUs = now;
}

// =================================================================
// 4. 初期設定
// =================================================================
void setup() {
    Serial.begin(Config::BAUD_RATE);
    
    // XIAO ESP32S3のシリアル準備待ち
    while (!Serial && millis() < 2000); 

    pinMode(Config::PIN_SNIFF, INPUT); // 外部プルアップ/ダウンがある前提
    
    lastEdgeTimeUs = micros();

    // 割り込み開始: CHANGEモードですべてのエッジを監視
    attachInterrupt(digitalPinToInterrupt(Config::PIN_SNIFF), handleEdgeInterrupt, CHANGE);

    Serial.println("\n=== Phase1: RAW EDGE LOGGER START ===");
    Serial.println("timestamp_us,level,dt_us");
}

// =================================================================
// 5. メインループ
// =================================================================
void loop() {
    bool hasData = false;
    EdgeLog currentEdge{};

    // データの取り出し (クリティカルセクション)
    noInterrupts();
    if (readIndex != writeIndex) {
        currentEdge = logBuffer[readIndex];
        readIndex = (readIndex + 1) % Config::LOG_BUFFER_SIZE;
        hasData = true;
    }
    interrupts();

    // シリアル出力 (割り込み禁止区間外で行う)
    if (hasData) {
        Serial.print(currentEdge.timestamp_us);
        Serial.print(",");
        Serial.print(currentEdge.level ? "H" : "L");
        Serial.print(",");
        Serial.println(currentEdge.delta_us);
    }
}