/**
 * ESP32_GPIO4_EdgeLogger_Step4B_TEST.ino
 *
 * ============================================================
 * 位置付け(必ずお読みください)
 * ============================================================
 * これは Step4B (Nano Every TC106エミュレータ TXの物理極性再現)の
 * 波形を、実ロジックアナライザ/オシロなしで「遠隔から」補助的に
 * 確認するための、診断専用の別テストスケッチです。
 *
 *   - 本番の tcReceiveFrame()（TC106→Main方向の意味変換）とは
 *     一切関係がなく、プロトコルのデコード(command判定、checksum
 *     検証、6byteフレームの意味解釈等)は一切行いません。
 *   - 本スケッチは本番ESP32コード(ESP32-S3のCore0 RX実装等)には
 *     統合されません。単体の診断用スケッチとして独立させています。
 *   - GPIO4への出力は一切行いません(INPUT専用、Nano/TC106側への
 *     補正・制御は一切行いません)。
 *   - 本ツールの時間分解能はESP32のmicros()精度とGPIO割込み
 *     レイテンシに依存する「補助的なRUNTIME確認」であり、
 *     Step4B TX波形の最終PASS判定は、実ロジックアナライザ/
 *     オシロスコープでの確認によって行います。本ツールはそれを
 *     置き換えるものではありません。
 *
 * 配線: Nano D10 → (既存基板、能動素子なしの分圧+クランプ経路) →
 *       ESP32-S3 GPIO4 (本スケッチはこの入力を読み取るのみ)
 *
 * ============================================================
 * 使い方
 * ============================================================
 *   1. ESP32-S3に書き込み、シリアルモニタ(115200bps)を開く
 *   2. シリアルから 'a' (ARM) を1文字送信するとキャプチャ開始
 *   3. バッファ満杯 または アイドルタイムアウトで自動的に停止し、
 *      記録済みの全エッジをまとめてダンプする
 *   4. ダンプ完了後は IDLE に戻るので、再度 'a' を送ると次の
 *      キャプチャを開始できる(one-shot方式、自動連続実行はしない)
 */

#include <Arduino.h>

// ============================================================
// 設定
// ============================================================
static const uint8_t  GPIO4_PIN          = 4;
static const uint16_t BUFFER_SIZE        = 512;    // 固定長静的バッファ(数フレーム分の余裕を確保)
static const uint32_t IDLE_TIMEOUT_US    = 5000000UL; // 5秒: これだけエッジが来なければキャプチャ終了とみなす
static const uint32_t SLOT_US            = 3300;   // Step4B 1slotの想定値(換算表示専用。デコードには使わない)

// ============================================================
// 状態
// ============================================================
enum CaptureState : uint8_t {
    CAP_IDLE = 0,   // 未アーム。Serialからの 'a' コマンド待ち
    CAP_ARMED,      // キャプチャ中(ISRがedgeを記録している)
    CAP_DONE,       // キャプチャ終了(dump待ち)
};

enum StopReason : uint8_t {
    STOP_NONE = 0,
    STOP_BUFFER_FULL,
    STOP_TIMEOUT,
};

// ISRとloop()の間で共有する状態。読み書きは必ずvolatile経由。
static volatile CaptureState capturing    = CAP_IDLE;
static volatile uint16_t     edgeCount    = 0;
static volatile uint32_t     lastEdgeUs   = 0;
static volatile StopReason   stopReason   = STOP_NONE;

// 固定長静的バッファ(ISR専用の書き込み対象)。
// 構造体配列ではなく2本の配列に分離し、ISR内の処理を単純化する。
static uint32_t timestamps[BUFFER_SIZE];
static uint8_t  levels[BUFFER_SIZE];

// loop()<->ISR間の短時間排他用(ESP32はデュアルコアのため、
// 単純なnoInterrupts()ではなくportMUX_TYPEスピンロックを用いる)。
static portMUX_TYPE muxCap = portMUX_INITIALIZER_UNLOCKED;

// arm時点の基準情報(loop()側のみが読み書きする。ISRからは触らない)
static uint32_t armMicros        = 0;
static uint8_t  levelBeforeArm   = 0;

// ============================================================
// ISR: GPIO4 CHANGE割込み
// ============================================================
// ここで行うのは「timestamp取得」「level取得」「バッファへの保存」
// 「indexの更新」の4点のみ。Serial出力は一切行わない。
//
// 唯一の例外として、バッファが満杯になった瞬間にcapturingを
// CAP_DONEへ遷移させる1行を含む。これは「index更新」の一部として、
// 配列範囲外書き込みを防ぐために不可欠な処理であり、それ以上の
// ロジック(判定・演算・出力)は一切追加していない。
void IRAM_ATTR onGpio4Change() {
    if (capturing != CAP_ARMED) {
        return;
    }

    const uint32_t t   = micros();
    const uint8_t  lvl  = (uint8_t)digitalRead(GPIO4_PIN);
    const uint16_t idx  = edgeCount;

    if (idx < BUFFER_SIZE) {
        timestamps[idx] = t;
        levels[idx]     = lvl;
        lastEdgeUs       = t;

        portENTER_CRITICAL_ISR(&muxCap);
        edgeCount = idx + 1;
        if (edgeCount >= BUFFER_SIZE) {
            capturing  = CAP_DONE;
            stopReason = STOP_BUFFER_FULL;
        }
        portEXIT_CRITICAL_ISR(&muxCap);
    }
    // idx >= BUFFER_SIZE のケースは理論上発生しない
    // (capturing は上のブロックで直ちに CAP_DONE へ遷移するため)。
    // 万一の保険としても、ここでは何も書き込まない(範囲外アクセス防止)。
}

// ============================================================
// キャプチャの開始(Serialコマンド 'a' 受信時、loop()側から呼ぶ)
// ============================================================
static void armCapture() {
    portENTER_CRITICAL(&muxCap);
    edgeCount  = 0;
    stopReason = STOP_NONE;
    portEXIT_CRITICAL(&muxCap);

    levelBeforeArm = (uint8_t)digitalRead(GPIO4_PIN);
    armMicros      = micros();
    lastEdgeUs     = armMicros;

    // 最後にcapturingをARMEDにする(ここまでの初期化が先に完了している必要があるため)
    portENTER_CRITICAL(&muxCap);
    capturing = CAP_ARMED;
    portEXIT_CRITICAL(&muxCap);

    Serial.println();
    Serial.println("=== ARMED: capturing GPIO4 edges (diagnostic only, no protocol decode) ===");
    Serial.print("Level at arm time: ");
    Serial.println(levelBeforeArm ? "HIGH" : "LOW");
    Serial.println("Waiting for edges...");
}

// ============================================================
// ダンプ(loop()側。キャプチャ完了後にのみ呼ばれる。dump中は新規capture不可)
// ============================================================
static void dumpCapture() {
    // 件数と停止理由をこの瞬間だけ排他的に読み取る(スナップショット)。
    uint16_t  n      = 0;
    StopReason reason = STOP_NONE;
    portENTER_CRITICAL(&muxCap);
    n      = edgeCount;
    reason = stopReason;
    portEXIT_CRITICAL(&muxCap);

    Serial.println();
    Serial.println("=== DUMP START ===");
    Serial.print("Total edges captured: ");
    Serial.println(n);
    Serial.print("Stop reason: ");
    switch (reason) {
        case STOP_BUFFER_FULL: Serial.println("BUFFER FULL (OVERFLOW)"); break;
        case STOP_TIMEOUT:     Serial.println("IDLE TIMEOUT");           break;
        default:               Serial.println("UNKNOWN");               break;
    }
    if (reason == STOP_BUFFER_FULL) {
        Serial.println("*** OVERFLOW: buffer capacity reached, some later edges may be missing ***");
    }
    Serial.println();
    Serial.println("edge#  R/F  level  timestamp_us   dt_us      dt/3300(slot)");

    uint32_t prevUs  = armMicros;
    uint8_t  prevLvl = levelBeforeArm;

    for (uint16_t i = 0; i < n; i++) {
        const uint32_t t   = timestamps[i];
        const uint8_t  lvl = levels[i];
        const uint32_t dt  = t - prevUs; // 単調増加前提(micros()のオーバーフローは約70分周期、本テストの時間スケールでは無視可)
        const char*    rf  = (lvl > prevLvl) ? "R" : (lvl < prevLvl) ? "F" : "?"; // "?"は理論上発生しない(CHANGE割込みのため)

        Serial.print(i);
        Serial.print("\t");
        Serial.print(rf);
        Serial.print("\t");
        Serial.print(lvl ? "HIGH" : "LOW");
        Serial.print("\t");
        Serial.print(t);
        Serial.print("\t");
        Serial.print(dt);
        Serial.print("\t");
        Serial.println((double)dt / (double)SLOT_US, 2);

        prevUs  = t;
        prevLvl = lvl;
    }

    Serial.println("=== DUMP END ===");
    Serial.println("Send 'a' to arm the next capture.");
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(GPIO4_PIN, INPUT); // 出力は絶対に行わない。プルアップ/ダウンも付与しない。

    Serial.println();
    Serial.println("=== ESP32_GPIO4_EdgeLogger_Step4B_TEST ===");
    Serial.println("Diagnostic-only edge logger for Step4B Nano TX waveform (remote aux check).");
    Serial.println("NOT the production tcReceiveFrame(). No protocol decode. No output to GPIO4.");
    Serial.println("Send 'a' to arm a one-shot capture.");

    attachInterrupt(digitalPinToInterrupt(GPIO4_PIN), onGpio4Change, CHANGE);
}

void loop() {
    // ---- Serialコマンド受付(ARM) ----
    if (Serial.available() > 0) {
        const char c = (char)Serial.read();
        if (c == 'a' || c == 'A') {
            CaptureState s;
            portENTER_CRITICAL(&muxCap);
            s = capturing;
            portEXIT_CRITICAL(&muxCap);

            if (s == CAP_IDLE) {
                armCapture();
            } else {
                Serial.println("Busy: capture already in progress or pending dump.");
            }
        }
    }

    // ---- タイムアウト監視(ISRはedge発生時にしか動かないため、
    //      「長時間edgeが来ない」の検出はloop()側で行う) ----
    CaptureState s;
    portENTER_CRITICAL(&muxCap);
    s = capturing;
    portEXIT_CRITICAL(&muxCap);

    if (s == CAP_ARMED) {
        const uint32_t now = micros();
        const uint32_t sinceLastEdge = now - lastEdgeUs; // lastEdgeUsはvolatile uint32_tの単純読み出し
        if (sinceLastEdge > IDLE_TIMEOUT_US) {
            portENTER_CRITICAL(&muxCap);
            if (capturing == CAP_ARMED) { // ISRが同時にBUFFER_FULLへ遷移させていないことを再確認
                capturing  = CAP_DONE;
                stopReason = STOP_TIMEOUT;
            }
            portEXIT_CRITICAL(&muxCap);
        }
    }

    // ---- dump(CAP_DONEのときだけ。dump中は新規captureを開始しない) ----
    portENTER_CRITICAL(&muxCap);
    s = capturing;
    portEXIT_CRITICAL(&muxCap);

    if (s == CAP_DONE) {
        dumpCapture();
        portENTER_CRITICAL(&muxCap);
        capturing = CAP_IDLE; // dump完了後、次のARMコマンド待ちへ
        portEXIT_CRITICAL(&muxCap);
    }
}
