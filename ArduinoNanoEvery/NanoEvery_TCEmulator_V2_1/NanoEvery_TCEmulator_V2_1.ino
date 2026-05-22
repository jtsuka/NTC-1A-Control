/**
 * NanoEvery_TCEmulator_V2_1.ino
 *
 * TC106 テンションコントローラ エミュレータ
 * Arduino Nano Every (ATmega4809) 用
 *
 * 目的:
 *   Phase 1.1:
 *     Nano Every から TC106 風 17スロット波形を周期送信し、
 *     ESP32-S3 側の GPIO4 / tcReceiveFrame() を育てる。
 *
 *   Phase 1.2:
 *     ESP32 からのコマンドを受信し、Nano が TC106 風応答を返す。
 *
 * 解析元:
 *   FW_TC106-1_v0_0_250201-UTF8.c の tx_300_1 / tx_300_2 / tx_300_3 / data_tx
 *
 * 17スロット構造:
 *   slot 0    : HIGH preamble
 *   slot 1-7  : 7bit data, LSB first
 *               data bit = 0 -> HIGH
 *               data bit = 1 -> LOW
 *   slot 8-16 : LOW footer
 *
 * 重要:
 *   TC106 firmware の data_tx() では tx_bitcntr=1〜7 の7回だけ tx_300_2() が呼ばれる。
 *   そのため、まずは「7bitデータ + footer」として再現する。
 *
 * 配線想定:
 *   Phase 1.1 Nano -> ESP32:
 *     Nano D10 -> R4(100Ω) -> Q1未実装 G-D代替抵抗 -> JP1
 *              -> SIG_FOR_5V -> R6/R8分圧 -> JP4=2-3 -> JP7 -> ESP32 D3(GPIO4)
 *
 *   Phase 1.2 ESP32 -> Nano:
 *     ESP32 D10(GPIO9) -> Q2 -> JP3=2-3 -> Nano D13
 *     ※ Q2経由では物理論理が反転する可能性があるため RX_INVERTED で切替。
 *
 * 推奨初期設定:
 *   まずは MODE_PERIODIC = 1 のまま使用する。
 */

#include <Arduino.h>

// ======================================================
// 動作モード
// ======================================================
// 1: Phase 1.1 周期送信モード
// 0: Phase 1.2 受信→応答モード
#define MODE_PERIODIC 1

// ======================================================
// ピン設定
// ======================================================
static const uint8_t TX_PIN = 10;   // Nano D10: TC106風波形送信
static const uint8_t RX_PIN = 13;   // Nano D13: ESP32からの入力確認用

// ======================================================
// タイミング設定
// ======================================================
// TC106 firmware計算値: 3300us/slot
// PMオシロ実測: 約3400us
// まず3300で開始し、不安定なら3400へ変更する。
static const uint32_t SLOT_US = 3300;

// ======================================================
// 受信論理設定 Phase 1.2用
// ======================================================
// ESP32 -> Q2 -> Nano 経路では、Q2 ON時にNano入力がLOWになる。
// 物理入力を「TC106バスの論理」として扱うため、必要に応じて反転する。
// まずは false で開始。ESP32送信テスト時に合わなければ true へ変更。
static const bool RX_INVERTED = false;

// D13入力が浮かないよう、Phase1.2では内部プルアップを使う。
static const bool RX_USE_INTERNAL_PULLUP = true;

// ======================================================
// フレーム設定
// ======================================================
static const uint8_t FRAME_LEN = 6;
static const uint32_t PERIODIC_INTERVAL_MS = 1000;
static const uint32_t RX_TIMEOUT_MS = 700;

static const uint8_t TEST_FRAMES[][FRAME_LEN] = {
    // lastlen[0], lastlen[1], lastlen[2], txtens[0], txtens[1], marker
    { 0x11, 0x22, 0x33, 0x44, 0x55, 0x7F },
    { 0x01, 0x00, 0x00, 0x1E, 0x00, 0x7F },  // 糸長=1相当, 張力=30相当
    { 0x64, 0x00, 0x00, 0x32, 0x00, 0x7F },  // 糸長=100相当, 張力=50相当
    { 0x00, 0x64, 0x00, 0x64, 0x00, 0x7F },  // 糸長=10000相当, 張力=100相当
};
static const uint8_t NUM_FRAMES = sizeof(TEST_FRAMES) / sizeof(TEST_FRAMES[0]);

// ======================================================
// ユーティリティ
// ======================================================

static void dumpFrame(const char* tag, const uint8_t* frame, uint8_t len) {
    Serial.print(tag);
    for (uint8_t i = 0; i < len; i++) {
        if (frame[i] < 0x10) Serial.print('0');
        Serial.print(frame[i], HEX);
        if (i + 1 < len) Serial.print(' ');
    }
    Serial.println();
}

static inline bool readRxLogical() {
    bool v = (digitalRead(RX_PIN) == HIGH);
    return RX_INVERTED ? !v : v;
}

static uint8_t checksum7(const uint8_t* data, uint8_t len) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0x7F);
}

// ======================================================
// TC106風 17スロット送信
// ======================================================

static void writeBusHigh() {
    digitalWrite(TX_PIN, HIGH);
}

static void writeBusLow() {
    digitalWrite(TX_PIN, LOW);
}

/**
 * 1バイトをTC106風17スロットで送信する。
 *
 * slot0    HIGH
 * slot1-7  bit0-bit6, LSB first, 0=HIGH, 1=LOW
 * slot8-16 LOW footer
 */
static void sendByte17Slot(uint8_t data) {
    // slot 0: preamble HIGH
    writeBusHigh();
    delayMicroseconds(SLOT_US);

    // slot 1-7: 7bit data, LSB first
    for (uint8_t bit = 0; bit < 7; bit++) {
        const bool bitIsOne = ((data >> bit) & 0x01) != 0;
        if (bitIsOne) {
            writeBusLow();    // bit=1 -> LOW
        } else {
            writeBusHigh();   // bit=0 -> HIGH
        }
        delayMicroseconds(SLOT_US);
    }

    // slot 8-16: footer LOW x 9
    writeBusLow();
    for (uint8_t i = 0; i < 9; i++) {
        delayMicroseconds(SLOT_US);
    }
}

static void sendFrame17Slot(const uint8_t* frame, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        sendByte17Slot(frame[i]);
    }

    // TC106 firmware上の送信完了状態に合わせ、ラインをHIGHへ戻す。
    writeBusHigh();
}

// ======================================================
// 17スロット受信 Phase 1.2用
// ======================================================

/**
 * preamble HIGHを待つ。
 *
 * 注意:
 *   idleもHIGHなので、完全なフレーム同期としては弱い。
 *   Phase1.2の簡易確認用。
 *   ESP32側本実装では、footer LOW -> preamble HIGH のエッジ検出、
 *   またはslot列の妥当性確認を入れる。
 */
static bool waitPreambleHigh(uint32_t timeoutMs) {
    const uint32_t start = millis();

    // まずLOW状態を一度見る。これにより、単なるidle HIGH誤検出を減らす。
    while (readRxLogical()) {
        if (millis() - start > timeoutMs) return false;
    }

    // LOWからHIGHになるのを待つ。
    while (!readRxLogical()) {
        if (millis() - start > timeoutMs) return false;
    }

    return true;
}

static bool receiveByte17Slot(uint8_t& out, uint32_t timeoutMs) {
    if (!waitPreambleHigh(timeoutMs)) {
        return false;
    }

    // preambleの中央付近から、次のslot1中央へ進める。
    delayMicroseconds(SLOT_US + (SLOT_US / 2));

    uint8_t value = 0;

    // slot1-7: 7bit data
    for (uint8_t bit = 0; bit < 7; bit++) {
        const bool busHigh = readRxLogical();

        // TC106 tx_300_2 準拠: LOW = bit 1, HIGH = bit 0
        if (!busHigh) {
            value |= (1 << bit);
        }

        delayMicroseconds(SLOT_US);
    }

    // footer slot8-16 を読み飛ばす。
    // ここでは簡易的に待つだけ。将来はLOW連続確認にする。
    for (uint8_t i = 0; i < 9; i++) {
        delayMicroseconds(SLOT_US);
    }

    out = value;
    return true;
}

static bool receiveFrame17Slot(uint8_t* buf, uint8_t len, uint32_t timeoutMs) {
    for (uint8_t i = 0; i < len; i++) {
        if (!receiveByte17Slot(buf[i], timeoutMs)) {
            return false;
        }
    }
    return true;
}

// ======================================================
// 応答フレーム生成 Phase 1.2用
// ======================================================

static void buildResponseFrame(const uint8_t* cmdFrame, uint8_t* respFrame) {
    const uint8_t cmd = cmdFrame[0] & 0x07;

    switch (cmd) {
        case 1: // reset相当
            respFrame[0] = 0x00;
            respFrame[1] = 0x00;
            respFrame[2] = 0x00;
            respFrame[3] = 0x1E; // 30相当
            respFrame[4] = 0x00;
            respFrame[5] = 0x7F;
            Serial.println("[CMD] RESET");
            break;

        case 2: // sens adj相当
            respFrame[0] = 0x00;
            respFrame[1] = 0x00;
            respFrame[2] = 0x00;
            respFrame[3] = 0x50; // 80相当
            respFrame[4] = 0x00;
            respFrame[5] = 0x7F;
            Serial.println("[CMD] SENS ADJ");
            break;

        case 3: // send相当
            respFrame[0] = cmdFrame[1] & 0x7F;
            respFrame[1] = cmdFrame[2] & 0x7F;
            respFrame[2] = 0x00;
            respFrame[3] = cmdFrame[3] & 0x7F;
            respFrame[4] = cmdFrame[4] & 0x7F;
            respFrame[5] = 0x7F;
            Serial.println("[CMD] SEND");
            break;

        default:
            memcpy(respFrame, TEST_FRAMES[0], FRAME_LEN);
            Serial.print("[CMD] UNKNOWN 0x");
            Serial.println(cmd, HEX);
            break;
    }
}

// ======================================================
// Arduino setup / loop
// ======================================================

static uint8_t frameIndex = 0;
static uint32_t lastSendMs = 0;

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(TX_PIN, OUTPUT);
    writeBusHigh(); // idle HIGH

#if MODE_PERIODIC
    // Phase1.1ではRXは使わない。
#else
    pinMode(RX_PIN, RX_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT);
#endif

    Serial.println();
    Serial.println("=== NanoEvery TC106 Emulator V2.1 ===");
    Serial.println("TC106-like 17-slot, 7bit data, LSB first");
    Serial.print("TX_PIN D"); Serial.println(TX_PIN);
    Serial.print("RX_PIN D"); Serial.println(RX_PIN);
    Serial.print("SLOT_US = "); Serial.println(SLOT_US);
    Serial.print("FRAME_LEN = "); Serial.println(FRAME_LEN);

#if MODE_PERIODIC
    Serial.println("Mode: PERIODIC / Phase 1.1");
#else
    Serial.println("Mode: RESPONSE / Phase 1.2");
    Serial.print("RX_INVERTED = "); Serial.println(RX_INVERTED ? "true" : "false");
#endif

    Serial.println("[ready]");
}

void loop() {
#if MODE_PERIODIC
    if (millis() - lastSendMs >= PERIODIC_INTERVAL_MS) {
        lastSendMs = millis();

        const uint8_t* frame = TEST_FRAMES[frameIndex];
        dumpFrame("[TX] ", frame, FRAME_LEN);
        sendFrame17Slot(frame, FRAME_LEN);

        frameIndex = (frameIndex + 1) % NUM_FRAMES;
    }
#else
    uint8_t rxBuf[FRAME_LEN];

    if (receiveFrame17Slot(rxBuf, FRAME_LEN, RX_TIMEOUT_MS)) {
        dumpFrame("[RX] ", rxBuf, FRAME_LEN);

        uint8_t resp[FRAME_LEN];
        buildResponseFrame(rxBuf, resp);

        dumpFrame("[TX] ", resp, FRAME_LEN);
        sendFrame17Slot(resp, FRAME_LEN);
    }
#endif
}

/**
 * 実験メモ:
 *
 * 1. Phase 1.1では MODE_PERIODIC=1 のまま使う。
 *    Nano単体のシリアルモニタに [TX] が出れば送信動作は開始している。
 *
 * 2. ESP32側は GPIO4(D3) で受信し、17スロット受信ロジックで
 *    [RX] 11 22 33 44 55 7F のように復元できることを確認する。
 *
 * 3. 受信が不安定な場合:
 *    - SLOT_US を 3300 -> 3400 に変更
 *    - ESP32側の中央サンプリング位置を調整
 *    - JP4=2-3 / JP7短絡 / Q1代替抵抗の接続を確認
 *
 * 4. Phase 1.2では MODE_PERIODIC=0 に変更する。
 *    ESP32 -> Nanoの経路がQ2で反転している場合は RX_INVERTED=true に変更する。
 */
