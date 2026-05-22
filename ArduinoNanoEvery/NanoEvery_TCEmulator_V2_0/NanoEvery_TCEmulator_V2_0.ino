/**
 * NanoEvery_TCEmulator_V2_0.ino
 *
 * TC106 テンションコントローラ エミュレータ (17スロット完全準拠版)
 * Arduino Nano Every (ATmega4809) 用
 *
 * 【解析元】FW_TC106-1_v0_0_250201-UTF8.c の tx_300 系関数
 *
 * 【17スロット構造 (TC106 firmware から忠実に再現)】
 *
 *   tx_300_1(): slot 0  → PORT_TX = 1 (HIGH = preamble)
 *   tx_300_2(): slot 1  → data bit0 (LSB first, 0=High / 1=Low)
 *   tx_300_2(): slot 2  → data bit1
 *   tx_300_2(): slot 3  → data bit2
 *   tx_300_2(): slot 4  → data bit3
 *   tx_300_2(): slot 5  → data bit4
 *   tx_300_2(): slot 6  → data bit5
 *   tx_300_2(): slot 7  → data bit6
 *   tx_300_3(): slot 8  → PORT_TX = 0 (LOW = footer 開始)
 *   tx_300_3(): slot 9  → LOW
 *   ...
 *   tx_300_3(): slot 16 → LOW (footer 9スロット)
 *
 * 【論理極性 (TC106 firmware tx_300_2 より)】
 *   bit=0 → PORT_TX = 1 (OC Q1 OFF = バス HIGH)
 *   bit=1 → PORT_TX = 0 (OC Q1 ON  = バス LOW)
 *   ※ Nano Every は push-pull なので:
 *     HIGH = 5V (バス HIGH 相当)
 *     LOW  = 0V (バス LOW  相当)
 *   → 論理は同じ (反転なし)
 *
 * 【6byte フレーム構成】
 *   byte 0: lastlen[0]  糸長カウンタ 1の位
 *   byte 1: lastlen[1]  糸長カウンタ 100の位
 *   byte 2: lastlen[2]  糸長カウンタ 万の位
 *   byte 3: txtens[0]   実測張力 1の位
 *   byte 4: txtens[1]   実測張力 10の位
 *   byte 5: 0x7F        固定終端マーカー
 *
 * 【Phase 対応】
 *   Phase 1.1: 周期的送信モード (片方向疎通テスト)
 *   Phase 1.2: 受信→応答モード (双方向ブリッジテスト)
 *
 * 【配線 (ESP32-S3 スニファ基板ベータ版)】
 *   Nano D10 → R4(100Ω) → Q1代替(1kΩ) → JP1 → 5V系分圧 → JP4=2-3 → ESP32 D3
 *   Nano D13 ← JP3=2-3 ← Q2ドレイン ← JP5 ← ESP32 D10
 *
 * 作成: 2026年
 * 解析: FW_TC106-1_v0_0_250201-UTF8.c の tx_300_1/2/3, tx_cntrset, data_tx
 */

// ======================================================
// 設定
// ======================================================

// 動作モード選択
// #define MODE_PERIODIC    // Phase 1.1: 周期的送信 (コメントアウトで MODE_RESPONSE に)
#define MODE_RESPONSE       // Phase 1.2: 受信後に応答

// ピン設定
static const uint8_t TX_PIN = 10;   // D10: 送信 (5V push-pull)
static const uint8_t RX_PIN = 13;   // D13: 受信 (ESP32 D10 → Q2 → JP3=2-3 → ここ)

// スロット時間: TC106 firmware より
//   TMR1 = 50us 周期
//   tx_timer = 32 → 50us × 33 = 1.65ms ごとに data_tx() 呼び出し
//   data_tx() は tx_cntr の偶数回のみ処理
//   → 実質 1.65ms × 2 = 3.3ms / スロット
static const uint32_t SLOT_US = 3300;  // 1スロット = 3.3ms (TC106 firmware準拠)

// 送信するテストフレームのバリエーション
// byte[5] は常に 0x7F (固定)
static const uint8_t TEST_FRAMES[][6] = {
    // lastlen[0,1,2], txtens[0,1], 0x7F
    { 0x11, 0x22, 0x33, 0x44, 0x55, 0x7F },  // frameA: 基本テストパターン
    { 0x01, 0x00, 0x00, 0x1E, 0x00, 0x7F },  // frameB: 糸長=1m, 張力=30gf
    { 0x64, 0x00, 0x00, 0x32, 0x00, 0x7F },  // frameC: 糸長=100m, 張力=50gf
    { 0x00, 0x64, 0x00, 0x64, 0x00, 0x7F },  // frameD: 糸長=10000m, 張力=100gf
};
static const uint8_t NUM_FRAMES = sizeof(TEST_FRAMES) / sizeof(TEST_FRAMES[0]);

// 周期送信モード (Phase 1.1) での送信間隔
static const uint32_t PERIODIC_INTERVAL_MS = 1000;

// 受信タイムアウト (Phase 1.2)
static const uint32_t RX_TIMEOUT_MS = 500;

// ======================================================
// TC106 準拠 送信関数
// ======================================================

/**
 * @brief 1バイトを17スロットで送信 (TC106 tx_300_1/2/3 準拠)
 *
 * slot 0   : HIGH (preamble = tx_300_1)
 * slot 1-7 : データビット LSB first (tx_300_2)
 *            bit=0 → HIGH (OC OFF)
 *            bit=1 → LOW  (OC ON)
 * slot 8-16: LOW (footer 9スロット = tx_300_3)
 */
static void sendByte17Slot(uint8_t data) {
    // Slot 0: preamble (HIGH)
    digitalWrite(TX_PIN, HIGH);
    delayMicroseconds(SLOT_US);

    // Slot 1-7: data bits LSB first (7ビット = bit0~bit6)
    // TC106 firmware tx_300_2:
    //   if (txreg_1 & 0x01) == 0: PORT_TX = 1 (HIGH)
    //   else                     : PORT_TX = 0 (LOW)
    //   → bit=0 ならHIGH、bit=1 ならLOW (反転論理)
    //
    // Nano Every (push-pull 5V) でも同じ論理で送出する
    uint8_t reg = data;
    for (uint8_t i = 0; i < 7; i++) {
        if ((reg & 0x01) == 0) {
            digitalWrite(TX_PIN, HIGH);
        } else {
            digitalWrite(TX_PIN, LOW);
        }
        reg >>= 1;
        delayMicroseconds(SLOT_US);
    }

    // Slot 8-16: footer (LOW × 9スロット = tx_300_3)
    digitalWrite(TX_PIN, LOW);
    for (uint8_t i = 0; i < 9; i++) {
        delayMicroseconds(SLOT_US);
    }
}

/**
 * @brief 6バイトフレームを送信 (TC106 6byte 応答フレーム準拠)
 *
 * フレーム送信後は TX_PIN を HIGH (アイドル) に戻す。
 * (TC106 firmware: tx_bytecntr == 6 で PORT_TX = 1 に固定)
 */
static void sendFrame(const uint8_t* frame, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        sendByte17Slot(frame[i]);
    }
    // 送信完了後アイドル状態に戻す
    // (TC106 firmware: tx_enable=0 + PORT_TX=1)
    digitalWrite(TX_PIN, HIGH);
}

// ======================================================
// TC106 準拠 受信関数 (Phase 1.2 用)
// ======================================================

/**
 * @brief ESP32 からのコマンドフレームを受信 (簡易版)
 *
 * ESP32 → Q2 → JP3=2-3 → Nano D13 経路で、
 * ESP32 が送る 17スロット形式のバイトを受信する。
 *
 * 注意: ESP32 → Nano の経路は Q2 (NMOS) 経由なので
 *       論理が反転する可能性あり。実測で確認すること。
 *       ここでは「HIGH = バスHIGH」と仮定して実装。
 *
 * @param out 受信したバイト格納先
 * @return true: 受信成功、false: タイムアウト
 */
static bool receiveByte(uint8_t& out) {
    uint32_t t0 = millis();

    // start bit (LOW) を待つ (preamble の HIGH の後の LOW を検出)
    // ※ TC106 のフレームでは preamble = HIGH、データ LOW が start bit に相当
    // まず HIGH (preamble) を待つ
    while (digitalRead(RX_PIN) == LOW) {
        if (millis() - t0 > RX_TIMEOUT_MS) return false;
    }

    // preamble HIGH を確認したら中央でサンプリング
    delayMicroseconds(SLOT_US / 2);  // preamble の中央でスキップ
    delayMicroseconds(SLOT_US);      // slot 0 (preamble) をスキップ

    // slot 1-7: 7ビット LSB first
    // 反転論理: HIGH=bit0、LOW=bit1
    uint8_t value = 0;
    for (uint8_t i = 0; i < 7; i++) {
        delayMicroseconds(SLOT_US / 2);  // スロット中央でサンプリング
        if (digitalRead(RX_PIN) == LOW) {
            value |= (1 << i);  // LOW = bit 1
        }
        delayMicroseconds(SLOT_US / 2);  // 残り半スロット
    }

    // footer (9スロット LOW) を読み飛ばす
    for (uint8_t i = 0; i < 9; i++) {
        delayMicroseconds(SLOT_US);
    }

    out = value;
    return true;
}

/**
 * @brief nバイトのフレームを受信
 *
 * @param buf     受信バッファ
 * @param len     受信バイト数
 * @return true: 受信成功、false: タイムアウト
 */
static bool receiveFrame(uint8_t* buf, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        if (!receiveByte(buf[i])) return false;
    }
    return true;
}

// ======================================================
// チェックサム (ESP32 ブリッジとの確認用)
// ======================================================

/**
 * @brief TC106 準拠の 7bit チェックサム計算
 * tc_packet_phase3.hpp の checksum7() と同一
 */
static uint8_t checksum7(const uint8_t* data, uint8_t len) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += data[i];
    return (uint8_t)(sum & 0x7F);
}

// ======================================================
// デバッグ用シリアル出力
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

// ======================================================
// setup / loop
// ======================================================

static uint8_t frameIndex = 0;
static uint32_t lastSendMs = 0;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("=== NanoEvery TC106 Emulator V2.0 ===");
    Serial.println("17-slot / byte, LSB first, preamble+footer");
    Serial.print("Slot time: ");
    Serial.print(SLOT_US);
    Serial.println(" us");
    Serial.print("Frame: 6 bytes, fixed 0x7F at byte[5]");
    Serial.println();

#ifdef MODE_PERIODIC
    Serial.println("Mode: PERIODIC (Phase 1.1)");
#else
    Serial.println("Mode: RESPONSE (Phase 1.2)");
#endif

    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, HIGH);  // アイドル = HIGH

#ifdef MODE_RESPONSE
    pinMode(RX_PIN, INPUT);      // 受信ピン (Q2 からの信号)
#endif

    Serial.println("[ready]");
}

void loop() {

#ifdef MODE_PERIODIC
    // ─────────────────────────────────────────────────
    // Phase 1.1: 周期的送信モード
    // 1秒ごとに frameA/B/C/D を順に送信
    // ─────────────────────────────────────────────────
    if (millis() - lastSendMs >= PERIODIC_INTERVAL_MS) {
        lastSendMs = millis();

        const uint8_t* frame = TEST_FRAMES[frameIndex];
        dumpFrame("[TX] ", frame, 6);
        sendFrame(frame, 6);

        frameIndex = (frameIndex + 1) % NUM_FRAMES;
    }

#else
    // ─────────────────────────────────────────────────
    // Phase 1.2: 受信 → 応答モード
    //
    // ESP32 ブリッジからのコマンドフレームを受信して、
    // TC106 応答フレームを返す。
    //
    // TC106 の txsend (rxsend 呼び出し後):
    //   1. rxsend() で tx_enable = 1
    //   2. data_tx() が 6byte を順次送信
    //   3. 6byte 完了で tx_enable = 0
    //
    // エミュレータとして、「コマンド受信 → 即応答」に簡略化。
    // ─────────────────────────────────────────────────
    uint8_t rxBuf[6];

    // コマンドフレームを受信 (6byte)
    if (receiveFrame(rxBuf, 6)) {
        dumpFrame("[RX] ", rxBuf, 6);

        // コマンド解析 (先頭バイトの下位 3bit)
        uint8_t cmd = rxBuf[0] & 0x07;

        // txsend 相当: 受信後に TC106 応答フレームを構築
        // (実機 TC106 は rxsend 後に tx_enable=1 で data_tx が動く)

        // 応答フレームを選択 (コマンドに応じて内容を変える)
        uint8_t resp[6];

        switch (cmd) {
            case 1:  // rxreset
                // 糸長カウンタをリセット状態で応答
                resp[0] = 0x00;  // lastlen[0] = 0
                resp[1] = 0x00;  // lastlen[1] = 0
                resp[2] = 0x00;  // lastlen[2] = 0
                resp[3] = 0x1E;  // txtens[0] = 30gf (仮)
                resp[4] = 0x00;  // txtens[1] = 0
                resp[5] = 0x7F;
                Serial.println("[CMD] RESET");
                break;

            case 2:  // rxadj (センサ調整)
                resp[0] = 0x00;
                resp[1] = 0x00;
                resp[2] = 0x00;
                resp[3] = 0x50;  // txtens[0] = 80gf (センサ調整用のおもり 80g 相当)
                resp[4] = 0x00;
                resp[5] = 0x7F;
                Serial.println("[CMD] SENS ADJ");
                break;

            case 3:  // rxsend (通常送信、張力値を含む)
                // 受信した設定張力を反映した応答を返す
                // (実機 TC106 は PID 制御後の実測値を返すが、エミュレータでは設定値を返す)
                resp[0] = frameIndex;   // lastlen[0] = 仮カウンタ
                resp[1] = 0x00;
                resp[2] = 0x00;
                resp[3] = rxBuf[1];     // txtens[0] = 受信した張力値の下位バイト (エコー)
                resp[4] = rxBuf[2];     // txtens[1] = 受信した張力値の上位バイト
                resp[5] = 0x7F;
                frameIndex++;
                if (frameIndex >= 100) frameIndex = 0;
                Serial.print("[CMD] SEND tens=");
                Serial.println(rxBuf[1] + rxBuf[2] * 10);
                break;

            default:
                // 未定義コマンド → テストパターンで応答
                memcpy(resp, TEST_FRAMES[0], 6);
                Serial.print("[CMD] unknown cmd=0x");
                Serial.println(cmd, HEX);
                break;
        }

        // 応答送信 (tx_enable = 1 相当)
        dumpFrame("[TX] ", resp, 6);
        sendFrame(resp, 6);
    }
    // receiveFrame がタイムアウトした場合は何もしない (次のループへ)

#endif
}

// ======================================================
// 補足コメント: TC106 firmware との対応表
// ======================================================
//
// [TC106 firmware]              [このスケッチ]
// ─────────────────────────────────────────────────────
// TMR1 50us 周期               delayMicroseconds(SLOT_US)
// tx_timer = 32 → 1.65ms      SLOT_US = 3300us (= 3.3ms)
// data_tx() 偶数回のみ実行     sendByte17Slot() 1回 = 1スロット
// tx_bitcntr = 0  tx_300_1()  slot 0: HIGH (preamble)
// tx_bitcntr = 1  tx_300_2()  slot 1: bit0 (LSB)
// ...
// tx_bitcntr = 7  tx_300_2()  slot 7: bit6
// tx_bitcntr = 8  tx_300_3()  slot 8: LOW (footer 開始)
// ...
// tx_bitcntr = 16 tx_300_3()  slot 16: LOW (footer 終了)
// tx_bytecntr = 6 → tx_enable=0  sendFrame() 完了後 HIGH に戻す
//
// 注意: TC106 firmware は 8bit 送信するが、
//       実際に意味があるのは 7bit のみ (bit7 は常に 0、0x7F = 最大値)。
//       byte[5] = 0x7F がその証拠。
//       エミュレータでは 7bit として扱っているが、
//       実機対応時は 8bit 全部送るよう拡張する必要あり。
//       → sendByte17Slot() の for ループを 7 → 8 に変更するだけ。
//
// 未実装: bit7 (8bit目) の処理
//   TC106 firmware の tx_300_2() は txreg_1 を右シフトしていくが、
//   tx_bitcntr=1~7 で 7回シフトしているため、実質 7bit 送信。
//   8bit 目は footer の最初のスロット (tx_bitcntr=8, tx_300_3) で
//   PORT_TX = 0 に固定される。
//   → byte[5] = 0x7F (0111 1111) の場合、bit7=0 は footer の LOW と一致。
//   → つまり 0x7F は「8bit 送ってもフッタに溶け込む」設計。
