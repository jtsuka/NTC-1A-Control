/**
 * NanoEvery_TCEmulator_V2_5.ino
 *
 * TC106 テンションコントローラ エミュレータ
 * Arduino Nano Every (ATmega4809, 5V 16MHz) 用
 *
 * ============================================================
 * 目的
 * ============================================================
 * TC106 実機がない環境で、TC106 が送信する 17スロット波形を
 * Nano Every で模擬し、ESP32-S3 ブリッジ基板の受信ロジック
 * (tcReceiveFrame) を育てるためのテストツール。
 *
 * Phase 1.1: Nano → ESP32 の片方向送信テスト
 *   TC106 風 17スロット波形を 1秒ごとに繰り返し送信する。
 *   ESP32 側の GPIO4 (D3) で受信できることを確認する。
 *
 * Phase 1.2: ESP32 → Nano → ESP32 の双方向テスト (将来)
 *   ESP32 からのコマンドを受信し、TC106 風応答フレームを返す。
 *   まずは Phase 1.1 を完了させてから使用する。
 *
 * ============================================================
 * TC106 firmware の解析結果 (Nittenser TC106_v012)
 * ============================================================
 *
 * ● タイミングの根拠 (FW_TC106-1_v0_0_250201-UTF8.c より)
 *
 *   TMR1 割込: 50us 周期
 *   tx_timer = 32 → 33カウント後に flag.tx_300 を立てる
 *   flag.tx_300 の発火周期: 50us × 33 = 1650us = 1.65ms
 *
 *   data_tx() は tx_cntr の偶数回のみ実処理する。
 *   (tx_cntr は 1 から 127 の間を循環)
 *   → 実質スロット時間: 1650us × 2 = 3300us = 3.3ms
 *
 *   PMオシロ実測値: +Pulse Width ≒ 3.4ms
 *   (TC106 の FRC 内部発振子の誤差 ±2% の範囲内で一致)
 *
 * ● 17スロット構造 (data_tx / tx_300_1 / tx_300_2 / tx_300_3 より)
 *
 *   1バイト = 17スロット × 3.3ms ≒ 56ms
 *
 *   slot  0     : HIGH  (preamble)           ← tx_300_1: PORT_TX = 1
 *   slot  1     : bit0  (LSB first)          ← tx_300_2: 0=HIGH, 1=LOW
 *   slot  2     : bit1
 *   slot  3     : bit2
 *   slot  4     : bit3
 *   slot  5     : bit4
 *   slot  6     : bit5
 *   slot  7     : bit6  (ここまでで 7ビット)  ← tx_300_2 は case 1〜7 の 7回のみ
 *   slot  8〜16 : LOW   (footer 9スロット)   ← tx_300_3: PORT_TX = 0
 *
 *   ※ bit7 (MSB) は送信されない。
 *      data_tx() の switch 文は case 1〜7 (7回) しか tx_300_2 を呼ばない。
 *      case 8 以降はすべて tx_300_3 (footer LOW) に入る。
 *      → 実質 7ビット送信。8ビット目は footer の LOW に吸収される。
 *
 * ● 論理極性 (tx_300_2 より)
 *
 *   TC106 内部 (OC 駆動):
 *     bit = 0 → PORT_TX = 1 → Q1 OFF → バス HIGH (中継基板プルアップで +15V)
 *     bit = 1 → PORT_TX = 0 → Q1 ON  → バス LOW  (GND へ引き下げ ≒ 200mV)
 *
 *   Nano Every (push-pull 5V):
 *     bit = 0 → D10 = HIGH (5V)   ← 同じ論理 (反転なし)
 *     bit = 1 → D10 = LOW  (0V)
 *
 *   → Nano Every は push-pull なのでプルアップ不要。TC106 と同じ論理で送出可能。
 *
 * ● 6バイトフレーム構成
 *
 *   byte 0: lastlen[0]  送出済み糸長 (1の位、0〜99)
 *   byte 1: lastlen[1]  送出済み糸長 (100の位、0〜99)
 *   byte 2: lastlen[2]  送出済み糸長 (万の位、0〜99)
 *   byte 3: txtens[0]   実測張力値 (1の位、tens_actual % 10)
 *   byte 4: txtens[1]   実測張力値 (10の位、tens_actual / 10)
 *   byte 5: 0x7F        固定終端マーカー (= 0111 1111)
 *
 *   ※ 0x7F の波形パターン:
 *      bit0〜6 がすべて 1 なので slot 1〜7 がすべて LOW。
 *      footer の 9スロット LOW と合わせて slot 1〜16 が連続 LOW になる。
 *      波形: H L L L L L L L L L L L L L L L L (1H + 16L)
 *      → オシロで見たときに「フレーム終端」として一目で判別できる。
 *      → ESP32 の tcReceiveFrame() でも終端確認として活用可能。
 *
 * ● tx_enable ガード (改造版 FW の追加機能)
 *
 *   rxsend() または rxreset() 受信時に tx_enable = 1 にセット。
 *   6バイト送信完了後に自動で tx_enable = 0 に戻る。
 *   → TC106 は「コマンドを受けたときだけ応答する」req-response 型。
 *   → Nano エミュレータは Phase 1.1 では自発的に周期送信するが、
 *      Phase 1.2 では tx_enable ガードと同じ動作を再現する。
 *
 * ============================================================
 * 配線 (ESP32-S3 スニファ基板ベータ版)
 * ============================================================
 *
 * Phase 1.1 (Nano → ESP32、片方向):
 *
 *   Nano D10
 *     → R4 (100Ω) [電流制限]
 *     → Q1 未実装 ゲート-ドレイン間 1kΩ ショート
 *     → JP1 (閉)
 *     → /SIG_FOR_5V
 *     → R6 (3.3kΩ) [分圧上側]
 *     → R8 (4.7kΩ) [分圧下側]
 *     → JP4 = 2-3 (5V系経路選択)
 *     → /TC_MCU_RX → D1/D2 BAT43 (3.3Vクランプ)
 *     → JP7 (閉)
 *     → ESP32 D3 (GPIO4)
 *
 *   GND → GND (基板共通)
 *
 *   分圧後電圧: 5V × 4.7/(3.3+4.7) = 2.94V
 *   → ESP32 VIH (2.48V) に対して +0.46V のマージン。
 *
 * Phase 1.2 (ESP32 → Nano、追加経路):
 *
 *   ESP32 D10 (GPIO9)
 *     → JP5 (閉)
 *     → R12 (220Ω) [ゲート電流制限]
 *     → Q2 (2N7000) ゲート
 *     → Q2 ドレイン → R7 (100Ω)
 *     → JP3 = 2-3 (Nano側ループバック経路。基板シルク上の並びは「3-2」)
 *     → /Nano_MOS_DRAIN
 *     → Nano D11 (MOSI/COPI)
 *
 *   ※ 当初 D13 と想定していたが、KiCadネットリスト精査の結果、
 *      Nano_MOS_DRAIN が実際に接続されるのは D11(MOSI) と判明。
 *      Nano Every は D13=SCK, D12=MISO(CIPO), D11=MOSI(COPI) という
 *      特殊なSPIピン配置のため、D13は無関係(未配線)。
 *
 *   ※ Q2 経由で信号が論理反転する可能性あり。
 *     実測で確認し、必要なら RX_INVERTED = true に変更する。
 *
 * ============================================================
 * ジャンパ設定 (Phase 1.1 用)
 * ============================================================
 *   JP1 = 閉    Q1 代替経路を有効化
 *   JP2 = 開    外部送信バスは使わない
 *   JP3 = 設定不要 (Phase 1.1 では未使用。Phase 1.2で 2-3 = Nano側)
 *   JP4 = 2-3   5V系経路を選択
 *   JP5 = 開    Q2 は使わない (Phase 1.1)
 *   JP6 = 開    Q2 プルアップなし
 *   JP7 = 閉    ESP32 GPIO4 に接続
 */

#include <Arduino.h>

// ============================================================
// 動作モード
// ============================================================
// MODE_PERIODIC:
//   1: Phase 1.1 周期送信モード (まずこちらで疎通確認)
//   0: Phase 1.2 受信→応答モード (Phase 1.1 完了後に使用)
// MODE_SIMULTANEOUS:
//   1にすると、MODE_PERIODICの値に関わらずPhase1.3.5-A
//   (時間窓方式の同時双方向モード)で動作する。
// MODE_TX_NONBLOCKING_TEST:
//   Phase1.5 Step1。1にすると他の全モードより優先され、
//   非ブロッキング版TX(tcTxStartNonBlocking/tcTxPollNonBlocking)を
//   MODE_PERIODICと同じ1秒周期・同じTEST_FRAMESで送信する。
//   旧blocking版(MODE_PERIODIC=1)と交互に有効化して、
//   波形・タイミングログを比較する単方向回帰試験用。
//   Nano RX / ESP32側は本モードでは一切使用しない。
//
// ------------------------------------------------------------
// Test mode selection
//
// Non-blocking TX regression (このV2.5版のデフォルト):
//   MODE_PERIODIC            = 0
//   MODE_SIMULTANEOUS        = 0
//   MODE_TX_NONBLOCKING_TEST = 1
//
// Legacy blocking baseline:
//   MODE_PERIODIC            = 1
//   MODE_SIMULTANEOUS        = 0
//   MODE_TX_NONBLOCKING_TEST = 0
//
// Legacy Phase1.3.5 simultaneous test:
//   MODE_PERIODIC            = 0
//   MODE_SIMULTANEOUS        = 1
//   MODE_TX_NONBLOCKING_TEST = 0
//
// IMPORTANT:
//   Enable exactly ONE mode.
// ------------------------------------------------------------
#define MODE_PERIODIC             0
#define MODE_SIMULTANEOUS         0
#define MODE_TX_NONBLOCKING_TEST  1

#if (MODE_PERIODIC + MODE_SIMULTANEOUS + MODE_TX_NONBLOCKING_TEST) != 1
#error "Select exactly ONE test mode"
#endif

// ============================================================
// ピン設定
// ============================================================
// Nano D10: TC106 風 17スロット波形の送信ピン
//   基板上の経路: D10 → R4 → Q1代替 → JP1 → 分圧 → ESP32 GPIO4
static const uint8_t TX_PIN = 10;

// Nano D11 (MOSI/COPI): ESP32 からのコマンド受信ピン (Phase 1.2 で使用)
//   基板上の経路: ESP32 GPIO9 → Q2 → JP3=2-3 → D11
static const uint8_t RX_PIN = 11;

// ============================================================
// タイミング設定
// ============================================================
// TC106 firmware 計算値: 50us × 33 × 2 = 3300us / スロット
// オシロ実測: 約 3400us (FRC 発振子誤差 ±2% の範囲内)
// → まず 3300us で開始し、受信が不安定なら 3400us へ変更する。
static const uint32_t SLOT_US = 3300;

// ============================================================
// Phase 1.2 受信設定
// ============================================================
// ESP32 D10 → Q2 (NMOS) → Nano D11(MOSI) の経路では、
// Q2 が ON のとき Nano D11 が LOW に引かれる (論理反転)。
// Q2 がOFF のとき D11 はプルアップで HIGH に戻る。
//
// まず false で動かし、受信値がおかしければ true に変更する。
static const bool RX_INVERTED = false;

// Phase 1.2 では D11 が Q2 経由でしか駆動されないため、
// Q2 OFF 時に D11 がフローティングにならないよう内部プルアップを使う。
static const bool RX_USE_INTERNAL_PULLUP = true;

// ============================================================
// フレーム設定
// ============================================================
static const uint8_t  FRAME_LEN            = 6;     // TC106 応答フレームは必ず 6バイト
static const uint32_t PERIODIC_INTERVAL_MS = 1000;  // 周期送信モードの送信間隔
static const uint32_t RX_TIMEOUT_MS        = 1600;  // 次フレーム開始待ちを含む受信タイムアウト

// テスト用フレーム (Phase 1.1 で順番に送信)
// 構成: lastlen[0], lastlen[1], lastlen[2], txtens[0], txtens[1], 0x7F (固定)
static const uint8_t TEST_FRAMES[][FRAME_LEN] = {
    // 基本テストパターン (値の多様性を確認するため全部違う値)
    { 0x11, 0x22, 0x33, 0x44, 0x55, 0x7F },

    // 糸長=1m相当, 張力=30gf相当 (最小付近の動作確認)
    { 0x01, 0x00, 0x00, 0x1E, 0x00, 0x7F },

    // 糸長=100m相当, 張力=50gf相当 (中間値の動作確認)
    { 0x64, 0x00, 0x00, 0x32, 0x00, 0x7F },

    // 糸長=10000m相当, 張力=100gf相当 (最大付近の動作確認)
    { 0x00, 0x64, 0x00, 0x64, 0x00, 0x7F },
};
static const uint8_t NUM_FRAMES = sizeof(TEST_FRAMES) / sizeof(TEST_FRAMES[0]);

#if MODE_SIMULTANEOUS
// Phase1.3.5-A専用: Nano→ESP32方向のテストパターン。
// 既存のTEST_FRAMES(ESP32→Nano方向と同一内容)とは別の値にして、
// 方向の取り違えを検出できるようにする。
// 本プロトコルは7bitデータのため、全バイト 0x00〜0x7E に収めること。
static const uint8_t P135_TEST_FRAMES[][FRAME_LEN] = {
    { 0x55, 0x2A, 0x55, 0x2A, 0x55, 0x7F },
    { 0x12, 0x24, 0x36, 0x48, 0x5A, 0x7F },
    { 0x0F, 0x1E, 0x2D, 0x3C, 0x4B, 0x7F },
    { 0x6E, 0x5D, 0x4C, 0x3B, 0x2A, 0x7F },
};
static const uint8_t P135_NUM_FRAMES =
    sizeof(P135_TEST_FRAMES) / sizeof(P135_TEST_FRAMES[0]);

// タイミング(仕様書 Phase1.3.5_試験仕様書.md の時間窓と対応)
static const uint32_t P135_RX_WINDOW_MS = 450;   // 450〜950ms: ESP32受信用の窓
static const uint32_t P135_TX_START_MS  = 0;     // 0ms: 自分のTX開始(周期の先頭)
static const uint32_t P135_CYCLE_MS     = 1000;  // 周期
#endif

// ============================================================
// ユーティリティ
// ============================================================

// フレーム内容を HEX でシリアルに出力する
// tag: "[TX] " や "[RX] " などの識別子
static void dumpFrame(const char* tag, const uint8_t* frame, uint8_t len) {
    Serial.print(tag);
    for (uint8_t i = 0; i < len; i++) {
        if (frame[i] < 0x10) Serial.print('0');
        Serial.print(frame[i], HEX);
        if (i + 1 < len) Serial.print(' ');
    }
    Serial.println();
}

// 連番+タイムスタンプ付き版。長時間試験でのログ突き合わせ(欠落・重複の特定)と、
// 周期乱れ・停止・異常遅延の検出のため、各フレームに通し番号とmillis()を付けて出力する。
static void dumpFrameSeq(const char* tag, uint32_t seq, const uint8_t* frame, uint8_t len) {
    Serial.print('[');
    Serial.print(millis());
    Serial.print(" ms] ");
    Serial.print(tag);
    Serial.print('#');
    Serial.print(seq);
    Serial.print(' ');
    for (uint8_t i = 0; i < len; i++) {
        if (frame[i] < 0x10) Serial.print('0');
        Serial.print(frame[i], HEX);
        if (i + 1 < len) Serial.print(' ');
    }
    Serial.println();
}

// RX_INVERTED フラグを反映した論理レベルを返す
// HIGH = バス HIGH (TC106 が LOW を送っていない状態)
// LOW  = バス LOW  (TC106 がバスを GND へ引き下げている状態)
static inline bool readRxLogical() {
    bool v = (digitalRead(RX_PIN) == HIGH);
    return RX_INVERTED ? !v : v;
}

// TC106 準拠の 7bit チェックサム計算
// ESP32 ブリッジ側の tc_packet_phase3.hpp / checksum7() と同一ロジック。
static uint8_t checksum7(const uint8_t* data, uint8_t len) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0x7F);
}

// ============================================================
// TC106 風 17スロット 送信
// ============================================================

// バスを HIGH にする (TC106 firmware: PORT_TX = 1 / Q1 OFF / バス HIGH)
static void writeBusHigh() { digitalWrite(TX_PIN, HIGH); }

// バスを LOW にする (TC106 firmware: PORT_TX = 0 / Q1 ON / バス LOW)
static void writeBusLow()  { digitalWrite(TX_PIN, LOW);  }

/**
 * @brief 1バイトを TC106 準拠の 17スロットで送信する
 *
 * TC106 firmware の data_tx() / tx_300_1/2/3 を忠実に再現する。
 *
 * [スロット構成]
 *   slot  0     HIGH (preamble)     tx_300_1 の PORT_TX = 1
 *   slot  1〜7  7bit データ         tx_300_2 の偶数スロット処理
 *               bit = 0 → HIGH
 *               bit = 1 → LOW
 *   slot  8〜16 LOW (footer)        tx_300_3 の PORT_TX = 0
 *
 * [論理]
 *   TC106 は OC (Q1) なので「bit=0 → バス HIGH, bit=1 → バス LOW」。
 *   Nano Every は push-pull だが同じ論理で出力する (反転なし)。
 *
 * @param data 送信するバイト値 (bit7 は無視される、実質 7bit 送信)
 */
static void sendByte17Slot(uint8_t data) {
    // slot 0: preamble (TC106 tx_300_1 の PORT_TX = 1)
    writeBusHigh();
    delayMicroseconds(SLOT_US);

    // slot 1〜7: データビット (7ビット、LSB first)
    // TC106 tx_300_2 準拠:
    //   (txreg_1 & 0x01) == 0 → PORT_TX = 1 (HIGH)
    //   (txreg_1 & 0x01) != 0 → PORT_TX = 0 (LOW)
    for (uint8_t bit = 0; bit < 7; bit++) {
        if ((data >> bit) & 0x01) {
            writeBusLow();    // bit = 1 → LOW
        } else {
            writeBusHigh();   // bit = 0 → HIGH
        }
        delayMicroseconds(SLOT_US);
    }

    // slot 8〜16: footer (TC106 tx_300_3 の PORT_TX = 0)
    // 9スロット連続 LOW。
    // ※ 0x7F の場合は slot 1〜7 もすべて LOW になるため、
    //    合計 slot 1〜16 = 16スロット連続 LOW となり、
    //    オシロで「フレーム終端マーカー」として視認できる。
    writeBusLow();
    for (uint8_t i = 0; i < 9; i++) {
        delayMicroseconds(SLOT_US);
    }
}

/**
 * @brief 6バイトフレームを送信し、送信後バスをアイドル状態に戻す
 *
 * TC106 firmware の tx_enable ガード解除後の動作を再現:
 *   送信完了後に PORT_TX = 1 (HIGH) に固定する。
 *
 * @param frame 送信するフレームのポインタ
 * @param len   フレームバイト数 (通常 6)
 */
static void sendFrame17Slot(const uint8_t* frame, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        sendByte17Slot(frame[i]);
    }
    // 送信完了後アイドル HIGH に戻す
    // (TC106 firmware: tx_bytecntr==6 で tx_enable=0, PORT_TX=1)
    writeBusHigh();
}

// ============================================================
// TC106 風 17スロット送信 (非ブロッキング状態機械版)
// ============================================================
//
// Phase1.5 設計表 v0.4 対応。上記 sendByte17Slot()/sendFrame17Slot()
// (blocking版)はそのまま残し、比較検証用に温存する。
// こちらは同じ 17-slot 構造(維持する通信仕様は変更なし)を、
// 「呼ばれるたびに1スロットだけ処理してすぐreturnする」
// 非ブロッキング形式で実装したもの。
//
// 設計表 v0.4 3節「catch-up禁止」原則に従い、処理が大きく遅延した
// 場合は複数スロットをまとめて処理せず、送信をabortしてIDLEへ戻し、
// timing overrunとしてログに記録する。
//
// [使い方]
//   tcTxStartNonBlocking(frame, len) で送信開始。
//   以後、loop() 内で毎回 tcTxPollNonBlocking() を呼ぶ。
//   戻り値が true になったら、そのフレームの送信が完了(または
//   overrunによりabort)している。tcTxBusy() で送信中かどうかを
//   確認できる。

// 1スロット分の遅延までは許容し、それを超えたら overrun として
// abortする。暫定値。実装・測定段階で実際のジッタ量を見て調整する。
static const uint32_t TX_SLOT_OVERRUN_US = SLOT_US;

enum TxNbState : uint8_t {
    TXNB_IDLE = 0,
    TXNB_SENDING,
};

static TxNbState txNbState   = TXNB_IDLE;
static uint8_t   txNbFrame[FRAME_LEN];
static uint8_t   txNbFrameLen  = 0;
static uint8_t   txNbByteIndex = 0;   // 0..txNbFrameLen-1
static uint8_t   txNbSlotIndex = 0;   // 次に処理すべきスロット番号(1..16)
static uint32_t  txNbByteT0Us  = 0;   // 現バイトのslot0基準時刻(micros())
static uint32_t  txNbOverrunCount = 0; // ログ用: overrun発生回数

// 現在送信中かどうか
static bool tcTxBusy() {
    return txNbState != TXNB_IDLE;
}

// 非ブロッキング送信を開始する。
// 既に送信中(IDLEでない)場合は false を返し、何もしない。
static bool tcTxStartNonBlocking(const uint8_t* frame, uint8_t len) {
    if (txNbState != TXNB_IDLE) return false;
    if (len == 0 || len > FRAME_LEN) return false;

    memcpy(txNbFrame, frame, len);
    txNbFrameLen  = len;
    txNbByteIndex = 0;

    // slot0: preamble (tx_300_1 の PORT_TX = 1 相当)。
    // 開始時点で即座に出力し、以後の1..16スロットをpollで進める。
    writeBusHigh();
    txNbByteT0Us  = micros();
    txNbSlotIndex = 1;
    txNbState     = TXNB_SENDING;
    return true;
}

// 毎loop()で呼ぶ。次のスロット時刻に達していれば1スロットだけ処理する。
// フレーム送信が完了(正常終了 or overrunによるabort)した場合に true を返す。
// まだ送信中(次スロット時刻に未到達、または送信途中)の場合は false を返す。
static bool tcTxPollNonBlocking() {
    if (txNbState == TXNB_IDLE) return false;

    const uint32_t now = micros();
    const uint32_t targetUs = txNbByteT0Us + (uint32_t)txNbSlotIndex * SLOT_US;

    // まだ次のスロット時刻に達していない → 何もせず戻る
    if ((int32_t)(now - targetUs) < 0) return false;

    // catch-up禁止: 許容量を超えて遅延していたら、
    // 追いつこうとせずここでabortしてIDLEへ戻す(設計表v0.4 3節)。
    if ((int32_t)(now - targetUs) > (int32_t)TX_SLOT_OVERRUN_US) {
        txNbOverrunCount++;
        Serial.print('[');
        Serial.print(millis());
        Serial.print(" ms] [TX-OVERRUN] #");
        Serial.print(txNbOverrunCount);
        Serial.print(" byte=");
        Serial.print(txNbByteIndex);
        Serial.print(" slot=");
        Serial.print(txNbSlotIndex);
        Serial.print(" lateBy_us=");
        Serial.println((int32_t)(now - targetUs));
        writeBusHigh(); // 安全のためアイドルHIGHへ戻す
        txNbState = TXNB_IDLE;
        return true; // このフレームの送信は終了(異常終了)扱い
    }

    // ---- このスロットの処理 ----
    if (txNbSlotIndex >= 1 && txNbSlotIndex <= 7) {
        // slot1〜7: データ7bit、LSB first (tx_300_2 相当)
        const uint8_t data = txNbFrame[txNbByteIndex];
        const bool bitIsOne = (data >> (txNbSlotIndex - 1)) & 0x01;
        if (bitIsOne) writeBusLow(); else writeBusHigh();
    } else if (txNbSlotIndex == 8) {
        // slot8: footer開始 (tx_300_3 相当)。slot9〜16はLOWのまま
        // 維持するだけなので、GPIO操作は不要(タイミングのみ消費)。
        writeBusLow();
    }
    // slot9〜16: 何もしない(footer継続、タイミング消費のみ)
    // slot17: バイト境界イベント(下記参照)。blocking版が
    // slot8〜16の9スロット分LOWを維持してから次バイトへ進むのと
    // 挙動を一致させるため、slot16の処理では遷移せず、
    // 「slot16も1スロット分LOWを維持し終えた」slot17到達時に
    // 次バイトのslot0(またはidle HIGH)へ切り替える。

    if (txNbSlotIndex == 17) {
        // バイト境界: 次バイトへ、またはフレーム完了
        txNbByteIndex++;
        if (txNbByteIndex >= txNbFrameLen) {
            // フレーム完了
            writeBusHigh(); // アイドルHIGHへ (tx_enable=0相当)
            txNbState = TXNB_IDLE;
            return true;
        }
        // 次バイトへ。slot0(preamble)を即座に出力し、
        // 基準時刻は「実測now」ではなく「スケジュール上の次の時刻」を
        // 使うことで、長時間送信時のドリフト蓄積を避ける
        // (この時点でoverrunチェックは通過済みなので許容範囲内)。
        writeBusHigh();
        txNbByteT0Us  = txNbByteT0Us + (uint32_t)17 * SLOT_US;
        txNbSlotIndex = 1;
        return false; // フレームはまだ完了していない
    }

    txNbSlotIndex++;
    return false;
}

// ============================================================
// TC106 風 17スロット 受信 (Phase 1.2 用)
// ============================================================
//
// V2.3 変更点:
//   旧方式は「LOW→HIGH を preamble 開始」とみなして1バイトずつ受信していた。
//   しかし TC106 形式では idle=HIGH かつ slot0(preamble)=HIGH のため、
//   フレーム先頭 byte0 の開始にはエッジが存在しない。
//   その結果、最初に見つかる LOW→HIGH は byte1 の開始境界になり、
//   先頭バイトを取りこぼす可能性があった。
//
//   V2.3では ESP32側 p11TryReceiveFrame() と同じ考え方に変更:
//     1) D11(MOSI) の全エッジをポーリングで記録
//     2) 26ms以上続く LOW を footer と判定
//     3) footer終了の LOW→HIGH = 次バイト slot0 開始として確定
//     4) 最初のバイトだけは byte1開始時刻 - 17slot で逆算
//     5) 各データslot中央をサンプリングして6バイトを復元
//
// Nano Everyでは3.3ms/slotと十分遅いため、Phase 1.2の検証では
// 割り込みを使わず digitalRead() の高速ポーリングでエッジを記録する。
// ============================================================

static const uint8_t  RX_EDGE_BUF_SIZE  = 128;
static const uint32_t RX_BYTE_US        = SLOT_US * 17UL;  // 56100us
static const uint32_t RX_FOOTER_MIN_US  = 26000UL;         // 7slot=23100us と 9slot=29700us の間
static const uint32_t RX_QUIET_MS       = 70;              // 最大連続LOW(0x7F: 52.8ms)より長くする

/**
 * @brief 6バイトフレームをエッジ記録から復元する
 *
 * @param buf       受信バッファ (6バイト)
 * @param len       受信バイト数。Phase 1.2では6固定
 * @param timeoutMs 次のフレームが動き始めるまで待つ最大時間
 * @return true: 6バイト復元成功、false: 同期/タイムアウト失敗
 */
static bool receiveFrame17Slot(uint8_t* buf, uint8_t len, uint32_t timeoutMs) {
    if (len != FRAME_LEN) return false;

    uint32_t edgeTimes[RX_EDGE_BUF_SIZE];
    uint8_t  edgeLevels[RX_EDGE_BUF_SIZE];
    uint8_t  edgeCount = 0;

    const uint32_t waitStartMs = millis();
    const uint32_t captureBaseUs = micros();

    bool prev = readRxLogical();
    bool started = false;
    uint32_t lastEdgeMs = millis();

    // --------------------------------------------------------
    // 1. 1フレーム分のエッジを記録
    // --------------------------------------------------------
    while (true) {
        const bool now = readRxLogical();

        if (now != prev) {
            if (edgeCount >= RX_EDGE_BUF_SIZE) {
                Serial.println("[RXERR] edge buffer overflow");
                return false;
            }

            edgeTimes[edgeCount]  = micros() - captureBaseUs; // 相対時刻
            edgeLevels[edgeCount] = now ? HIGH : LOW;         // エッジ後の論理レベル
            edgeCount++;

            prev = now;
            started = true;
            lastEdgeMs = millis();
        }

        if (!started) {
            // まだフレームが始まっていない間だけ timeoutMs を適用
            if (millis() - waitStartMs > timeoutMs) {
                return false;
            }
        } else {
            // 0x7F は最大52.8ms連続LOWになるので、それより長い無変化を
            // 「フレーム終了後のidle HIGH」と判断する。
            if (millis() - lastEdgeMs > RX_QUIET_MS) {
                break;
            }
        }
    }

    if (edgeCount < 2) {
        Serial.println("[RXERR] too few edges");
        return false;
    }

    // --------------------------------------------------------
    // 2. 長いLOW区間の終了時刻を集める
    //
    // データ部だけの最長LOWは7slot = 23.1ms。
    // footerを含むLOWは最低9slot = 29.7ms。
    // 26msを境界にすれば両者を区別できる。
    //
    // LOW区間の終了(LOW→HIGH)は「次バイトのslot0開始」。
    // --------------------------------------------------------
    uint32_t footerEnd[FRAME_LEN];
    uint8_t footerCount = 0;

    for (uint8_t i = 0; i + 1 < edgeCount && footerCount < FRAME_LEN; i++) {
        if (edgeLevels[i] == LOW) {
            const uint32_t lowDur = edgeTimes[i + 1] - edgeTimes[i];
            if (lowDur >= RX_FOOTER_MIN_US) {
                footerEnd[footerCount++] = edgeTimes[i + 1];
            }
        }
    }

    // byte0→1, 1→2, 2→3, 3→4, 4→5 の5境界があれば
    // 6バイト全部の開始時刻を決められる。
    if (footerCount < FRAME_LEN - 1) {
        Serial.print("[RXERR] footer boundaries=");
        Serial.println(footerCount);
        return false;
    }

    // --------------------------------------------------------
    // 3. 各バイトのslot0開始時刻を決定
    // --------------------------------------------------------
    uint32_t byteT0[FRAME_LEN];

    // 最初のバイトは idle HIGH → preamble HIGH でエッジが無いので、
    // byte1開始時刻から1バイト長を引いて逆算する。
    if (footerEnd[0] < RX_BYTE_US) {
        Serial.println("[RXERR] first byte start underflow");
        return false;
    }

    byteT0[0] = footerEnd[0] - RX_BYTE_US;

    for (uint8_t b = 1; b < FRAME_LEN; b++) {
        byteT0[b] = footerEnd[b - 1];
    }

    // 指定時刻における論理レベルをエッジ列から復元する。
    // 最初のエッジより前は idle/preamble と同じ HIGH とみなせる。
    auto levelAt = [&](uint32_t t) -> bool {
        bool lvl = true; // HIGH
        for (uint8_t i = 0; i < edgeCount; i++) {
            if (edgeTimes[i] <= t) {
                lvl = (edgeLevels[i] == HIGH);
            } else {
                break;
            }
        }
        return lvl;
    };

    // --------------------------------------------------------
    // 4. slot1〜7の中央を読み、7bit LSB-firstを復元
    //    HIGH=bit0, LOW=bit1
    // --------------------------------------------------------
    for (uint8_t b = 0; b < FRAME_LEN; b++) {
        uint8_t value = 0;

        for (uint8_t bit = 0; bit < 7; bit++) {
            const uint32_t sampleAt =
                byteT0[b]
                + (uint32_t)(bit + 1) * SLOT_US
                + (SLOT_US / 2);

            if (!levelAt(sampleAt)) {
                value |= (uint8_t)(1U << bit); // LOW = bit1
            }
        }

        buf[b] = value;
    }

    return true;
}

// ============================================================
// 応答フレーム生成 (Phase 1.2 用)
// ============================================================

/**
 * @brief 受信コマンドに応じた TC106 風応答フレームを生成する
 *
 * TC106 firmware の rxreset / rxadj / rxsend に対応する応答を返す。
 * 実機 TC106 は PID 制御後の実測値を返すが、エミュレータでは
 * 固定値またはコマンドのエコーバックで代替する。
 *
 * コマンド識別: buf[0] の下位 3bit (TC106 firmware の table_rx 準拠)
 *   case 1: rxreset  → 糸長カウンタリセット相当の応答
 *   case 2: rxadj    → センサ調整中相当の応答 (80g おもり掛け)
 *   case 3: rxsend   → 通常運転中の張力・糸長応答 (受信値エコー)
 *
 * @param cmdFrame  受信したコマンドフレーム (6バイト)
 * @param respFrame 生成した応答フレームの格納先 (6バイト)
 */
static void buildResponseFrame(const uint8_t* cmdFrame, uint8_t* respFrame) {
    // コマンドコードは先頭バイトの下位 3bit
    // (TC106 firmware の table_rx: switch (b & 0x07))
    const uint8_t cmd = cmdFrame[0] & 0x07;

    switch (cmd) {
        case 1: // rxreset: リセット直後 → 糸長 0、張力 30gf 相当
            respFrame[0] = 0x00;  // lastlen[0] = 0 (糸長 0m)
            respFrame[1] = 0x00;  // lastlen[1] = 0
            respFrame[2] = 0x00;  // lastlen[2] = 0
            respFrame[3] = 0x1E;  // txtens[0] = 30 (30gf の 1の位)
            respFrame[4] = 0x00;  // txtens[1] = 0  (30gf の 10の位)
            respFrame[5] = 0x7F;  // 固定終端マーカー
            Serial.println("[CMD] RESET -> resp: len=0, tens=30gf");
            break;

        case 2: // rxadj: センサ調整中 → 80g おもり掛け相当
            // TC106 マニュアル: 調整時は 80g のおもりを使用
            respFrame[0] = 0x00;
            respFrame[1] = 0x00;
            respFrame[2] = 0x00;
            respFrame[3] = 0x50;  // txtens[0] = 80 (80gf の 1の位 = 0)
            respFrame[4] = 0x08;  // txtens[1] = 8  (80gf の 10の位 = 8)
            respFrame[5] = 0x7F;
            Serial.println("[CMD] SENS ADJ -> resp: tens=80gf");
            break;

        case 3: // rxsend: 通常運転 → 受信データをエコーバック (TC106 応答確認用)
            // 実機では PID 制御後の実測張力値を返すが、エミュレータでは
            // 受信値をそのまま返すことで ESP32 の送受信が正しく動いているか確認できる。
            // bit7 を除去 (&0x7F) して 7bit 送信プロトコルに合わせる。
            respFrame[0] = cmdFrame[1] & 0x7F;  // lastlen[0] = 受信 byte1 をエコー
            respFrame[1] = cmdFrame[2] & 0x7F;  // lastlen[1] = 受信 byte2 をエコー
            respFrame[2] = 0x00;
            respFrame[3] = cmdFrame[3] & 0x7F;  // txtens[0]  = 受信 byte3 をエコー
            respFrame[4] = cmdFrame[4] & 0x7F;  // txtens[1]  = 受信 byte4 をエコー
            respFrame[5] = 0x7F;
            Serial.print("[CMD] SEND -> resp echo: tens=");
            Serial.println((cmdFrame[4] & 0x7F) * 10 + (cmdFrame[3] & 0x7F));
            break;

        default: // 未定義コマンド → テストパターン frameA を返す
            memcpy(respFrame, TEST_FRAMES[0], FRAME_LEN);
            Serial.print("[CMD] UNKNOWN cmd=0x");
            Serial.println(cmd, HEX);
            break;
    }
}

// ============================================================
// Arduino 標準関数
// ============================================================

static uint8_t  frameIndex = 0;
static uint32_t lastSendMs = 0;
static uint32_t txSeq = 0;  // 連番(Phase1.3-B長時間試験でのログ突き合わせ用)
static uint32_t rxSeq = 0;  // 連番(Phase1.3.5-A用、受信側)

void setup() {
    Serial.begin(115200);
    delay(500);

    // TX ピン初期化: アイドル状態は HIGH
    // (TC106 firmware: 送信完了後 PORT_TX = 1)
    pinMode(TX_PIN, OUTPUT);
    writeBusHigh();

    // RX ピン初期化 (Phase 1.2 または Phase1.3.5-A、受信を行うモード)
#if MODE_SIMULTANEOUS || !MODE_PERIODIC
    pinMode(RX_PIN, RX_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT);
#endif

    // 起動メッセージ
    Serial.println();
    Serial.println("=== NanoEvery TC106 Emulator V2.5 (Phase1.5 Step1: TX non-blocking) ===");
    Serial.println("TC106-like 17-slot, 7bit data, LSB first");
    Serial.print  ("TX_PIN  : D"); Serial.println(TX_PIN);
    Serial.print  ("RX_PIN  : D"); Serial.println(RX_PIN);
    Serial.print  ("SLOT_US : ");  Serial.print(SLOT_US);
    Serial.println(" us (3300=firmware値, 3400=実測値)");
    Serial.print  ("1 byte  : ");  Serial.print(SLOT_US * 17 / 1000);
    Serial.println(" ms");
    Serial.print  ("6 bytes : ");  Serial.print(SLOT_US * 17 * 6 / 1000);
    Serial.println(" ms");

#if MODE_TX_NONBLOCKING_TEST
    Serial.println("Mode: TX_NONBLOCKING_TEST (Phase 1.5 Step1)");
    Serial.print  ("Interval: "); Serial.print(PERIODIC_INTERVAL_MS);
    Serial.println(" ms");
    Serial.print  ("Frames  : "); Serial.println(NUM_FRAMES);
    Serial.print  ("TX_SLOT_OVERRUN_US: "); Serial.println(TX_SLOT_OVERRUN_US);
#elif MODE_SIMULTANEOUS
    Serial.println("Mode: SIMULTANEOUS (Phase 1.3.5-A, time-windowed)");
    Serial.print  ("RX window: "); Serial.print(P135_RX_WINDOW_MS);
    Serial.println("-950 ms");
    Serial.print  ("Cycle: "); Serial.print(P135_CYCLE_MS); Serial.println(" ms");
    Serial.print  ("TX frames (Nano->ESP32): "); Serial.println(P135_NUM_FRAMES);
#elif MODE_PERIODIC
    Serial.println("Mode: PERIODIC (Phase 1.1)");
    Serial.print  ("Interval: "); Serial.print(PERIODIC_INTERVAL_MS);
    Serial.println(" ms");
    Serial.print  ("Frames  : "); Serial.println(NUM_FRAMES);
#else
    Serial.println("Mode: RESPONSE (Phase 1.2)");
    Serial.print  ("RX_INVERTED       : ");
    Serial.println(RX_INVERTED ? "true (Q2経由反転あり)" : "false (反転なし)");
    Serial.print  ("RX_USE_PULLUP     : ");
    Serial.println(RX_USE_INTERNAL_PULLUP ? "true" : "false");
    Serial.print  ("RX_TIMEOUT_MS     : "); Serial.println(RX_TIMEOUT_MS);
#endif

    Serial.println("[ready]");
}

void loop() {
#if MODE_TX_NONBLOCKING_TEST
    // ──────────────────────────────────────────────────────
    // Phase 1.5 Step1: 非ブロッキングTXの単方向回帰試験モード
    //
    // 旧 MODE_PERIODIC (blocking sendFrame17Slot) と同じ
    // PERIODIC_INTERVAL_MS周期・同じTEST_FRAMESを使い、
    // 非ブロッキング版(tcTxStartNonBlocking/tcTxPollNonBlocking)で
    // 送信する。Nano RX・ESP32側は使用しない。
    //
    // loop()自体はbusy-waitを含まず、他の処理と共存できることを
    // 示すため、ダミーのカウンタ更新をpollと並行して行う。
    // ──────────────────────────────────────────────────────
    static uint32_t idleLoopCount = 0; // busy-waitしていないことの確認用
    idleLoopCount++;

    if (!tcTxBusy() && (millis() - lastSendMs >= PERIODIC_INTERVAL_MS)) {
        lastSendMs = millis();

        const uint8_t* frame = TEST_FRAMES[frameIndex];
        txSeq++;
        dumpFrameSeq("[TX-NB] ", txSeq, frame, FRAME_LEN);
        tcTxStartNonBlocking(frame, FRAME_LEN);

        frameIndex = (frameIndex + 1) % NUM_FRAMES;
    }

    // 送信中なら1スロットずつ進める。完了したらログに残す。
    if (tcTxBusy()) {
        if (tcTxPollNonBlocking()) {
            Serial.print('[');
            Serial.print(millis());
            Serial.println(" ms] [TX-NB] frame done (or aborted)");
        }
    }

#elif MODE_SIMULTANEOUS
    // ──────────────────────────────────────────────────────
    // Phase 1.3.5-A: 時間窓方式の同時双方向モード
    //
    // 17スロット送受信ロジック(sendFrame17Slot/receiveFrame17Slot)自体は
    // Phase1.3/1.3-Bで実証済みのため変更しない。ここでは、周期の中で
    // 「今は送信の番か、受信の番か」を判断する時間管理のみを行う。
    //
    //   周期 n
    //   0ms          336ms       450  500ms       836ms   950  1000ms
    //   │ Nano TX ────►│          │    ESP32 TX ────►│      │
    //   │              │          │                  │      │
    //   │ (送信中)      │          │ Nano RX窓         │      │
    //   └──────────────┘          └──────────────────┘
    //
    // Nanoは周期開始(0ms)で自分のTXを行い、続けて450〜950msを
    // ESP32からの受信用の窓とする。窓終了後は次周期の0msまで待機する。
    // ──────────────────────────────────────────────────────
    static uint32_t p135CycleStart = 0;
    static bool p135Initialized = false;
    if (!p135Initialized) {
        p135CycleStart = millis();
        p135Initialized = true;
    }

    // --- 0ms: 自分のTX ---
    {
        const uint8_t* frame = P135_TEST_FRAMES[frameIndex];
        txSeq++;
        dumpFrameSeq("[TX] ", txSeq, frame, FRAME_LEN);
        sendFrame17Slot(frame, FRAME_LEN);
        frameIndex = (frameIndex + 1) % P135_NUM_FRAMES;
    }

    // --- 450〜950ms: ESP32からの受信窓 ---
    while (millis() - p135CycleStart < P135_RX_WINDOW_MS) {
        // 窓開始(450ms)まで待機
    }
    {
        uint32_t elapsed = millis() - p135CycleStart;
        const uint32_t RX_WINDOW_END_MS = 950; // 受信窓の終わり
        uint32_t remaining = (elapsed < RX_WINDOW_END_MS) ? (RX_WINDOW_END_MS - elapsed) : 0;
        if (remaining > 0) {
            uint8_t rxBuf[FRAME_LEN];
            if (receiveFrame17Slot(rxBuf, FRAME_LEN, remaining)) {
                rxSeq++;
                dumpFrameSeq("[RX] ", rxSeq, rxBuf, FRAME_LEN);
            }
        }
    }

    // --- 次周期(1000ms)まで待機 ---
    while (millis() - p135CycleStart < P135_CYCLE_MS) {
        // 次周期まで待機
    }
    p135CycleStart += P135_CYCLE_MS;

#elif MODE_PERIODIC
    // ──────────────────────────────────────────────────────
    // Phase 1.1: 周期的送信モード
    //
    // 1秒ごとに TEST_FRAMES を順番に送信する。
    // Nano のシリアルモニタに [TX] が出れば送信動作は開始している。
    //
    // 確認手順:
    // 1. このモードで Nano を動かす。
    // 2. ESP32 に 17スロット受信ロジック (tcReceiveFrame 実装版) を書き込む。
    // 3. ESP32 のシリアルモニタに [RX] XX XX XX XX XX 7F が出れば疎通成功。
    // ──────────────────────────────────────────────────────
    if (millis() - lastSendMs >= PERIODIC_INTERVAL_MS) {
        lastSendMs = millis();

        const uint8_t* frame = TEST_FRAMES[frameIndex];
        txSeq++;
        dumpFrameSeq("[TX] ", txSeq, frame, FRAME_LEN);
        sendFrame17Slot(frame, FRAME_LEN);

        // 次のフレームに進む (循環)
        frameIndex = (frameIndex + 1) % NUM_FRAMES;
    }

#else
    // ──────────────────────────────────────────────────────
    // Phase 1.2: 受信→応答モード
    //
    // ESP32 からのコマンドフレーム (6バイト、17スロット) を受信し、
    // TC106 風の応答フレームを返す。
    //
    // TC106 firmware の動作を模擬:
    //   rxsend() → tx_enable = 1 → data_tx() で 6バイト送信 → tx_enable = 0
    //
    // 確認手順:
    // 1. ESP32 に 17スロット送信ロジック (tcSendFrame 実装版) を書き込む。
    // 2. Pi から SEND コマンドを送る。
    // 3. Nano のシリアルモニタに [RX] と [TX] が交互に出れば成功。
    // 4. ESP32 → Pi の経路で応答が届くことを Pi 側で確認する。
    // ──────────────────────────────────────────────────────
    uint8_t rxBuf[FRAME_LEN];

    // コマンドフレームを受信 (タイムアウトなら次のループへ)
    if (receiveFrame17Slot(rxBuf, FRAME_LEN, RX_TIMEOUT_MS)) {
        dumpFrame("[RX] ", rxBuf, FRAME_LEN);

        // 受信したフレームの最終バイトが 0x7F かを確認 (簡易フレーム検証)
        if (rxBuf[FRAME_LEN - 1] != 0x7F) {
            Serial.println("[WARN] 末尾が 0x7F でない: フレーム不正の可能性");
        }

        // 応答フレームを生成して送信 (tc_enable = 1 → data_tx 相当)
        uint8_t resp[FRAME_LEN];
        buildResponseFrame(rxBuf, resp);

        dumpFrame("[TX] ", resp, FRAME_LEN);
        sendFrame17Slot(resp, FRAME_LEN);
    }
    // タイムアウト時は何もしない (次の受信待ちへ)
#endif
}

/*
 * ============================================================
 * TC106 firmware との対応表 (保守・デバッグ用)
 * ============================================================
 *
 * [TC106 firmware]              [このスケッチ]
 * ─────────────────────────────────────────────────────────────
 * TMR1 50us 周期               delayMicroseconds(SLOT_US)
 * tx_timer=32 → 1.65ms        SLOT_US = 3300us (1スロット)
 * data_tx() 偶数回のみ実行     sendByte17Slot() が 1スロット分処理
 * tx_bitcntr=0  tx_300_1()    slot 0: writeBusHigh() (preamble)
 * tx_bitcntr=1  tx_300_2()    slot 1: bit0 送出
 * ...
 * tx_bitcntr=7  tx_300_2()    slot 7: bit6 送出 (7ビット目)
 * tx_bitcntr=8  tx_300_3()    slot 8: writeBusLow() (footer 開始)
 * ...
 * tx_bitcntr=16 tx_300_3()    slot 16: writeBusLow() (footer 終了)
 * tx_bytecntr=6               sendFrame17Slot() 完了 → writeBusHigh()
 * tx_enable=0                 (writeBusHigh() でアイドル HIGH に戻す)
 *
 * ─────────────────────────────────────────────────────────────
 * [送信フレームの 0x7F の特殊波形]
 *
 * 0x7F = 0b0111_1111 → bit0〜6 がすべて 1
 * slot 1〜7 がすべて LOW、slot 8〜16 も LOW
 * → preamble の HIGH 1スロット + 16スロット連続 LOW
 *
 * オシロ上では「1H + 16L」のパターンが 6バイトフレームの末尾に来る。
 * ESP32 の tcReceiveFrame() でのフレーム終端確認に活用できる。
 *
 * ─────────────────────────────────────────────────────────────
 * [Phase 1.2 RX 論理反転について]
 *
 * ESP32 D10(GPIO9) → Q2(2N7000) → Nano D11(MOSI/COPI) の経路では:
 *   ESP32 D10 = HIGH → Q2 ON  → Nano D11 = LOW (R10プルアップを引き下げ)
 *   ESP32 D10 = LOW  → Q2 OFF → Nano D11 = HIGH (プルアップで HIGH 復帰)
 *
 * つまり ESP32 が送る 17スロット波形が Nano 側で論理反転して届く。
 * readRxLogical() の RX_INVERTED フラグで吸収できる。
 * 実測で判断し、受信値が期待と逆なら RX_INVERTED = true に変更する。
 */
