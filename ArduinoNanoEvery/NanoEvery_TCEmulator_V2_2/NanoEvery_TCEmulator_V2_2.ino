/**
 * NanoEvery_TCEmulator_V2_2.ino
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
 *     → JP3 = 2-3 (Nano側ループバック経路)
 *     → /Nano_MOS_DRAIN
 *     → Nano D13
 *
 *   ※ Q2 経由で信号が論理反転する可能性あり。
 *     実測で確認し、必要なら RX_INVERTED = true に変更する。
 *
 * ============================================================
 * ジャンパ設定 (Phase 1.1 用)
 * ============================================================
 *   JP1 = 閉    Q1 代替経路を有効化
 *   JP2 = 開    外部送信バスは使わない
 *   JP3 = 1-2   (Phase 1.1 では関係なし)
 *   JP4 = 2-3   5V系経路を選択
 *   JP5 = 開    Q2 は使わない (Phase 1.1)
 *   JP6 = 開    Q2 プルアップなし
 *   JP7 = 閉    ESP32 GPIO4 に接続
 */

#include <Arduino.h>

// ============================================================
// 動作モード
// ============================================================
// 1: Phase 1.1 周期送信モード (まずこちらで疎通確認)
// 0: Phase 1.2 受信→応答モード (Phase 1.1 完了後に使用)
#define MODE_PERIODIC 1

// ============================================================
// ピン設定
// ============================================================
// Nano D10: TC106 風 17スロット波形の送信ピン
//   基板上の経路: D10 → R4 → Q1代替 → JP1 → 分圧 → ESP32 GPIO4
static const uint8_t TX_PIN = 10;

// Nano D13: ESP32 からのコマンド受信ピン (Phase 1.2 で使用)
//   基板上の経路: ESP32 GPIO9 → Q2 → JP3=2-3 → D13
static const uint8_t RX_PIN = 13;

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
// ESP32 D10 → Q2 (NMOS) → Nano D13 の経路では、
// Q2 が ON のとき Nano D13 が LOW に引かれる (論理反転)。
// Q2 がOFF のとき D13 はプルアップで HIGH に戻る。
//
// まず false で動かし、受信値がおかしければ true に変更する。
static const bool RX_INVERTED = false;

// Phase 1.2 では D13 が Q2 経由でしか駆動されないため、
// Q2 OFF 時に D13 がフローティングにならないよう内部プルアップを使う。
static const bool RX_USE_INTERNAL_PULLUP = true;

// ============================================================
// フレーム設定
// ============================================================
static const uint8_t  FRAME_LEN            = 6;     // TC106 応答フレームは必ず 6バイト
static const uint32_t PERIODIC_INTERVAL_MS = 1000;  // 周期送信モードの送信間隔
static const uint32_t RX_TIMEOUT_MS        = 700;   // 1バイト受信のタイムアウト

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
// TC106 風 17スロット 受信 (Phase 1.2 用)
// ============================================================

/**
 * @brief 次のバイトの preamble (HIGH) が来るのを待つ
 *
 * [設計上の注意]
 *   アイドル状態もバス HIGH なので、単純に HIGH を待つだけでは
 *   アイドルを preamble と誤検出してしまう。
 *   そのため、先に footer の LOW を確認してから次の HIGH 立ち上がりを待つ。
 *
 *   手順:
 *   1. まず LOW になるのを待つ (footer または送信中のデータ LOW)
 *   2. 次に HIGH になるのを待つ (= preamble の立ち上がりエッジ)
 *
 * @param timeoutMs タイムアウト時間 [ms]
 * @return true: preamble 検出成功、false: タイムアウト
 */
static bool waitPreambleHigh(uint32_t timeoutMs) {
    const uint32_t start = millis();

    // 1. LOW になるのを待つ (前のバイトの footer 終端か、次の LOW データ)
    while (readRxLogical()) {
        if (millis() - start > timeoutMs) return false;
    }

    // 2. LOW → HIGH の立ち上がりを待つ (= preamble の先頭)
    while (!readRxLogical()) {
        if (millis() - start > timeoutMs) return false;
    }

    return true;
}

/**
 * @brief 1バイトを 17スロットで受信する
 *
 * waitPreambleHigh() で preamble の立ち上がりエッジを検出した後、
 * slot 1〜7 の中央でサンプリングして 7ビットを復元する。
 *
 * [サンプリングタイミング]
 *   preamble 立ち上がり後:
 *     + SLOT_US     → preamble の終端 (slot 0 が終わる)
 *     + SLOT_US/2   → slot 1 の中央 (サンプリング最適位置)
 *   以降 SLOT_US ごとに次のスロット中央へ進む。
 *
 * [論理]
 *   TC106 tx_300_2 準拠:
 *     バス HIGH → bit = 0
 *     バス LOW  → bit = 1
 *
 * @param out      受信したバイト値の格納先
 * @param timeoutMs タイムアウト時間 [ms]
 * @return true: 受信成功、false: タイムアウト
 */
static bool receiveByte17Slot(uint8_t& out, uint32_t timeoutMs) {
    // preamble (HIGH) の立ち上がりを待つ
    if (!waitPreambleHigh(timeoutMs)) return false;

    // preamble の終端 + slot 1 の中央まで進む
    // (立ち上がりエッジ検出直後から 1.5スロット分待つ)
    delayMicroseconds(SLOT_US + (SLOT_US / 2));

    uint8_t value = 0;

    // slot 1〜7: 7ビット データ (LSB first)
    // TC106 tx_300_2 準拠: LOW = bit 1, HIGH = bit 0
    for (uint8_t bit = 0; bit < 7; bit++) {
        if (!readRxLogical()) {
            value |= (1 << bit);  // LOW = bit 1
        }
        delayMicroseconds(SLOT_US);
    }

    // slot 8〜16: footer (9スロット LOW) を読み飛ばす
    // 将来は LOW 連続を検証してフレーム正常性確認にできるが、
    // Phase 1.2 の簡易版では待つだけにする。
    for (uint8_t i = 0; i < 9; i++) {
        delayMicroseconds(SLOT_US);
    }

    out = value;
    return true;
}

/**
 * @brief 複数バイトを連続して受信する
 *
 * 各バイトごとに receiveByte17Slot() を呼ぶ。
 * footer の後すぐに次のバイトの preamble が来る想定。
 *
 * @param buf      受信バッファ
 * @param len      受信バイト数
 * @param timeoutMs 1バイトあたりのタイムアウト時間 [ms]
 * @return true: 全バイト受信成功、false: いずれかのバイトでタイムアウト
 */
static bool receiveFrame17Slot(uint8_t* buf, uint8_t len, uint32_t timeoutMs) {
    for (uint8_t i = 0; i < len; i++) {
        if (!receiveByte17Slot(buf[i], timeoutMs)) return false;
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

void setup() {
    Serial.begin(115200);
    delay(500);

    // TX ピン初期化: アイドル状態は HIGH
    // (TC106 firmware: 送信完了後 PORT_TX = 1)
    pinMode(TX_PIN, OUTPUT);
    writeBusHigh();

    // RX ピン初期化 (Phase 1.2 のみ)
#if !MODE_PERIODIC
    pinMode(RX_PIN, RX_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT);
#endif

    // 起動メッセージ
    Serial.println();
    Serial.println("=== NanoEvery TC106 Emulator V2.2 ===");
    Serial.println("TC106-like 17-slot, 7bit data, LSB first");
    Serial.print  ("TX_PIN  : D"); Serial.println(TX_PIN);
    Serial.print  ("RX_PIN  : D"); Serial.println(RX_PIN);
    Serial.print  ("SLOT_US : ");  Serial.print(SLOT_US);
    Serial.println(" us (3300=firmware値, 3400=実測値)");
    Serial.print  ("1 byte  : ");  Serial.print(SLOT_US * 17 / 1000);
    Serial.println(" ms");
    Serial.print  ("6 bytes : ");  Serial.print(SLOT_US * 17 * 6 / 1000);
    Serial.println(" ms");

#if MODE_PERIODIC
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
#if MODE_PERIODIC
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
        dumpFrame("[TX] ", frame, FRAME_LEN);
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
 * ESP32 D10(GPIO9) → Q2(2N7000) → Nano D13 の経路では:
 *   ESP32 D10 = HIGH → Q2 ON  → Nano D13 = LOW (R10プルアップを引き下げ)
 *   ESP32 D10 = LOW  → Q2 OFF → Nano D13 = HIGH (プルアップで HIGH 復帰)
 *
 * つまり ESP32 が送る 17スロット波形が Nano 側で論理反転して届く。
 * readRxLogical() の RX_INVERTED フラグで吸収できる。
 * 実測で判断し、受信値が期待と逆なら RX_INVERTED = true に変更する。
 */
