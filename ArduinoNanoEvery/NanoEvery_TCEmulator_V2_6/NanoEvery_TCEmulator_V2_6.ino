/**
 * NanoEvery_TCEmulator_V2_6.ino
 *
 * TC106 テンションコントローラ エミュレータ
 * Arduino Nano Every (ATmega4809, 5V 16MHz) 用
 *
 * V2.6 (Phase1.5 Step2): Nano RX 非ブロッキング化
 *   Main→TC106方向(check_port()/data_rx()相当)を実コードから
 *   起こした状態遷移表(V2.6_RX仕様抽出_ドラフトv4.md)どおりに実装。
 *   TX(V2.5, tcTxStartNonBlocking/tcTxPollNonBlocking)・
 *   旧blocking版(sendFrame17Slot/receiveFrame17Slot)・ESP32側は
 *   一切変更しない。RX engineとprotocol handler(検証用ログのみ、
 *   モータ/auto_zero/EEPROM等の物理機能は未実装)を分離して追加。
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
// MODE_RX_NONBLOCKING_TEST:
//   Phase1.5 Step2。1にすると他の全モードより優先され、
//   非ブロッキング版RX(rxNbPoll、V2.6_RX仕様抽出_ドラフトv4.md準拠)で
//   RX_PIN(D11)を継続的にpollし、受信したbyte/commandをログ出力する。
//   ESP32側は現状ブロッキングの仮スタブ(tcSend9BitByte、11bit固定・
//   BIT_US)のままのため、本モードでの単体試験はESP32との結合を
//   前提とせず、疑似波形(方式は別途決定)またはオシロ実測データ相当の
//   入力をRX_PINへ与えて検証する想定。TX(V2.5)・ESP32側は一切使用しない。
// MODE_RX_LOOPBACK_TEST:
//   Phase1.5 Step2、TcMainTx実装後の単体試験モード。1にすると他の
//   全モードより優先される。TcMainTx(Main Controller ASM忠実再現の
//   TXコア)とV2.6 RX engineを同一Nano上で同時にpollし、TX_DUMMY_PIN
//   (D9)からRX_PIN(D11)へ実配線ジャンパでループバックする。
//   RESET/SEND/SENS.ADJを既知パターンで順番に送信し、RX側のログと
//   突き合わせて復号の正しさを確認する。ESP32・既存TX(D10/V2.5)は
//   一切使用しない。
//
// ------------------------------------------------------------
// Test mode selection
//
// RX loopback regression (このV2.6版のデフォルト。D9→D11ジャンパ必須):
//   MODE_PERIODIC            = 0
//   MODE_SIMULTANEOUS        = 0
//   MODE_TX_NONBLOCKING_TEST = 0
//   MODE_RX_NONBLOCKING_TEST = 0
//   MODE_RX_LOOPBACK_TEST    = 1
//
// Non-blocking RX regression (外部波形入力用):
//   MODE_PERIODIC            = 0
//   MODE_SIMULTANEOUS        = 0
//   MODE_TX_NONBLOCKING_TEST = 0
//   MODE_RX_NONBLOCKING_TEST = 1
//   MODE_RX_LOOPBACK_TEST    = 0
//
// Non-blocking TX regression (V2.5基準):
//   MODE_PERIODIC            = 0
//   MODE_SIMULTANEOUS        = 0
//   MODE_TX_NONBLOCKING_TEST = 1
//   MODE_RX_NONBLOCKING_TEST = 0
//   MODE_RX_LOOPBACK_TEST    = 0
//
// Legacy blocking baseline:
//   MODE_PERIODIC            = 1
//   MODE_SIMULTANEOUS        = 0
//   MODE_TX_NONBLOCKING_TEST = 0
//   MODE_RX_NONBLOCKING_TEST = 0
//   MODE_RX_LOOPBACK_TEST    = 0
//
// Legacy Phase1.3.5 simultaneous test:
//   MODE_PERIODIC            = 0
//   MODE_SIMULTANEOUS        = 1
//   MODE_TX_NONBLOCKING_TEST = 0
//   MODE_RX_NONBLOCKING_TEST = 0
//   MODE_RX_LOOPBACK_TEST    = 0
//
// IMPORTANT:
//   Enable exactly ONE mode.
// ------------------------------------------------------------
#define MODE_PERIODIC             0
#define MODE_SIMULTANEOUS         0
#define MODE_TX_NONBLOCKING_TEST  0
#define MODE_RX_NONBLOCKING_TEST  0
#define MODE_RX_LOOPBACK_TEST     1

#if (MODE_PERIODIC + MODE_SIMULTANEOUS + MODE_TX_NONBLOCKING_TEST + MODE_RX_NONBLOCKING_TEST + MODE_RX_LOOPBACK_TEST) != 1
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

// Nano D9: TcMainTx(Main Controller ASM忠実再現TXコア)の出力ピン。
// MODE_RX_LOOPBACK_TESTでのみ使用。D9とD11を実配線ジャンパで直結する
// (基板上の既存経路は使わない、Nano単体でのループバック試験専用)。
static const uint8_t TX_DUMMY_PIN = 9;

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
// V2.6 (Phase1.5 Step2): Main→TC106 非ブロッキングRX
// ============================================================
//
// TC106 firmware の check_port()/data_rx() を実コードから起こした
// 状態遷移表(V2.6_RX仕様抽出_ドラフトv4.md)に準拠する。
// TC106→Main方向(上のtcTx*、下のsendFrame17Slot/receiveFrame17Slot)
// とは別プロトコル(8bit data + command/dataマーカー1bit、
// byte単位で毎回STARTを取り直す可変長フレーム)であり、
// 流用せず新規に実装する。
//
// 極性: readRxLogical()==true(受信HIGH) → bit=1
//       (TC106 data_rx()のPORT_RX==1→bit=1と同じ論理。
//        ESP32↔Nano間には実基板の反転ハードウェアが無いため、
//        ASMの生ピン規則ではなくTC106 C側の論理をそのまま使う)
//
// タイミング: TC106 Timer1(50us周期)の N+1 tick 換算により、
//   RX_HALF_SLOT_US = 1700us (rx_timer初期値33 → 34tick×50us)
//   RX_SLOT_US      = 3300us (rx_timer通常65   → 66tick×50us)
//   (Step1実測3293usからの33:65比換算は不採用。原Cのtick計算を正とする)
//
// [独立レビュー反映・v2]
//   1. Serial出力の分離:
//      115200bpsのSerial.print()は文字数に応じて数百us〜数ms
//      ブロックしうる。RX_SAMPLING/RX_FINALIZE_WAIT中にSerial出力を
//      挟むと、次に処理すべきスロット時刻を追い越してしまい、
//      それ自体がoverrun/誤サンプリングの原因になる。
//      そのためrxNbPoll()とrxProtoHandleCommand()はSerial出力を
//      一切行わず、軽量なログキュー(rxLogPush、配列書き込みのみ)に
//      積むだけにする。実際のSerial出力はrxNbDrainLog()が行うが、
//      これも「RXが受信中でない(RXNB_IDLEの時だけ)」かつ
//      「1回のloop()につき最大1件だけ」に制限し、受信タイミングに
//      影響しないようにする。
//   2. RESET checksum: byteCounter==5時に
//      (buf[0]+buf[1]+buf[2]+buf[3])&0x7F と buf[4] を比較し、
//      原C(rxreset() L1006-1061)どおりOK/NGどちらの場合も
//      処理後はbyteCounterを0へ戻す(原Cはrxreset()成功時は
//      自身でrx_bytecntr=0、失敗時・bytecntr!=5時はrxerror()経由で
//      rx_bytecntr=0にしており、結局どの経路でも0に戻る)。
//   3. RX_SLOT_OVERRUN_US: 3300(1slot)は不適切。サンプル点は各スロット
//      開始から1700us後(RX_HALF_SLOT_US)にあり、次スロット境界までは
//      3300-1700=1600usしかない。1600us以上遅延すると次スロットの
//      データを誤って読む可能性があるため、余裕を見て1000usを候補と
//      する(境界までの1600usに対し600usの安全マージン)。実測で
//      ジッタが大きいようならここを再調整する。
//   4. RX_MAX_BYTES: 原C rx_buffer[6]・rx_bytecntr==6でrxerror()に
//      合わせ、8→6へ修正。

static const uint32_t RX_SLOT_US       = 3300;  // 通常スロット周期
static const uint32_t RX_HALF_SLOT_US  = 1700;  // START検出後、最初のサンプルまで

// サンプル点はスロット開始から1700us後にあり、次スロット境界まで
// 3300-1700=1600us。それを超えて遅延すると隣のスロットを誤読する
// おそれがあるため、1600usより十分小さい値を暫定採用する。
static const uint32_t RX_SLOT_OVERRUN_US = 1000;

static const uint8_t  RX_WAIT_SLOTS    = 17;    // byte間タイムアウト(rx_waitcntr相当)
static const uint32_t RX_WAIT_DEADLINE_US = (uint32_t)RX_WAIT_SLOTS * RX_SLOT_US; // 56100us
static const uint8_t  RX_MAX_BYTES     = 6;     // 原C rx_buffer[6]に合わせる

// RESETの2セット目開始まで(1セット目の確定byte finalizeから)は
// 約8.25msしかなく、その間にSerial出力を挟むと次STARTのpollを
// 遅延させ見逃す恐れがある(独立レビュー指摘1)。そのため
// RXNB_IDLE中であっても、直近のRX活動からRX_LOG_QUIET_US以上
// 経過して初めてログを出力する。70000us(70ms)は暫定候補で、
// 実運用のコマンド送信間隔を見て再調整する。
static const uint32_t RX_LOG_QUIET_US = 70000;

enum RxNbState : uint8_t {
    RXNB_IDLE = 0,          // START(LOW)待ち
    RXNB_SAMPLING,          // bitIndex 1..10 を1slotごとにサンプル中
    RXNB_FINALIZE_WAIT,     // bitIndex10サンプル後、1slot待ってbyte確定
    RXNB_WAIT_NEXT_BYTE,    // byte確定(data)後、次byteのSTART or timeout待ち
    RXNB_RESYNC_WAIT_HIGH,  // overrun abort後、HIGHを1度確認するまで待機
};

// イベント種別: rxNbPoll()呼び出し後、この値で何が起きたかを判定する
// (呼び出し側での即時分岐用。実際のログ出力はrxLogQueue経由)
enum RxNbEvent : uint8_t {
    RXNB_EVENT_NONE = 0,
    RXNB_EVENT_DATA_BYTE,   // 1byte確定、データとしてrxNbRxBufferへ格納した
    RXNB_EVENT_COMMAND,     // 1byte確定、command/dataマーカーが立っていた(dispatch)
    RXNB_EVENT_TIMEOUT,     // RX_WAIT_NEXT_BYTE中にdeadline超過、byteCounterをリセットした
    RXNB_EVENT_OVERRUN,     // 許容量を超えて遅延しabortした
};

static RxNbState rxNbState        = RXNB_IDLE;
static uint8_t   rxNbBitIndex     = 0;      // 1..10
static uint8_t   rxNbByteReg      = 0;      // シフトレジスタ(TC106 rx_buffer_1相当、9回挿入・8bit保持)
static bool      rxNbCommandFlag  = false;
static uint32_t  rxNbNextDueUs    = 0;
static uint32_t  rxNbWaitDeadlineUs = 0;
static uint8_t   rxNbByteCounter  = 0;      // TC106 rx_bytecntr相当
static uint8_t   rxNbRxBuffer[RX_MAX_BYTES];
static uint32_t  rxNbOverrunCount = 0;
static uint32_t  rxNbTimeoutCount = 0;

// 直近にRX activity(IDLE以外の処理)があった時刻。rxNbDrainLog()が
// 「十分quietか」を判定するのに使う(独立レビュー指摘1対応)。
static uint32_t  lastRxActivityUs = 0;

// 直近イベントの詳細(呼び出し側が読む。ログキューにも同時に積まれる)
static uint8_t rxNbLastByteValue      = 0;  // EVENT_DATA_BYTE/EVENT_COMMAND時のbyteReg値
static uint8_t rxNbLastCmdCode        = 0;  // EVENT_COMMAND時: byteReg & 0x07
static uint8_t rxNbByteCounterAtEvent = 0;  // イベント発生時点(handler呼び出し前)のbyteCounter

static bool tcRxBusy() {
    return rxNbState != RXNB_IDLE;
}

// overrun abort時に呼ぶ。旧フレームのbyte途中の状態を一切残さず、
// 完全にクリアしてからRESYNC_WAIT_HIGHへ入る(独立レビュー指摘2対応)。
// byteCounter(複数byteコマンドの継続状態)も含めて破棄する。
static void rxNbAbortToResync() {
    rxNbByteCounter  = 0;
    rxNbBitIndex     = 0;
    rxNbByteReg      = 0;
    rxNbCommandFlag  = false;
    rxNbState        = RXNB_RESYNC_WAIT_HIGH;
}

// ------------------------------------------------------------
// ログキュー(修正1対応)
//
// rxNbPoll()/rxProtoHandleCommand()はここへ積むだけで、
// Serial.print()を直接呼ばない。実際の出力はrxNbDrainLog()が、
// RXNB_IDLE中に1loopあたり最大1件だけ行う。
// ------------------------------------------------------------
enum RxLogType : uint8_t {
    RXLOG_DATA_BYTE,
    RXLOG_COMMAND_BYTE,
    RXLOG_TIMEOUT,
    RXLOG_OVERRUN,
    RXLOG_OVERFLOW,
    RXLOG_CMD_RESET,
    RXLOG_CMD_ADJ,
    RXLOG_CMD_SEND,
    RXLOG_CMD_UNKNOWN,
};

struct RxLogEntry {
    uint32_t   ms;
    RxLogType  type;
    uint8_t    a;      // 用途はtypeごとに異なる(下記drain関数のswitch参照)
    uint8_t    b;
    int32_t    extra;  // lateBy_us等、符号付きが必要な値
};

static const uint8_t RX_LOG_QUEUE_SIZE = 16;
static RxLogEntry rxLogQueue[RX_LOG_QUEUE_SIZE];
static uint8_t  rxLogHead = 0;
static uint8_t  rxLogTail = 0;
static uint32_t rxLogDroppedCount = 0;

// 軽量: 配列書き込みのみ、Serial呼び出しなし。RX engineの
// タイミングクリティカルな経路から安全に呼べる。
static void rxLogPush(RxLogType type, uint8_t a, uint8_t b, int32_t extra) {
    uint8_t nextHead = (uint8_t)((rxLogHead + 1) % RX_LOG_QUEUE_SIZE);
    if (nextHead == rxLogTail) {
        // キュー満杯: 今回分は破棄し、破棄件数だけ数える
        // (満杯時に古いものを詰め直す処理自体もタイミングに影響するため行わない)
        rxLogDroppedCount++;
        return;
    }
    RxLogEntry& e = rxLogQueue[rxLogHead];
    e.ms = millis();
    e.type = type;
    e.a = a;
    e.b = b;
    e.extra = extra;
    rxLogHead = nextHead;
}

// ------------------------------------------------------------
// protocol handler (検証用ログのみ。TC106のrxreset/rxadj/rxsendに
// 対応するが、モータ制御・auto_zero()・EEPROM書き込み等の物理機能は
// Step2では実装しない。Serial出力は行わずrxLogPush()に積むのみ
// (修正1)。
//
// TODO(将来のTC106機能エミュレーション向け):
//   TC106実機は (flag2.byte&0x07)==0 または ==7 のときのみ
//   table_rx()からのdispatchを実行し、それ以外はrx_bytecntrを
//   リセットするだけで無視する(多重dispatch防止と思われるガード)。
//   Step2はRX復号確認が目的のためこのガードは未実装。
// ------------------------------------------------------------
static void rxProtoHandleCommand(uint8_t cmdCode, uint8_t* byteCounterInOut,
                                  const uint8_t* buf) {
    switch (cmdCode) {
        case 1: { // RESET相当 (rxreset() L999-1061)
            if (*byteCounterInOut == 5) {
                uint8_t calc = (uint8_t)((buf[0] + buf[1] + buf[2] + buf[3]) & 0x7F);
                bool ok = (calc == buf[4]);
                // a: byteCounter(常に5のはず), b: 1=OK / 0=NG
                rxLogPush(RXLOG_CMD_RESET, *byteCounterInOut, ok ? 1 : 0, 0);
            } else {
                // 原Cではrx_bytecntr!=5のときrxerror()経由でbytecntr=0
                // (b=2: 想定外のbyteCounterでdispatchされたことを示す)
                rxLogPush(RXLOG_CMD_RESET, *byteCounterInOut, 2, 0);
            }
            // 原C: 成功時はrxreset()自身がrx_bytecntr=0、失敗時・
            // bytecntr!=5時もrxerror()経由でrx_bytecntr=0になる。
            // → どちらの場合も必ず0に戻す(2セット目を正しく
            //    受信するために必須。ここを5のまま維持していたのが
            //    前バージョンの誤りだった)。
            *byteCounterInOut = 0;
            break;
        }

        case 2: // SENS.ADJ相当 (rxadj())
            rxLogPush(RXLOG_CMD_ADJ, *byteCounterInOut, 0, 0);
            *byteCounterInOut = 0;
            break;

        case 3: // SEND相当 (rxsend())
            if (*byteCounterInOut == 1) {
                // a=1: 張力採用相当、b: buf[0]の値
                rxLogPush(RXLOG_CMD_SEND, 1, buf[0], 0);
                // rxsend()同様、byteCounterはここでは変更しない
            } else if (*byteCounterInOut == 2) {
                // a=2: 2組目確認、byteCounterリセットのみ(buf[1]未参照)
                rxLogPush(RXLOG_CMD_SEND, 2, 0, 0);
                *byteCounterInOut = 0;
            } else {
                // a=0: 想定外のbyteCounterでdispatch、bにその値を積む
                // 原C: rxsend()は1/2以外だとrxerror()相当でrx_bytecntr=0に
                // なる(暗黙のelse)。ログのみでcounterを残していたのは誤り。
                rxLogPush(RXLOG_CMD_SEND, 0, *byteCounterInOut, 0);
                *byteCounterInOut = 0;
            }
            break;

        default:
            rxLogPush(RXLOG_CMD_UNKNOWN, cmdCode, 0, 0);
            break;
    }
}

// 毎loop()で呼ぶ。1pollで最大1ステップだけ状態を進める(catch-up禁止)。
// イベントが発生した場合はそのRxNbEventを返す(発生しなければNONE)。
// Serial出力は一切行わない(修正1)。詳細ログはrxLogQueue経由で
// rxNbDrainLog()が別途出力する。
static RxNbEvent rxNbPoll() {
    const uint32_t now = micros();

    // このpoll開始時点で既に受信活動中(IDLE以外)なら、活動時刻を更新する。
    // rxNbDrainLog()はこれを見て「十分quietか」を判定する(指摘1対応)。
    if (rxNbState != RXNB_IDLE) {
        lastRxActivityUs = now;
    }

    switch (rxNbState) {

        case RXNB_IDLE: {
            // START(LOW)判定はレベル検出でよい(原Cもcheck_port()の
            // ポーリングであり、厳密なエッジ割込みではない)
            if (!readRxLogical()) {
                rxNbBitIndex  = 0;
                rxNbByteReg   = 0;
                rxNbNextDueUs = now + RX_HALF_SLOT_US; // bitIndex=1のサンプル時刻
                rxNbState     = RXNB_SAMPLING;
            }
            return RXNB_EVENT_NONE;
        }

        case RXNB_SAMPLING: {
            if ((int32_t)(now - rxNbNextDueUs) < 0) return RXNB_EVENT_NONE; // まだ

            // catch-up禁止: 許容量(RX_SLOT_OVERRUN_US)を超えて
            // 遅延していたらabort
            if ((int32_t)(now - rxNbNextDueUs) > (int32_t)RX_SLOT_OVERRUN_US) {
                rxNbOverrunCount++;
                rxLogPush(RXLOG_OVERRUN, rxNbBitIndex, 0,
                          (int32_t)(now - rxNbNextDueUs));
                rxNbAbortToResync();
                return RXNB_EVENT_OVERRUN;
            }

            rxNbBitIndex++;

            if (rxNbBitIndex <= 9) {
                // bitIndex 1..9: 9回シフト挿入。最初の1回(bitIndex=1、
                // START自身のレベル)は後段8回のシフトで押し出される。
                // TC106 data_rx()と同じ「右シフト+MSB挿入」を再現する。
                if (readRxLogical()) {
                    rxNbByteReg = (uint8_t)((rxNbByteReg >> 1) | 0x80);
                } else {
                    rxNbByteReg = (uint8_t)(rxNbByteReg >> 1);
                }
                rxNbNextDueUs += RX_SLOT_US;
                return RXNB_EVENT_NONE;
            }

            // bitIndex == 10: command/dataマーカーをサンプル(byteRegへは挿入しない)
            rxNbCommandFlag = readRxLogical();
            rxNbNextDueUs += RX_SLOT_US;
            rxNbState = RXNB_FINALIZE_WAIT;
            return RXNB_EVENT_NONE;
        }

        case RXNB_FINALIZE_WAIT: {
            if ((int32_t)(now - rxNbNextDueUs) < 0) return RXNB_EVENT_NONE; // まだ

            if ((int32_t)(now - rxNbNextDueUs) > (int32_t)RX_SLOT_OVERRUN_US) {
                rxNbOverrunCount++;
                rxLogPush(RXLOG_OVERRUN, 11, 0, (int32_t)(now - rxNbNextDueUs));
                rxNbAbortToResync();
                return RXNB_EVENT_OVERRUN;
            }

            // bitIndex == 11相当: サンプルなし、byte確定処理のみ
            rxNbLastByteValue      = rxNbByteReg;
            rxNbByteCounterAtEvent = rxNbByteCounter;

            if (!rxNbCommandFlag) {
                // データbyte: rxBufferへ格納
                if (rxNbByteCounter < RX_MAX_BYTES) {
                    rxNbRxBuffer[rxNbByteCounter] = rxNbByteReg;
                }
                rxNbByteCounter++;
                rxLogPush(RXLOG_DATA_BYTE, rxNbByteCounterAtEvent, rxNbLastByteValue, 0);

                if (rxNbByteCounter >= RX_MAX_BYTES) {
                    // 原C: rx_bytecntr==6 → rxerror()相当(bytecntr=0)。
                    // rxerror()はbyte間waitに入らずここで完結するため、
                    // RXNB_WAIT_NEXT_BYTEを経由せず直接IDLEへ戻す
                    // (独立レビュー指摘4対応。以前はWAIT_NEXT_BYTE経由で、
                    // 無通信時に不要なTIMEOUTログが出ていた)。
                    rxLogPush(RXLOG_OVERFLOW, rxNbByteCounter, 0, 0);
                    rxNbByteCounter = 0;
                    rxNbState = RXNB_IDLE;
                } else {
                    rxNbWaitDeadlineUs = now + RX_WAIT_DEADLINE_US;
                    rxNbState = RXNB_WAIT_NEXT_BYTE;
                }
                return RXNB_EVENT_DATA_BYTE;
            } else {
                // コマンドbyte: protocol handlerへ委譲(byteCounterは
                // handler側で必要に応じて書き換える)
                rxNbLastCmdCode = (uint8_t)(rxNbByteReg & 0x07);
                rxLogPush(RXLOG_COMMAND_BYTE, rxNbByteCounterAtEvent, rxNbLastByteValue, 0);
                rxProtoHandleCommand(rxNbLastCmdCode, &rxNbByteCounter, rxNbRxBuffer);

                if (rxNbByteCounter != 0) {
                    // handlerがカウンタを維持した = 続きのbyteを待つ
                    rxNbWaitDeadlineUs = now + RX_WAIT_DEADLINE_US;
                    rxNbState = RXNB_WAIT_NEXT_BYTE;
                } else {
                    rxNbState = RXNB_IDLE;
                }
                return RXNB_EVENT_COMMAND;
            }
        }

        case RXNB_WAIT_NEXT_BYTE: {
            if (!readRxLogical()) {
                // 新規START: 即座に半スロット待ちへ(byteCounterは維持したまま)
                rxNbBitIndex  = 0;
                rxNbByteReg   = 0;
                rxNbNextDueUs = now + RX_HALF_SLOT_US;
                rxNbState     = RXNB_SAMPLING;
                return RXNB_EVENT_NONE;
            }
            if ((int32_t)(now - rxNbWaitDeadlineUs) >= 0) {
                // catch-upはしない。単純にタイムアウト破棄。
                rxNbTimeoutCount++;
                rxLogPush(RXLOG_TIMEOUT, rxNbByteCounter, 0, 0);
                rxNbByteCounter = 0;
                rxNbState = RXNB_IDLE;
                return RXNB_EVENT_TIMEOUT;
            }
            return RXNB_EVENT_NONE; // 何もしない
        }

        case RXNB_RESYNC_WAIT_HIGH: {
            // overrun abort直後、LOWが継続中の可能性があるため、
            // 一度HIGHを確認してから次のLOWを正式なSTARTとして受理する。
            if (readRxLogical()) {
                rxNbState = RXNB_IDLE;
            }
            return RXNB_EVENT_NONE;
        }
    }

    return RXNB_EVENT_NONE; // 到達しないはずだが念のため
}

// ------------------------------------------------------------
// ログキューの出力(独立レビュー指摘1・v2対応)
//
// RXNB_IDLE = 「安全にSerial出力できる状態」ではなく、単に
// 次STARTを監視中というだけ。RESET等の複数byteコマンドは
// セット間の間隔が約8.25msしかなく、その間にIDLEへ戻ることが
// あるため、IDLE判定だけでは不十分。
// 直近のRX活動(lastRxActivityUs)からRX_LOG_QUIET_US以上
// 経過して初めて「本当に一連の受信が終わった」とみなし、
// 1回のloop()あたり最大1件だけSerial出力する。
// ------------------------------------------------------------
static void rxNbDrainLog() {
    if (rxNbState != RXNB_IDLE) return;   // 受信中は出力しない
    if ((int32_t)(micros() - lastRxActivityUs) < (int32_t)RX_LOG_QUIET_US) return; // まだquietではない
    if (rxLogTail == rxLogHead) return;   // キューが空

    RxLogEntry e = rxLogQueue[rxLogTail];
    rxLogTail = (uint8_t)((rxLogTail + 1) % RX_LOG_QUEUE_SIZE);

    Serial.print('[');
    Serial.print(e.ms);
    Serial.print(" ms] ");

    switch (e.type) {
        case RXLOG_DATA_BYTE:
            rxSeq++;
            Serial.print("[RX-NB] #"); Serial.print(rxSeq);
            Serial.print(" data byteCounter(before)="); Serial.print(e.a);
            Serial.print(" value=0x");
            if (e.b < 0x10) Serial.print('0');
            Serial.println(e.b, HEX);
            break;

        case RXLOG_COMMAND_BYTE:
            rxSeq++;
            Serial.print("[RX-NB] #"); Serial.print(rxSeq);
            Serial.print(" command byteCounter(before)="); Serial.print(e.a);
            Serial.print(" value=0x");
            if (e.b < 0x10) Serial.print('0');
            Serial.print(e.b, HEX);
            Serial.print(" cmd="); Serial.println(e.b & 0x07);
            break;

        case RXLOG_TIMEOUT:
            Serial.print("[RX-TIMEOUT] byteCounter(before reset)=");
            Serial.println(e.a);
            break;

        case RXLOG_OVERRUN:
            Serial.print("[RX-OVERRUN] bitIndex="); Serial.print(e.a);
            Serial.print(" lateBy_us="); Serial.println(e.extra);
            break;

        case RXLOG_OVERFLOW:
            Serial.print("[RX-ERR] byteCounter overflow(>=");
            Serial.print(RX_MAX_BYTES);
            Serial.println("), resetting (rxerror()相当)");
            break;

        case RXLOG_CMD_RESET:
            Serial.print("[RX-CMD] RESET byteCounter="); Serial.print(e.a);
            if (e.b == 1) {
                Serial.println(" checksum=OK(検証のみ、物理動作なし)");
            } else if (e.b == 0) {
                Serial.println(" checksum=NG(検証のみ、物理動作なし)");
            } else {
                Serial.println(" 想定外byteCounterでdispatch(rxerror()相当)");
            }
            break;

        case RXLOG_CMD_ADJ:
            Serial.println("[RX-CMD] SENS.ADJ dispatch(検証のみ、物理動作なし)");
            break;

        case RXLOG_CMD_SEND:
            if (e.a == 1) {
                Serial.print("[RX-CMD] SEND: 張力採用相当 buf[0]=0x");
                if (e.b < 0x10) Serial.print('0');
                Serial.println(e.b, HEX);
            } else if (e.a == 2) {
                Serial.println("[RX-CMD] SEND: 2組目確認、byteCounterリセットのみ(buf[1]未参照)");
            } else {
                Serial.print("[RX-CMD] SEND dispatch想定外byteCounter=");
                Serial.println(e.b);
            }
            break;

        case RXLOG_CMD_UNKNOWN:
            Serial.print("[RX-CMD] 未定義/未実装 cmd=0x");
            Serial.println(e.a, HEX);
            break;
    }

    if (rxLogDroppedCount > 0) {
        Serial.print("[RX-LOG] queue full, dropped=");
        Serial.println(rxLogDroppedCount);
        rxLogDroppedCount = 0;
    }
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

// ============================================================
// TcMainTx: Main Controller ASM(zhukongban.asm)忠実再現・
// 移植可能な非ブロッキングTXコア(Phase1.5 Step2試験治具)
// ============================================================
//
// 正本はASMのみ。現行ESP32スタブ(tcSend9BitByte)には一切合わせない。
// 出力する論理レベルはTC106が受信すべき論理値を直接生成する
// (V2.6 RX engineのreadRxLogical()と対称、反転ハードウェアなし前提):
//   idle: HIGH / START(slot0): LOW / bit=1: HIGH、bit=0: LOW(LSB first)
//   command/dataマーカー(slot9): 確定byteならHIGH、データbyteならLOW
//   guard: HIGH
//
// GPIOアクセスはwriteLogical()の1箇所のみに局所化。ESP32移植時は
// ここだけ差し替えれば済む。
//
// ASM確認済み事項(独立レビュー反映):
//   ・RESET/SENDの張力byteは送信前に1bit左シフトされる
//     (send_reset: txbyte_cntr==3/9でbcf STATUS,C; rlf rreg_1、
//      send_send: byte0/byte2のtens_ch1/ch3読み出し直後で同様)。
//     API引数tensはMain内部値(シフト前)と定義し、内部でtxTens=tens<<1
//     を送信する。
//   ・RESET checksumは (len0+len1+len2+txTens)&0x7F
//     (原ASMのchecksum計算がtens_ch1を2回加算しているのと数学的に等価、
//      Draft0.3で確認済み)。

static const uint32_t TX_SLOT_OVERRUN_US_TCMAINTX = 1000; // RX側と同じ根拠で1slot(3300)は不採用

class TcMainTx {
public:
    void begin(uint8_t pin) {
        _pin = pin;
        pinMode(_pin, OUTPUT);
        writeLogical(true); // idle = 論理HIGH
        _state = TXTX_IDLE;
        _doneFlag = false;
        _aborted = false;
    }

    bool busy() const { return _state != TXTX_IDLE; }

    // RESET: tensはMain内部値(シフト前)。内部でtxTens=tens<<1を送信し、
    // checksum=(len0+len1+len2+txTens)&0x7Fを計算、同一内容を2セット
    // (確定byteのみ0xF1→0xF9)連続送信する。
    bool sendReset(uint8_t len0, uint8_t len1, uint8_t len2, uint8_t tens) {
        if (busy()) return false;
        const uint8_t txTens = (uint8_t)(tens << 1);
        const uint8_t checksum = (uint8_t)((len0 + len1 + len2 + txTens) & 0x7F);

        _frame.byte[0]  = { len0,     false };
        _frame.byte[1]  = { len1,     false };
        _frame.byte[2]  = { len2,     false };
        _frame.byte[3]  = { txTens,   false };
        _frame.byte[4]  = { checksum, false };
        _frame.byte[5]  = { 0xF1,     true  };
        _frame.byte[6]  = { len0,     false };
        _frame.byte[7]  = { len1,     false };
        _frame.byte[8]  = { len2,     false };
        _frame.byte[9]  = { txTens,   false };
        _frame.byte[10] = { checksum, false };
        _frame.byte[11] = { 0xF9,     true  };
        _frame.byteCount    = 12;
        _frame.slotsPerByte = 13;
        startFrame();
        return true;
    }

    // SEND: ch1Tens/ch3Tensともtens<<1で送信。ASM再確認により
    // send_send_2_1(byte0=tens_ch1)・send_send_2_2(byte2=tens_ch3)の
    // 両方でbcf STATUS,C; rlf rreg_1 (左シフト)を確認済み。
    // CH1単体試験ではch3Tens=0(既定)でよい。
    bool sendTension(uint8_t ch1Tens, uint8_t ch3Tens = 0) {
        if (busy()) return false;
        const uint8_t txTens1 = (uint8_t)(ch1Tens << 1);
        const uint8_t txTens3 = (uint8_t)(ch3Tens << 1);

        _frame.byte[0] = { txTens1, false };
        _frame.byte[1] = { 0xF3,    true  };
        _frame.byte[2] = { txTens3, false };
        _frame.byte[3] = { 0xFB,    true  };
        _frame.byteCount    = 4;
        _frame.slotsPerByte = 13;
        startFrame();
        return true;
    }

    // SENS.ADJ (isBreak=falseで0xF2、trueで0xF4/BREAK)
    bool sendSensAdj(bool isBreak = false) {
        if (busy()) return false;
        _frame.byte[0] = { (uint8_t)(isBreak ? 0xF4 : 0xF2), true };
        _frame.byteCount    = 1;
        _frame.slotsPerByte = 11;
        startFrame();
        return true;
    }

    // 毎loop()で呼ぶ。1pollで最大1step、catch-up禁止。Serial出力は
    // 一切行わない(呼び出し側がconsumeDoneFlag()で完了を検知する)。
    void poll() {
        if (_state == TXTX_IDLE) return;

        const uint32_t now = micros();
        const int32_t lateBy = (int32_t)(now - _nextDueUs);
        if (lateBy < 0) return; // まだ

        if (lateBy > (int32_t)TX_SLOT_OVERRUN_US_TCMAINTX) {
            _overrunCount++;
            writeLogical(true); // idleへ強制復帰(V2.6 RXのRESYNC_WAIT_HIGHと対になる安全策)
            _state = TXTX_IDLE;
            _doneFlag = true;
            _aborted = true;
            return;
        }

        if (_slotIndex >= 1 && _slotIndex <= 8) {
            const bool bit = ((_frame.byte[_byteIndex].value >> (_slotIndex - 1)) & 0x01) != 0;
            writeLogical(bit); // LSB first: bit=1→HIGH, bit=0→LOW
            _nextDueUs += SLOT_US;
            _slotIndex++;
            return;
        }

        if (_slotIndex == 9) {
            writeLogical(_frame.byte[_byteIndex].isCommand); // マーカー: 確定byte=HIGH
            _nextDueUs += SLOT_US;
            _slotIndex++;
            return; // byte境界判定はここでしない(下記参照)
        }

        if (_slotIndex >= 10 && _slotIndex < _frame.slotsPerByte) {
            // guard: 最初の1回(slotIndex==10)だけHIGHへ、以降はレベルを
            // 変えずタイミングだけ消費する。
            if (_slotIndex == 10) {
                writeLogical(true);
            }
            _nextDueUs += SLOT_US;
            _slotIndex++;
            return; // byte境界判定はここでしない(下記参照)
        }

        // slotIndex == slotsPerByte: byte境界を独立イベントとして扱う
        // (Step1のslot16/17境界修正と同じ考え方)。
        // これにより、直前のguard最終スロット(13-stepならslot12、
        // 11-stepならslot10)がSLOT_US分きちんと保持されてから、
        // 次byteのSTART(またはフレーム完了)へ遷移する。
        // 以前はここをslot9/guardの処理と同じpoll()呼び出し内で
        // 即座に判定していたため、最終guardスロットが1slot分
        // 短くなっていた(独立レビュー指摘1で修正)。
        advanceByte();
    }

    // フレーム完了(正常/overrun異常とも)を1回だけ通知する。
    // outAbortedにoverrunによる異常終了だったかを返す。
    bool consumeDoneFlag(bool* outAborted = nullptr) {
        if (!_doneFlag) return false;
        _doneFlag = false;
        if (outAborted) *outAborted = _aborted;
        _aborted = false;
        return true;
    }

    uint32_t overrunCount() const { return _overrunCount; }

private:
    struct TxByteSpec { uint8_t value; bool isCommand; };
    struct TxFrame {
        TxByteSpec byte[12]; // RESET(12byte)が最大
        uint8_t    byteCount;
        uint8_t    slotsPerByte;
    };
    enum TxTxState : uint8_t { TXTX_IDLE, TXTX_SENDING };

    void startFrame() {
        _byteIndex = 0;
        writeLogical(false); // slot0 START = 論理LOW、即時出力
        _slotIndex = 1;
        _nextDueUs = micros() + SLOT_US;
        _state = TXTX_SENDING;
    }

    void advanceByte() {
        _byteIndex++;
        if (_byteIndex >= _frame.byteCount) {
            writeLogical(true); // 全byte送信完了、idleへ
            _state = TXTX_IDLE;
            _doneFlag = true;
            return;
        }
        // 次byteのSTART(論理LOW)を即時出力(slot0は他のbyteと同様、
        // 待機なしで即時出力する)。このpoll呼び出し自体が
        // 「now == 前byteのbyte境界時刻」で起きているため、次に
        // 待つべきslot1の時刻は「今 + SLOT_US」。境界判定側の
        // poll()はここに来る前に_nextDueUsを進めていない(独立
        // イベント化した際に増分をここへ移した)ため、ここで
        // 明示的に加算する。
        writeLogical(false);
        _slotIndex = 1;
        _nextDueUs += SLOT_US;
    }

    void writeLogical(bool high) {
        digitalWrite(_pin, high ? HIGH : LOW);
    }

    uint8_t   _pin = 0;
    TxTxState _state = TXTX_IDLE;
    TxFrame   _frame;
    uint8_t   _byteIndex = 0;
    uint8_t   _slotIndex = 0;
    uint32_t  _nextDueUs = 0;
    bool      _doneFlag = false;
    bool      _aborted = false;
    uint32_t  _overrunCount = 0;
};

static TcMainTx tcMainTx;

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
    Serial.println("=== NanoEvery TC106 Emulator V2.6 (Phase1.5 Step2: RX non-blocking) ===");
    Serial.println("TC106-like 17-slot, 7bit data, LSB first (TX) / Main-TC106 command RX (RX)");
    Serial.print  ("TX_PIN  : D"); Serial.println(TX_PIN);
    Serial.print  ("RX_PIN  : D"); Serial.println(RX_PIN);
    Serial.print  ("SLOT_US : ");  Serial.print(SLOT_US);
    Serial.println(" us (3300=firmware値, 3400=実測値)");
    Serial.print  ("1 byte  : ");  Serial.print(SLOT_US * 17 / 1000);
    Serial.println(" ms");
    Serial.print  ("6 bytes : ");  Serial.print(SLOT_US * 17 * 6 / 1000);
    Serial.println(" ms");

#if MODE_RX_LOOPBACK_TEST
    Serial.println("Mode: RX_LOOPBACK_TEST (Phase 1.5 Step2, TcMainTx)");
    Serial.print  ("TX_DUMMY_PIN(TcMainTx)  : D"); Serial.println(TX_DUMMY_PIN);
    Serial.print  ("RX_PIN(V2.6 RX engine)  : D"); Serial.println(RX_PIN);
    Serial.println("D9-D11間を配線ジャンパで直結してください(実GPIOループバック)。");
    Serial.print  ("RX_INVERTED        : ");
    Serial.println(RX_INVERTED ? "true (Q2経由反転あり)" : "false (反転なし)");
    Serial.print  ("RX_SLOT_US         : "); Serial.println(RX_SLOT_US);
    Serial.print  ("RX_HALF_SLOT_US    : "); Serial.println(RX_HALF_SLOT_US);
    Serial.print  ("RX_SLOT_OVERRUN_US : "); Serial.println(RX_SLOT_OVERRUN_US);
    Serial.print  ("TX_SLOT_OVERRUN_US(TcMainTx): "); Serial.println(TX_SLOT_OVERRUN_US_TCMAINTX);
    Serial.println("ESP32・既存TX(D10/V2.5)は使用しない。");
    tcMainTx.begin(TX_DUMMY_PIN);
#elif MODE_RX_NONBLOCKING_TEST
    Serial.println("Mode: RX_NONBLOCKING_TEST (Phase 1.5 Step2)");
    Serial.print  ("RX_INVERTED        : ");
    Serial.println(RX_INVERTED ? "true (Q2経由反転あり)" : "false (反転なし)");
    Serial.print  ("RX_USE_PULLUP      : ");
    Serial.println(RX_USE_INTERNAL_PULLUP ? "true" : "false");
    Serial.print  ("RX_SLOT_US         : "); Serial.println(RX_SLOT_US);
    Serial.print  ("RX_HALF_SLOT_US    : "); Serial.println(RX_HALF_SLOT_US);
    Serial.print  ("RX_SLOT_OVERRUN_US : "); Serial.println(RX_SLOT_OVERRUN_US);
    Serial.print  ("RX_WAIT_DEADLINE_US: "); Serial.println(RX_WAIT_DEADLINE_US);
    Serial.print  ("RX_MAX_BYTES       : "); Serial.println(RX_MAX_BYTES);
    Serial.print  ("RX_LOG_QUIET_US    : "); Serial.println(RX_LOG_QUIET_US);
    Serial.println("ESP32/TX(V2.5)は使用しない。RX_PINへの入力のみで動作。");
#elif MODE_TX_NONBLOCKING_TEST
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
#if MODE_RX_LOOPBACK_TEST
    // ──────────────────────────────────────────────────────
    // Phase 1.5 Step2: TcMainTx + V2.6 RX engine 実GPIOループバック試験
    //
    // TX_DUMMY_PIN(D9)からRX_PIN(D11)へ配線ジャンパで直結する前提。
    // TcMainTxとrxNbPollを同一loop()内で並行してpollする。
    // 両者ともSerial出力をpoll内で行わない設計のため、ここで
    // まとめて処理してもタイミングには影響しない。
    // ──────────────────────────────────────────────────────
    static uint32_t lastSendMs = 0;
    static uint8_t  patternIndex = 0;
    static const uint32_t LOOPBACK_INTERVAL_MS = 1000; // Step2初回試験の推奨値

    tcMainTx.poll();
    rxNbPoll();
    rxNbDrainLog();

    // 完了ログの確認を、次コマンド開始判定より先に行う。
    // RESETは12×13×3300=514800usかかる一方、LOOPBACK_INTERVAL_MSの
    // 条件はその間に満たされてしまうため、順序を誤ると次コマンドの
    // slot0 START出力後にこの完了ログが割り込む恐れがある
    // (独立レビュー指摘対応)。
    {
        bool aborted = false;
        if (tcMainTx.consumeDoneFlag(&aborted)) {
            if (aborted) {
                Serial.print("[TX-CORE] ABORTED (overrun) count=");
                Serial.println(tcMainTx.overrunCount());
            } else {
                Serial.println("[TX-CORE] frame done");
            }
        }
    }

    if (!tcMainTx.busy() && (millis() - lastSendMs >= LOOPBACK_INTERVAL_MS)) {
        lastSendMs = millis();
        switch (patternIndex) {
            case 0:
                Serial.println("[TX-CORE] sendReset(len=0x00,0x00,0x00, tens=0x1E)");
                tcMainTx.sendReset(0x00, 0x00, 0x00, 0x1E);
                break;
            case 1:
                Serial.println("[TX-CORE] sendTension(ch1=0x32)");
                tcMainTx.sendTension(0x32);
                break;
            case 2:
                Serial.println("[TX-CORE] sendSensAdj()");
                tcMainTx.sendSensAdj();
                break;
        }
        patternIndex = (patternIndex + 1) % 3;
    }

#elif MODE_RX_NONBLOCKING_TEST
    // ──────────────────────────────────────────────────────
    // Phase 1.5 Step2: 非ブロッキングRXの単体試験モード
    //
    // RX_PIN(D11)を継続的にpollし、受信したbyte/commandを
    // ログ出力するだけのモード。ESP32・TX(V2.5)は使用しない。
    // 入力波形(疑似波形の生成方式)は別途決定するため、
    // ここでは「何らかの入力がRX_PINに来る」ことだけを前提とする。
    //
    // loop()自体はbusy-waitを含まない(rxNbPoll()は1pollで
    // 最大1ステップしか進めない、catch-up禁止)。
    //
    // Serial出力はrxNbPoll()/rxProtoHandleCommand()の中では行わず、
    // rxNbDrainLog()がRXNB_IDLE中(=タイミングにクリティカルでない
    // 期間)にのみ、1loopあたり最大1件だけ出力する(独立レビュー
    // 指摘1対応)。
    // ──────────────────────────────────────────────────────
    static uint32_t idleLoopCount = 0; // busy-waitしていないことの確認用
    idleLoopCount++;

    rxNbPoll();      // 状態機械を最大1ステップ進める(Serial出力なし)
    rxNbDrainLog();  // IDLE中のみ、キューから最大1件だけSerial出力

#elif MODE_TX_NONBLOCKING_TEST
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
