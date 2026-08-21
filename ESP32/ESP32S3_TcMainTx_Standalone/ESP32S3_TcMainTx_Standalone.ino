/**
 * ESP32S3_TcMainTx_Standalone.ino
 *
 * Phase 1.5 Step2 (ESP32-S3側): TcMainTx 移植・単体TX試験スケッチ
 *
 * Main Controller ASM(zhukongban.asm)忠実再現の非ブロッキングTXコア
 * TcMainTx を、Nano Every V2.6(D9-D11自己ループバックでPASS済み)から
 * ESP32-S3へ移植する。
 *
 * 【重要】TcMainTxクラスの状態機械・フレーム構築・タイミングロジックは
 * NanoEvery_TCEmulator_V2_6.ino から1行も変更していない。変更したのは
 * private: の writeLogical() 1箇所のみ(GPIOアクセスの局所化という
 * 設計意図どおりのplatform adaptation)。
 *
 * 正式PHY(仕様書 ESP32S3_TcMainTx単体スケッチ_仕様案.md §2/§3で確定):
 *   ESP32 GPIO9 → JP5(閉) → R12(220Ω) → Q2 gate
 *   Q2 drain → R7(100Ω) → JP3(2-3) → Nano D11(MOSI)
 *   Q2 source = GND (標準的なローサイドNMOSスイッチ)
 *   D11のHIGHはNano自身の内蔵プルアップ(RX_USE_INTERNAL_PULLUP=true)で形成
 *   JP6 = まず開(ESP32自身の5Vプルアップは今回使わない)
 *
 * 極性(既存 ESP32_TC_Bridge_RTOS_Phase3_Skeleton.ino の
 * tcWriteLogicalBit()/TC_OC_ACTIVE_LOW=true と同一):
 *   logical HIGH → GPIO LOW  → Q2 OFF → D11 HIGH(Nano内蔵プルアップ)
 *   logical LOW  → GPIO HIGH → Q2 ON  → D11 LOW(R7経由でGNDへ)
 *
 * スコープ外(このスケッチには含めない):
 *   - ESP32_TC_Bridge_RTOS_Phase3_Skeleton.ino のRTOS構成(Core0/Core1、
 *     FreeRTOSタスク)、Pi UART通信、OLED表示、旧tcSend9BitByte()仮スタブ
 *   - RX機能(ESP32側では受信しない。Nano V2.6 RXへ送るだけの一方向試験)
 *
 * 配線前チェックリスト(実施すること):
 *   1. JP5 = 閉、JP3 = 2-3、JP6 = 開 に設定
 *   2. Nano側: NanoEvery_TCEmulator_V2_6.ino を MODE_RX_NONBLOCKING_TEST
 *      に切り替えて書き込む(RX_USE_INTERNAL_PULLUP=true のまま)
 *   3. 起動直後(idle、GPIO9=LOW固定)のD11をオシロで確認:
 *        GPIO9=LOW(Q2 OFF) → D11 ≈ HIGH
 *      本スケッチはGPIO9=HIGH側を能動的に作る機能を持たないため、
 *      静的に確認できるのはidle状態のみ。GPIO9=HIGH(Q2 ON)側の
 *      極性確認は、RESET送信開始後にロジアナ2ch観測(下記4)で行う。
 *   4. ロジアナ2ch同時観測(推奨): CH0=ESP32 GPIO9、CH1=Nano D11
 */

#include <Arduino.h>

// ============================================================
// 定数
// ============================================================
static const uint32_t SLOT_US = 3300;

// GPIO9 (= 現行Phase3スケルトンのPIN_TC_TX_TRIG=D10と同一ピン)。
// 既存基板のJP5→R12→Q2ゲート経路をそのまま利用する。
// 配線前提: JP5=閉、JP3=2-3、JP6=開。極性はtcWriteLogicalBit()と同じ。
static const uint8_t TX_PIN = 9;

// Nano側テストと同じ間隔にして比較しやすくする
static const uint32_t SEND_INTERVAL_MS = 1000;

// ============================================================
// TcMainTx: Main Controller ASM(zhukongban.asm)忠実再現・
// 移植可能な非ブロッキングTXコア(Phase1.5 Step2試験治具)
// ============================================================
//
// NanoEvery_TCEmulator_V2_6.ino から移植。状態機械・フレーム構築・
// タイミングロジックは無変更。private: の writeLogical() のみ、
// 既存基板のGPIO9→Q2→R7→JP3→Nano D11経路(Q2による論理反転を
// 吸収する)に合わせて変更している。それ以外は1行も変更していない。
//
// 出力する論理レベルはTC106が受信すべき論理値(V2.6 RX engineの
// readRxLogical()と対称):
//   idle: HIGH / START(slot0): LOW / bit=1: HIGH、bit=0: LOW(LSB first)
//   command/dataマーカー(slot9): 確定byteならHIGH、データbyteならLOW
//   guard: HIGH
//
// ASM確認済み事項:
//   ・RESET/SENDの張力byteは送信前に1bit左シフトされる
//     (send_reset: txbyte_cntr==3/9でbcf STATUS,C; rlf rreg_1、
//      send_send: byte0/byte2のtens_ch1/ch3読み出し直後で同様)。
//     API引数tensはMain内部値(シフト前)と定義し、内部でtxTens=tens<<1
//     を送信する。
//   ・RESET checksumは (len0+len1+len2+txTens)&0x7F
//     (原ASMのchecksum計算がtens_ch1を2回加算しているのと数学的に等価)。

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

    // ★ここだけがNano版からの変更点(platform adaptation)★
    //
    // Nano版: digitalWrite(_pin, high ? HIGH : LOW);
    //   (直結、反転なし。Nano自身がTC106の論理値をそのまま出力していた)
    //
    // ESP32-S3版: 既存基板のQ2(NMOS)による論理反転を吸収する。
    //   既存 ESP32_TC_Bridge_RTOS_Phase3_Skeleton.ino の
    //   tcWriteLogicalBit()(TC_OC_ACTIVE_LOW=true)と同一極性:
    //     logical HIGH → GPIO LOW  → Q2 OFF → D11 HIGH(Nano内蔵プルアップ)
    //     logical LOW  → GPIO HIGH → Q2 ON  → D11 LOW(R7経由でGNDへ)
    //   状態機械・フレーム構築・タイミングロジックには一切手を入れて
    //   いない。ここだけがESP32-S3の電気インターフェースへの適応。
    void writeLogical(bool high) {
        digitalWrite(_pin, high ? LOW : HIGH);
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
static uint8_t  patternIndex = 0;
static uint32_t lastSendMs = 0;

// ============================================================
// setup() / loop()
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=== ESP32-S3 TcMainTx Standalone Test ===");
    Serial.print("TX_PIN: GPIO"); Serial.println(TX_PIN);
    Serial.print("SLOT_US: "); Serial.println(SLOT_US);
    Serial.println("Nano V2.6 RXへ既存Q2経路で送信します。");
    Serial.println("JP5=CLOSE / JP3=2-3 / JP6=OPEN");
    Serial.println("起動直後のidleをオシロ確認:");
    Serial.println("  GPIO9=LOW(Q2 OFF) -> D11 approx HIGH");
    Serial.println("送信開始後はGPIO9/D11を2ch観測しQ2反転を確認");
    tcMainTx.begin(TX_PIN);
}

void loop() {
    tcMainTx.poll();

    // 完了ログの確認を、次コマンド開始判定より先に行う
    // (Nano版MODE_RX_LOOPBACK_TESTで確定した順序を踏襲)。
    bool aborted = false;
    if (tcMainTx.consumeDoneFlag(&aborted)) {
        if (aborted) {
            Serial.print("[TX-CORE] ABORTED (overrun) count=");
            Serial.println(tcMainTx.overrunCount());
        } else {
            Serial.println("[TX-CORE] frame done");
        }
    }

    if (!tcMainTx.busy() && (millis() - lastSendMs >= SEND_INTERVAL_MS)) {
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
}
