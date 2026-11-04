/**
 * TC106_RawFrameTest.ino  (試験専用スケッチ。ブリッジ本体のファームではない)
 *
 * 目的: TC106のFWを変えずに、異常フレーム(チェックサム不正のRESET、
 *       F3のみのSEND など)を ESP32 から送って、TC106の rxerror 動作
 *       (LEDの赤緑交互点滅)を確認する。
 *
 * 元: ESP32S3_TcMainTx_Standalone.ino の TcMainTx クラス(ゴールド資産)。
 *     クラス本体は無変更。追加したのは public メソッド sendRaw() のみ。
 *     ブリッジ本体(TCBridge_EvidenceV3.ino)には一切手を入れない。
 *
 * 配線・状態(従来の送信試験と同じ):
 *   GPIO9 -> JP5(閉) -> R12 -> Q2(ゲート)、Q2ドレイン -> R7 -> JP3 -> TC106
 *   JP6 = 開(ESP32の5Vプルアップは使わない)
 *   ESP32(USB)を先に給電してから、バスをつなぐ。+24Vは接続しない。
 *
 * 使い方: シリアルモニター 115200bps、改行付きで1行ずつ送る。
 *   プリセット名 または 16進のバイト列(最大12個)を入力。
 *   80h以上=コマンドbyte、未満=データbyte。
 *   F1 / F9 を含む行(RESET系)は、続けて yes を入力した時だけ送信。
 *
 * 異常フレームを送ると、TC106は rxerror(flag2=7)になる。
 * 戻すには TC106 の電源を入れ直す(次の試験の前に毎回)。
 */

#include <Arduino.h>

static const uint32_t SLOT_US = 3300;
static const uint8_t  TX_PIN  = 9;   // GPIO9 (Standaloneと同じ)
static const uint32_t TX_SLOT_OVERRUN_US_TCMAINTX = 1000;

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

    // ---- 試験専用の追加(クラス本体の他の部分は無変更) -------------
    // 任意のバイト列をそのまま送る。value>=0x80 はコマンドbyte(slot9=HIGH)、
    // それ以外はデータbyte(slot9=LOW)。チェックサム等は一切補正しない。
    // 1byteのコマンドだけは正規のSENS.ADJと同じ11スロット、それ以外は13スロット。
    bool sendRaw(const uint8_t* vals, uint8_t n) {
        if (busy()) return false;
        if (n < 1 || n > 12) return false;
        for (uint8_t i = 0; i < n; i++) {
            _frame.byte[i] = { vals[i], (vals[i] >= 0x80) };
        }
        _frame.byteCount    = n;
        _frame.slotsPerByte = (n == 1 && vals[0] >= 0x80) ? 11 : 13;
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

struct Preset { const char* name; const char* bytes; const char* note; };
static const Preset PRESETS[] = {
  {"reset_ok",       "00 00 0A 0A 14 F1 00 00 0A 0A 14 F9", "正常RESET(これまでの記録と同じ。EEPROM書込みあり)"},
  {"reset_bad",      "00 00 0A 0A 15 F1 00 00 0A 0A 15 F9", "チェックサム不正RESET(前半・後半とも) 期待: rxerror"},
  {"reset_bad_first","00 00 0A 0A 15 F1 00 00 0A 0A 14 F9", "前半だけ不正 期待: 前半rxerror。後半は処理される可能性(要観察)"},
  {"adj",            "F2",                                   "SENS.ADJ"},
  {"send_ok",        "14 F3 00 FB",                          "正常SEND(張力バイト0x14。RESETの0x0Aと違う値)"},
  {"send_f3only",    "F3",                                   "F3のみ(データなし) 期待: rxerror"},
  {"send_3data",     "01 02 03 F3",                          "データ3個+F3 期待: rxerror"},
  {"send_nofb",      "14 F3 FB",                             "データ,F3,FB(2個目のデータ欠落) 期待: エラーなし(cnt=1が残る)"},
  {"data6",          "01 02 03 04 05 06",                    "データ6個 期待: 6個目でrxerror"},
};
static const int NPRESETS = sizeof(PRESETS) / sizeof(PRESETS[0]);

static String lineBuf;
static bool   pendingConfirm = false;
static uint8_t pendBytes[12];
static uint8_t pendN = 0;

static void printHelp() {
  Serial.println("--- プリセット(名前を入力) ---");
  for (int i = 0; i < NPRESETS; i++) {
    Serial.print("  "); Serial.print(PRESETS[i].name);
    Serial.print("  = "); Serial.print(PRESETS[i].bytes);
    Serial.print("   "); Serial.println(PRESETS[i].note);
  }
  Serial.println("--- または16進バイト列(例: 00 00 0A 0A 15 F1) ---");
  Serial.println("  80h以上=コマンドbyte。F1/F9を含む行はyesで確認。 help: この表示");
}

static void printBytes(const uint8_t* v, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    if (v[i] < 0x10) Serial.print('0');
    Serial.print(v[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

static bool parseHexLine(const String& s, uint8_t* out, uint8_t* n) {
  *n = 0;
  int i = 0, len = s.length();
  while (i < len) {
    while (i < len && (s[i] == ' ' || s[i] == ',' || s[i] == '\t')) i++;
    if (i >= len) break;
    int j = i;
    while (j < len && s[j] != ' ' && s[j] != ',' && s[j] != '\t') j++;
    String tok = s.substring(i, j);
    char* endp = nullptr;
    long v = strtol(tok.c_str(), &endp, 16);
    if (*endp != '\0' || v < 0 || v > 0xFF) return false;
    if (*n >= 12) return false;
    out[(*n)++] = (uint8_t)v;
    i = j;
  }
  return *n >= 1;
}

static void doSend(const uint8_t* v, uint8_t n) {
  if (tcMainTx.sendRaw(v, n)) {
    Serial.print("[SEND] "); printBytes(v, n);
  } else {
    Serial.println("[SEND] 送信中のため無視しました。完了後にもう一度");
  }
}

static bool hasResetCmd(const uint8_t* v, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) if (v[i] == 0xF1 || v[i] == 0xF9) return true;
  return false;
}

static void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  String low = s; low.toLowerCase();

  if (pendingConfirm) {
    pendingConfirm = false;
    if (low == "yes") { doSend(pendBytes, pendN); }
    else { Serial.println("[CANCEL] 送信しませんでした"); }
    return;
  }
  if (low == "help" || low == "?") { printHelp(); return; }

  uint8_t v[12]; uint8_t n = 0;
  bool ok = false;
  for (int i = 0; i < NPRESETS; i++) {
    if (low == PRESETS[i].name) { ok = parseHexLine(String(PRESETS[i].bytes), v, &n); break; }
  }
  if (!ok) ok = parseHexLine(s, v, &n);
  if (!ok) { Serial.println("[ERR] 解釈できません。help で一覧"); return; }

  if (hasResetCmd(v, n)) {
    memcpy(pendBytes, v, n); pendN = n; pendingConfirm = true;
    Serial.print("[CONFIRM] RESET系: "); printBytes(v, n);
    Serial.println("  正しいチェックサムならTC106のEEPROMに書込みます。送るなら yes と入力");
    return;
  }
  doSend(v, n);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== TC106 RawFrameTest (試験専用) ===");
  Serial.print("TX_PIN: GPIO"); Serial.println(TX_PIN);
  Serial.println("JP5=閉 / JP6=開。TC106は毎回、電源を入れ直してから試験。");
  tcMainTx.begin(TX_PIN);
  printHelp();
}

void loop() {
  tcMainTx.poll();
  bool aborted = false;
  if (tcMainTx.consumeDoneFlag(&aborted)) {
    if (aborted) { Serial.print("[TX] ABORTED(overrun) count="); Serial.println(tcMainTx.overrunCount()); }
    else         { Serial.println("[TX] done"); }
  }
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineBuf.length() > 0) { String s = lineBuf; lineBuf = ""; handleLine(s); }
    } else if (lineBuf.length() < 120) {
      lineBuf += c;
    }
  }
}
