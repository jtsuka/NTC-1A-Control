/*
  ESP32-S3 TC Bridge RTOS Phase3 Skeleton
  --------------------------------------
  V0.9からOLED/I2Cを完全撤去したRTOS土台。

  方針:
    Core0 : TC106側 300bps相当 BitBang/OC 物理層専用
    Core1 : Raspberry Pi側 9600bps UART / command起点parser / ログ
    Queue : Core間のTcMessage受け渡し
    OLED  : 使用しない。デバッグは USB Serial + Pi側ログ + LED。

  注意:
    これはPhase3用の「ひな型」です。
    TC106実機の17タイムスロット送受信は、tcSendFrame()/tcReceiveFrame()
    の中身を実測結果に合わせて差し替えてください。
*/

#include <Arduino.h>
#include <HardwareSerial.h>
#include "tc_packet_phase3.hpp"
#include "tc_message.hpp"

// ======================================================
// ピン設定
// ======================================================
// Pi側 UART。基板・配線に合わせて最終確認すること。
// 注: XIAO ESP32S3 のパッド名 (D0〜D10) を使用。
//     これは Seeed の variant ファイルが提供するマクロで、
//     XIAO_ESP32S3 ボード選択時のみ有効。
//     ボードパッケージ v2.0.8 以降が必要。
static constexpr int PIN_PI_RX = D1;  // Pi_Tx_MCU -> ESP32 RX  (= GPIO2)
static constexpr int PIN_PI_TX = D0;  // ESP32 TX -> Pi_Rx_MCU  (= GPIO1)

// TC106側。現行Snifar回路図の読み直しでは RX=D3(GPIO4), TX_TRIG=D10(GPIO9) が有力。
static constexpr int PIN_TC_RX      = D3;   // TC_MCU_RX <- JP7 <- クランプ <- 分圧  (= GPIO4)
static constexpr int PIN_TC_TX_TRIG = D10;  // MCU_TX_TRIG -> JP5 -> R12 -> Q2 gate  (= GPIO9)

// 開発用LED。XIAO ESP32S3のオンボードLEDは環境で異なるため必要に応じて変更。
static constexpr int PIN_STATUS_LED = LED_BUILTIN;

// ======================================================
// 通信パラメータ
// ======================================================
static constexpr uint32_t PI_BAUD = 9600;
static constexpr uint32_t TC_BAUD = 300;
static constexpr uint32_t BIT_US  = 1000000UL / TC_BAUD;

// TC側はOCドライバ想定。
// Q2 gate HIGH -> Q2 ON -> bus LOW、Q2 gate LOW -> bus released。
static constexpr bool TC_OC_ACTIVE_LOW = true;

// ======================================================
// Phase1.1: 17スロットBitBangプロトコル定数
// ======================================================
// Nano Every側(NanoEvery_TCEmulator_V2_2.ino)と同じ値。
// slot0=HIGHプリアンブル、slot1-7=7bitデータ(LSB first, bit0->HIGH, bit1->LOW)、
// slot8-16=LOW footer(9スロット)。
static constexpr uint32_t P11_SLOT_US       = 3300;
static constexpr uint8_t  P11_DATA_SLOTS    = 7;
static constexpr uint8_t  P11_SLOTS_PER_BYTE = 17;
static constexpr uint8_t  P11_FRAME_BYTES   = 6;   // 6バイト/フレーム
static constexpr uint32_t P11_BYTE_US       = P11_SLOT_US * P11_SLOTS_PER_BYTE; // 56100us

// footer(9スロット=29700us)と判定するための下限しきい値。
// データ部だけで作れる最長連続LOWは7スロット(=全data bit=1, 23100us)なので、
// その間(26000us=26ms)に設定すれば両者を確実に区別できる。
static constexpr uint32_t P11_FOOTER_MIN_US = 26000;

// 1フレーム分のエッジ(信号変化)を記録するバッファ。
// 1バイトあたり最大17エッジ×6バイト分の余裕を持たせる。
static constexpr uint16_t P11_EDGE_BUF_SIZE = 300;

static volatile uint32_t p11EdgeTimes[P11_EDGE_BUF_SIZE];
static volatile uint8_t  p11EdgeLevels[P11_EDGE_BUF_SIZE]; // そのエッジの後に確定するレベル
static volatile uint16_t p11EdgeCount = 0;

// GPIO4(PIN_TC_RX)のレベルが変化するたびに呼ばれる。
// 記録するだけで、Serial等の重い処理は一切行わない(ISR内厳禁のため)。
static void IRAM_ATTR p11TcRxIsr() {
  if (p11EdgeCount < P11_EDGE_BUF_SIZE) {
    p11EdgeTimes[p11EdgeCount]  = micros();
    p11EdgeLevels[p11EdgeCount] = (uint8_t)digitalRead(PIN_TC_RX);
    p11EdgeCount++;
  }
}

// Queue設定
static constexpr uint8_t QUEUE_DEPTH_PI_TO_TC = 8;
static constexpr uint8_t QUEUE_DEPTH_TC_TO_PI = 8;

// TC応答待ちタイムアウト。実機確認後に調整。
static constexpr uint32_t TC_RESPONSE_TIMEOUT_MS = 150;

HardwareSerial SerialPi(1);

static QueueHandle_t qPiToTc = nullptr;
static QueueHandle_t qTcToPi = nullptr;

// TCバス送信を複数箇所から同時に叩かないための保険。
// 現時点ではTCタスクのみが使うが、将来拡張用に残す。
static SemaphoreHandle_t tcBusMutex = nullptr;

// ======================================================
// ユーティリティ
// ======================================================
static void debugDumpMessage(const char* tag, const tc::TcMessage& m) {
  Serial.print(tag);
  Serial.print(" len=");
  Serial.print(m.len);
  Serial.print(" flags=0x");
  Serial.print(m.flags, HEX);
  Serial.print(" data=");
  m.dumpTo(Serial);
  Serial.println();
}

static void setStatusLed(bool on) {
#ifdef LED_BUILTIN
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
#endif
}

// ======================================================
// TcMainTx: Main Controller ASM(zhukongban.asm)忠実再現・
// 非ブロッキングTXコア(Standalone実機PASS済み、無変更のまま統合)
// ======================================================
//
// 出典: ESP32S3_TcMainTx_Standalone.ino(Step3A実機PASS)。
// クラス本体は1行も変更していない(golden asset)。
// GPIO9→Q2→R7→JP3→Nano D11経路、TC_OC_ACTIVE_LOW=trueと同一極性。
//
// ASM確認済み事項:
//   ・RESET/SENDの張力byteは送信前に1bit左シフトされる。
//     API引数tensはMain内部値(シフト前)と定義し、内部でtxTens=tens<<1
//     を送信する。
//   ・RESET checksumは (len0+len1+len2+txTens)&0x7F。

// TcMainTx(Standalone)が参照する定数。Standalone本体と同一名・同一値。
// Skeleton側のP11_SLOT_US(=3300、MODE1/2/4用の別シンボル)とは独立して
// 定義する(TcMainTxクラス本体を無変更のまま統合するため)。
static const uint32_t SLOT_US = 3300;

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

// ======================================================
// Phase1.1: 17スロット受信の実装
// ======================================================
//
// 課題: アイドル時(HIGH)とslot0プリアンブル(HIGH)が同一レベルなので、
// 「バイト1がいつ始まったか」はレベルの変化だけでは判定できない。
//
// 解決策:
//  1. GPIO4の変化(エッジ)を割り込みで全部記録しておく
//  2. 記録の中から「9スロット分(29700us)続く連続LOW」= footerを探す
//     (データ部だけで作れる最長連続LOWは7スロット=23100usなので、
//      しきい値26000usで確実に区別できる)
//  3. footerの終わり(LOWからHIGHに戻る瞬間)は「次のバイトのslot0開始」
//     と確定的に分かる
//  4. byte1だけはfooterが手前に無い(idleから直接続く)ので、
//     byte2の開始時刻から1バイト分(56100us)引いて逆算する
//  5. 各バイトの開始時刻が分かれば、あとは各データスロットの中央を
//     サンプリングするだけでビットを復元できる
//
// 1回の呼び出しで6バイト(1フレーム)を復元する。
static bool p11TryReceiveFrame(uint8_t out[P11_FRAME_BYTES], uint32_t timeoutMs) {
  const uint32_t giveUpAtMs = millis() + timeoutMs;

  // 記録バッファをリセットしてから待つ
  noInterrupts();
  p11EdgeCount = 0;
  interrupts();

  // 最初のエッジ(何か信号が動き出した)を待つ
  while (true) {
    if ((int32_t)(millis() - giveUpAtMs) > 0) return false;
    noInterrupts();
    uint16_t n = p11EdgeCount;
    interrupts();
    if (n > 0) break;
    vTaskDelay(1);
  }

  // エッジが増えなくなる(=送信が一段落した)まで待つ
  const uint32_t QUIET_MS = 50;
  uint16_t lastCount = 0;
  uint32_t lastChangeAtMs = millis();
  while (true) {
    if ((int32_t)(millis() - giveUpAtMs) > 0) return false;

    noInterrupts();
    uint16_t n = p11EdgeCount;
    interrupts();

    if (n != lastCount) {
      lastCount = n;
      lastChangeAtMs = millis();
    } else if (millis() - lastChangeAtMs > QUIET_MS) {
      break; // 一連のバースト(1フレーム分)が終わったとみなす
    }
    vTaskDelay(1);
  }

  // スナップショットをローカルにコピー(volatileのままだと扱いにくいため)
  static uint32_t times[P11_EDGE_BUF_SIZE];
  static uint8_t  levels[P11_EDGE_BUF_SIZE];
  uint16_t n;
  noInterrupts();
  n = p11EdgeCount;
  for (uint16_t i = 0; i < n; i++) {
    times[i]  = p11EdgeTimes[i];
    levels[i] = p11EdgeLevels[i];
  }
  p11EdgeCount = 0; // 次回に備えてクリア
  interrupts();

  if (n < 2) return false;

  // footer(連続LOWが26000us以上)の「終わりの時刻」を集める。
  // これがbyte2〜byte6それぞれのslot0開始時刻になる。
  uint32_t footerEnd[P11_FRAME_BYTES];
  uint8_t footerCount = 0;
  for (uint16_t i = 0; i + 1 < n && footerCount < P11_FRAME_BYTES; i++) {
    if (levels[i] == LOW) {
      uint32_t dur = times[i + 1] - times[i];
      if (dur >= P11_FOOTER_MIN_US) {
        footerEnd[footerCount++] = times[i + 1];
      }
    }
  }

  // byte1<->2, 2<->3, 3<->4, 4<->5, 5<->6 の5個の境界が最低限必要。
  if (footerCount < P11_FRAME_BYTES - 1) {
    return false; // フッターを十分見つけられなかった(ノイズ/未接続など)
  }

  uint32_t byteT0[P11_FRAME_BYTES];
  byteT0[0] = footerEnd[0] - P11_BYTE_US; // byte1は逆算
  for (uint8_t b = 1; b < P11_FRAME_BYTES; b++) {
    byteT0[b] = footerEnd[b - 1];
  }

  // 指定時刻における信号レベルを、記録したエッジ列から求める
  auto levelAt = [&](uint32_t t) -> int {
    if (n == 0 || t < times[0]) return HIGH; // 最初のエッジより前はidle(HIGH)とみなす
    int lvl = HIGH;
    for (uint16_t i = 0; i < n; i++) {
      if (times[i] <= t) {
        lvl = levels[i];
      } else {
        break;
      }
    }
    return lvl;
  };

  // 各バイトのデータスロット(1〜7)の中央をサンプリングしてビット復元
  // bit=0->HIGH, bit=1->LOW, LSB first
  for (uint8_t b = 0; b < P11_FRAME_BYTES; b++) {
    uint8_t value = 0;
    for (uint8_t k = 1; k <= P11_DATA_SLOTS; k++) {
      uint32_t target = byteT0[b] + (uint32_t)k * P11_SLOT_US + (P11_SLOT_US / 2);
      if (levelAt(target) == LOW) {
        value |= (uint8_t)(1u << (k - 1));
      }
    }
    out[b] = value;
  }

  return true;
}

// ======================================================
// Phase1.2: 17スロット送信の実装
// ======================================================
//
// 受信側(p11TryReceiveFrame)と対になる送信ロジック。
// Q2はオープンドレイン駆動(TC_OC_ACTIVE_LOW=true)なので、
// bit=0(HIGH) -> Q2 gate LOW  -> Q2 OFF -> バス解放(プルアップでHIGHへ)
// bit=1(LOW)  -> Q2 gate HIGH -> Q2 ON  -> バスをLOWへ引き込む
// という極性になる(tcWriteLogicalBit()と同じ考え方)。
//
// delayMicroseconds()の累積ではなく、micros()の絶対時刻を基準に
// 各スロットの終わりまで待つ方式にして、ドリフト(誤差の蓄積)を防ぐ。

static void p11SetBusLogical(bool logicalHigh) {
  if (TC_OC_ACTIVE_LOW) {
    digitalWrite(PIN_TC_TX_TRIG, logicalHigh ? LOW : HIGH);
  } else {
    digitalWrite(PIN_TC_TX_TRIG, logicalHigh ? HIGH : LOW);
  }
}

// targetMicros に達するまでビジーウェイトする。
// スロット長(3300us)が十分長いので、busy-waitが最も正確。
// (Phase1.2/1.3-Bで実績のある元の実装。Watchdog発火の根本原因は
//  こちらではなく、呼び出し側の周期管理(catch-upループ)だったため、
//  ここは変更しない。詳細は taskTcBus 内 TC_TEST_MODE==4 のコメント参照。)
static void p11WaitUntilMicros(uint32_t targetMicros) {
  while ((int32_t)(micros() - targetMicros) < 0) {
    // busy wait
  }
}

// 1バイト(17スロット)を送信する。
// slot0=HIGHプリアンブル、slot1-7=7bitデータ(LSB first, bit0->HIGH, bit1->LOW)、
// slot8-16=LOW footer(9スロット、まとめて1回のウェイトでOK)。
static void p11SendByte17Slot(uint8_t value) {
  const uint32_t t0 = micros();

  // slot0: プリアンブル(HIGH)
  p11SetBusLogical(true);
  p11WaitUntilMicros(t0 + P11_SLOT_US);

  // slot1-7: データ7bit、LSB first
  for (uint8_t k = 1; k <= P11_DATA_SLOTS; k++) {
    const bool bitIsOne = (value >> (k - 1)) & 0x01;
    p11SetBusLogical(!bitIsOne); // bit=0->HIGH(true), bit=1->LOW(false)
    p11WaitUntilMicros(t0 + (uint32_t)(k + 1) * P11_SLOT_US);
  }

  // slot8-16: footer(LOW、9スロットまとめて待つ)
  p11SetBusLogical(false);
  p11WaitUntilMicros(t0 + (uint32_t)P11_SLOTS_PER_BYTE * P11_SLOT_US);
}

// 6バイト(1フレーム)を送信する。送信後はバスをidle(HIGH/解放)に戻す。
static void p11SendFrame17Slot(const uint8_t data[P11_FRAME_BYTES]) {
  for (uint8_t i = 0; i < P11_FRAME_BYTES; i++) {
    p11SendByte17Slot(data[i]);
  }
  p11SetBusLogical(true); // idle解放
}


// Phase3差し替えポイント(コマンド/応答モデル用)。
// Phase1.1/1.2ではそれぞれ p11TryReceiveFrame()/p11SendFrame17Slot() を
// taskTcBus から直接使うため、この関数は呼ばれない(下の TC_TEST_MODE 参照)。
// Phase3再開時に、ここへ17スロット応答受信ロジックを合わせ込む。
//
// 注意: tc::TcMessage/tc::MsgType 経由のラップは tc_message.hpp の
// 正確なAPI(MsgTypeの列挙子名など)をこちらで確認できていないため、
// ここでは未実装のままにしてある(Phase3再開時、tc_message.hppの
// 中身と突き合わせて実装すること)。
static bool tcReceiveFrame(tc::TcMessage& out, uint32_t timeoutMs) {
  (void)out;
  (void)timeoutMs;
  return false; // Phase3再開時に実装
}

// ======================================================
// Core1: Raspberry Pi側タスク
// ======================================================
static void taskPiUart(void* pv) {
  // Stage2: command起点parser(WAIT_CMD/COLLECTING)。
  // 旧PacketFactory::tryParse()の{12,8,6}sliding-window方式(ring/head/lastSig)を廃止し、
  // raw command byteでexpected lengthを一意に決定する状態機械へ置換した。
  // command判定は必ずbuf[0]直読み。tc::Packet::cmd()(&0x07マスク)は使用しない
  // (0x11 SAFE OFFが0x01 RESETへ衝突するため)。
  enum PiParserState : uint8_t { WAIT_CMD, COLLECTING };
  PiParserState piState = WAIT_CMD;
  uint8_t piBuf[tc::PI_MAX]{};
  uint8_t piCollected = 0;
  uint8_t piExpectedLen = 0;

  Serial.println("[PiTask] start on Core1");

  while (true) {
    // Pi -> ESP32
    while (SerialPi.available()) {
      const uint8_t b = static_cast<uint8_t>(SerialPi.read());

      if (piState == WAIT_CMD) {
        uint8_t expLen = 0;
        switch (b) {
          case 0x01: expLen = tc::PI_LEN_RESET; break; // RESET     12byte
          case 0x02: expLen = tc::PI_LEN_SEND;  break; // SEND       6byte
          case 0x03: expLen = tc::PI_LEN_SENS;  break; // SENS.ADJ   8byte
          case 0x10: expLen = tc::PI_LEN_SEND;  break; // SAFE ON    6byte
          case 0x11: expLen = tc::PI_LEN_SEND;  break; // SAFE OFF   6byte
          default:   expLen = 0;                break; // 無効command
        }
        if (expLen == 0) {
          // 無効command byte。この1byteだけ破棄しWAIT_CMDのまま再同期。
          continue;
        }
        piBuf[0] = b;
        piCollected = 1;
        piExpectedLen = expLen;
        piState = COLLECTING;
        continue;
      }

      // COLLECTING: expectedLenまでは1byteずつ収集するのみで、
      // 途中でchecksum判定は一切行わない(早期確定の禁止)。
      piBuf[piCollected] = b;
      piCollected++;
      if (piCollected < piExpectedLen) {
        continue;
      }

      // expectedLen分そろった時点で初めてchecksum判定。
      if (tc::checksum7(piBuf, piExpectedLen - 1) == piBuf[piExpectedLen - 1]) {
        tc::Packet p;
        p.len = piExpectedLen;
        memcpy(p.buf, piBuf, piExpectedLen);

        tc::TcMessage msg = tc::TcMessage::fromPacket(
          p, tc::MsgSource::Pi, tc::MsgType::Command
        );

        if (xQueueSend(qPiToTc, &msg, 0) != pdTRUE) {
          Serial.println("[PiTask] qPiToTc full, dropping packet");
        } else {
          debugDumpMessage("[PiTask RX]", msg);
        }
      }
      // checksum OK/NGいずれの場合も、bufを丸ごと破棄しWAIT_CMDへ復帰する
      // (部分再利用しない。旧ring方式のような非consumeスライド判定は行わない)。
      piState = WAIT_CMD;
      piCollected = 0;
      piExpectedLen = 0;
    }

    // TC -> Pi
    tc::TcMessage rx;
    while (xQueueReceive(qTcToPi, &rx, 0) == pdTRUE) {
      debugDumpMessage("[PiTask TX]", rx);

      // Phase3ではPi側プロトコルを最終確定する。
      // 現時点では6/8/12byte応答をそのまま返す。
      if (rx.len > 0) {
        SerialPi.write(rx.data, rx.len);
        SerialPi.flush();
      }
    }

    vTaskDelay(1);
  }
}

// ======================================================
// Core0: TC106側タスク
// ======================================================
//
// TC_TEST_MODE で動作モードを切り替える:
//   1 = Phase1.1: 受信専用テスト(Nano Every -> ESP32、GPIO4を受信し続けてログ出力)
//   2 = Phase1.2: 送信専用テスト(ESP32 -> Nano Every、テストパターンを周期送信)
//   4 = Phase1.3.5-A: 時間窓方式の同時双方向テスト(下記参照)
//   0 = Phase3  : Piからのコマンド/応答モデル(元のロジック)
//
// Phase1.2実施時は、JP3=2-3・JP5=閉に設定し、Nano側は
// NanoEvery_TCEmulator_V2_2.ino の MODE_PERIODIC=0 に変更しておくこと。
// 受信確認はNano側のシリアルモニタで行う(D13で受信したバイト列を見る)。
#define TC_TEST_MODE 0

#if TC_TEST_MODE == 2 || TC_TEST_MODE == 4
// Nano側の4パターンと同じものを使う(相互比較しやすいように)。
static const uint8_t kP12TestFrames[][P11_FRAME_BYTES] = {
  {0x00, 0x64, 0x00, 0x64, 0x00, 0x7F},
  {0x11, 0x22, 0x33, 0x44, 0x55, 0x7F},
  {0x01, 0x00, 0x00, 0x1E, 0x00, 0x7F},
  {0x64, 0x00, 0x00, 0x32, 0x00, 0x7F},
};
static constexpr uint8_t kP12TestFrameCount =
  sizeof(kP12TestFrames) / sizeof(kP12TestFrames[0]);
#endif

#if TC_TEST_MODE == 4
// ============================================================
// Phase1.3.5-A: 時間窓方式の状態機械
// ============================================================
// 仕様書(Phase1.3.5_試験仕様書.md)のタイミング図に対応。
//
//   周期 n
//   0ms          336ms       450  500ms       836ms   950  1000ms
//   │ Nano TX ────►│          │    ESP32 TX ────►│      │
//   │ ESP32 RX窓   │          │    Nano RX窓     │      │
//   └──────────────┘          └──────────────────┘
//
// ESP32は「0〜450msをNano受信用の窓」「500msで自分のTX」という周期を繰り返す。
// 17スロット送受信ロジック(p11SendFrame17Slot/p11TryReceiveFrame)自体は
// Phase1.3/1.3-Bで実証済みのため変更しない。ここで新規に作るのは、
// その外側で「今は受信の番か、送信の番か」を判断する時間管理のみ。
static constexpr uint32_t P135_RX_WINDOW_MS = 450;   // 0〜450ms: 受信を試みる窓
static constexpr uint32_t P135_TX_START_MS  = 500;   // 500ms: 自分の送信を開始
static constexpr uint32_t P135_CYCLE_MS     = 1000;  // 周期
#endif

static void taskTcBus(void* pv) {
  Serial.println("[TcTask] start on Core0");

  // OCバスをidle/release状態へ
  digitalWrite(PIN_TC_TX_TRIG, LOW);

#if TC_TEST_MODE == 1
  Serial.println("[TcTask] Phase1.1 standalone listen mode (Pi queue is ignored)");

  uint8_t frame[P11_FRAME_BYTES];
  uint32_t rxSeq = 0;
  while (true) {
    // タイムアウトは長めに。Nano側の送信間隔(1000ms)より余裕を持たせる。
    if (p11TryReceiveFrame(frame, 3000)) {
      setStatusLed(true);
      rxSeq++;
      Serial.print('[');
      Serial.print(millis());
      Serial.print(" ms] [TcTask RX] #");
      Serial.print(rxSeq);
      Serial.print(" ");
      for (uint8_t i = 0; i < P11_FRAME_BYTES; i++) {
        if (frame[i] < 0x10) Serial.print('0');
        Serial.print(frame[i], HEX);
        Serial.print(' ');
      }
      Serial.println();
      setStatusLed(false);
    }
    // 受信できなくてもループを回し続け、次のフレームを待つ。
  }

#elif TC_TEST_MODE == 2
  Serial.println("[TcTask] Phase1.2 standalone send mode (Pi queue is ignored)");
  Serial.println("[TcTask] Check Nano Every's serial monitor for received bytes.");

  uint8_t idx = 0;
  uint32_t txSeq = 0;
  while (true) {
    const uint8_t* f = kP12TestFrames[idx];

    setStatusLed(true);
    p11SendFrame17Slot(f);
    setStatusLed(false);

    txSeq++;
    Serial.print('[');
    Serial.print(millis());
    Serial.print(" ms] [TcTask TX] #");
    Serial.print(txSeq);
    Serial.print(" ");
    for (uint8_t i = 0; i < P11_FRAME_BYTES; i++) {
      if (f[i] < 0x10) Serial.print('0');
      Serial.print(f[i], HEX);
      Serial.print(' ');
    }
    Serial.println();

    idx = (idx + 1) % kP12TestFrameCount;
    vTaskDelay(pdMS_TO_TICKS(1000)); // Nano側と同じく1000ms周期
  }

#elif TC_TEST_MODE == 4
  Serial.println("[TcTask] Phase1.3.5-A time-windowed simultaneous mode");
  Serial.println("[TcTask] RX window: 0-450ms, TX at 500ms, cycle=1000ms");

  uint8_t txIdx = 0;
  uint32_t txSeq = 0;
  uint32_t rxSeq = 0;
  uint32_t cycleStart = millis();

  while (true) {
    uint32_t elapsed = millis() - cycleStart;

    // --- 受信窓 (0〜450ms) ---
    if (elapsed < P135_RX_WINDOW_MS) {
      uint32_t remaining = P135_RX_WINDOW_MS - elapsed;
      uint8_t frame[P11_FRAME_BYTES];
      if (p11TryReceiveFrame(frame, remaining)) {
        rxSeq++;
        Serial.print('[');
        Serial.print(millis());
        Serial.print(" ms] [RX] #");
        Serial.print(rxSeq);
        Serial.print(" ");
        for (uint8_t i = 0; i < P11_FRAME_BYTES; i++) {
          if (frame[i] < 0x10) Serial.print('0');
          Serial.print(frame[i], HEX);
          Serial.print(' ');
        }
        Serial.println();
      }
      // 受信を試みても試みなくても、窓の終わりまでは待つ
      while (millis() - cycleStart < P135_RX_WINDOW_MS) {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }

    // --- 送信時刻(500ms)まで待機 ---
    while (millis() - cycleStart < P135_TX_START_MS) {
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    // --- 送信 ---
    const uint8_t* f = kP12TestFrames[txIdx];
    setStatusLed(true);
    txSeq++;
    Serial.print('[');
    Serial.print(millis());
    Serial.print(" ms] [TX] #");
    Serial.print(txSeq);
    Serial.print(" ");
    for (uint8_t i = 0; i < P11_FRAME_BYTES; i++) {
      if (f[i] < 0x10) Serial.print('0');
      Serial.print(f[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
    p11SendFrame17Slot(f);
    setStatusLed(false);
    txIdx = (txIdx + 1) % kP12TestFrameCount;

    // --- 次周期の開始(1000ms)まで待機 ---
    // 重要: cycleStart += P135_CYCLE_MS のように過去の周期を律儀に
    // 追いかけると、一度でも処理が1周期分以上遅れた場合、次周期以降も
    // millis()-cycleStart が常にP135_CYCLE_MS超過のままとなり、
    // 全ての待機がスキップされて送信だけが連続実行される
    // 「catch-upループ」に陥る(実機試験で確認済み: 336〜337ms間隔の
    // 連続送信としてログに現れ、これがTask Watchdog発火の根本原因だった)。
    // ここでは、遅れた周期は追いかけず、現在時刻を新たな周期の起点として
    // 使う(取りこぼした過去の周期は捨てて、次の未来の周期へ進む)。
    while (millis() - cycleStart < P135_CYCLE_MS) {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    cycleStart = millis();
  }

#else
  // ============================================================
  // Phase3(MODE0): TcMainTx正式統合(Stage 3)
  // ============================================================
  // Standalone実機PASS済みTcMainTxを、Core0所有の唯一のTC送信経路とする。
  // TcMainTxクラス本体・begin()呼び出し位置(この#else分岐冒頭のみ)は
  // Step3C_Skeleton正式統合_変更点一覧_v0.9.md §8確定通り。
  //
  // Stage3のスコープ:
  //   qPiToTc -> raw command判定(data[0]) -> TcMainTx -> Q2 -> Nano
  // SAFE/fault/recovery RESET/ACK/NAK/qTcToPi応答は今回実装しない。
  static TcMainTx tcMainTx;
  tcMainTx.begin(PIN_TC_TX_TRIG);

  // Stage4: SAFE状態。taskTcBus()の生存期間中(ループ外)で保持する。
  // Core0のみが読み書きし、Core1・共有atomicとは一切やり取りしない。
  bool localSafeEnabled = false;

  while (true) {
    // TcMainTxのpoll()はtiming-criticalなslot駆動状態機械のため、
    // busy中は delay/yield を一切挟まず高頻度で呼び続ける。
    tcMainTx.poll();

    bool aborted = false;
    if (tcMainTx.consumeDoneFlag(&aborted)) {
      // Stage3ではcompletionログのみ。SAFE/fault/recovery処理は行わない。
      if (aborted) {
        Serial.print("[TcTask] ABORTED (overrun) count=");
        Serial.println(tcMainTx.overrunCount());
      } else {
        Serial.println("[TcTask] frame done");
      }

      // frame終了直後はtiming-critical区間外のため、
      // ここでIdle0タスクへ実行機会を与える。
      vTaskDelay(1);
    }

    if (!tcMainTx.busy()) {
      tc::TcMessage cmd;

      // queueが空の間は、このxQueueReceive自体のblockingで
      // Idle0タスクへ実行機会が生まれる。
      if (xQueueReceive(qPiToTc, &cmd, pdMS_TO_TICKS(1)) == pdTRUE) {
        debugDumpMessage("[TcTask CMD]", cmd);

        // command判定は必ずraw command byte(data[0])を直読みする。
        // Packet::cmd()(&0x07マスク)は使用しない
        // (0x11 SAFE OFFが0x01 RESETへ衝突するため)。
        switch (cmd.data[0]) {
          case 0x01: { // RESET
            if (localSafeEnabled) {
              Serial.println("[TcTask] SAFE DROP cmd=0x01");
              break;
            }
            const uint32_t value = ((uint32_t)cmd.data[1] << 16)
                                  | ((uint32_t)cmd.data[2] << 8)
                                  |  (uint32_t)cmd.data[3];
            const uint8_t tens = cmd.data[4];
            const uint8_t len0 = (uint8_t)(value / 10000);
            const uint8_t len1 = (uint8_t)((value / 100) % 100);
            const uint8_t len2 = (uint8_t)(value % 100);
            if (!tcMainTx.sendReset(len0, len1, len2, tens)) {
              Serial.println("[TcTask] sendReset failed (busy)");
            }
            break;
          }
          case 0x02: { // SEND (CH1専用。CH3相当は既定の0)
            if (localSafeEnabled) {
              Serial.println("[TcTask] SAFE DROP cmd=0x02");
              break;
            }
            if (!tcMainTx.sendTension(cmd.data[1])) {
              Serial.println("[TcTask] sendTension failed (busy)");
            }
            break;
          }
          case 0x03: { // SENS.ADJ (BREAKはPhase3対象外、常にfalse)
            if (localSafeEnabled) {
              Serial.println("[TcTask] SAFE DROP cmd=0x03");
              break;
            }
            if (!tcMainTx.sendSensAdj(false)) {
              Serial.println("[TcTask] sendSensAdj failed (busy)");
            }
            break;
          }
          case 0x10: // SAFE ON
            localSafeEnabled = true;
            Serial.println("[TcTask] SAFE ON");
            break;
          case 0x11: // SAFE OFF
            localSafeEnabled = false;
            Serial.println("[TcTask] SAFE OFF");
            break;
          default:
            // Core1のparserで既に弾かれているはずだが、防御的に無視する。
            Serial.print("[TcTask] unrecognized command 0x");
            Serial.println(cmd.data[0], HEX);
            break;
        }
      }
    }
  }
#endif
}

// ======================================================
// setup / loop
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== ESP32-S3 TC Bridge RTOS Phase3 Skeleton v0.1 ===");
  Serial.println("OLED disabled. Debug via USB Serial / Pi log / LED.");
  Serial.println("Core0=TC BitBang/OC, Core1=Pi UART.");

  pinMode(PIN_STATUS_LED, OUTPUT);
  setStatusLed(false);

  pinMode(PIN_TC_TX_TRIG, OUTPUT);
  digitalWrite(PIN_TC_TX_TRIG, LOW);  // OC release
  pinMode(PIN_TC_RX, INPUT_PULLUP);

  // Phase1.1: 17スロット受信用のエッジキャプチャ割り込みを常時有効化
  attachInterrupt(digitalPinToInterrupt(PIN_TC_RX), p11TcRxIsr, CHANGE);

  SerialPi.begin(PI_BAUD, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);

  qPiToTc = xQueueCreate(QUEUE_DEPTH_PI_TO_TC, sizeof(tc::TcMessage));
  qTcToPi = xQueueCreate(QUEUE_DEPTH_TC_TO_PI, sizeof(tc::TcMessage));
  tcBusMutex = xSemaphoreCreateMutex();

  if (!qPiToTc || !qTcToPi || !tcBusMutex) {
    Serial.println("[FATAL] RTOS object creation failed");
    while (true) {
      setStatusLed(true);
      delay(100);
      setStatusLed(false);
      delay(100);
    }
  }

  // ESP32 ArduinoではCore1にArduino loopがいるため、スタックと優先度は控えめにする。
  // TC側はタイミング重視のためCore0へ固定。
  xTaskCreatePinnedToCore(taskTcBus,  "TcBus",  4096, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(taskPiUart, "PiUart", 4096, nullptr, 3, nullptr, 1);

  Serial.println("[setup] tasks created");
}
void loop() {
  // 何もしない。タスク側で処理する。
  vTaskDelay(portMAX_DELAY);
}
