/*
  TC Emulator for Arduino Nano Every (初期版)

  - 通信速度は define で変更可能（デフォルト 9600bps）
  - 6バイト固定フレームを受信し、TCステータス6バイトを返信
  - コマンド (buf[5] の下位3bit):
      1: RESET   (長さ3桁 + 張力 + チェックサム)
      2: SENSADJ (センサゼロ調整)
      3: SEND    (張力のみ変更)
*/

#define TC_BAUD 9600       // ★ ここを 300 / 9600 などに変更可能
#define HOST_BAUD 115200   // PCデバッグ用

// シリアルの割り当て
#define HOST_SERIAL  Serial
#define TC_SERIAL    Serial1

// コマンドID（下位3bit）
#define CMD_RESET   1
#define CMD_SENSADJ 2
#define CMD_SEND    3

// 内部状態（PICコードを簡略化したもの）
uint8_t setlen[3]      = {0, 0, 0};  // 設定長さ3桁
uint8_t len_1[3]       = {0, 0, 0};  // 実際の長さ表示用（とりあえず同じ値）
uint8_t settens        = 0;          // 張力設定値 (0-127想定)
uint16_t tens_set      = 0;          // AD換算後の張力設定（簡易換算）
uint16_t tens_actual   = 0;          // 実測張力（疑似値）
uint16_t tens_temp     = 0;          // tens_actual からの派生値
uint16_t pwm_vol       = 0;          // 表示用PWMボリューム 0-999
uint8_t  txtens[2]     = {0, 0};     // tens_actual の10進2桁表示

// 受信バッファ
static const uint8_t FRAME_LEN = 6;
uint8_t rxBuf[FRAME_LEN] = {0};

// ----------------------------------------------------
// 疑似的な「張力補正」：settens から tens_set を決める簡易版
// 本物では張力レンジごとに Kp/Ki/Kd を切り替えているが、
// ここでは単純にスケーリングのみ。
// ----------------------------------------------------
void tension_compensate_dummy()
{
  // settens (0-127?) → tens_set (0-1000くらい) の適当な換算
  tens_set = (uint16_t)settens * 8;  // ざっくり 8倍
  if (tens_set > 1000) tens_set = 1000;
}

// ----------------------------------------------------
// 疑似的な「モータ制御」：tens_actual を tens_set に寄せながら
// pwm_vol を決める簡易版
// ----------------------------------------------------
void motor_control_dummy()
{
  // tens_actual を tens_set に徐々に近づける
  if (tens_actual < tens_set) {
    tens_actual++;
  } else if (tens_actual > tens_set) {
    tens_actual--;
  }

  // tens_temp は tens_actual をコピー（本物は AD 値からの処理）
  tens_temp = tens_actual;

  // pwm_vol は tens_set に比例させる（0-999）
  pwm_vol = (tens_set * 999UL) / 1000UL;
}

// ----------------------------------------------------
// ステータス6バイトの計算
// PICコードの tx_300_1() を簡易再現
// ----------------------------------------------------
void buildStatusFrame(uint8_t out[6])
{
  // PWMボリューム分解
  out[0] = pwm_vol % 100;    // 下2桁
  out[1] = pwm_vol / 100;    // 上桁

  // 張力センサ値の縮小
  out[2] = tens_temp / 8;    // 仮でそのまま縮小

  // tens_actual を 2桁の10進数に分割（0-99想定）
  uint16_t ta = tens_actual;
  if (ta > 99) ta = 99;
  txtens[0] = ta % 10;
  txtens[1] = ta / 10;

  out[3] = txtens[0];
  out[4] = txtens[1];

  // 終端
  out[5] = 0x7F;
}

// ----------------------------------------------------
// RESETコマンドの処理
// フレーム: [len0][len1][len2][settens][chk][CMD_RESET]
// chk = (len0 + len1 + len2 + settens) & 0x7F
// ----------------------------------------------------
void handleReset(const uint8_t buf[FRAME_LEN])
{
  uint8_t len0 = buf[0];
  uint8_t len1 = buf[1];
  uint8_t len2 = buf[2];
  uint8_t st   = buf[3];
  uint8_t chk  = buf[4];

  uint8_t c = (len0 + len1 + len2 + st) & 0x7F;

  if (c != chk) {
    HOST_SERIAL.println(F("[TC] RESET checksum error"));
    return; // エラーだが、ここでは単に無視
  }

  // 長さと張力を内部状態に反映
  setlen[0] = len0;
  setlen[1] = len1;
  setlen[2] = len2;

  len_1[0] = len0;
  len_1[1] = len1;
  len_1[2] = len2;

  settens = st;
  tension_compensate_dummy();

  HOST_SERIAL.print(F("[TC] RESET len="));
  HOST_SERIAL.print(len2);
  HOST_SERIAL.print(len1);
  HOST_SERIAL.print(len0);
  HOST_SERIAL.print(F("  settens="));
  HOST_SERIAL.println(settens);
}

// ----------------------------------------------------
// SENSADJ コマンドの処理（簡易）
// 本物はゼロ点調整＋EEPROMバックアップなどを行うが、
// ここでは tens_actual を 0 にリセットする程度。
// ----------------------------------------------------
void handleSensAdj(const uint8_t buf[FRAME_LEN])
{
  (void)buf;
  tens_actual = 0;
  tens_temp   = 0;

  HOST_SERIAL.println(F("[TC] SENSADJ (zero adjust)"));
}

// ----------------------------------------------------
// SEND コマンドの処理
// フレーム: [settens][(未使用)][(未使用)][(未使用)][(未使用)][CMD_SEND]
// とりあえず buf[0] だけを張力設定値として使う。
// ----------------------------------------------------
void handleSend(const uint8_t buf[FRAME_LEN])
{
  uint8_t st = buf[0];
  settens = st;
  tension_compensate_dummy();

  HOST_SERIAL.print(F("[TC] SEND settens="));
  HOST_SERIAL.println(settens);
}

// ----------------------------------------------------
// 受信フレーム処理
// ----------------------------------------------------
void processFrame(const uint8_t buf[FRAME_LEN])
{
  uint8_t cmd = buf[5] & 0x07; // 下位3bitで判定（PICの table_rx と同じ考え方）

  switch (cmd) {
    case CMD_RESET:
      handleReset(buf);
      break;
    case CMD_SENSADJ:
      handleSensAdj(buf);
      break;
    case CMD_SEND:
      handleSend(buf);
      break;
    default:
      HOST_SERIAL.print(F("[TC] Unknown cmd="));
      HOST_SERIAL.println(cmd);
      break;
  }

  // 内部状態を少し進めて、ステータスを返す
  motor_control_dummy();

  uint8_t status[6];
  buildStatusFrame(status);

  // TCライン側に返信
  for (uint8_t i = 0; i < 6; i++) {
    TC_SERIAL.write(status[i]);
  }

  // デバッグ用にも表示
  HOST_SERIAL.print(F("[TC] Status TX: "));
  for (uint8_t i = 0; i < 6; i++) {
    HOST_SERIAL.print(status[i]);
    HOST_SERIAL.print(' ');
  }
  HOST_SERIAL.println();
}

// ----------------------------------------------------
// セットアップ
// ----------------------------------------------------
void setup()
{
  HOST_SERIAL.begin(HOST_BAUD);
  while (!HOST_SERIAL) {
    ; // USBシリアル接続待ち（必要なら）
  }

  TC_SERIAL.begin(TC_BAUD);

  HOST_SERIAL.println(F("TC Emulator (Nano Every) start"));
  HOST_SERIAL.print(F("TC_BAUD = "));
  HOST_SERIAL.println(TC_BAUD);
}

// ----------------------------------------------------
// メインループ
// - TCライン（Serial1）から 6バイトたまったら1フレーム処理
// ----------------------------------------------------
void loop()
{
  // TCラインからの受信をポーリング
  static uint8_t idx = 0;

  while (TC_SERIAL.available() > 0) {
    int b = TC_SERIAL.read();
    if (b < 0) break;

    rxBuf[idx++] = (uint8_t)b;

    if (idx >= FRAME_LEN) {
      // 6バイトそろったので処理
      processFrame(rxBuf);
      idx = 0;
    }
  }

  // 必要ならここで他の処理（タイマ更新など）も入れられる
}
