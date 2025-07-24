// TC Repeater with BitBang Response Timeout - ESP32-S3 (修正版)
// ----------------------------------------
// - UART RX/TX (to Pi): GPIO44 / GPIO43
// - BitBang TX/RX (to TC): GPIO2 / GPIO3
// - Baud rate: UART = 9600bps, BitBang = 300bps
// - Wait up to 100ms for TC response, then reply to Pi
// - Safe with FreeRTOS & taskENTER_CRITICAL
// - 2025.07.23 5th Try Version
// ----------------------------------------

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define UART_TX_PIN        43   // Grove JP7(白 D6/TX)
#define UART_RX_PIN        44   // Grove JP7(黄 D7/RX)
#define BITBANG_TX_PIN     2    // Grove JP2(白 D1)
#define BITBANG_RX_PIN     3    // Grove JP2(黄 D2)

#define UART_BAUD_RATE     9600
#define BITBANG_DELAY_US   3340
#define RESPONSE_TIMEOUT_MS 200
#define MAX_PACKET_LEN     32
#define FIXED_PACKET_LEN   6
#define LED_PIN 21

// --- AE-LCNV-LVCH16T245 control pins ---
#define PIN_OE_A 5      // 1OE  (J5 白)   LOW = 出力有効
#define PIN_DIR_A 6     // 1DIR (J5 黄)   LOW = B→A (TC→ESP32)
#define PIN_OE_B 7      // 2OE  (J7 白)   LOW = 出力有効
#define PIN_DIR_B 8     // 2DIR (J7 黄)   HIGH = A→B (ESP32→TC)

#define START_OFFSET 1.94f
#define BYTE_GAP  1
// ---------------- tunable range ----------------
#define DELTA_MIN_US  (-300)   // ここから
#define DELTA_MAX_US  (300)   // ここまで自動スキャン
#define DELTA_STEP_US  25   // 微調ステップ

QueueHandle_t bitbangRxQueue;
portMUX_TYPE bitbangMux = portMUX_INITIALIZER_UNLOCKED;

// --- 追加：直前に送った 6 byte を保持するバッファ
static uint8_t lastSent[FIXED_PACKET_LEN] = {0};
static bool    lastValid = false;                // まだ何も送っていない状態

/* ---- 自動チューニング用 ---- */
//static uint32_t delta_now  = (DELTA_MIN_US + DELTA_MAX_US)/2;
static int32_t delta_now  = 0;
static bool     delta_fixed = false;
static uint16_t syncFail   = 0;
static uint16_t syncOK     = 0;
static uint32_t lastEvalMs = 0;

/* ★ start 判定結果を保持する */
static bool startOK = false;   // true = waitValidStart() success


void uartInit() {
  Serial1.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
}

/* 8bit を左右反転するユーティリティ */
static inline uint8_t rev8(uint8_t v)
{
  v = (v >> 4) | (v << 4);
  v = ((v & 0xCC) >> 2) | ((v & 0x33) << 2);
  v = ((v & 0xAA) >> 1) | ((v & 0x55) << 1);
  return v;
}

/* ========= ノイズ除去付きスタート検出 ========= */
static bool waitValidStart()
{
    const uint32_t debounce_us = 300;                  // ノイズ除去時間
    const uint32_t halfbit_us  = BITBANG_DELAY_US / 2; // 1670 µs (300 bps)

    /* 1. スタート Low を最大 30 ms 待つ ---------------------- */
    uint32_t t0 = micros();
    while (digitalRead(BITBANG_RX_PIN) == HIGH) {      // HIGH→LOW を待つ
        if (micros() - t0 > 30000) return false;       // タイムアウト
    }
    /* ここに来たとき RX は LOW (スタート候補) */

    /* 2. 300 µs 待って still LOW なら本物、HIGH ならグリッチ ---- */
    delayMicroseconds(debounce_us);
    if (digitalRead(BITBANG_RX_PIN) == LOW) return false; // ノイズだった

    /* 3. 半ビット＋Δ だけ進めてビット中央へ ------------------ */
    delayMicroseconds(halfbit_us + delta_now);

    return true;                                       // ★ 正常にスタート捕捉
}

int uartReceivePacket(uint8_t *buf) {
  int len = 0;
  unsigned long start = millis();
  while ((millis() - start) < 100 && len < MAX_PACKET_LEN) {
    if (Serial1.available()) {
      buf[len++] = Serial1.read();
    }
  }
  return len;
}

void uartSendPacket(const uint8_t *buf, int len) {
  Serial.print("[SEND Pi] ");
  Serial.println();
 for (int i = 0; i < len; i++) {
    Serial.printf("%02X ", buf[i]);
    Serial.println();
 }
  Serial1.write(buf, len);
  Serial1.flush();
  delay(1);
}

/* 送信 1byte */
void bitBangSendByte(uint8_t b) {
  /* ─── 送信開始前に必ず OUTPUT へ戻す！ ─── */
  pinMode(BITBANG_TX_PIN, OUTPUT);          // ← これが無いと Hi-Z のまま

   /* ---------- Drive start～stop bit ---------- */
  taskENTER_CRITICAL(&bitbangMux);
  digitalWrite(BITBANG_TX_PIN, LOW);   // Start LOW
  delayMicroseconds(BITBANG_DELAY_US);
  for (int i = 0; i < 8; i++) {
    digitalWrite(BITBANG_TX_PIN, (b >> i) & 1);   // データ反転しない
    delayMicroseconds(BITBANG_DELAY_US);
  }
  digitalWrite(BITBANG_TX_PIN, HIGH);    // Stop HIGH
  delayMicroseconds(BITBANG_DELAY_US);
  taskEXIT_CRITICAL(&bitbangMux);
  
  /* ---------- ここが肝心！TX を Hi-Z へ ---------- */
  pinMode(BITBANG_TX_PIN, INPUT);       // Tx ライン手放し
  /* 300 bps なので 3.3 ms 待つ間に RX が安定する */
}

void bitBangSendPacket(const uint8_t *buf, int len) {
  for (int i = 0; i < len; i++) {
    bitBangSendByte(rev8(buf[i]));
    delayMicroseconds(BITBANG_DELAY_US * BYTE_GAP);   // ←バイト間ギャップ 3 -> 2 ->1 へ
  }
}

int bitBangReceivePacket(uint8_t *buf, int maxLen)
{
  int byteCount = 0;

  while (byteCount < maxLen)
  {
    /* ---- スタート検出 ---- */
    uint32_t t0 = micros();               // ←★ ここで現在時刻を保存
     /* ---- スタート検出（デバウンス付き） ---- */
    startOK = waitValidStart();          // ← 成否を保存
    if (!startOK) return 0;

    /* ---- 8 bit 読み取り ---- */
    uint8_t b = 0;
    for (int i = 0; i < 8; i++) {
      b |= (digitalRead(BITBANG_RX_PIN) << i);
      delayMicroseconds(BITBANG_DELAY_US);
    }
    if (byteCount == 0 || byteCount == 5) {
//    ESP_EARLY_LOGI("BB", "b%d=%02X idle=%d",
//                   byteCount, b, digitalRead(BITBANG_RX_PIN));
    }
    /* Stop ビット (HIGH) は “捨て読み” のみに変更 */
    delayMicroseconds(BITBANG_DELAY_US);

    /* ---- 最初のバイトだけ簡易チェック ---- */
    if (byteCount == 0 && (b == 0x00 || b == 0xFF)) {
      Serial.println("[WARN] Invalid start byte (00/FF)");
      return 0;                    // グリッチか極性ズレ
    }

    buf[byteCount++] = b;
    if (byteCount >= FIXED_PACKET_LEN) break;
  }
  return byteCount;
}


void TaskBitBangReceive(void *pvParameters) {
  uint8_t rxBuf[MAX_PACKET_LEN];
  static uint32_t dbgCount[7]={0};
  int cnt=0;
  startOK=false;
  while (1) {
    int len = bitBangReceivePacket(rxBuf, MAX_PACKET_LEN);
    // for Debug
      /* ★ 統計カウントをここで更新 */
      if (startOK) syncOK++;
      else         syncFail++;

    if (len == FIXED_PACKET_LEN) {
      uint8_t* copyBuf = (uint8_t*)malloc(len);
      if (copyBuf != nullptr) {
        memcpy(copyBuf, rxBuf, len);
        if (xQueueSend(bitbangRxQueue, &copyBuf, pdMS_TO_TICKS(10)) != pdTRUE) {
          free(copyBuf);  // キューがいっぱい → メモリ解放
          copyBuf = nullptr;
        }
      }
    }

    // ---- Δ 自動チューニング ----------------------------------
    uint32_t now = millis();
    if (!delta_fixed && now - lastEvalMs > 250) {   // 250 ms ごとに評価
        uint16_t total = syncFail + syncOK;         // 試行回数
        if (total > 30) {                           // データ十分？
            uint32_t failPermil = (syncFail * 1000) / total;  // 0.1 % 単位
            ESP_EARLY_LOGI("AUTO",
                "delta=%ld  fail=%u.%u%%",
                (long)delta_now, failPermil / 10, failPermil % 10);

            if (failPermil < 150) {                 // 成功率 85 % 以上
                if (syncOK > 30) {                  // 30 回連続 OK → 確定
                    delta_fixed = true;
                } else if (delta_now > DELTA_MIN_US + DELTA_STEP_US) {
                    delta_now -= DELTA_STEP_US / 2; // さらに中央へ寄せる
                }
            } else {                                // 失敗率が高い
                if (delta_now < DELTA_MAX_US - DELTA_STEP_US) {
                    delta_now += DELTA_STEP_US;     // 範囲を広げる
                } else {
                    delta_fixed = true;             // 端まで来たら確定
                }
            }
            syncFail = syncOK = 0;                  // 統計リセット
        }
        lastEvalMs = now;
    }
  }
}

void TaskUartReceive(void *pvParameters)
{
  uint8_t buf[MAX_PACKET_LEN];

  while (1)
  {
    /* Pi → ESP32 から受信して buf[] に格納 */
    int len = uartReceivePacket(buf);

    /* ───────── 新規パケットが来た？ ───────── */
    if (len == FIXED_PACKET_LEN)
    {
      /* 過去と同じ内容ならスキップ（再送ループ防止） */
      if (lastValid && memcmp(buf, lastSent, FIXED_PACKET_LEN) == 0)
      {
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;                      // ★ 同じなので送らない
      }

      /* ---------- Bit-Bang で TC に送信 ---------- */
      Serial.println("[INFO] Pi -> TC へ送信開始");
      bitBangSendPacket(buf, FIXED_PACKET_LEN);

      /* 送れたので今回の内容を記録 */
      memcpy(lastSent, buf, FIXED_PACKET_LEN);
      lastValid = true;

      /* ---------- TC からの応答待ち ---------- */
      uint8_t *echoBuf = nullptr;
      if (xQueueReceive(bitbangRxQueue,
                        &echoBuf,
                        pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS))
          == pdTRUE)
      {

        /* LSB→MSB へビット反転して Pi へ返す */
        uint8_t txTmp[FIXED_PACKET_LEN];
        for (int i = 0; i < FIXED_PACKET_LEN; i++) {
            txTmp[i] = rev8(echoBuf[i]);   // ★ 反転
        }
        uartSendPacket(txTmp, FIXED_PACKET_LEN);      // Pi へ返信

        free(echoBuf);
        echoBuf = nullptr;      /*   次ループで “残りカス” を誤判定しない */
      }
      else
      {
//        Serial.println("[WARN] TC応答なし (timeout)");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void dumpCtrl(const char* tag){
  Serial.printf("[%s] DIRA=%d  OEA=%d  DIRB=%d  OEB=%d\n",
      tag,
      digitalRead(PIN_DIR_A), digitalRead(PIN_OE_A),
      digitalRead(PIN_DIR_B), digitalRead(PIN_OE_B));
}

void setup() {
  Serial.begin(115200);
  uartInit();

  // for Look()
  pinMode(LED_PIN, OUTPUT);

  pinMode(BITBANG_RX_PIN, INPUT); // 高インピーダンス
  Serial.printf("[DEBUG] RXB idle level = %d\n", digitalRead(BITBANG_RX_PIN));

  // for AE-LLCNV-LVC8T245
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);
  pinMode(PIN_OE_A, OUTPUT);
  pinMode(PIN_OE_B, OUTPUT);

  // 前出力をHi-Z
  digitalWrite(PIN_OE_A, HIGH);   // 出力無効（OE=H）
  digitalWrite(PIN_OE_B, HIGH);   // 出力無効（OE=H）

  // 方向を確定
  digitalWrite(PIN_DIR_A, LOW);   // B→A (TC -> ESP32)
  digitalWrite(PIN_DIR_B, HIGH);  // A→B (ESP32 -> TC)
  delay(50);  // DIR状態反映待ち

  // 出力を有効化
  digitalWrite(PIN_OE_A, LOW);   // 出力有効（OE_A=L）
  digitalWrite(PIN_OE_B, LOW);   // 出力有効（OE_B=L）

  delay(100);  // 状態反映待ち

  // Step 2: 読み戻して確認
//  pinMode(PIN_OE, INPUT);  // 強制的に入力に戻して電圧状態確認
  int OElevel1 = digitalRead(PIN_OE_A);
  int OElevel2 = digitalRead(PIN_OE_B);
  int level2 = digitalRead(PIN_DIR_A);
  int level3 = digitalRead(PIN_DIR_B);

  Serial.printf("[診断] OE GPIO%d の状態: %s\n", PIN_OE_A,
                OElevel1 == LOW ? "LOW" : "HIGH");
  Serial.printf("[診断] GPIO%d の状態: %s\n", PIN_DIR_A,
                level2 == LOW ? "LOW" : "HIGH");
  Serial.printf("[診断] OE GPIO%d の状態: %s\n", PIN_OE_B,
                OElevel2 == LOW ? "LOW" : "HIGH");
  Serial.printf("[診断] GPIO%d の状態: %s\n", PIN_DIR_B,
                level3 == LOW ? "LOW" : "HIGH");


  // Bitbang
  pinMode(BITBANG_TX_PIN, OUTPUT);
  digitalWrite(BITBANG_TX_PIN, HIGH);  // idle HIGH
  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);  // ★プルアップを戻す
  gpio_set_pull_mode((gpio_num_t)BITBANG_RX_PIN,
                   GPIO_PULLUP_ONLY);   // ★強制 47k → 10k 相当へ

//  pinMode(BITBANG_RX_PIN, INPUT);      // ← PULLUPを削除

  bitbangRxQueue = xQueueCreate(4, sizeof(uint8_t*));
  xTaskCreatePinnedToCore(TaskBitBangReceive, "BitBangRX", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskUartReceive, "UartRX", 4096, NULL, 1, NULL, 1);

  Serial.println("[START] TC Repeater Ready.");
  Serial.print("[DEBUG] RXB idle level = ");
  Serial.println(digitalRead(BITBANG_RX_PIN));   // 0 なら LOW、1 なら HIGH

  /* 既存の setup の最後に … */
  ESP_EARLY_LOGI("AUTO","Δ sweep %u–%u µs  step=%u",
                 DELTA_MIN_US, DELTA_MAX_US, DELTA_STEP_US);
}

// loop()にLED点滅処理を追加
void loop() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  if (millis() - lastBlink >= 100) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastBlink = millis();
//    dumpCtrl("AFTER_INIT");   // 期待: 0 0 1 0
  }

  // loop()は必ず何かdelay入れる（CPU占有防止）
  delay(10);
}
