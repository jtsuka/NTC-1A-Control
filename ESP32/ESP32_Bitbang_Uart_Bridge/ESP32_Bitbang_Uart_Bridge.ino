// TC Repeater with BitBang Response Timeout - ESP32-S3 (修正版)
// ----------------------------------------
// - UART RX/TX (to Pi): GPIO44 / GPIO43
// - BitBang TX/RX (to TC): GPIO2 / GPIO3
// - Baud rate: UART = 9600bps, BitBang = 300bps
// - Wait up to 100ms for TC response, then reply to Pi
// - Safe with FreeRTOS & taskENTER_CRITICAL
// - 2025.07.14 3th Try Version
// ----------------------------------------

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define UART_RX_PIN        44
#define UART_TX_PIN        43
#define BITBANG_TX_PIN     2
#define BITBANG_RX_PIN     3

#define UART_BAUD_RATE     9600
#define BITBANG_DELAY_US   3340
#define RESPONSE_TIMEOUT_MS 200
#define MAX_PACKET_LEN     32
#define FIXED_PACKET_LEN   6

#define START_OFFSET 1.94f
#define BYTE_GAP  1
// ---------------- tunable range ----------------
#define DELTA_MIN_US  380   // ここから
#define DELTA_MAX_US  720   // ここまで自動スキャン
#define DELTA_STEP_US  20   // 微調ステップ

QueueHandle_t bitbangRxQueue;
portMUX_TYPE bitbangMux = portMUX_INITIALIZER_UNLOCKED;

// --- 追加：直前に送った 6 byte を保持するバッファ
static uint8_t lastSent[FIXED_PACKET_LEN] = {0};
static bool    lastValid = false;                // まだ何も送っていない状態

// for tuneable range
static uint32_t delta_now = (DELTA_MIN_US + DELTA_MAX_US)/2;
static bool delta_fixed   = false;

// ---- 失敗統計用 ----
static uint16_t syncFail   = 0;
static uint16_t syncOK     = 0;


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

/* ========= 追加：ノイズ除去付きスタート検出 ========= */
static bool waitValidStart()
{
  /* 中央へ移動 – ½bit + Δ */
 // const uint32_t halfBit = BITBANG_DELAY_US / 2;   // 1 665 µs
 // const uint32_t delta   = 571;                    // ← 試す値 (+25µs ずつ)

    const uint32_t debounce_us = BITBANG_DELAY_US / 3;
    uint32_t t0 = micros();

    while (digitalRead(BITBANG_RX_PIN) == HIGH) {
        if (micros() - t0 > delta_now) return false;   // ← ★ ここ
    }
    /* スタート Low を 30 ms だけ待つ */
//    uint32_t t0 = micros();
    while (digitalRead(BITBANG_RX_PIN) == HIGH) {
        if (micros() - t0 > 30000) return false;
    }

    /* ノイズ除去（ここでは 300 µs）*/
    delayMicroseconds(300);
    if (digitalRead(BITBANG_RX_PIN) == HIGH) return false;

    /* ---------- 中央へ ½bit 移動 ---------- */
    delayMicroseconds(debounce_us + delta_now);

    return true;                               // ここが bit0 中央
}

#if 0
static bool waitValidStart()
{
  const uint32_t debounce_us = BITBANG_DELAY_US / 3;   // ≒ 1.1 ms
  uint32_t t0 = micros();

  /* LOW に落ちるのを 30 ms だけ待つ */
  while (digitalRead(BITBANG_RX_PIN) == HIGH) {
    if (micros() - t0 > 30000) return false;           // タイムアウト
  }

  /* ¼bit 後にまだ LOW か？（短いグリッチならここで弾く） */
  delayMicroseconds(debounce_us);
  if (digitalRead(BITBANG_RX_PIN) == HIGH) return false;

  /* さらに ¼bit 後も確認（より確実に）*/
  delayMicroseconds(debounce_us);
  if (digitalRead(BITBANG_RX_PIN) == HIGH) return false;

  /* 本物確定 ─ 中央まで移動
     3340-2*1113 = 1114 µs だと“ほんの少し前寄り”だったので +10µs */
   const uint32_t center_us = (uint32_t)(BITBANG_DELAY_US * START_OFFSET);
  delayMicroseconds(center_us);

  return true;
}
#endif

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
//  Serial.print("[SEND TC] ");
  for (int i = 0; i < len; i++) {
    bitBangSendByte(rev8(buf[i]));
    delayMicroseconds(BITBANG_DELAY_US * BYTE_GAP);   // ←バイト間ギャップ 3 -> 2 ->1 へ
//    Serial.printf("%02X ", buf[i]);
  }
//  Serial.println();
}

int bitBangReceivePacket(uint8_t *buf, int maxLen)
{
  int byteCount = 0;

  while (byteCount < maxLen)
  {
    /* ---- スタート検出 ---- */
    uint32_t t0 = micros();               // ←★ ここで現在時刻を保存
    bool got = waitValidStart();
  //  ESP_EARLY_LOGI("SYN","start=%s time=%lu us",
  //             got ? "OK" : "FAIL",
  //             micros()-t0);
    /* ---- スタート検出（デバウンス付き） ---- */
    if (!got) return 0;   // ノイズ or 30 ms 超

    /* ---- 8 bit 読み取り ---- */
    uint8_t b = 0;
    for (int i = 0; i < 8; i++) {
      b |= (digitalRead(BITBANG_RX_PIN) << i);
      delayMicroseconds(BITBANG_DELAY_US);
    }
    if (byteCount == 0 || byteCount == 5) {
    ESP_EARLY_LOGI("BB", "b%d=%02X idle=%d",
                   byteCount, b, digitalRead(BITBANG_RX_PIN));
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
  // bit反転
//  for (int i = 0; i < maxLen; i++ ) {
//   buf[i] = rev8(buf[i]);   // bitBangReceivePacket の最後で
      /* ★受信側では bit 反転しない（Pi へ返す直前だけ rev8） */
//  }

  return byteCount;
}


void TaskBitBangReceive(void *pvParameters) {
  uint8_t rxBuf[MAX_PACKET_LEN];
  static uint32_t dbgCount[7]={0};
  int cnt=0;
  while (1) {
    int len = bitBangReceivePacket(rxBuf, MAX_PACKET_LEN);
    // for Debug
//    ESP_EARLY_LOGI("BBRX", "len=%d", len);           // ★追加
    if (len<=6) dbgCount[len]++;
//    if (len > 0) {
//      ESP_EARLY_LOGI("BBRX", "b0=%02X", rxBuf[0]); // ★追加
//    } else if ( len = 0 && cnt++ < 3 ) {
//      ESP_EARLY_LOGI("BBRX", "len=%d", len);           // ★追加
//    }



//    if ((dbgCount[0]+dbgCount[6]) % 500 == 0) {   // 500回に1度だけ
//      ESP_EARLY_LOGI("BBRX", "len[0]=%u len[6]=%u", dbgCount[0], dbgCount[6]);
//    }

//   Serial.printf("[DBG TCraw] len=%d", len);
//   Serial.println();
//    for(int i=0;i<len;i++) {
//      Serial.printf(" %02X", rxBuf[i]);
//      Serial.println();
//    }

    if (len == FIXED_PACKET_LEN) {
      uint8_t* copyBuf = (uint8_t*)malloc(len);
      if (copyBuf != nullptr) {
        memcpy(copyBuf, rxBuf, len);
        if (xQueueSend(bitbangRxQueue, &copyBuf, pdMS_TO_TICKS(10)) != pdTRUE) {
          free(copyBuf);  // キューがいっぱい → メモリ解放
          copyBuf = nullptr;
//         Serial.println("[ERROR] xQueueSend failed. Buffer discarded.");
        }
        // ★ TaskBitBangReceive の Queue 成功直後
//        Serial.println("[DBG] RX→Queue OK");
//        Serial.println();
      } else {
//        Serial.println("[ERROR] malloc failed in BitBangReceive");
//        Serial.println();
      }
    }
//    if(!startOK) syncFail++; else syncOK++;

    uint32_t now = millis();
    if(!delta_fixed && now - lastEvalMs > 250) {          // 250 ms毎に評価
        uint16_t total = syncFail + syncOK;
        if(total > 30) {                                  // データ十分？
            float failRate = (float)syncFail / total;
            ESP_EARLY_LOGI("AUTO","delta=%lu  fail=%.1f%%",(unsigned long)delta_now,
                          failRate*100.0);

            if(failRate < 0.15) {
                // 充分良い → step/2 ずつ縮めてゼロを狙う
                if(delta_now > DELTA_MIN_US + DELTA_STEP_US)
                    delta_now -= DELTA_STEP_US/2;
                else delta_fixed = true;
            } else {
                // 悪い → 広げる
                if(delta_now < DELTA_MAX_US - DELTA_STEP_US)
                    delta_now += DELTA_STEP_US;
                else delta_fixed = true;                  // 端まで来たら FIX
            }
            syncFail = syncOK = 0;
        }
       lastEvalMs = now;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
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
        // ★ Pi → ESP32 が 6 バイト入った直後
        if (len == FIXED_PACKET_LEN) {
//            Serial.print("[DBG RX] ");
//            for (int i=0;i<len;i++) Serial.printf("%02X ", buf[i]);
//            Serial.println();
        }

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
        /* デバッグで内容を必ず出力して確認 */
//        Serial.print("[DBG] echoBuf =");
//        for(int i=0;i<FIXED_PACKET_LEN;i++) Serial.printf(" %02X", echoBuf[i]);
//        Serial.println();


        /* LSB→MSB へビット反転して Pi へ返す */
        uint8_t txTmp[FIXED_PACKET_LEN];
        for (int i = 0; i < FIXED_PACKET_LEN; i++) {
            txTmp[i] = rev8(echoBuf[i]);   // ★ 反転
        }
        /* 2) デバッグ表示 */
//        Serial.print("[DBG] Pi<-TC:");
//        for (int i = 0; i < FIXED_PACKET_LEN; i++) Serial.printf(" %02X", txTmp[i]);
//        Serial.println();
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

void setup() {
  Serial.begin(115200);
  uartInit();

  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);
  Serial.printf("[DBG] idle-level=%d\n", digitalRead(BITBANG_RX_PIN));

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

void loop() {
  // loopなし
}
