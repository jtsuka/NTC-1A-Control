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
#define LED_PIN 21

// AE-LLCNV-LVC8T245基板セットアップ
#define RX_DIR_GPIO 7
#define RX_OE_GPIO 8
#define TX_DIR_GPIO 9
#define TX_OE_GPIO 4

#define START_OFFSET 1.94f
#define BYTE_GAP  1
// ---------------- tunable range ----------------
#define DELTA_MIN_US  200   // ここから
#define DELTA_MAX_US  1200   // ここまで自動スキャン
#define DELTA_STEP_US  25   // 微調ステップ

QueueHandle_t bitbangRxQueue;
portMUX_TYPE bitbangMux = portMUX_INITIALIZER_UNLOCKED;

// --- 追加：直前に送った 6 byte を保持するバッファ
static uint8_t lastSent[FIXED_PACKET_LEN] = {0};
static bool    lastValid = false;                // まだ何も送っていない状態

/* ---- 自動チューニング用 ---- */
static uint32_t delta_now  = (DELTA_MIN_US + DELTA_MAX_US)/2;
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

/* ========= 追加：ノイズ除去付きスタート検出 ========= */
static bool waitValidStart()
{
    const uint32_t debounce_us = BITBANG_DELAY_US / 3;
    uint32_t t0 = micros();

    /* スタート Low を 30 ms だけ待つ */
    while (digitalRead(BITBANG_RX_PIN) == HIGH) {
        if (micros() - t0 > 30000) return false;
    }

    /* ノイズ除去（ここでは 300 µs）*/
    delayMicroseconds(300);
    if (digitalRead(BITBANG_RX_PIN) == LOW) return false;

    /* ---------- 中央へ ½bit 移動 ---------- */
    delayMicroseconds(debounce_us + delta_now);

    return true;                               // ここが bit0 中央
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

    uint32_t now = millis();
    if(!delta_fixed && now - lastEvalMs > 250) {          // 250 ms毎に評価
        uint16_t total = syncFail + syncOK;
        if(total > 30) {                                  // データ十分？
            uint32_t failPermil = (syncFail * 1000 )/ total; // 0.1% 単位
            ESP_EARLY_LOGI("AUTO","delta=%lu  fail=%u.%u%%",(unsigned long)delta_now,
                          failPermil/10, failPermil%10);

            if(failPermil < 150) {
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

void setup() {
  Serial.begin(115200);
  uartInit();

  // for Look()
  pinMode(LED_PIN, OUTPUT);
//  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);  // 内部P-UP
//  Serial.printf("[DBG] idle-level=%d\n", digitalRead(BITBANG_RX_PIN));
  pinMode(BITBANG_RX_PIN, INPUT); // 高インピーダンス
  Serial.printf("[DEBUG] RXB idle level = %d\n", digitalRead(BITBANG_RX_PIN));

  // RX側 for AE-LLCNV-LVC8T245(1)
  pinMode(RX_DIR_GPIO, OUTPUT); digitalWrite(RX_DIR_GPIO, LOW);   // B→A
  pinMode(RX_OE_GPIO, OUTPUT);  digitalWrite(RX_OE_GPIO, LOW);    // 有効

  // TX側 for AE-LLCNV-LVC8T245(2)
  pinMode(TX_DIR_GPIO, OUTPUT); digitalWrite(TX_DIR_GPIO, HIGH);  // A→B
  pinMode(TX_OE_GPIO, OUTPUT);  digitalWrite(TX_OE_GPIO, LOW);    // 有効

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
  }

  // loop()は必ず何かdelay入れる（CPU占有防止）
  delay(10);
}
