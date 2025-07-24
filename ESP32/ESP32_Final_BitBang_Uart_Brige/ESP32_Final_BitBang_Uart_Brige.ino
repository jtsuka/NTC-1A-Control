// TC Repeater with BitBang Response Timeout - ESP32-S3 (FIXED)
// ------------------------------------------------------------
// - UART RX/TX (to Pi):  GPIO44 / GPIO43
// - BitBang TX/RX (to TC): GPIO2 / GPIO3
// - UART  : 9 600 bps
// - BB    : 300 bps  (1 bit = 3340 µs)
// ------------------------------------------------------------

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define UART_TX_PIN        43
#define UART_RX_PIN        44
#define BITBANG_TX_PIN      2
#define BITBANG_RX_PIN      3

#define UART_BAUD_RATE      9600
#define BITBANG_DELAY_US    3340            // 1 bit
#define RESPONSE_TIMEOUT_MS 200
#define MAX_PACKET_LEN      32
#define FIXED_PACKET_LEN     6
#define LED_PIN             21

/* --- SN74LVC8T245 制御 -------------------------- */
#define PIN_OE_A  5   // 1OE  LOW = enable  (TC→ESP32 方向)
#define PIN_DIR_A 6   // 1DIR LOW = B→A
#define PIN_OE_B  7   // 2OE  LOW = enable  (ESP32→TC 方向)
#define PIN_DIR_B 8   // 2DIR HIGH = A→B

/* Δ 自動チューニング範囲 ------------------------- */
#define DELTA_MIN_US  (-300)
#define DELTA_MAX_US  ( 300)
#define DELTA_STEP_US   25

/* グローバル ------------------------------------ */
QueueHandle_t bitbangRxQueue;
portMUX_TYPE  bitbangMux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t lastSent[FIXED_PACKET_LEN];
static bool    lastValid = false;

static int32_t  delta_now   = DELTA_MIN_US;   // ★FIX 左端スタート
static bool     delta_fixed = false;
static uint8_t  sweepCount  = 0;
static uint32_t lastEvalMs  = 0;
/* ---- 統計用カウンタ ---- */
static volatile uint16_t syncOK   = 0;
static volatile uint16_t syncNG   = 0;
static volatile uint16_t startFail = 0;   // ★追加
static volatile uint16_t stopFail  = 0;   // ★追加


/* =============== ユーティリティ ================= */
static inline uint8_t rev8(uint8_t v)      // LSB/MSB 反転
{
  v = (v >> 4) | (v << 4);
  v = ((v & 0xCC) >> 2) | ((v & 0x33) << 2);
  v = ((v & 0xAA) >> 1) | ((v & 0x55) << 1);
  return v;
}

/* ============ UART HELPERS ===================== */
void uartInit() {
  Serial1.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
}
int uartReceivePacket(uint8_t *buf) {
  int len = 0;
  unsigned long t0 = millis();
  while ((millis() - t0) < 100 && len < MAX_PACKET_LEN) {
    if (Serial1.available()) buf[len++] = Serial1.read();
  }
  return len;
}
void uartSendPacket(const uint8_t *buf, int len) {
  Serial1.write(buf, len);
  Serial1.flush();
}

/* ============ STARTビット検出 =================== */
static bool waitValidStart()
{
  const uint32_t debounce_us = 500;                // ★FIX 300→500
  const uint32_t halfbit_us  = BITBANG_DELAY_US / 2;

  uint32_t t0 = micros();
  while (digitalRead(BITBANG_RX_PIN) == HIGH) {    // LOW待ち
    if (micros() - t0 > 30000) return false;
  }
  delayMicroseconds(debounce_us);
  if (digitalRead(BITBANG_RX_PIN) == HIGH) return false;

  delayMicroseconds(halfbit_us + delta_now);       // 中央へ
  return true;
}

/* ============ Bit-Bang 送信 ===================== */
void bitBangSendByte(uint8_t b)
{
  pinMode(BITBANG_TX_PIN, OUTPUT);

  taskENTER_CRITICAL(&bitbangMux);
  digitalWrite(BITBANG_TX_PIN, LOW);
  delayMicroseconds(BITBANG_DELAY_US);

  for (int i = 0; i < 8; ++i) {
    digitalWrite(BITBANG_TX_PIN, (b >> i) & 1);
    delayMicroseconds(BITBANG_DELAY_US);
  }

  /* --- ★NEW: Stop = HIGH を 200 µs 駆動してから Hi-Z ---- */
  digitalWrite(BITBANG_TX_PIN, HIGH);
  delayMicroseconds(200);
  taskEXIT_CRITICAL(&bitbangMux);

  pinMode(BITBANG_TX_PIN, INPUT);                  // Hi-Z
}

void bitBangSendPacket(const uint8_t *buf, int len)
{
  for (int i = 0; i < len; ++i) {
    bitBangSendByte(rev8(buf[i]));
    delayMicroseconds(BITBANG_DELAY_US);           // byte gap = 1bit
  }
}

/* ============ Bit-Bang 受信 ===================== */
int bitBangReceivePacket(uint8_t *buf, int maxLen)
{
  int n = 0;
  while (n < maxLen) {
    if (!waitValidStart()) { syncNG++; return 0; }

    uint8_t b = 0;
    for (int i = 0; i < 8; ++i) {
      b |= digitalRead(BITBANG_RX_PIN) << i;
      delayMicroseconds(BITBANG_DELAY_US);
    }
    b = rev8(b);

    /* --- ★NEW: Stop ビット確認 ------------------ */
    bool stopOk = digitalRead(BITBANG_RX_PIN);
    delayMicroseconds(BITBANG_DELAY_US);
    if (!stopOk) { syncNG++; return 0; }

    buf[n++] = b;
    if (n >= FIXED_PACKET_LEN) break;
  }
  syncOK++;
  return n;
}

/* ============ FreeRTOS Tasks =================== */
void TaskBitBangReceive(void*)
{
  uint8_t rxBuf[MAX_PACKET_LEN];

  while (1) {
    int len = bitBangReceivePacket(rxBuf, MAX_PACKET_LEN);
    if (len == FIXED_PACKET_LEN) {
      uint8_t* p = (uint8_t*)malloc(len);
      if (p) {
        memcpy(p, rxBuf, len);
        xQueueSend(bitbangRxQueue, &p, 0);
      }
    }

    /* --- Δ オートチューニング ------------------ */
    uint32_t now = millis();
    if (!delta_fixed && now - lastEvalMs > 250) {
      uint16_t total = syncOK + syncNG;
      if (total < 10) {                         // まだデータ不足
        ESP_EARLY_LOGI("AUTO",
          "Δ=%ld  OBSERVED=%u まだ評価待ち…",
         (long)delta_now, total);
      } else {
        uint32_t failPermil = syncNG * 1000UL / total;
        Serial.printf("[AUTO] Δ=%ld  OK=%u NG=%u  fail=%u.%u%%\n",
                      (long)delta_now, syncOK, syncNG,
                      failPermil/10, failPermil%10);

        if (failPermil < 150) {
          delta_fixed = true;                       // 85 %↑ でFIX
        } else {
          delta_now += DELTA_STEP_US;               // 次へ
          if (delta_now > DELTA_MAX_US) {
            delta_now = DELTA_MIN_US;
            if (++sweepCount >= 3) delta_fixed = true;
          }
        }
        syncOK = syncNG = 0;
      }
      /* ---- ② Δ を必ず 25 µs 進める ---- */
      if (total >= 10) {          // 10 件たまったら必ず次へ
        delta_now += DELTA_STEP_US;
        if (delta_now > DELTA_MAX_US) delta_now = DELTA_MIN_US;
        syncNG = syncOK = 0;
        startFail = stopFail = 0;
        lastEvalMs = now;
      }
      lastEvalMs = now;

      if (delta_fixed) vTaskSuspend(NULL);          // ★FIX: 以降停止
    }
    vTaskDelay(1);
  }
}

void TaskUartReceive(void*)
{
  uint8_t buf[MAX_PACKET_LEN];

  while (1) {
    int len = uartReceivePacket(buf);
    if (len == FIXED_PACKET_LEN) {

      if (lastValid && !memcmp(buf, lastSent, FIXED_PACKET_LEN)) {
        vTaskDelay(1); continue;                    // 同一パケットskip
      }
      bitBangSendPacket(buf, FIXED_PACKET_LEN);
      memcpy(lastSent, buf, FIXED_PACKET_LEN);
      lastValid = true;

      uint8_t *echo = nullptr;
      if (xQueueReceive(bitbangRxQueue, &echo,
                        pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS)) == pdTRUE)
      {
        uint8_t tx[FIXED_PACKET_LEN];
        for (int i = 0; i < FIXED_PACKET_LEN; ++i) tx[i] = rev8(echo[i]);
        uartSendPacket(tx, FIXED_PACKET_LEN);
        free(echo);
      }
    }
    vTaskDelay(1);
  }
}

/* ============ SETUP / LOOP ===================== */
void setup()
{
  Serial.begin(115200);
  uartInit();

  pinMode(LED_PIN, OUTPUT);

  /* ★NEW: 強プルアップ */
  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);
  gpio_set_pull_mode((gpio_num_t)BITBANG_RX_PIN, GPIO_PULLUP_ONLY);
  gpio_pullup_en((gpio_num_t)BITBANG_RX_PIN);

  /* LVC8T245 設定 */
  pinMode(PIN_DIR_A, OUTPUT); pinMode(PIN_DIR_B, OUTPUT);
  pinMode(PIN_OE_A,  OUTPUT); pinMode(PIN_OE_B,  OUTPUT);

  digitalWrite(PIN_DIR_A, LOW);   // TC→ESP32
  digitalWrite(PIN_DIR_B, HIGH);  // ESP32→TC
  digitalWrite(PIN_OE_A,  LOW);
  digitalWrite(PIN_OE_B,  LOW);

  Serial.printf("[DEBUG] RXB idle = %d\n", digitalRead(BITBANG_RX_PIN));

  bitbangRxQueue = xQueueCreate(4, sizeof(uint8_t*));
  xTaskCreatePinnedToCore(TaskBitBangReceive, "BB-RX", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskUartReceive,   "UART-RX",4096, NULL, 1, NULL, 1);

  Serial.println("[START] Repeater ready");
}

void loop()
{
  static uint32_t prev = 0;
  if (millis() - prev > 1000) {                 // 1 s 心拍LED
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    prev = millis();
  }
  delay(10);
}
