// TC Repeater with HardWare UART Response Timeout - ESP32-S3 (FIXED)
// ------------------------------------------------------------
// - UART RX/TX (to Pi):  GPIO44 / GPIO43
// - UART TX/RX (to TC):  GPIO2 / GPIO3
// - HW UART   : 9600 bps
// - HW UART   : 300 bps
// ------------------------------------------------------------

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define UART_TX_PIN        43
#define UART_RX_PIN        44
#define TC_UART_TX_PIN      2
#define TC_UART_RX_PIN      3

#define UART_BAUD_RATE      9600
//#define BITBANG_DELAY_US    3340            // 1 bit
//#define RESPONSE_TIMEOUT_MS 200
#define MAX_PACKET_LEN      32
#define FIXED_PACKET_LEN     6
#define LED_PIN             21

/* --- SN74LVC8T245 制御 -------------------------- */
#define PIN_OE_A  5   // 1OE  LOW = enable  (TC→ESP32 方向)
#define PIN_DIR_A 6   // 1DIR LOW = B→A
#define PIN_OE_B  7   // 2OE  LOW = enable  (ESP32→TC 方向)
#define PIN_DIR_B 8   // 2DIR HIGH = A→B

/* Δ 自動チューニング範囲 ------------------------- */
//#define DELTA_MIN_US  (-300)
//#define DELTA_MAX_US  ( 300)
//#define DELTA_STEP_US   25

/* ハードウエアシリアル */
HardwareSerial PiSerial      = Serial1;   // Raspberry Pi 用 (UART0)
HardwareSerial TCserial(2);               // TC 用 (UART1)  ※Serial2 と同義

/* グローバル ------------------------------------ */
QueueHandle_t bitbangRxQueue;
portMUX_TYPE  bitbangMux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t lastSent[FIXED_PACKET_LEN];
static bool    lastValid = false;

//static int32_t  delta_now   = DELTA_MIN_US;   // ★FIX 左端スタート
//static bool     delta_fixed = false;
//static uint16_t syncOK = 0, syncNG = 0;
//static uint8_t  sweepCount  = 0;
//static uint32_t lastEvalMs  = 0;
/* ---- 統計用カウンタ ---- */
//static volatile bool startOK   = false;   //  ← ここを追加
//static volatile uint16_t syncOK   = 0;
//static volatile uint16_t syncNG   = 0;
//static volatile uint16_t startFail = 0;   // ★追加
//static volatile uint16_t stopFail  = 0;   // ★追加


/* =============== ユーティリティ ================= */
static inline uint8_t rev8(uint8_t v)      // LSB/MSB 反転
{
  v = (v >> 4) | (v << 4);
  v = ((v & 0xCC) >> 2) | ((v & 0x33) << 2);
  v = ((v & 0xAA) >> 1) | ((v & 0x55) << 1);
  return v;
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


/* ============ FreeRTOS Tasks =================== */
void TaskTCUartReceive(void*)
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
//  uartInit();
  Serial1.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);  // Pi
  Serial2.setRxBufferSize(256);             // ★バッファ拡大
  Serial2.begin(300,  SERIAL_8N1, TC_UART_RX_PIN, TC_UART_TX_PIN);    // TC

  pinMode(LED_PIN, OUTPUT);

  /* LVC16T245 設定 */
  pinMode(PIN_DIR_A, OUTPUT); pinMode(PIN_DIR_B, OUTPUT);
  pinMode(PIN_OE_A,  OUTPUT); pinMode(PIN_OE_B,  OUTPUT);

  digitalWrite(PIN_DIR_A, LOW);   // TC→ESP32
  digitalWrite(PIN_DIR_B, HIGH);  // ESP32→TC
  digitalWrite(PIN_OE_A,  LOW);
  digitalWrite(PIN_OE_B,  LOW);

//  Serial.printf("[DEBUG] RXB idle = %d\n", digitalRead(BITBANG_RX_PIN));

  /* 2) Raspberry Pi ⇆ ESP32 (9600 bps) --------------------- */
  //  RX=GPIO44, TX=GPIO43 → XIAO 基板の Grove JP7 に相当
  PiSerial.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  /* 3) TC ⇆ ESP32 (300 bps) ------------------------------- */
  //  RX=GPIO3,  TX=GPIO2  → Grove JP2
  TCserial.begin(300, SERIAL_8N1, TC_UART_RX_PIN, TC_UART_TX_PIN);

  bitbangRxQueue = xQueueCreate(4, sizeof(uint8_t*));
  xTaskCreatePinnedToCore(TaskTCUartReceive, "TC-RX", 4096, NULL, 1, NULL, 1);
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
