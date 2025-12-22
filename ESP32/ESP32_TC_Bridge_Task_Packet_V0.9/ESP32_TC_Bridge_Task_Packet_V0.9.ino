/**
 * ESP32S3_TC_Bridge_Full_VarLen.ino
 *  - Core1 (high priority): Communication transaction (Pi 9600bps <-> TC 300bps)
 *  - Core0 (low priority): OLED UI + Serial logging
 *
 * Variable request length: 6 / 8 / 12 bytes (RESET=12, SENS.ADJ=8, SEND=6)
 * Response length: default 6 bytes (必要なら同様に可変化できる)
 *
 * Pins:
 *  - Pi UART: RX=GPIO44, TX=GPIO43 (UART1)
 *  - TC UART: RX=GPIO3,  TX=GPIO2  (UART2)
 *  - SN74LVC16T245 fixed:
 *      OE1=GPIO5 LOW, DIR1=GPIO6 HIGH   (ESP -> TC)
 *      OE2=GPIO7 LOW, DIR2=GPIO8 LOW    (TC -> ESP)
 */

#include <Arduino.h>
#include <HardwareSerial.h>

#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

/* ----------------- Config ----------------- */
static constexpr uint32_t BAUD_PI = 9600;
static constexpr uint32_t BAUD_TC = 300;

static constexpr int UART_PI_TX = 43;   // ESP -> Pi
static constexpr int UART_PI_RX = 44;   // Pi -> ESP
static constexpr int UART_TC_TX = 2;    // ESP -> TC
static constexpr int UART_TC_RX = 3;    // TC -> ESP

static constexpr int PIN_OE1  = 5;
static constexpr int PIN_DIR1 = 6;
static constexpr int PIN_OE2  = 7;
static constexpr int PIN_DIR2 = 8;

static constexpr int OLED_ADDR = 0x3C;
static constexpr int OLED_W = 128;
static constexpr int OLED_H = 64;

/* 可変長 */
static constexpr size_t REQ_LEN_MIN = 6;
static constexpr size_t REQ_LEN_MAX = 12;

/* 応答長（必要なら可変に変更OK） */
static constexpr size_t RESP_LEN_DEFAULT = 6;

/* timeouts */
static constexpr uint32_t PI_RX_TIMEOUT_MS = 300;        // 12Bでも余裕
static constexpr uint32_t TC_RX_TIMEOUT_MS = 1000;       // 相手処理込み
static constexpr uint32_t INTER_BYTE_TIMEOUT_MS = 150;   // 300bpsでも余裕

/* OLED update throttle */
static constexpr uint32_t OLED_MIN_UPDATE_MS = 200;

/* ----------------- UART ----------------- */
HardwareSerial SerialPi(1);
HardwareSerial SerialTC(2);

/* ----------------- OLED ----------------- */
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire);

/* ----------------- Queues ----------------- */
struct DisplayData {
  uint32_t seq;
  char     line1[22];
  char     line2[22];
  char     line3[22];
  bool     isError;
};

struct LogLine {
  char s[96];
};

static QueueHandle_t qDisplay = nullptr; // len=1 overwrite
static QueueHandle_t qLog     = nullptr; // logs

/* ----------------- Shared state ----------------- */
static volatile bool comm_busy = false;
static volatile uint32_t seq_counter = 0;

/* ----------------- Utils ----------------- */
static inline uint32_t msNow() { return (uint32_t)millis(); }

static void flushInput(HardwareSerial &ser) {
  while (ser.available() > 0) (void)ser.read();
}

static void logf(const char *fmt, ...) {
  if (!qLog) return;
  LogLine line{};
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line.s, sizeof(line.s), fmt, ap);
  va_end(ap);
  (void)xQueueSend(qLog, &line, 0);
}

static void pushDisplay(bool isErr, const char *l1, const char *l2, const char *l3) {
  if (!qDisplay) return;
  DisplayData d{};
  d.seq = seq_counter;
  d.isError = isErr;
  strncpy(d.line1, l1 ? l1 : "", sizeof(d.line1)-1);
  strncpy(d.line2, l2 ? l2 : "", sizeof(d.line2)-1);
  strncpy(d.line3, l3 ? l3 : "", sizeof(d.line3)-1);
  (void)xQueueOverwrite(qDisplay, &d);
}

/**
 * read exactly n bytes with overall timeout + inter-byte timeout
 */
static bool readExact(HardwareSerial &ser,
                      uint8_t *dst,
                      size_t n,
                      uint32_t timeout_ms,
                      uint32_t inter_byte_timeout_ms)
{
  uint32_t t_start = msNow();
  uint32_t t_last  = msNow();
  size_t got = 0;

  while (got < n) {
    while (ser.available() > 0 && got < n) {
      int v = ser.read();
      if (v < 0) break;
      dst[got++] = (uint8_t)v;
      t_last = msNow();
    }

    uint32_t now = msNow();
    if ((now - t_start) > timeout_ms) return false;
    if (got > 0 && (now - t_last) > inter_byte_timeout_ms) return false;

    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return true;
}

/**
 * Determine expected request length by first byte.
 * ここは tc_packet.hpp の仕様に合わせて “ビットマスク” を調整する必要があります。
 *
 * Gemini案では headByte & 0x07 をcmdとしていました。
 * まずはそれに合わせて実装（違ったらここだけ直せばOK）。
 */
static size_t expectedReqLenFromFirst(uint8_t firstByte) {
  uint8_t cmd = firstByte & 0x07;  // ←ここが仕様と違うなら変更

  // 仮のコマンド割当（あなたの定義に合わせて数字を直す）
  // 例: CMD_RESET=??, CMD_SENS_ADJ=??, CMD_SEND=?? (SENDは6B)
  // ひとまず “RESET=12 / SENS=8 / else=6” の形にしておく。
  //
  // ★あなたの tc_packet.hpp に定数があるなら、ここをそれに合わせてください。
  static constexpr uint8_t CMD_RESET    = 0x01; // 仮
  static constexpr uint8_t CMD_SENS_ADJ = 0x02; // 仮

  if (cmd == CMD_RESET)    return 12;
  if (cmd == CMD_SENS_ADJ) return 8;
  return 6;
}

/* ----------------- Tasks ----------------- */
void taskComm(void *pv) {
  (void)pv;

  uint8_t req[REQ_LEN_MAX];
  uint8_t resp[REQ_LEN_MAX];

  flushInput(SerialPi);
  flushInput(SerialTC);

  logf("[COMM] started on core %d", xPortGetCoreID());

  while (true) {
    comm_busy = false;

    // 先頭バイトが来るまで待つ
    if (SerialPi.available() <= 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    // 先頭を覗いて長さ確定（peekが不安なら readして戻す構造に変更可）
    int p = SerialPi.peek();
    if (p < 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    size_t reqLen = expectedReqLenFromFirst((uint8_t)p);
    if (reqLen < REQ_LEN_MIN || reqLen > REQ_LEN_MAX) {
      // 想定外 → 1バイト捨てて同期回復を狙う
      (void)SerialPi.read();
      logf("[COMM] bad reqLen -> drop 1 byte");
      continue;
    }

    // トランザクション開始
    comm_busy = true;
    seq_counter++;

    // Piから reqLen 受信
    memset(req, 0, sizeof(req));
    if (!readExact(SerialPi, req, reqLen, PI_RX_TIMEOUT_MS, INTER_BYTE_TIMEOUT_MS)) {
      logf("[COMM] Pi RX timeout seq=%lu", (unsigned long)seq_counter);
      pushDisplay(true, "ERR: Pi timeout", "req not complete", "");
      comm_busy = false;
      continue;
    }

    // 表示
    char l2[22], l3[22];
    snprintf(l2, sizeof(l2), "Pi->TC %uB seq=%lu", (unsigned)reqLen, (unsigned long)seq_counter);
    // 先頭6Bだけ表示（OLED幅）
    snprintf(l3, sizeof(l3), "%02X %02X %02X %02X %02X %02X",
             req[0], req[1], req[2], req[3], req[4], req[5]);
    pushDisplay(false, "TX: Pi -> TC", l2, l3);

    // TCへ送信（flushしない）
    size_t w = SerialTC.write(req, reqLen);
    if (w != reqLen) {
      logf("[COMM] SerialTC.write short: %u/%u", (unsigned)w, (unsigned)reqLen);
      pushDisplay(true, "ERR: write TC", "short write", "");
      flushInput(SerialTC);
      comm_busy = false;
      continue;
    }

    // 応答（まずは6B固定。必要ならここも可変化）
    size_t respLen = RESP_LEN_DEFAULT;

    memset(resp, 0, sizeof(resp));
    bool ok = readExact(SerialTC, resp, respLen, TC_RX_TIMEOUT_MS, INTER_BYTE_TIMEOUT_MS);
    if (!ok) {
      logf("[COMM] TC RX timeout seq=%lu", (unsigned long)seq_counter);
      pushDisplay(true, "ERR: TC timeout", "no response", "");
      flushInput(SerialTC);
      comm_busy = false;
      continue;
    }

    // Piへ返送
    size_t wp = SerialPi.write(resp, respLen);
    if (wp != respLen) {
      logf("[COMM] SerialPi.write short: %u/%u", (unsigned)wp, (unsigned)respLen);
      pushDisplay(true, "ERR: write Pi", "short write", "");
      comm_busy = false;
      continue;
    }

    logf("[COMM] OK seq=%lu req=%uB resp=%uB",
         (unsigned long)seq_counter, (unsigned)reqLen, (unsigned)respLen);

    snprintf(l2, sizeof(l2), "TC->Pi %uB seq=%lu", (unsigned)respLen, (unsigned long)seq_counter);
    snprintf(l3, sizeof(l3), "%02X %02X %02X %02X %02X %02X",
             resp[0], resp[1], resp[2], resp[3], resp[4], resp[5]);
    pushDisplay(false, "RX: TC -> Pi", l2, l3);

    comm_busy = false;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void taskUiLog(void *pv) {
  (void)pv;

  bool oled_ok = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  uint32_t last_oled_ms = 0;

  if (oled_ok) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("TC Bridge Ready");
    display.println("UI on Core0");
    display.display();
  } else {
    Serial.println("[UI] OLED init failed (continue without OLED)");
  }

  logf("[UI] started on core %d", xPortGetCoreID());

  DisplayData d{};
  LogLine line{};

  while (true) {
    // logs
    for (int i = 0; i < 10; i++) {
      if (xQueueReceive(qLog, &line, 0) == pdTRUE) Serial.println(line.s);
      else break;
    }

    // OLED: latest only, throttled, and not during comm
    if (oled_ok) {
      if (xQueuePeek(qDisplay, &d, 0) == pdTRUE) {
        uint
