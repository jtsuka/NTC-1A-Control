#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "tc_packet.hpp"

using namespace tc;

/* ----------------- Pins / UART ----------------- */
static constexpr uint32_t BAUD_PI = 9600;
static constexpr uint32_t BAUD_TC = 300;

static constexpr int UART_PI_TX = 43;
static constexpr int UART_PI_RX = 44;
static constexpr int UART_TC_TX = 2;
static constexpr int UART_TC_RX = 3;

static constexpr int PIN_OE1  = 5;
static constexpr int PIN_DIR1 = 6;
static constexpr int PIN_OE2  = 7;
static constexpr int PIN_DIR2 = 8;

/* ----------------- OLED ----------------- */
static constexpr int OLED_ADDR = 0x3C;
static constexpr int OLED_W = 128;
static constexpr int OLED_H = 64;
static constexpr uint32_t OLED_MIN_UPDATE_MS = 200;

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire);

/* ----------------- Timeouts ----------------- */
static constexpr uint32_t INTER_BYTE_TIMEOUT_MS = 180; // 300bpsでも余裕
static constexpr uint32_t TC_RESP_TIMEOUT_MS    = 1200;

/* ----------------- UART instances ----------------- */
HardwareSerial SerialPi(1);
HardwareSerial SerialTC(2);

/* ----------------- Queues ----------------- */
struct DisplayData {
  uint32_t seq;
  char line1[22];
  char line2[22];
  char line3[22];
  bool isError;
};

struct LogLine {
  char s[96];
};

static QueueHandle_t qDisplay = nullptr; // len=1 overwrite
static QueueHandle_t qLog     = nullptr;

/* ----------------- Shared ----------------- */
static volatile bool comm_busy = false;
static volatile uint32_t seq_counter = 0;

static inline uint32_t msNow(){ return (uint32_t)millis(); }

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

static void flushInput(HardwareSerial &ser){
  while(ser.available() > 0) (void)ser.read();
}

/* read bytes until a packet is parsed, or timeout */
static Packet* readPacketStream(HardwareSerial &ser,
                                uint8_t *ring,
                                uint8_t &widx,
                                uint32_t overallTimeoutMs,
                                uint32_t interByteTimeoutMs)
{
  uint32_t t_start = msNow();
  uint32_t t_last  = msNow();
  bool gotAny = false;

  while(true){
    while(ser.available() > 0){
      int v = ser.read();
      if(v < 0) break;

      ring[widx] = (uint8_t)v;
      widx = (uint8_t)((widx + 1) & (RING_SIZE - 1));
      gotAny = true;
      t_last = msNow();

      if (auto p = PacketFactory::tryParse(ring, widx)) {
        return p;
      }
    }

    uint32_t now = msNow();
    if ((now - t_start) > overallTimeoutMs) return nullptr;
    if (gotAny && (now - t_last) > interByteTimeoutMs) return nullptr;

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* ----------------- Tasks ----------------- */
void taskComm(void *pv){
  (void)pv;

  uint8_t ringPi[RING_SIZE]{};
  uint8_t ringTc[RING_SIZE]{};
  uint8_t wPi = 0;
  uint8_t wTc = 0;

  flushInput(SerialPi);
  flushInput(SerialTC);

  logf("[COMM] started core=%d", xPortGetCoreID());
  pushDisplay(false, "Boot", "Waiting Pi pkt", "");

  while(true){
    comm_busy = false;

    // 1) Piから「成立したパケット」を待つ（6/8/12自動）
    Packet* req = readPacketStream(SerialPi, ringPi, wPi,
                                   /*overall*/ 2000,
                                   /*inter*/   INTER_BYTE_TIMEOUT_MS);
    if(!req){
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    comm_busy = true;
    seq_counter++;

    char l2[22], l3[22];
    snprintf(l2, sizeof(l2), "Pi->TC %uB seq=%lu", req->len, (unsigned long)seq_counter);
    snprintf(l3, sizeof(l3), "%02X %02X %02X %02X %02X %02X",
             req->buf[0], req->buf[1], req->buf[2], req->buf[3], req->buf[4], req->buf[5]);
    pushDisplay(false, "TX: Pi -> TC", l2, l3);

    // 2) TCへ送信（flushしない）
    uint8_t out[FRAME_MAX];
    req->toBytes(out, false);
    size_t w = SerialTC.write(out, req->len);
    if(w != req->len){
      logf("[COMM] write TC short %u/%u", (unsigned)w, (unsigned)req->len);
      pushDisplay(true, "ERR: write TC", "short write", "");
      flushInput(SerialTC);
      comm_busy = false;
      continue;
    }

    // 3) TC応答パケットを待つ（こちらも tryParse で成立判定）
    Packet* resp = readPacketStream(SerialTC, ringTc, wTc,
                                    /*overall*/ TC_RESP_TIMEOUT_MS,
                                    /*inter*/   INTER_BYTE_TIMEOUT_MS);
    if(!resp){
      logf("[COMM] TC timeout seq=%lu", (unsigned long)seq_counter);
      pushDisplay(true, "ERR: TC timeout", "no resp pkt", "");
      flushInput(SerialTC);
      comm_busy = false;
      continue;
    }

    // 4) Piへ返送
    size_t wp = SerialPi.write(resp->buf, resp->len);
    if(wp != resp->len){
      logf("[COMM] write Pi short %u/%u", (unsigned)wp, (unsigned)resp->len);
      pushDisplay(true, "ERR: write Pi", "short write", "");
      comm_busy = false;
      continue;
    }

    logf("[COMM] OK seq=%lu req=%uB resp=%uB cmd=%u",
         (unsigned long)seq_counter, (unsigned)req->len, (unsigned)resp->len, (unsigned)resp->cmd());

    snprintf(l2, sizeof(l2), "TC->Pi %uB seq=%lu", resp->len, (unsigned long)seq_counter);
    snprintf(l3, sizeof(l3), "%02X %02X %02X %02X %02X %02X",
             resp->buf[0], resp->buf[1], resp->buf[2], resp->buf[3], resp->buf[4], resp->buf[5]);
    pushDisplay(false, "RX: TC -> Pi", l2, l3);

    comm_busy = false;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void taskUiLog(void *pv){
  (void)pv;

  bool oled_ok = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  uint32_t last_oled_ms = 0;

  if(oled_ok){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("TC Bridge Ready");
    display.println("UI on Core0");
    display.display();
  } else {
    Serial.println("[UI] OLED init failed (continue)");
  }

  logf("[UI] started core=%d", xPortGetCoreID());

  DisplayData d{};
  LogLine line{};

  while(true){
    // logs
    for(int i=0;i<10;i++){
      if(xQueueReceive(qLog, &line, 0) == pdTRUE) Serial.println(line.s);
      else break;
    }

    // OLED: latest only, throttled, and not during comm
    if(oled_ok){
      if(xQueuePeek(qDisplay, &d, 0) == pdTRUE){
        uint32_t now = msNow();
        if((now - last_oled_ms) >= OLED_MIN_UPDATE_MS){
          if(!comm_busy){
            (void)xQueueReceive(qDisplay, &d, 0);
            display.clearDisplay();
            display.setCursor(0,0);
            display.println(d.line1);
            display.println(d.line2);
            display.println(d.line3);
            display.println(d.isError ? "Status: ERR" : "Status: OK");
            display.display();
            last_oled_ms = now;
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/* ----------------- Setup / Loop ----------------- */
void setup(){
  Serial.begin(115200);
  delay(100);

  // Level shifter fixed (あなたの確定値)
  pinMode(PIN_OE1, OUTPUT);  digitalWrite(PIN_OE1, LOW);
  pinMode(PIN_DIR1, OUTPUT); digitalWrite(PIN_DIR1, HIGH);
  pinMode(PIN_OE2, OUTPUT);  digitalWrite(PIN_OE2, LOW);
  pinMode(PIN_DIR2, OUTPUT); digitalWrite(PIN_DIR2, LOW);

  SerialPi.begin(BAUD_PI, SERIAL_8N1, UART_PI_RX, UART_PI_TX);
  SerialTC.begin(BAUD_TC, SERIAL_8N1, UART_TC_RX, UART_TC_TX);

  qDisplay = xQueueCreate(1, sizeof(DisplayData));
  qLog     = xQueueCreate(64, sizeof(LogLine));

  pushDisplay(false, "Booting...", "Waiting packets", "");

  // 通信=Core1高優先度 / UI=Core0低優先度
  xTaskCreatePinnedToCore(taskComm,  "COMM", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(taskUiLog, "UI",   4096, nullptr, 1, nullptr, 0);

  Serial.println("[MAIN] System Initialized");
}

void loop(){
  vTaskDelay(portMAX_DELAY);
}
