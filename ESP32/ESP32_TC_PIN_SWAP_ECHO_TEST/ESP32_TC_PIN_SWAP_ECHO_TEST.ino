/**
 * TC Bridge: Raspberry Pi 側 疎通確認テスト (9600bps)
 * --------------------------------------------------
 * 【本番基板（SWAP配線）専用ピンアサイン】
 * Pi 側：TX=GPIO 44, RX=GPIO 43
 * TC 側：TX=GPIO 1,  RX=GPIO 2 (テスト中はアイドル維持)
 * * 役割：
 * 1. Pi から有効なパケット(チェックサム付)を受信
 * 2. 受信内容を Mac (Serial) に出力
 * 3. 同じ内容を Pi (SerialPi) に送り返す
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <tc_packet.hpp>

// =====================
// 0. 型定義 (コンパイルエラー対策)
// =====================
struct TcPacket {
  static constexpr int LEN_MAX = 12;
  uint8_t b[LEN_MAX];
  uint8_t len;

  void dumpTo(Print &p) const {
    for (int i = 0; i < len; i++) {
      if (b[i] < 0x10) p.print('0');
      p.print(b[i], HEX);
      if (i < len - 1) p.print(' ');
    }
  }
};

// =====================
// 1. ピンアサイン (本番基板 SWAP 配線)
// =====================

// 秋月レベルシフタ制御
static constexpr int PIN_LSHIFT_OE1  = 6;
static constexpr int PIN_LSHIFT_DIR1 = 5;
static constexpr int PIN_LSHIFT_OE2  = 7;
static constexpr int PIN_LSHIFT_DIR2 = 8;

// Pi側 (Raspberry Pi) 通信ピン: SWAP配線
static constexpr int PIN_PI_UART_RX = 43; // Pi TX -> ESP32 RX
static constexpr int PIN_PI_UART_TX = 44; // ESP32 TX -> Pi RX

// TC側 (Nano Every) 通信ピン (保護のため定義)
static constexpr int PIN_TC_TX = 1;
static constexpr int PIN_TC_RX = 2;

// =====================
// 2. 通信設定
// =====================
static constexpr uint32_t PI_BAUD = 9600;
HardwareSerial SerialPi(1); 

// FreeRTOS用キュー
static QueueHandle_t qEcho = nullptr;

// =====================
// 3. Pi 受信タスク (Core 0)
// =====================
static void taskPiRx(void *pv) {
  uint8_t ring[tc::RING_SIZE]{};
  uint8_t head = 0;
  uint64_t lastSig = 0;

  Serial.println("[Test] Pi Receive Task Started (Core 0).");

  while (true) {
    while (SerialPi.available() > 0) {
      ring[head] = (uint8_t)SerialPi.read();
      head = (uint8_t)((head + 1) & (tc::RING_SIZE - 1));

      // パケット解析 (チェックサムが正しいものだけを通す)
      if (const tc::Packet* p = tc::PacketFactory::tryParse(ring, head, lastSig)) {
        
        TcPacket echoPkt;
        echoPkt.len = p->len;
        memcpy(echoPkt.b, p->buf, p->len);

        // --- Mac へ表示 ---
        Serial.print("[Pi -> ESP] Received: ");
        echoPkt.dumpTo(Serial);
        Serial.println();

        // 送信タスクへ転送
        (void)xQueueSend(qEcho, &echoPkt, 0);
      }
    }
    vTaskDelay(1);
  }
}

// =====================
// 4. Pi 送信タスク (Core 0)
// =====================
static void taskPiTx(void *pv) {
  TcPacket p{};
  while (true) {
    if (xQueueReceive(qEcho, &p, portMAX_DELAY) == pdTRUE) {
      // --- Pi へ送り返す ---
      SerialPi.write(p.b, p.len);
      
      Serial.print("[ESP -> Pi] Echoed: ");
      p.dumpTo(Serial);
      Serial.println();
    }
  }
}

// =====================
// 5. 初期設定
// =====================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- [ESP32 Pi-side Pin Checker] ---");

  // レベルシフタ安全起動 (方向は固定)
  pinMode(PIN_LSHIFT_OE1, OUTPUT);  digitalWrite(PIN_LSHIFT_OE1, HIGH);
  pinMode(PIN_LSHIFT_OE2, OUTPUT);  digitalWrite(PIN_LSHIFT_OE2, HIGH);
  pinMode(PIN_LSHIFT_DIR1, OUTPUT); digitalWrite(PIN_LSHIFT_DIR1, HIGH); // A->B
  pinMode(PIN_LSHIFT_DIR2, OUTPUT); digitalWrite(PIN_LSHIFT_DIR2, LOW);  // B->A

  // TC側ピンの保護 (アイドルHIGH)
  pinMode(PIN_TC_TX, OUTPUT); digitalWrite(PIN_TC_TX, HIGH);
  pinMode(PIN_TC_RX, INPUT_PULLUP);

  // Pi側 UART開始
  SerialPi.begin(PI_BAUD, SERIAL_8N1, PIN_PI_UART_RX, PIN_PI_UART_TX);

  delay(100);
  digitalWrite(PIN_LSHIFT_OE1, LOW); // ゲート開放
  digitalWrite(PIN_LSHIFT_OE2, LOW);

  qEcho = xQueueCreate(10, sizeof(TcPacket));

  // Pi側タスクのみ起動 (Core 0)
  xTaskCreatePinnedToCore(taskPiRx, "PiRx", 4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(taskPiTx, "PiTx", 4096, nullptr, 3, nullptr, 0);

  Serial.printf("Ready. Please send packets from Pi. (Pins: RX=%d, TX=%d)\n", PIN_PI_UART_RX, PIN_PI_UART_TX);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}