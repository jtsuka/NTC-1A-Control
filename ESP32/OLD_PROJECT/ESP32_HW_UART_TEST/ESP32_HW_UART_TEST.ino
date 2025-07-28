#include <Arduino.h>

// UART 定義
#define UART_PI_RX    44    // (黄)RX Pi->ESP32
#define UART_PI_TX    43    // (白)TX ESP32->Pi
#define UART_TC_RX     3    // (黄)RX TC->ESP32
#define UART_TC_TX     2    // (白)TX ESP32->TC

#define PACKET_LEN     6
#define TIMEOUT_MS    100

HardwareSerial SerialPi(1); // UART1 ←→ Pi
HardwareSerial SerialTC(2); // UART2 ←→ TC

// LVC16T245 制御ピン
#define PIN_OE1        5   // (白)TX 出力有効 → 常時 LOW
#define PIN_DIR1       6   // (黄)TX方向：ESP32 → TC → HIGH 固定
#define PIN_OE2        7   // (白)RX 出力有効 → 常時 LOW
#define PIN_DIR2       8   // (黄)RX方向：TC → ESP32 → LOW 固定

// ---- 1バイトをビット単位で左右反転 ----
static inline uint8_t rev8(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("[BOOT] TC Repeater: Pi_RX = 無限待ちモード");

  SerialPi.begin(9600, SERIAL_8N1, UART_PI_RX, UART_PI_TX);
  Serial.println("Initializing SerialTC...");
  SerialTC.begin(300, SERIAL_8N1, UART_TC_RX, UART_TC_TX);
  Serial.println("SerialTC initialized.");

  pinMode(PIN_DIR1, OUTPUT); digitalWrite(PIN_DIR1, HIGH);
  pinMode(PIN_DIR2, OUTPUT); digitalWrite(PIN_DIR2, LOW);
  pinMode(PIN_OE1, OUTPUT); digitalWrite(PIN_OE1, LOW);
  pinMode(PIN_OE2, OUTPUT); digitalWrite(PIN_OE2, LOW);
}

void loop() {
  static uint8_t piBuf[PACKET_LEN];
  static uint8_t tcBuf[PACKET_LEN];

  // ◆ Pi → TC：受信を無限ループで待機
  while (true) {
    if (SerialPi.available() >= PACKET_LEN) {
      SerialPi.readBytes(piBuf, PACKET_LEN);
      break;
    }
    // 必要に応じて少し delay（例：1ms）を入れてCPU負荷軽減
    delay(1);
  }

  Serial.print("[Pi → TC] ");
  for (int i = 0; i < PACKET_LEN; ++i) Serial.printf("%02X ", piBuf[i]);
  Serial.println();

  int result = SerialTC.write(piBuf, PACKET_LEN);
  Serial.printf("[DEBUG] SerialTC.write() result: %d\n", result);
  SerialTC.flush();

  // ◆ TC → Pi：応答待機（タイムアウト付き）
  unsigned long start = millis();
  int received = 0;
  while (received < PACKET_LEN && millis() - start < TIMEOUT_MS) {
    if (SerialTC.available()) {
      tcBuf[received++] = SerialTC.read();
    }
  }

  if (received == PACKET_LEN) {
    Serial.print("[TC → Pi] ");
    for (int i = 0; i < PACKET_LEN; ++i) Serial.printf("%02X ", tcBuf[i]);
    Serial.println();
    // --- TC から PACKET_LEN バイト取り終えた直後 ---
    uint8_t rev_pkt[PACKET_LEN];
    for (int i = 0; i < PACKET_LEN; i++) rev_pkt[i] = rev8(tcBuf[i]);
    Serial1.write(rev_pkt, PACKET_LEN);   // Pi へ送信（GPIO43）

    SerialPi.flush();
  } else {
    Serial.println("[WARN] TC応答なし or 不完全");
  }

  // このループを繰り返して next packet を待ち受け
}
