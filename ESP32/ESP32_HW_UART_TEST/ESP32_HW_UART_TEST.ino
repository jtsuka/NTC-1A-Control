#include <Arduino.h>

HardwareSerial SerialPi(1);  // UART1 ←→ Raspberry Pi
HardwareSerial SerialTC(2);  // UART2 ←→ TC側（300bps）

// =================== ピン定義 ===================
#define UART_PI_RX    44
#define UART_PI_TX    43
#define UART_TC_RX     3
#define UART_TC_TX     2

#define PIN_DIR1       6   // TX: ESP32 → TC, DIR1=HIGH
#define PIN_OE1        7   // OE1: TX出力許可
#define PIN_DIR2       8   // RX: TC → ESP32, DIR2=LOW
#define PIN_OE2        9   // OE2: RX出力許可

#define PACKET_LEN     6
#define TIMEOUT_MS     100

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("[BOOT] TC Repeater with HW UART + Level Shifter");

  // UART初期化
  SerialPi.begin(9600, SERIAL_8N1, UART_PI_RX, UART_PI_TX);
  SerialTC.begin(300, SERIAL_8N1, UART_TC_RX, UART_TC_TX);

  // DIRピン固定設定
  pinMode(PIN_DIR1, OUTPUT); digitalWrite(PIN_DIR1, HIGH);  // TX方向
  pinMode(PIN_DIR2, OUTPUT); digitalWrite(PIN_DIR2, LOW);   // RX方向

  // OEピン初期化（無効化状態から開始）
  pinMode(PIN_OE1, OUTPUT); digitalWrite(PIN_OE1, HIGH); // TX出力無効
  pinMode(PIN_OE2, OUTPUT); digitalWrite(PIN_OE2, HIGH); // RX出力無効
}

void loop() {
  static uint8_t piBuf[PACKET_LEN];
  static uint8_t tcBuf[PACKET_LEN];

  // Pi → TC 送信処理
  if (SerialPi.available() >= PACKET_LEN) {
    SerialPi.readBytes(piBuf, PACKET_LEN);

    Serial.print("[Pi → TC] ");
    for (int i = 0; i < PACKET_LEN; ++i) Serial.printf("%02X ", piBuf[i]);
    Serial.println();

    // OE1 有効化（TX出力をオン）
    digitalWrite(PIN_OE1, LOW);
    delayMicroseconds(10);  // 安定化のため
    SerialTC.write(piBuf, PACKET_LEN);
    SerialTC.flush();
    digitalWrite(PIN_OE1, HIGH);  // 出力無効に戻す
  }

  // TC → Pi 応答処理（受信待ち）
  if (SerialTC.available() > 0) {
    // OE2 有効化
    digitalWrite(PIN_OE2, LOW);
    delayMicroseconds(10);  // 安定化のため

    unsigned long start = millis();
    int received = 0;
    while (received < PACKET_LEN && millis() - start < TIMEOUT_MS) {
      if (SerialTC.available()) {
        tcBuf[received++] = SerialTC.read();
      }
    }
    digitalWrite(PIN_OE2, HIGH);  // 受信完了→OEオフ

    if (received == PACKET_LEN) {
      Serial.print("[TC → Pi] ");
      for (int i = 0; i < PACKET_LEN; ++i) Serial.printf("%02X ", tcBuf[i]);
      Serial.println();

      SerialPi.write(tcBuf, PACKET_LEN);
      SerialPi.flush();
    } else {
      Serial.println("[WARN] TC応答なし or 不完全");
    }
  }
}
