/**********************************************************************
  BitBang UART + キュー + 擬似スレッド + MSB/LSB切替 完全スケッチ
  - UART: Serial (ハードウェア)
  - BitBang通信: D2 (TX), D3 (RX)
**********************************************************************/

#define BITBANG_TX_PIN 2
#define BITBANG_RX_PIN 3
#define QUEUE_SIZE 32
#define UART_BAUD 1200
#define USE_MSB_FIRST  // ← MSB/LSB切替はこちらで制御

uint8_t txQueue[QUEUE_SIZE];
volatile uint8_t txHead = 0;
volatile uint8_t txTail = 0;

// UARTキュー処理
bool enqueue(uint8_t val) {
  uint8_t next = (txHead + 1) % QUEUE_SIZE;
  if (next == txTail) return false; // full
  txQueue[txHead] = val;
  txHead = next;
  return true;
}

bool dequeue(uint8_t &val) {
  if (txHead == txTail) return false; // empty
  val = txQueue[txTail];
  txTail = (txTail + 1) % QUEUE_SIZE;
  return true;
}

// BitBang TX (1byte)
void bitbangWrite(uint8_t b) {
  digitalWrite(BITBANG_TX_PIN, LOW);
  delayMicroseconds(3333);  // 300bps相当

  for (int i = 0; i < 8; i++) {
#ifdef USE_MSB_FIRST
    digitalWrite(BITBANG_TX_PIN, (b & (0x80 >> i)) ? HIGH : LOW);
#else
    digitalWrite(BITBANG_TX_PIN, (b & (1 << i)) ? HIGH : LOW);
#endif
    delayMicroseconds(3333);
  }

  digitalWrite(BITBANG_TX_PIN, HIGH);
  delayMicroseconds(3333);
}

// BitBang RX (1byte) - 同期簡易版
bool bitbangRead(uint8_t &b) {
  if (digitalRead(BITBANG_RX_PIN) == LOW) {
    delayMicroseconds(1666); // Start bit中心へ
    b = 0;
    for (int i = 0; i < 8; i++) {
      delayMicroseconds(3333);
#ifdef USE_MSB_FIRST
      b |= digitalRead(BITBANG_RX_PIN) << (7 - i);
#else
      b |= digitalRead(BITBANG_RX_PIN) << i;
#endif
    }
    delayMicroseconds(3333); // Stop bit
    return true;
  }
  return false;
}

void setup() {
  pinMode(BITBANG_TX_PIN, OUTPUT);
  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);
  digitalWrite(BITBANG_TX_PIN, HIGH);

  Serial.begin(UART_BAUD);
  while (!Serial);

  Serial.println(F("[INIT] BitBang+UART 擬似スレッド開始"));
}

void loop() {
  // UART受信→キュー格納
  if (Serial.available()) {
    uint8_t c = Serial.read();
    enqueue(c);
  }

  // キューから取り出しBitBang送信
  uint8_t outByte;
  if (dequeue(outByte)) {
    bitbangWrite(outByte);
    Serial.print(F("[TX→TC] ")); Serial.println(outByte, HEX);
  }

  // BitBang受信
  uint8_t recvByte;
  if (bitbangRead(recvByte)) {
    Serial.print(F("[←RX TC] ")); Serial.println(recvByte, HEX);
  }
}
