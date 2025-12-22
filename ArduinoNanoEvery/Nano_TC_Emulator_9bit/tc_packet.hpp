#include <Arduino.h>
#include "tc_packet.hpp"

#define PIN_TX 2
#define PIN_RX 3
#define TC_INVERT_LOGIC true
#define BIT_US 3333

bool tcRead() {
  bool val = (digitalRead(PIN_RX) == HIGH);
  return TC_INVERT_LOGIC ? !val : val;
}

void tcWrite(bool logical) {
  bool out = TC_INVERT_LOGIC ? !logical : logical;
  digitalWrite(PIN_TX, out ? HIGH : LOW);
  delayMicroseconds(BIT_US);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RX, INPUT_PULLUP);
  pinMode(PIN_TX, OUTPUT);
  tcWrite(true);
  Serial.println("Nano TC Emulator Fusion READY.");
}

void loop() {
  static uint8_t data[5];
  static uint8_t di = 0;
  
  if (tcRead() == false) { // Start検出
    delayMicroseconds(BIT_US + BIT_US/2); // データ中央へ
    uint8_t d = 0;
    for (int i=0; i<8; i++) {
      if (tcRead()) d |= (1 << i);
      delayMicroseconds(BIT_US);
    }
    bool isCmd = tcRead();
    
    if (!isCmd) {
      if (di < 5) data[di++] = d;
      Serial.print(d, HEX); Serial.print(" ");
    } else {
      Serial.print("\n[CMD] 0x"); Serial.println(d, HEX);
      di = 0; // 次のパケットへ
    }
    // ストップビット待ちタイムアウト
    uint32_t t0 = micros();
    while(tcRead() == false && (micros()-t0) < (BIT_US * 2));
  }
}