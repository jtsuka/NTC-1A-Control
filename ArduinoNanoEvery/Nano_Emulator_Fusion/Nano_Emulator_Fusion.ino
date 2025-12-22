#include <Arduino.h>
#include "tc_packet.hpp"

/*
  TC Emulator Fusion Final V1.2.1
*/

#define PIN_TX 2
#define PIN_RX 3
#define TC_INVERT_LOGIC true
#define BIT_US 3333
#define ECHO_REPLY true

void tcWrite(bool logical) {
  bool out = TC_INVERT_LOGIC ? !logical : logical;
  digitalWrite(PIN_TX, out ? HIGH : LOW);
  delayMicroseconds(BIT_US);
}

bool tcRead() {
  bool val = (digitalRead(PIN_RX) == HIGH);
  return TC_INVERT_LOGIC ? !val : val;
}

void send9bit(uint8_t d, bool isCmd) {
  tcWrite(false); // Start
  for (int i=0; i<8; i++) tcWrite((d >> i) & 0x01);
  tcWrite(isCmd); // 9th
  tcWrite(true);  // Stop
  tcWrite(true);  // Gap
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RX, INPUT_PULLUP);
  pinMode(PIN_TX, OUTPUT);
  
  // 初期電位：Idle HIGH
  digitalWrite(PIN_TX, TC_INVERT_LOGIC ? LOW : HIGH);
  
  Serial.println("Nano TC Emulator Fusion v1.2.1 READY.");
}

void loop() {
  static uint8_t data[tc::TC_DATA_LEN];
  static uint8_t di = 0;
  
  if (tcRead() == false) { // 1. スタートビット(論理LOW)検出
    delayMicroseconds(BIT_US / 2); // 2. 半ビット待つ
    
    if (tcRead() == true) return; // 3. デグリッチ：ノイズなら捨てる
    
    delayMicroseconds(BIT_US); // 4. データ1ビット目の中央へ
    
    uint8_t d = 0;
    for (int i=0; i<8; i++) {
      if (tcRead()) d |= (1 << i);
      delayMicroseconds(BIT_US);
    }
    bool isCmd = tcRead();
    
    if (!isCmd) {
      if (di < tc::TC_DATA_LEN) data[di++] = d;
      Serial.print(d, HEX); Serial.print(" ");
    } else {
      Serial.print("\n[CMD] 0x"); Serial.println(d, HEX);
      if (ECHO_REPLY && di == tc::TC_DATA_LEN) {
        for (int i=0; i<3; i++) tcWrite(true); 
        for (int i=0; i<tc::TC_DATA_LEN; i++) send9bit(data[i], false);
        send9bit(d, true); 
        Serial.println("[ECHO] Sent.");
      }
      di = 0; 
    }
    // Stop待ちタイムアウト (フリーズ防止)
    uint32_t t0 = micros();
    while(tcRead() == false && (micros()-t0) < (BIT_US * 2));
  }
}