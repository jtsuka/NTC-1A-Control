#include <Arduino.h>
#include "tc_packet.hpp"

/*
  TC Emulator Fusion Final V1.2.5
  - ESP32 v1.2.5 とタイミングを同期 (3334us)
  - Nano Every でのコンパイルエラー(printf)を修正
*/

#define PIN_TX 2
#define PIN_RX 3
#define TC_INVERT_LOGIC true
#define ECHO_REPLY true

// 300bps: 3334us に設定 (ESP32側と同期)
#define BIT_US 3334 

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
  for (int i = 0; i < 8; i++) tcWrite((d >> i) & 0x01);
  tcWrite(isCmd); // 9bit
  tcWrite(true);  // Stop
  tcWrite(true);  // Gap
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RX, INPUT_PULLUP);
  pinMode(PIN_TX, OUTPUT);
  
  // 初期電位：Idle HIGH (反転時は物理LOW)
  digitalWrite(PIN_TX, TC_INVERT_LOGIC ? LOW : HIGH);
  
  Serial.println(F("\n--- [Nano TC Emulator v1.2.5-Final] ---"));
  
  // printfエラー回避のための分割出力
  Serial.print(F("BIT_US: "));   Serial.print(BIT_US);
  Serial.print(F(", Invert: ")); Serial.print(TC_INVERT_LOGIC ? "ON" : "OFF");
  Serial.print(F(", Echo: "));   Serial.println(ECHO_REPLY ? "ON" : "OFF");
  
  Serial.println(F("Ready to monitor Pi -> ESP32 -> TC chain."));
}

void loop() {
  static uint8_t data[tc::TC_DATA_LEN];
  static uint8_t di = 0;
  
  if (tcRead() == false) { 
    delayMicroseconds(BIT_US / 2); // 半ビット待つ
    
    if (tcRead() == true) return; // デグリッチ
    
    delayMicroseconds(BIT_US); // データ中央へ
    
    uint8_t d = 0;
    for (int i = 0; i < 8; i++) {
      if (tcRead()) d |= (1 << i);
      delayMicroseconds(BIT_US);
    }
    bool isCmd = tcRead(); 
    
    if (!isCmd) {
      if (di < tc::TC_DATA_LEN) {
        data[di++] = d;
        Serial.print(d, HEX); Serial.print(" ");
      }
    } else {
      Serial.print(F("\n[COMMAND] 0x")); Serial.println(d, HEX);
      
      if (ECHO_REPLY && di == tc::TC_DATA_LEN) {
        for (int i = 0; i < 3; i++) tcWrite(true); 
        for (int i = 0; i < tc::TC_DATA_LEN; i++) send9bit(data[i], false);
        send9bit(d, true); 
        Serial.println(F("[ECHO] Sent response to ESP32."));
      }
      di = 0; 
    }
    
    uint32_t t0 = micros();
    while(tcRead() == false && (micros() - t0) < (BIT_US * 2));
  }
}