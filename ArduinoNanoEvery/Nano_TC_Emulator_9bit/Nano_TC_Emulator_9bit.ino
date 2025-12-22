#include <Arduino.h>
#include "tc_packet.hpp"

#define PIN_RX 3
#define BIT_US 3333
#define INVERT_LOGIC true

uint8_t ring[tc::RING_SIZE];
uint8_t head = 0;

bool tcRead() {
    bool val = (digitalRead(PIN_RX) == HIGH);
    return INVERT_LOGIC ? !val : val; // 論理レベル読み取り
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_RX, INPUT_PULLUP);
    Serial.println("Nano Every 9-bit Emulator Ready.");
}

void loop() {
    if (tcRead() == false) { // Start検出
        delayMicroseconds(BIT_US + BIT_US/2); // データ中央へ
        
        uint8_t data = 0;
        for (int i=0; i<8; i++) {
            if (tcRead()) data |= (1 << i);
            delayMicroseconds(BIT_US);
        }
        
        bool isCmd = tcRead(); // 9th bit
        
        if (!isCmd) {
            ring[head] = data;
            head = (head + 1) % tc::RING_SIZE;
            if (auto p = tc::PacketFactory::tryParse(ring, head)) {
                Serial.print("[PKT RECV] CMD ID: "); Serial.println(p->cmd());
            }
        } else {
            Serial.print("[CMD FRAME] 0x"); Serial.println(data, HEX);
        }
        while(tcRead() == false); // Stop待ち
    }
}