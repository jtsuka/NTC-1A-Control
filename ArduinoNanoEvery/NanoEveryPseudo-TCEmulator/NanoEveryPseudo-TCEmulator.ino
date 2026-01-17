/**
 * Nano Every Pseudo-TC Emulator v1.1.1 (修正版)
 * --------------------------------------------------
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <tc_packet_fixed.hpp> 

SoftwareSerial tcSerial(10, 11); 

static const uint32_t BAUD_TC = 300;

void make_reply(const uint8_t rx[tc::Fixed6::LEN], uint8_t tx[tc::Fixed6::LEN]) {
  tx[0] = 0xF1;         
  tx[1] = rx[1];        
  tx[2] = 0x00;         
  tx[3] = 0x04;         
  tx[4] = 0x07;         
  tx[5] = tc::checksum7(tx, 5); 
}

void setup() {
  Serial.begin(115200);
  tcSerial.begin(BAUD_TC);
  Serial.println(F("--- Nano Pseudo-TC v1.1.1 Ready ---"));
}

void loop() {
  static uint8_t buf[tc::Fixed6::LEN];
  static uint8_t idx = 0;
  static uint32_t lastByteTime = 0;

  while (tcSerial.available()) {
    buf[idx++] = (uint8_t)tcSerial.read();
    lastByteTime = millis();

    if (idx >= tc::Fixed6::LEN) {
      uint8_t reply[tc::Fixed6::LEN];
      make_reply(buf, reply);

      delay(2); 
      
      tcSerial.write(reply, tc::Fixed6::LEN);
      idx = 0;

      Serial.print(F("[Pseudo-TC TX] "));
      for(int i=0; i<tc::Fixed6::LEN; i++) {
        if(reply[i]<0x10) Serial.print('0');
        Serial.print(reply[i], HEX); Serial.print(" ");
      }
      Serial.println();
    }
  }

  if (idx > 0 && (millis() - lastByteTime > 100)) {
    idx = 0;
  }
}