/*********************************************************************
 TCエミュレーター（ESP32-S3用）+ OLED_DisplayManager クラス使用版
 - BitBang通信 (GPI43: TX, GPI44: RX)
 - OLED表示 (I2C)
 - スリープモード (GPIO8 = LOW)
 - FreeRTOSタスク + OLED表示クラス使用
  - 2025.07.01 OLEDログ表示クラス付き スケッチ統合版
*********************************************************************/


#include <Wire.h>
#include "OLED_DisplayManager.h"

#define BB_TX 43
#define BB_RX 44
#define SLEEP_MODE_PIN 8
#define PACKET_SIZE 6
#define BAUD_RATE 300
#define BIT_DELAY_US (1000000UL / BAUD_RATE)
#define HALF_DELAY_US (BIT_DELAY_US / 2)
#define BYTE_GAP_US 800

OLED_DisplayManager oled(128, 64);

uint8_t rx_buf[PACKET_SIZE];
uint8_t tx_buf[PACKET_SIZE];

bool receive_packet(uint8_t* buf) {
  unsigned long t_start = millis();
  for (int i = 0; i < PACKET_SIZE; i++) {
    while (digitalRead(BB_RX) == HIGH) {
      if (millis() - t_start > 1000) return false;
    }
    delayMicroseconds(HALF_DELAY_US);
    if (digitalRead(BB_RX) != LOW) return false;

    uint8_t val = 0;
    for (int b = 0; b < 8; b++) {
      delayMicroseconds(BIT_DELAY_US);
      val |= (digitalRead(BB_RX) << b);
    }
    buf[i] = val;
    delayMicroseconds(BYTE_GAP_US);
  }
  return true;
}

void send_byte(uint8_t val) {
  noInterrupts();
  digitalWrite(BB_TX, LOW);
  delayMicroseconds(BIT_DELAY_US);
  for (int i = 0; i < 8; i++) {
    digitalWrite(BB_TX, (val >> i) & 0x01);
    delayMicroseconds(BIT_DELAY_US);
  }
  digitalWrite(BB_TX, HIGH);
  delayMicroseconds(BIT_DELAY_US);
  interrupts();
}

void send_packet(const uint8_t* buf) {
  for (int i = 0; i < PACKET_SIZE; i++) {
    send_byte(buf[i]);
    delayMicroseconds(BYTE_GAP_US);
  }
}

void task_main(void* arg) {
  for (;;) {
//    if (digitalRead(SLEEP_MODE_PIN) == LOW) {
//      oled.clearAll();
//      oled.printMessage("== SLEEP MODE ==", 0);
//      vTaskDelay(pdMS_TO_TICKS(1000));
//      continue;
//    }

    if (receive_packet(rx_buf)) {
      oled.printHexLine("RECV", rx_buf, 0);
      delayMicroseconds(BIT_DELAY_US * 4);
      memcpy(tx_buf, rx_buf, PACKET_SIZE);
      send_packet(tx_buf);
      oled.printHexLine("SEND", tx_buf, 2);
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void setup() {
  pinMode(BB_RX, INPUT_PULLUP);
  pinMode(BB_TX, OUTPUT);
  digitalWrite(BB_TX, HIGH);
  pinMode(SLEEP_MODE_PIN, INPUT_PULLUP);

  Wire.begin();
  oled.begin();
  oled.printMessage("TC Emulator Ready", 0);

  xTaskCreatePinnedToCore(task_main, "TCMain", 4096, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(100));
}
