#include <Arduino.h>

static const uint32_t BAUD_USB = 115200;
static const uint32_t BAUD_TC  = 300;

static void printPinInfo() {
  Serial.println("=== Nano Every: Serial1 Pin Check ===");

#ifdef PIN_SERIAL1_RX
  Serial.print("PIN_SERIAL1_RX = ");
  Serial.println(PIN_SERIAL1_RX);
#else
  Serial.println("PIN_SERIAL1_RX is not defined in this core.");
#endif

#ifdef PIN_SERIAL1_TX
  Serial.print("PIN_SERIAL1_TX = ");
  Serial.println(PIN_SERIAL1_TX);
#else
  Serial.println("PIN_SERIAL1_TX is not defined in this core.");
#endif

  // RXピンがアイドルHIGHになっているか（UARTは通常アイドルHIGH）
#ifdef PIN_SERIAL1_RX
  pinMode(PIN_SERIAL1_RX, INPUT);
  delay(50);
  Serial.print("digitalRead(PIN_SERIAL1_RX) idle = ");
  Serial.println(digitalRead(PIN_SERIAL1_RX)); // 1が期待
#endif

  Serial.println("Start Serial1 @300bps...");
  Serial1.begin(BAUD_TC);
  Serial.println("Ready. Send bytes to Serial1 RX and watch HEX.");
}

void setup() {
  Serial.begin(BAUD_USB);
  while (!Serial) {}
  delay(50);

  printPinInfo();
}

void loop() {
  if (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();

    static uint32_t last_us = 0;
    uint32_t now_us = micros();
    uint32_t dt = (last_us == 0) ? 0 : (now_us - last_us);
    last_us = now_us;

    Serial.print("dt(us)=");
    Serial.print(dt);
    Serial.print("  b=0x");
    if (b < 0x10) Serial.print('0');
    Serial.println(b, HEX);
  }
}
