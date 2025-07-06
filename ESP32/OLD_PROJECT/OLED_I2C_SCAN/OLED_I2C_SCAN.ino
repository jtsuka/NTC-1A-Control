#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("I2C Scanner Start");

  Wire.begin(4, 5);  // Grove J5用に明示的に指定

  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
    }
  }

  Serial.println("Scan done.");
}

void loop() {}
