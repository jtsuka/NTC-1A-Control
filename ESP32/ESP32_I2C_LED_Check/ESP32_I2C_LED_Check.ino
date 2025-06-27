#include <Wire.h>

void setup() {
  Wire.begin(6,5);  // SDA=5, SCL=4
  Serial.begin(115200);
  while (!Serial);

  Serial.println("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found address: ");
      Serial.print(i, HEX);
      Serial.println();
      count++;
      delay(5);
    }
  }
  Serial.print("Done. Found ");
  Serial.print(count, DEC);
  Serial.println(" device(s).");
}

void loop() {}
