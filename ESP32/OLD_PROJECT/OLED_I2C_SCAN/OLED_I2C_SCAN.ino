#include <Wire.h>

void setup() {
  Wire.begin(4, 5); // SDA=GPIO4, SCL=GPIO5
  Serial.begin(115200);
  delay(1000);
  Serial.println("I2C Scanner Start");

  for (byte address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(address, HEX);
    }
  }

  Serial.println("Scan done.");
}

void loop() {
}
