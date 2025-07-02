#include <Wire.h>

#define LED_PIN 21  // XIAO ESP32S3 の内蔵LED

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Wire.begin(5, 6);  // Grove I2CポートのSDA/SCL
  Serial.begin(115200);
  while (!Serial);

  Serial.println("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found address: 0x");
      Serial.println(i, HEX);
      count++;
      delay(5);
    }
  }

  Serial.print("Done. Found ");
  Serial.print(count, DEC);
  Serial.println(" device(s).");
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}
