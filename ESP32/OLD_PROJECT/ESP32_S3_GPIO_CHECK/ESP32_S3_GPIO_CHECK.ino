/* --------------------------------------------------
   ESP32-S3  GPIO3 (ADC1_CH2) アイドル電圧モニタ
   --------------------------------------------------
   - BITBANG_RX_PIN : GPIO3
   - 分解能         : 12bit (=4096 step)
   - 取得間隔       : 1 s
   - シリアル       : 115200 bps
   -------------------------------------------------- */

#include <Arduino.h>

#define BITBANG_RX_PIN  3        // TC → ESP32 の Rx ライン

void setup()
{
  Serial.begin(115200);
  delay(50);

  // --- ADC 準備 ---
  analogReadResolution(12);               // 0‥4095
  analogSetAttenuation(ADC_11db);         // 0-3.3 V フルスケール相当

  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);  // プルアップを有効にして実際の電位を確認

  Serial.println(F("=== RXB idle-voltage monitor ==="));
  Serial.println(F("RAW  = 0-4095 (12bit)"));
  Serial.println(F("mV   = analogReadMilliVolts()"));
  Serial.println();
}

void loop()
{
  uint16_t raw   = analogRead(BITBANG_RX_PIN);
  uint32_t milli = analogReadMilliVolts(BITBANG_RX_PIN);

  Serial.printf("RAW = %4u  \t≈ %4u mV\n", raw, milli);

  delay(1000);     // 1 s 周期で測定
}
