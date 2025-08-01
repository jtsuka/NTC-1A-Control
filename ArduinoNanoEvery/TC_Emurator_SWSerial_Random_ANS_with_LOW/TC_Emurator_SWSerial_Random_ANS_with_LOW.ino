/**********************************************************************
  TC Emulator – random telemetry version  (printf → print 置換済み)
**********************************************************************/
#include <Arduino.h>
#include <SoftwareSerial.h>

/* ==== パラメータ ================================================== */
constexpr uint8_t  PWM_MIN        =   0;
constexpr uint8_t  PWM_MAX        = 200;
constexpr uint16_t ADC_MIN        = 100;
constexpr uint16_t ADC_MAX        = 800;
constexpr uint8_t  KG_MIN         =  10;
constexpr uint8_t  KG_MAX         =  60;
constexpr uint32_t TX_INTERVAL_MS = 100;
/* ================================================================== */

constexpr uint8_t  BB_TX_PIN = 2;
constexpr uint8_t  BB_RX_PIN = 3;
SoftwareSerial tcSerial(BB_RX_PIN, BB_TX_PIN, true);  // inverse logic

uint32_t t_last = 0;

/* --- 疑似 6B フレームを作成 --------------------------------------- */
void buildRandomPacket(uint8_t pkt[6])
{
  uint8_t pwm = random(PWM_MIN, PWM_MAX + 1);
  pkt[0] = pwm % 100;
  pkt[1] = pwm / 100;

  uint16_t adc = random(ADC_MIN, ADC_MAX + 1);
  pkt[2] = adc >> 3;

  uint8_t kg = random(KG_MIN, KG_MAX + 1);
  pkt[3] = kg % 10;
  pkt[4] = kg / 10;

  pkt[5] = 0x7F;
}

/* --- 16 進を 00～FF で表示するユーティリティ ---------------------- */
void printHexByte(uint8_t b)
{
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

void setup()
{
  randomSeed(analogRead(A0));
  tcSerial.begin(300);
  Serial.begin(115200);
  Serial.println(F("TC random-telemetry emulator started"));
}

void loop()
{
  if (millis() - t_last >= TX_INTERVAL_MS) {
    uint8_t pkt[6];
    buildRandomPacket(pkt);
    tcSerial.write(pkt, 6);

    /* デバッグモニタ出力 */
    Serial.print(F("[TX] "));
    for (uint8_t b : pkt) {
      printHexByte(b);
      Serial.print(' ');
    }
    Serial.println();

    t_last = millis();
  }
}
