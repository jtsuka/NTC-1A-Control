/**********************************************************************
  TC Emulator – random telemetry version
  - SoftwareSerial : D2 (TX) / D3 (RX) , inverse logic = true
  - 300 bps / 8 N 1 , 6-byte frame identical to FW_TC106-1
    [0] PWM LSB2   (0-99)
    [1] PWM /100   (0-2)         ※PWM 0-255 なら 0-2
    [2] tens_temp  (ADC / 8 ⇒ 0-127)
    [3] tension 1’s digit
    [4] tension 10’s digit
    [5] 0x7F       (fixed trailer)
**********************************************************************/
#include <Arduino.h>
#include <SoftwareSerial.h>

/* ==== ユーザーが変えやすいレンジ定義 ============================ */
constexpr uint8_t  PWM_MIN        =   0;   // 0-255 の好きな範囲で
constexpr uint8_t  PWM_MAX        = 200;
constexpr uint16_t ADC_MIN        = 100;   // 0-1023 の好きな範囲で
constexpr uint16_t ADC_MAX        = 800;
constexpr uint8_t  KG_MIN         =  10;   // 表示張力 (kg) 0-99
constexpr uint8_t  KG_MAX         =  60;
constexpr uint32_t TX_INTERVAL_MS = 100;   // 100 ms 周期で送信
/* ================================================================ */

constexpr uint8_t  BB_TX_PIN = 2;
constexpr uint8_t  BB_RX_PIN = 3;
SoftwareSerial tcSerial(BB_RX_PIN, BB_TX_PIN, true);  // inverse logic

uint32_t t_last = 0;

/* --- 疑似フレームを組み立てる ------------------------------------ */
void buildRandomPacket(uint8_t pkt[6])
{
  /* ① PWM 0-255 --------------------------------------------------- */
  uint8_t pwm = random(PWM_MIN, PWM_MAX + 1);
  pkt[0] = pwm % 100;        // LSB2
  pkt[1] = pwm / 100;        // 0-2

  /* ② tens_temp: ADC 0-1023 を /8 --------------------------------- */
  uint16_t adc = random(ADC_MIN, ADC_MAX + 1);   // 0-1023
  pkt[2] = adc >> 3;         // /8 (0-127)

  /* ③ tension_actual (kg) 0-99 → 2 桁に分離 ----------------------- */
  uint8_t kg = random(KG_MIN, KG_MAX + 1);
  pkt[3] = kg % 10;          // 1 の位
  pkt[4] = kg / 10;          // 10 の位

  /* ④ 固定トレーラ ------------------------------------------------ */
  pkt[5] = 0x7F;             // 本家は加算 CS 再計算せず固定
}

/* ------------------------------------------------------------------ */
void setup()
{
  randomSeed(analogRead(A0));        // 適当な未接続 pin で乱数初期化
  tcSerial.begin(300);
  Serial.begin(115200);
  Serial.println(F("TC random-telemetry emulator started"));
}

void loop()
{
  /* 周期送信 ------------------------------------------------------- */
  if (millis() - t_last >= TX_INTERVAL_MS) {
    uint8_t pkt[6];
    buildRandomPacket(pkt);
    tcSerial.write(pkt, 6);          // ← TC / ESP32 へ

    /* デバッグモニタ出力（任意） */
    Serial.print(F("[TX] "));
    for (uint8_t b : pkt) {
      Serial.printf("%02X ", b);
    }
    Serial.println();

    t_last = millis();
  }

  /* ★受信→エコーも残したい場合はここで追加処理しても良い ------- */
}
