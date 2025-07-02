#include <esp_system.h>
#include "esp_system.h"   // ← esp_read_mac() の宣言
#include "esp_mac.h"      // ← ESP_MAC_WIFI_STA の定義

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(500);  // 起動直後の安定化

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);  // 直接読み出し（工場設定済）

  char buf[18];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.println("=== ESP32-S3 MAC Address ===");
  Serial.println(buf);
  Serial.println("============================");
}
void loop() {}
