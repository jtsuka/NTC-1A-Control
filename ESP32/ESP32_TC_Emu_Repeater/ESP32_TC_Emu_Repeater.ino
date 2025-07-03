// =========================================
// ESP32 TCエミュレーター＆リピーター統合スケッチ（完全版）
// - FreeRTOSベース
// - OLED排他制御付き表示
// - UART（Pi接続）<-> BitBang（TC接続）中継
// - MACアドレスによる自動モード切替
// - GPIO8=HIGH時にテストパケット送信（リピーターモードのみ）
// - シリアルとOLEDにログ表示
// - 対応機種：XIAO ESP32S3 + Grove Shield
// - 日付 2025.07.02 バイナリ統合版
// -   Update TCエミュレーターモードのシリアルデバックログ
// =========================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ========== OLED ==========
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SemaphoreHandle_t oledMutex;

// ========== モード定義 ==========
#define MODE_REPEATER 1
#define MODE_EMULATOR 2
uint8_t current_mode = 0;
// ========= ログレベル定義 =========
#define ENABLE_ALIVE_LOG 0              // デフォルトはAliveログのみ
#define LOG_MODE_PIN 10                 // GPIO10をスイッチに使用
#define ALIVE_TIME 3000                 // 死活確認時間 3秒
bool enableVerboseLog = false;          // 詳細ログ有効フラグ
unsigned long lastAlive = 0;            // タスク死活フラグ
unsigned long lastAliveSend = 0;        // タスク死活フラグ


// ===== テストモード制御用グローバル変数 =====
bool testMode = false;
bool lastTestPinState = LOW;
unsigned long lastSendTime = 0;
unsigned long lastBlinkTime = 0;
bool ledState = false;

// ======== エミュレータ用キューと定数 ========
QueueHandle_t echoQueue;
#define ECHO_QUEUE_LENGTH 4
#define ECHO_PACKET_SIZE  6

// ========== MACアドレス定義 ==========
const uint8_t REPEATER_MAC[6] = {0x98, 0x3D, 0xAE, 0x60, 0x55, 0x1C};
const uint8_t EMULATOR_MAC[6] = {0x8C, 0xBF, 0xEA, 0x8E, 0x57, 0xF4};

// ========== ピン定義 ==========
#define PI_UART_TX_PIN 43
#define PI_UART_RX_PIN 44
#define TC_UART_TX_PIN 1
#define TC_UART_RX_PIN 0
#define TEST_PIN 8
#define LED_PIN        21  // 内蔵LED

// ========== 通信定義 ==========
#define UART_BAUDRATE 9600
#define BITBANG_BPS 300
#define BIT_DELAY_US (1000000 / BITBANG_BPS)

// ========== テストパケット ==========
const uint8_t testPacket[6] = {0x01, 0x06, 0x05, 0x00, 0x00, 0x0C};

// ========== OLEDログ関数 ==========
void logToOLED(const String& upper, const String& lower) {
  if (xSemaphoreTake(oledMutex, portMAX_DELAY) == pdTRUE) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println(upper);   // 上段（RECV）
    display.println(lower);   // 下段（SEND）
    display.display();
    xSemaphoreGive(oledMutex);
  }
  Serial.println(upper + " | " + lower);
}
#if 0
void logToOLED(const String& line1, const String& line2) {
  if (xSemaphoreTake(oledMutex, portMAX_DELAY) == pdTRUE) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println(line1);
    display.println(line2);
    display.display();
    xSemaphoreGive(oledMutex);
  }
  Serial.println(line1 + " | " + line2);
}
#endif

// ========== BitBang送信 ==========
void bitbangSendByte(uint8_t b) {
  digitalWrite(TC_UART_TX_PIN, LOW); delayMicroseconds(BIT_DELAY_US); // Start
  for (int i = 0; i < 8; i++) {
    digitalWrite(TC_UART_TX_PIN, (b >> i) & 1);
    delayMicroseconds(BIT_DELAY_US);
  }
  digitalWrite(TC_UART_TX_PIN, HIGH); delayMicroseconds(BIT_DELAY_US); // Stop
}

void bitbangSendPacket(const uint8_t* packet, int len) {
  for (int i = 0; i < len; i++) bitbangSendByte(packet[i]);
}

// ========== BitBang受信 ==========
uint8_t bitbangReceiveByte() {
  while (digitalRead(TC_UART_RX_PIN) == HIGH); // Wait for start bit
  delayMicroseconds(BIT_DELAY_US + BIT_DELAY_US/2);
  uint8_t b = 0;
  for (int i = 0; i < 8; i++) {
    b |= (digitalRead(TC_UART_RX_PIN) << i);
    delayMicroseconds(BIT_DELAY_US);
  }
  return b;
}

void bitbangReceivePacket(uint8_t* buf, int len) {
  for (int i = 0; i < len; i++) buf[i] = bitbangReceiveByte();
}

// ========== テストパケット送信 ==========
void sendTestPacket() {
  bitbangSendPacket(testPacket, 6);
  logToOLED("Test Packet", "Sent via BitBang");
}

// ========== リピーター：Pi->TC ==========
void uartToBitbangTask(void* pv) {
  while (1) {
    if (Serial2.available() >= 6) {
      uint8_t buf[6];
      Serial2.readBytes(buf, 6);
      bitbangSendPacket(buf, 6);
      logToOLED("UART->BitBang", String("SENT: ") + String(buf[0], HEX));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ========== リピーター：TC->Pi ==========
void bitbangToUartTask(void* pv) {
  while (1) {
    if (digitalRead(TC_UART_RX_PIN) == LOW) {
      uint8_t buf[6];
      bitbangReceivePacket(buf, 6);
      Serial2.write(buf, 6);
      logToOLED("BitBang->UART", String("RECV: ") + String(buf[0], HEX));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ========== エミュレーター受信:Rep->Me ==========
void emulatorReceiverTask(void* pv) {
  while (1) {
    if (digitalRead(TC_UART_RX_PIN) == LOW) {
      uint8_t buf[ECHO_PACKET_SIZE];
      bitbangReceivePacket(buf, ECHO_PACKET_SIZE);
      xQueueSend(echoQueue, buf, portMAX_DELAY);

      if (enableVerboseLog) {
        String msg = "RECV: ";
        for (int i = 0; i < 6; i++) msg += String(buf[i], HEX) + " ";
        logToOLED(msg, "→ Queued for echo");
      }
    }
#if ENABLE_ALIVE_LOG
    if (millis() - lastAlive > ALIVE_TIME) {
      Serial.println("[EmuRecv] Alive");
      lastAlive = millis();
    }
#endif
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ========== エミュレーター送信:Me->Rep ==========
void emulatorSenderTask(void* pv) {
  uint8_t buf[ECHO_PACKET_SIZE];
  while (1) {
    if (xQueueReceive(echoQueue, buf, portMAX_DELAY) == pdTRUE) {
      bitbangSendPacket(buf, ECHO_PACKET_SIZE);

      if (enableVerboseLog) {
        String msg = "SEND: ";
        for (int i = 0; i < 6; i++) msg += String(buf[i], HEX) + " ";
        logToOLED("Echoed Back", msg);
      }
    }
#if ENABLE_ALIVE_LOG
    if (millis() - lastAliveSend > ALIVE_TIME) {
      Serial.println("[EmuSend] Alive");
      lastAliveSend = millis();
    }
#endif
  }
}

#if 0
// ========== エミュレーター処理 ==========
void emulatorTask(void* pv) {
  uint8_t buf[6];
  while (1) {
    // BitBangでパケット受信を試みる
    if (digitalRead(TC_UART_RX_PIN) == LOW) {
      bitbangReceivePacket(buf, 6);         // ← これは void なので単独で使う
      bitbangSendPacket(buf, 6);            // そのままエコーバック
      logToOLED("Emulator Mode", "Echo TC packet");
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // 無駄なCPU回しを回避
  }
}
#endif

// ========== setup ==========
void setup() {
  WiFi.mode(WIFI_STA);  // ← これを追加
  Serial.begin(115200);
  pinMode(TEST_PIN, INPUT);
  pinMode(TC_UART_TX_PIN, OUTPUT); digitalWrite(TC_UART_TX_PIN, HIGH);
  pinMode(TC_UART_RX_PIN, INPUT);
  // LEDPINセット
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // ログモードスイッチ設定
  pinMode(LOG_MODE_PIN, INPUT_PULLUP);  // LOWで詳細ログON
  enableVerboseLog = (digitalRead(LOG_MODE_PIN) == LOW);

  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  oledMutex = xSemaphoreCreateMutex();

  detectMode();

  if (current_mode == MODE_REPEATER) {
    Serial2.begin(UART_BAUDRATE, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
    logToOLED("Mode: REPEATER", "GPIO8=HIGH => Test");
    xTaskCreatePinnedToCore(uartToBitbangTask, "UART2BB", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(bitbangToUartTask, "BB2UART", 4096, NULL, 1, NULL, 1);
  } else if (current_mode == MODE_EMULATOR) {
    logToOLED("Mode: EMULATOR", "Starting echo tasks");
    echoQueue = xQueueCreate(ECHO_QUEUE_LENGTH, ECHO_PACKET_SIZE);
    xTaskCreatePinnedToCore(emulatorReceiverTask, "EmuRecv", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(emulatorSenderTask, "EmuSend", 4096, NULL, 1, NULL, 1);
  } else {
    logToOLED("ERROR", "Unknown MAC address");
  }

#if 0
  } else if (current_mode == MODE_EMULATOR) {
    logToOLED("Mode: EMULATOR", "Starting task...");
    xTaskCreatePinnedToCore(emulatorTask, "Emulator", 4096, NULL, 1, NULL, 1);
  }
#endif
}

// ========== MACアドレスチェック ==========
void detectMode() {
  uint8_t mac[6];
  WiFi.macAddress(mac);  // ← これが正しい！

  Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if (memcmp(mac, REPEATER_MAC, 6) == 0) current_mode = MODE_REPEATER;
  else if (memcmp(mac, EMULATOR_MAC, 6) == 0) current_mode = MODE_EMULATOR;
  else current_mode = 0;
}

// 死活の本体LED点滅と、擬似パケット送信モードSW対応
void loop() {
  bool currentState = digitalRead(TEST_PIN);

  // トグルスイッチ変化検出
  if (currentState != lastTestPinState) {
    lastTestPinState = currentState;

    if (currentState == HIGH) {
      logToOLED("TestMode ON", "Sending Start");
      testMode = true;
      lastSendTime = millis();
    } else {
      logToOLED("TestMode OFF", "Sending Stop");
      testMode = false;
      digitalWrite(LED_PIN, LOW);  // 状態リセット
    }
  }

  // ★ リピーターモードの動作
  if (current_mode == MODE_REPEATER) {
    if (testMode) {
      // 擬似送信（TCとPi両方へ）＆ LED点灯
      if (millis() - lastSendTime >= 1000) {
        sendTestPacket();                   // BitBang送信
        Serial2.write(testPacket, 6);       // Pi送信
        logToOLED("REPEATER TEST", "Sent to TC+Pi");
        lastSendTime = millis();
      }
      digitalWrite(LED_PIN, HIGH);  // 送信中は常時点灯
    } else {
      // 通常時は500ms点滅
      if (millis() - lastBlinkTime >= 500) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        lastBlinkTime = millis();
      }
    }
  }

  // ★ エミュレーターモードの動作（常に点滅＋必要なら送信）
  else if (current_mode == MODE_EMULATOR) {
    if (testMode && millis() - lastSendTime >= 1000) {
      Serial2.write(testPacket, 6);
      logToOLED("EMULATOR TEST", "Sent to Pi");
      lastSendTime = millis();
    }

    // 常に点滅
    if (millis() - lastBlinkTime >= 500) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlinkTime = millis();
    }
  }
}

#if 0
void loop() {
  bool currentState = digitalRead(TEST_PIN);

  // トグル状態の検出
  if (currentState != lastTestPinState) {
    lastTestPinState = currentState;

    if (currentState == HIGH) {
      logToOLED("TestMode ON", "Sending Start");
      testMode = true;
      lastSendTime = millis();
    } else {
      logToOLED("TestMode OFF", "Sending Stop");
      testMode = false;
      digitalWrite(LED_PIN, LOW);
    }
  }

// パケット送信とLED点滅（TC:BitBang送信 + Pi:UART送信）
  if (current_mode == MODE_REPEATER && testMode) {
    if (millis() - lastSendTime >= 1000) {
      sendTestPacket();               // TC側（BitBang）
      Serial2.write(testPacket, 6);   // Pi側（UART）
      logToOLED("TestMode ON", "Sent to TC+Pi");
      lastSendTime = millis();
    }

    if (millis() - lastBlinkTime >= 500) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlinkTime = millis();
    }
  }
}
#endif