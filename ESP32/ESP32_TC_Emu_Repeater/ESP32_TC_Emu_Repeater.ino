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
#define LOG_MODE_PIN 8                 // GPIO8をスイッチに使用
#define ALIVE_TIME 3000                 // 死活確認時間 3秒
bool enableVerboseLog = true;          // 詳細ログ有効フラグ
unsigned long lastAlive = 0;            // タスク死活フラグ
unsigned long lastAliveSend = 0;        // タスク死活フラグ


// ===== テストモード制御用グローバル変数 =====
bool testMode = true;
bool lastTestPinState = LOW;
unsigned long lastSendTime = 0;
unsigned long lastBlinkTime = 0;
bool ledState = false;

// ======== エミュレータ用キューと定数 ========
QueueHandle_t echoQueue;
#define ECHO_QUEUE_LENGTH 4
#define ECHO_PACKET_SIZE  6

// ========== MACアドレス定義 ==========
//const uint8_t EMULATOR_MAC[6] = {0x98, 0x3D, 0xAE, 0x60, 0x55, 0x1C};  // 故障
const uint8_t REPEATER_MAC[6] = {0x8C, 0xBF, 0xEA, 0x8E, 0x57, 0xF4};
const uint8_t EMULATOR_MAC[6] = {0xD8, 0x3B, 0xDA, 0x74, 0x82, 0x78};


// ========== ピン定義 ==========
#define PI_UART_TX_PIN 43   // ESP32からPiへのTX
#define PI_UART_RX_PIN 44   // PiからESP32へのRX
#define TC_UART_TX_PIN 2    // BitBang送信用(1->2)
#define TC_UART_RX_PIN 3    // BitBang受信用(0->3)
#define TEST_PIN 8          // テスト用トグルスイッチ
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
  Serial.printf("[DEBUG] Received byte: %02X\n", b); // ← NEW!
  return b;
}

void bitbangReceivePacket(uint8_t* buf, int len) {
  // for debug
  Serial.println("[DEBUG] Start bit detected");
  for (int i = 0; i < len; i++) buf[i] = bitbangReceiveByte();
}

// ========== テストパケット送信 ==========
void sendTestPacket(const uint8_t *packet, int lengs) {
  bitbangSendPacket(packet, lengs);
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
      Serial.println("[DEBUG] Start bit LOW detected (EmuRecv)");
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
        String LogMsg = "[EmuRecv] Packet:";
        for (int i = 0; i < 6; i++) msg += String(buf[i], HEX) + " ";
        logToOLED(msg, "→ Queued for echo");
        // for debug log
        for (int i = 0; i < ECHO_PACKET_SIZE; i++) {
          LogMsg += " " + String(buf[i], HEX);
        }
        Serial.println(msg);
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
        String LogMsg = "[EmuSend] Echoed:";
        for (int i = 0; i < 6; i++) msg += String(buf[i], HEX) + " ";
        logToOLED("Echoed Back", msg);
        for (int i = 0; i < ECHO_PACKET_SIZE; i++) {
          LogMsg += " " + String(buf[i], HEX);
        }
        Serial.println(msg);
      }
    }
  } 

#if ENABLE_ALIVE_LOG
    if (millis() - lastAliveSend > ALIVE_TIME) {
      Serial.println("[EmuSend] Alive");
      lastAliveSend = millis();
    }
#endif
}

// ========== setup ==========
void setup() {
  // ← WIFIのMACアドレスで個体識別をする
  WiFi.mode(WIFI_STA);
  // Arduino IDEのシリアル設定
  Serial.begin(115200);
  // トグルスイッチピンの設定
  pinMode(TEST_PIN, INPUT);
  // UART PIN 設定
  pinMode(TC_UART_TX_PIN, OUTPUT); digitalWrite(TC_UART_TX_PIN, HIGH);
  pinMode(TC_UART_RX_PIN, INPUT);
  // LEDPINセット
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // ログモードスイッチ設定
  pinMode(LOG_MODE_PIN, INPUT_PULLUP);  // LOWで詳細ログON
  enableVerboseLog = (digitalRead(LOG_MODE_PIN) == LOW);

  // OLED初期設定
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  // OLED排他制御用 Mutex
  oledMutex = xSemaphoreCreateMutex();

  // Seeeduino XIAO ESP32S3 MacAddr 個体識別の為に取得しモードを決定する
  detectMode();

  // 個体識別をして、リピーターモードかTCエミュレーターモードを決定して初期化する
  switch (current_mode) {
    case MODE_REPEATER:
      Serial2.begin(UART_BAUDRATE, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
      logToOLED("Mode: REPEATER", "GPIO8=HIGH => Test");
      xTaskCreatePinnedToCore(uartToBitbangTask, "UART2BB", 4096, NULL, 1, NULL, 1);
      xTaskCreatePinnedToCore(bitbangToUartTask, "BB2UART", 4096, NULL, 1, NULL, 1);
      // for debug
      pinMode(TEST_PIN, INPUT);
      pinMode(LED_PIN, OUTPUT);
      Serial.printf("TEST_PIN = %d\n", digitalRead(TEST_PIN));
      break;

    case MODE_EMULATOR:
      logToOLED("Mode: EMULATOR", "Starting echo tasks");
      echoQueue = xQueueCreate(ECHO_QUEUE_LENGTH, ECHO_PACKET_SIZE);
      xTaskCreatePinnedToCore(emulatorReceiverTask, "EmuRecv", 4096, NULL, 1, NULL, 1);
      xTaskCreatePinnedToCore(emulatorSenderTask, "EmuSend", 4096, NULL, 1, NULL, 1);
      // for Debug
      Serial.println("[Setup] EMULATOR Receiver task created");
      break;

    default:
      logToOLED("ERROR", "Unknown MAC address");
      break;
  }

  return;
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
  // loop()内でのチャタリング対策（例）
  unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;
  // トグルスイッチ変化検出
  bool currentState = digitalRead(TEST_PIN);
  if (currentState != lastTestPinState) {
    lastDebounceTime = millis();  // 状態変化を検出
  }

  // 今のスイッチと前のスイッチのモードを比較してモードを表示する
  if (currentState != lastTestPinState) {
    lastTestPinState = currentState;

    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (currentState != testMode) {
        testMode = currentState;
        if (testMode) {
          logToOLED("TestMode ON", "Sending Start");
          lastSendTime = millis();
        } else {
          logToOLED("TestMode OFF", "Sending Stop");
          digitalWrite(LED_PIN, LOW);
        }
      }
    }
    lastTestPinState = currentState;

    switch (current_mode) {
      case MODE_REPEATER:                 // ★ リピーターモードの動作
        if (testMode) {
          // 擬似送信（TCとPi両方へ）＆ LED点灯
          if (millis() - lastSendTime >= 1000) {
            sendTestPacket(testPacket, ECHO_PACKET_SIZE);       // BitBang送信
            Serial2.write(testPacket, ECHO_PACKET_SIZE);        // Pi送信
            logToOLED("REPEATER TEST", "Sent to TC+Pi");
            lastSendTime = millis();
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState);
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
        break;
      case MODE_EMULATOR:               // ★ エミュレーターモードの動作（常に点滅＋必要なら送信
        // ★ エミュレーターモードの動作（常に点滅＋必要なら送信）
        if (testMode && millis() - lastSendTime >= 1000) {
          Serial2.write(testPacket, 6);
          logToOLED("EMULATOR TEST", "Send to Repeater");
          lastSendTime = millis();
        }
        // 通常時は500ms点滅
        if (millis() - lastBlinkTime >= 500) {
          ledState = !ledState;
          digitalWrite(LED_PIN, ledState);
          lastBlinkTime = millis();
        }
        break;
    }
#if 0
    // 常に点滅
    if (millis() - lastBlinkTime >= 500) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlinkTime = millis();
    }
#endif
  }
}


