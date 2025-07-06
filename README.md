# NTC-1A-Control
NTC-1A-Control for TEST
Initial Version 2025.06.13

## NTC-1A MAIN - Raspberry Pi用 Python + tkinter(GUI)

## NTC-1A MAIN - Seeeduino Nano版 -> Seeed Studio XIAO ESP32S3 版へ

本スケッチは、Raspberry PiとのUART通信と、テンションコントローラー(TC)との300-1200bps Bit-Bang通信を中継するSeeeduino Nano用プログラムです。

### 機能概要
- Piからの6バイト固定パケットをキュー経由でTCへ送信
- TCからの応答をPiへ中継
- ~~SoftwareSerialの割り込み影響を排除し、Bit-Bang通信の安定性を確保~~
- OLED表示対応（切替可能）



上記の通信中継機能が非同期であり、ビットバング通信として、中継する為の信号同期が厳しいことがわかり。

真のマルチタスク(FreeRTOS)が利用できる安価なSeeed Studio XSAO ESP32S3 (2core)を採用し、ラズパイとTCとの橋渡しとすることに改変する。2025.07.04

### ピンアサイン
- ~~Pi RX/TX: D4/D5~~
- ~~TC RX/TX: D3/D2~~
- ~~OLED: I2C (A4/A5)~~



| ポート名 | 接続対象      | GPIO                 | 接続種別                | 備考                                   |
| -------- | ------------- | -------------------- | ----------------------- | -------------------------------------- |
| **J2**   | TC（BitBang） | TX: GPIO2RX: GPIO3   | 300bps ビットバンギング | **要クロス接続**                       |
| **J5**   | OLED          | SDA: GPIO4SCL: GPIO5 | I2C                     | SSD1306                                |
| **J6**   | Raspberry Pi  | TX: GPIO43RX: GPIO44 | UART（115200bps）       | TCエミュレーターでは使用しない場合あり |


# ESP32S3 マイコンによる通信仕様書

**現在のプロジェクトで一貫して使われている通信仕様・GPIO割り当て・定数マクロ定義**をまとめた「最新版・通信仕様書（2025年7月2日現在）」として記録しておきます。

---

## 📘 NTC-1A 通信仕様書（2025年7月2日更新）

### 🧩 機器構成

- **Raspberry Pi（メイン）**
- **ESP32-S3（中継器/リピーター）**
- **ESP32-S3（TCエミュレーター）※旧：Arduino Nano Every**
- **Grove Shield for Seeeduino XIAO v1.0**

### 🎮 ピン定義（スケッチ内定義）

```c
#define TC_UART_TX_PIN 2    // BitBang送信用 (→TC RX)
#define TC_UART_RX_PIN 3    // BitBang受信用 (←TC TX)
#define PI_UART_TX_PIN 43   // UART送信（→ Pi）
#define PI_UART_RX_PIN 44   // UART受信（← Pi）
#define LOG_MODE_PIN 10     // ログ詳細スイッチ（LOW時に詳細）
#define TEST_PIN 8          // テスト送信モード（HIGHでON）
#define LED_PIN 21          // 動作表示用LED
```

### 🧷 Groveポート割当（上から基板を見た図に基づく）

| ポート名 | 接続対象 | GPIO | 接続種別 | 備考 |
| --- | --- | --- | --- | --- |
| **J2** | TC（BitBang） | TX: GPIO2RX: GPIO3 | 300bps ビットバンギング | **要クロス接続** |
| **J5** | OLED | SDA: GPIO4SCL: GPIO5 | I2C | SSD1306 |
| **J6** | Raspberry Pi | TX: GPIO43RX: GPIO44 | UART（115200bps） | TCエミュレーターでは使用しない場合あり |

---

---

### 📡 UART通信仕様（Raspberry Pi ↔ ESP32リピーター）

| 項目 | 内容 |  |
| --- | --- | --- |
| 通信方式 | UART（ハードウェア） |  |
| ボーレート | `9600bps`（※定義済みマクロ使用） |  |
| TXピン（ESP32） | GPIO **43**（→ Pi RX） |  |
| RXピン（ESP32） | GPIO **44**（← Pi TX） |  |
| 備考 | Groveポート **J6** で運用 |  |
| マクロ定義例 | `#define UART_BAUDRATE 9600
#define PI_UART_TX_PIN 43
#define PI_UART_RX_PIN 44` |  |

---

### 🌀 BitBang通信仕様（ESP32 ↔ TC）

| 項目 | 内容 |
| --- | --- |
| 通信方式 | ソフトウェアビットバンギング（LSB先送） |
| ボーレート | 300bps |
| TXピン（ESP32） | GPIO **2**（→ TC RX） |
| RXピン（ESP32） | GPIO **3**（← TC TX） |
| 備考 | Groveポート **J2** を使用（※クロス接続必要） |
| マクロ定義例 | `#define TC_UART_TX_PIN 2
#define TC_UART_RX_PIN 3` |

📌 **※注意：BitBang接続はTX ↔ RXのクロス結線が必須です。**

---

### 📺 OLED表示用（共通）

| 項目 | 内容 |
| --- | --- |
| I2C SDA | GPIO **4** |
| I2C SCL | GPIO **5** |
| Groveポート | J5 |

---

### 💡 MACアドレス一覧（現行）

| 用途 | MACアドレス | 備考 |
| --- | --- | --- |
| リピーター | `8C:BF:EA:8E:57:F4` | 現在稼働中 |
| 旧エミュレーター | `98:3D:AE:60:55:1C` | 故障機（使用不可） |

---

### ✅ 追加メモ（記録）

- **通信速度の切り替え**は `#define UART_BAUDRATE` にて柔軟に変更可能。

- BitBangはクロック精度が必要なため、ESP32-S3のタイマーとFreeRTOSを併用。

- OLEDログは、**上段：送信パケット／下段：受信パケット**で分離表示。

- モードごとの点滅動作やログ出力は、全モードで共通性を保つ設計方針。

  Grove Shield for Seeeduino XIAO v1.0 基板図面あり

![Grove for XIAO-ESP32S3](E:\Users\塚田淳一\Documents\GitHub\NTC-1A-Control\Grove for XIAO-ESP32S3.png)



Seeed Studio XIAO ESP32-S3の仕様書

------

## 📦 製品概要

**Seeed Studio XIAO ESP32‑S3**

- **プロセッサ**：240 MHz 動作、Xtensa 32bit LX7 デュアルコア
- **ワイヤレス**：2.4 GHz Wi‑Fi（802.11 b/g/n）、Bluetooth 5.0 / BLE メッシュ
- **アンテナ**：2.4 GHz ロッドアンテナ搭載
- **電力管理**：リチウムバッテリ充電対応、ディープスリープで消費14 μA
- **用途例**：IoT機器、スマートホーム、ウェアラブル、ロボットなど 

------

## 🔌 電源・消費電力

- **入力電圧**
  - USB Type‑C：5 V
  - リチウム電池(BAT)：最大4.2 V
- **動作時消費電流**（XIAO ESP32S3 / XIAO ESP32S3 Sense） 
  - **Type‑C／BAT 待機時（通常動作時）**：約19 mA／22 mA
  - **Sense モジュール接続時**：38 mA／43.2 mA
  - **モデムスリープ時**：約25 mA
  - **ライトスリープ時**：約2 mA
  - **ディープスリープ時**：約14 μA

------

## ⚙️ ハードウェア仕様

- **サイズ**：21 × 17.8 mm（親指サイズ） 
- **メモリ**：8 MB PSRAM、8 MB Flash（Plus版は16 MB Flash） 
- **インタフェース**：
  - UART×1
  - I²C×1
  - I²S×1
  - SPI×1
  - GPIO（PWM対応）×11
  - ADC チャネル×9
  - ユーザー LED×1、充電LED×1
  - リセットボタン、ブートボタン 各×1 

------

## 🌐 ワイヤレス機能

- **Wi‑Fi**：IEEE 802.11 b/g/n、20/40 MHz帯域、最大150 Mbps、複数仮想インタフェース対応 
- **Bluetooth**：Bluetooth 5、BLE メッシュ、転送速度125 kbps～2 Mbps、高出力20 dBm対応、広告拡張対応 

------

## 🌙 低消費電力モード

ESP32‑S3 SoC のパワーマネジメントは高度で、多様な低消費力モードをサポート：

- **アクティブモード**：CPU・RF・周辺回路動作
- **モデムスリープ**：CPU動作、クロックダウン
- **ディープスリープ**：ULPコプロセッサ＋RTCのみ動作 🎯
- **ULP（超低電力）コプロセッサ**搭載 

------

## 🧠 ESP32‑S3 システム仕様

- **CPU**：Xtensa® LX7 デュアルコア、最大240 MHz
- **メモリ**：384 KB ROM、512 KB SRAM + 16 KB RTC SRAM
- **周辺回路**：SPI×4、LCD／Camera IF、UART×3、I²C×2、I²S×2、PWM、USB OTG、CAN（TWAI）、ADC、タッチ／温度センサー等 
- **セキュリティ**：Secure Boot、AES/SHA/RSA/HMAC/RNG ハードウェアアクセラレータ、Flash暗号化、OTP 

------

## ✅ 主な強み

1. **小型＆多機能**：21×17.8 mm サイズに高性能CPU、ワイヤレス、低電力種モードを集約
2. **電池駆動設計**：リチウム充電管理と低消費で、バッテリ駆動に最適
3. **豊富なI/O**：SPI/I²C/UARTなど開発に必要なインタフェース充実
4. **AI／セキュリティ対応**：PSRAM・ハードウェア暗号により、TinyMLや安全通信用途にも有効

------

## 🌟 XIAO ESP32S3 シリーズ比較

| モデル              | フラッシュ | PSRAM | カメラ/マイク      | SDカード     | 用途                        |
| ------------------- | ---------- | ----- | ------------------ | ------------ | --------------------------- |
| XIAO ESP32‑S3       | 8 MB       | 8 MB  | –                  | –            | 汎用 IoT／ウェアラブルなど  |
| XIAO ESP32‑S3 Sense | 8 MB       | 8 MB  | ○（OV2640/OV3660） | ○ (最大32GB) | 音声・映像処理付きエッジ AI |
| XIAO ESP32‑S3 Plus  | 16 MB      | 8 MB  | –                  | –            | I/O 拡張が必要な開発向け    |

※Sense版は OV2660（OV3660）または OV2640 カメラ、SD カードスロット搭載 
 Plus版は背面に SMDボード接続用で最大9個の GPIO Pad追加 

------

## 📝 まとめ

Seeed Studio XIAO ESP32‑S3シリーズは、限られたスペースに高性能CPU・Wi‑Fi・BLEを搭載し、低消費＆電池充電にも対応。用途に応じた3ラインナップ（汎用／AI搭載Sense／I/O追加Plus）から選べ、TinyML、音声・映像IoT、ウェアラブルなどに最適です。