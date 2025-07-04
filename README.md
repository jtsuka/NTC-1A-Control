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
