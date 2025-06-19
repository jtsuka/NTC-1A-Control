# NTC-1A-Control
NTC-1A-Control for TEST
Initial Version 2025.06.13

## NTC-1A MAIN - Raspberry Pi用 Python + tkinter(GUI)

## NTC-1A MAIN - Seeeduino Nano版

本スケッチは、Raspberry PiとのUART通信と、テンションコントローラー(TC)との300-1200bps Bit-Bang通信を中継するSeeeduino Nano用プログラムです。

### 機能概要
- Piからの6バイト固定パケットをキュー経由でTCへ送信
- TCからの応答をPiへ中継
- SoftwareSerialの割り込み影響を排除し、Bit-Bang通信の安定性を確保
- OLED表示対応（切替可能）

### ピンアサイン
- Pi RX/TX: D4/D5
- TC RX/TX: D3/D2
- OLED: I2C (A4/A5)

