# ESP32_TC_EmuRepeater

ESP32-S3ベースのテンションコントローラー中継・エミュレーター統合スケッチです。

## 機能概要
- **REPEATERモード**: UART/BitBang間の中継（今後実装予定）
- **EMULATORモード**: エコーバックによる受信テスト対応
- OLED表示によるログ出力
- GPIO8によるTESTモードスイッチ対応（HIGH=点滅、LOW=消灯）

## 構成ファイル
- `ESP32_TC_EmuRepeater.ino`: メインスケッチ
- `config.h`: 各種定義・ピン設定
- `oled_log.*`: OLED表示処理
- `task_bridge.*`: 中継処理（REPEATER用）
- `task_emulator.*`: エミュレーター処理
- `task_led_oled.*`: LED/OLED制御・TEST_PIN処理

## モード切替
- MACアドレスが `D8:3B:DA:74:82:78` の場合、**エミュレーター**
- その他は **リピーター**

今後の拡張としてUART受信やBitBang送受信処理を追加予定です。
