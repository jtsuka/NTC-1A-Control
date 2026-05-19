# TC106 Bridge Project

## 概要
製糸工場の Nittenser テンションコントローラシステム (TC-102/TC106) の
メインコントローラ (NTC-1A) を Raspberry Pi + ESP32-S3 で代替する個人プロジェクト。

## システム階層
- メインコントローラ NTC-1A (PIC16F73)
- 中継基板 (ハブユニット、6 錘ぶら下がり、デイジーチェーン)
- テンションユニット TC-102/TC106 (dsPIC33FJ16MC102)

## 進捗
- [x] TC106 firmware 解析完了
- [x] NTC-1A ASM 解析完了
- [x] 中継基板回路解析完了
- [x] ESP32 スニファ基板設計完了 (PCB 5 枚)
- [x] Pi 側 Pygame GUI 試作完了
- [ ] Phase 1 ループバックテスト (基板 #2 組立後)
- [ ] Phase 2 実機波形取得
- [ ] Phase 3 能動中継テスト
- [ ] Phase 4 本番運用

## アーキテクチャ
[Pi (Pygame GUI)] ↔ UART 9600bps ↔ [ESP32-S3] ↔ 300bps 17スロット OC ↔ [TC106]

## ファイル構成
- `firmware/TC106/` - TC106 firmware (dsPIC33FJ16MC102)
- `firmware/NTC-1A/` - 主控板 ASM (PIC16F73)
- `firmware/ESP32/` - ESP32 ブリッジファーム
- `firmware/Nano/` - Nano エミュレータ
- `pi/` - Pygame GUI と pyserial 通信層
- `schematics/` - 回路図 (ESP32 sniffer, 中継基板)
- `docs/manual/` - TC-102 マニュアル PDF
- `docs/analysis/` - 解析メモ

## 次のアクション
基板 #2 を組立、Phase 1.1 ループバック疎通テスト
