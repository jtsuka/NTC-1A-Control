# TC106 Step4B PulseView Decoder v0.1

TC106 / Nano Every -> Main / ESP32 の Direction B 用の
sigrok / PulseView カスタムデコーダ初版です。

## 対象仕様

- 17 slot / byte
- 3.3 ms / slot
- 7bit LSB first
- 6 byte telemetry
- footer `0x7F`
- Step4B暫定極性
  - idle LOW
  - slot0 LOW
  - bit0 LOW
  - bit1 HIGH
  - slot8..16 HIGH

## 重要

現在のStep4B仕様では `idle LOW -> byte0 slot0 LOW` なので、
フレーム先頭に必ずしもエッジがありません。

v0.1では「最初の立下り=START」と決め打ちせず、
観測エッジ周辺から候補slot位相を試し、
6byte全体・guard・footer `0x7F` が成立した候補だけを
TELEMETRYとして採用する方針です。

これは実TC106のSTART同期方式を確定したものではありません。
実TC106入手後に物理極性と先頭同期を再確認します。

## ファイル構成

tc106_step4b/
- __init__.py
- pd.py

## 最初の試験推奨

Nano側の既知データ:

`01 02 03 04 05 7F`

PulseView上で以下のような表示を狙います:

- BYTE0 0x01
- BYTE1 0x02
- BYTE2 0x03
- BYTE3 0x04
- BYTE4 0x05
- BYTE5 0x7F
- LEN0 / LEN1 / LEN2 / TENS0 / TENS1 / END
- TELEMETRY

## 推奨取得条件

- sample rate: 1 MHz
- slot_us: 3300
- polarity: step4b
- show_bits: yes
- show_slots: no

## ステータス

v0.1は実装初版です。
PulseView + 実際の `.sr` で認識確認するまでは RUNTIME PASS ではありません。
認識できない場合は、その `.sr` を基準に位相探索を調整します。
