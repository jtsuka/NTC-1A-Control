# TC106 Pi READY → ESP RESET Fail-Safe 回路（ブレッドボード版）

対象: KiCad 10.0

v0.2: legacy schematic の `Text Notes` / `Text Label` 行に余分なトークンが入っていたため修正。

このフォルダーには、ブレッドボード確認用の回路図を入れています。

## 開き方

1. `TC106_Pi_READY_RESET_Breadboard.sch` を KiCad 10 の Schematic Editor で開きます。
2. KiCad 10 は legacy `.sch` を読み込み可能です。
3. 最初に **名前を付けて保存** すると、KiCad 10 ネイティブの `.kicad_sch` に変換できます。
4. 変換後の `.kicad_sch` を今後の正式回路図ベースとして使ってください。

## 回路

- Q1: 2N7000
- Q2: 2N7000
- R1: 100kΩ（XIAO 3.3V → HOLD）
- R2: 100kΩ（Q2 Gate → GND）
- R3: 1kΩ（Pi BCM17/ESP_RUN → Q2 Gate）
- TP1: HOLD
- TP2: CHIP_PU/RESET
- J1: Pi側（BCM17/ESP_RUN, GND）
- J2: XIAO側（3.3V, CHIP_PU/RESET, GND）

### 動作

Pi未起動 / GPIO Hi-Z / ESP_RUN LOW:
Q2 OFF → R1でHOLD HIGH → Q1 ON → CHIP_PU LOW → ESP RESET保持

Pi READY / ESP_RUN HIGH:
Q2 ON → HOLD LOW → Q1 OFF → XIAO側pull-upでCHIP_PU HIGH → ESP起動

## 重要

この回路図の2N7000記号は論理的な S/G/D を示すためのものです。
**手持ち2N7000の実物ピン配列は、メーカー・型番のデータシートで必ず確認してから配線してください。**

R1/R2/R3は現時点の仮定値です。CHIP_PU立上り時間・電源投入波形の実測後にFIXします。
PCB footprintは意図的に未確定です。ブレッドボードPASS後、本番Sniffer回路へ組み込む段階で正式部品とfootprintを確定します。

## v0.3 修正
KiCad 10 の legacy schematic parser が空行を `unrecognized token` として扱ったため、
`.sch` 内の空行を全削除しました。前版の line 24 は空行でした。
