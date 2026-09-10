# Main Controller 原典PDF → KiCad解析用転記 v0.1

## 目的
このフォルダは、原典 `Main controller` 回路図PDFを、ChatGPT/Claude/KiCadでネット関係を追いやすくするための**解析用転記**です。

**製造用回路図ではありません。**

## v0.1で転記した範囲
原典PDF 1ページの左中央にある通信系を優先しました。

- `ch1送信`
- `ch2送信`
- `ch1糸長入力`
- `ch2糸長入力`
- Main PIC側へのCH1/CH2独立受信経路
- ASMとの対応を示す解析用ネット名

原典PDFから確認できる最重要点は、`ch1糸長入力` と `ch2糸長入力` がMain Controller側で別々の物理入力回路になっていることです。

## KiCadファイル
`Main_Controller_Communication_Analysis_v0_1.sch`

KiCad legacy Eeschema形式です。現行KiCadで開いた場合は新しい `.kicad_sch` 形式へ変換保存できます。

## 解析用ネット名
- `MAIN_CH1_RX`
- `MAIN_CH2_RX`
- `CH1_SEND_TO_TC`
- `CH2_SEND_TO_TC`

ASM対応注記:
- `MAIN_CH1_RX -> rx_ch1 -> Data_110bps_1 -> windlen1_* / realtens1_*`
- `MAIN_CH2_RX -> rx_ch2 -> Data_110bps_2 -> windlen2_* / realtens2_*`

## 重要な解釈
Main側CH1/CH2は独立しています。一方、中継基板の `TC_TX_REP` 共有は各CHグループ内部の共有として扱います。

したがって「Direction-Bはシステム全体で1本」という解釈は禁止します。

## v0.1で意図的に省略・抽象化したもの
スキャンから精密に転記できない部品値・型番・ピン番号は推定していません。
受信整形回路や送信ドライバの一部は解析用ブロックとして抽象化しています。

特に以下は今後の原典照合対象です。
1. PICの `rx_ch1` / `rx_ch2` の正確なPORTC bit / pin番号
2. 原典の受信整形回路の抵抗・トランジスタ等の正確な値と接続
3. CN番号・各pin番号の完全転記
4. `windlen1_*`～`windlen4_*` と2本の物理入力の論理対応
5. 中継基板実装枚数とCH1/CH2への実配線

## 更新ルール
- PDFで直接読めたもの: CONFIRMED
- 原典コードとの照合で導出したもの: DERIVED
- 配線上もっとも整合するが未確認: HYPOTHESIS
- 判別不能: UNRESOLVED

不明箇所を推測で埋めず、原典PDFを常に最終照合元として残します。
