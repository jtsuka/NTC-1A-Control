# ★最新★ 船舶値付用紙 PS版 本番版（2026-10-22 LayoutFix 第4版 r4）

> 検証用PCでは 2026-10-23 の夜間試験から使用。別PC（テスト環境）では、RK-10シナリオ経由のDocuWorks→PDF結果とログで実機判定する（未実施）。

| 項目 | 内容 |
|---|---|
| スクリプト | `Print-ValueTag_Ship_20261022_LayoutFix_r4.ps1` |
| SHA256 | `28757C496A7BC4B1738ABD430D1A6B47CDB71C2D5BB9C3E4B40F60429A2BABD7` |
| 設定 | `config\` の7ファイル（r2・r3から変更なし。`label_layout_full.json` SHA256 `C4E7F77B0D38049689953761BA69EC6F6AFEA86FF4C1F173942200BF5681F328`） |
| 修正元 | `Print-ValueTag_Ship_20261014_LayoutFix_r3.ps1`（第3版 r3） |
| 文字コード | UTF-8（BOM付き）。BOMを消さないこと |

## 第3版 r3 からの修正（第4版 r4）
11明細以上の手配（10明細ごとに複数頁へ分割）の縮尺を、10明細以下と同じ実測式にした。変更はこの縮尺式だけ。
- 従来は単純な縮小率で、行間が約3%狭く、下の明細ほど原典VBAからずれた（2026-10-22 の手配 E26B010640 で位置の基準外161件）。
- r4は固定Zoom（DocuWorks Printer 56%／その他 59%）に、列幅=round(134×Zoom)・行高=round(122.7×Zoom)（600dpi画素）を適用する。
- 差分は `差分_r3_to_r4.diff`。

## 検証結果（位置の基準：横4mm以内・縦2mm以内）
| 日付 | 総合 | 位置の基準外 | 備考 |
|---|---|---:|---|
| 10/22（検証用PC、DocuWorks Printer） | PASS | 0 | 印字値の値差0、バーコードNG0。11明細の手配は1頁目459要素・2頁目65要素とも許容内 |

## 使い方
スクリプトと `config\` を同じフォルダに置き、Windows PowerShell 5.1 で実行する。
```
powershell -NoProfile -ExecutionPolicy Bypass -File Print-ValueTag_Ship_20261022_LayoutFix_r4.ps1 -CsvPath <未計上明細_218001_YYYYMMDD.csv> -ConfigDir .\config -PrinterName "DocuWorks Printer" -MaxOrders 0
```
本番フォルダには印刷スクリプト（`Print-ValueTag*.ps1`）を1本だけ置く。

## 収録ファイル
- `Print-ValueTag_Ship_20261022_LayoutFix_r4.ps1` … 本番スクリプト
- `config\*.json` … 設定7ファイル（変更なし）
- `差分_r3_to_r4.diff` … 第3版からの差分
- `SHA256SUMS.txt` … 全ファイルのハッシュ

## 未検証
- DocuWorks Printer以外（59%の経路）。
- 12明細以上、複数頁の最後の頁、一式金額欄を含む手配の実例。
- 2頁目以降の原因付け（最終チェックの `gate3_attribute.py`）は未対応。
- 本ファイルに業務データ（CSV・明細・個人名）は含まない。