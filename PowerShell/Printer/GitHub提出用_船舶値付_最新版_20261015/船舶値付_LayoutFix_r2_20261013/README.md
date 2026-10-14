# 船舶値付用紙 PS版（2026-10-13 LayoutFix 第2版 r2）【旧版：2026-10-14 に第3版 r3 へ置換】

> **この版は第3版（r3）に置き換わりました。** 最新は `00_最新_船舶値付_本番版_20261014_LayoutFix_r3`。r2は 2026-10-14 の日次試験（1日）で使用。

| 項目 | 内容 |
|---|---|
| スクリプト | `Print-ValueTag_Ship_20261013_LayoutFix_r2.ps1` |
| SHA256 | `7A1727AB938EA2E8DE66F6B546E4049A582D5CE29D1CF8EDD1B9DAD6576FB6D6` |
| 設定 | `config\` の7ファイル（`label_layout_full.json` のみ変更、SHA256 `C4E7F77B0D38049689953761BA69EC6F6AFEA86FF4C1F173942200BF5681F328`） |
| 修正元 | `Print-ValueTag_Ship_20261007_FIX.ps1`（2026-10-07 正式採用、Codex最終版） |
| 本番配置先 | `C:\HPDB\AI_TEST\ShipValueTag\`（ルート直下の Print-ValueTag はこの1本のみ） |
| 文字コード | UTF-8（BOM付き）。BOMを消さないこと |

## 修正の要点
1. 中央寄せ・右寄せをテンプレートどおりに描画（従来は常に左寄せ）
2. 原典ブック model シートの横位置設定33セルを `label_layout_full.json` に追加
3. 縮小率・列幅・行高・配置を原典VBA（Excel）の実出力に合わせた（1〜7明細71%／8明細65%／9明細58%／10明細54%）
4. 合計欄・一式金額欄の円記号を原典VBAと同じ半角「¥」表示に（一式金額は第2版で追加）

詳細は `修正仕様書_LayoutFix_20261013.md`。

## 検証結果（位置の基準：横4mm以内・縦2mm以内）
| 日付 | 修正前の基準外 | 修正版の基準外 |
|---|---:|---:|
| 10/13（26手配） | 1,261 | 6 |
| 10/06（44手配） | 1,880 | 14 |
| 10/09（27手配） | 1,240 | 4 |

残った基準外は全件を画像で確認し、VBAと同じ位置（計測の誤照合）であることを確認済み。

## 使い方
スクリプトと `config\` を同じフォルダに置き、Windows PowerShell 5.1 で実行する。
```
powershell -NoProfile -ExecutionPolicy Bypass -File Print-ValueTag_Ship_20261013_LayoutFix_r2.ps1 -CsvPath <未計上明細_218001_YYYYMMDD.csv> -ConfigDir .\config -PrinterName "DocuWorks Printer" -MaxOrders 0
```

## 収録ファイル
- `Print-ValueTag_Ship_20261013_LayoutFix_r2.ps1` … 本番スクリプト
- `config\*.json` … 設定7ファイル
- `修正仕様書_LayoutFix_20261013.md` … 修正内容と検証結果
- `差分_20261007_FIX_to_20261013_LayoutFix_r2.diff` … 修正元からのスクリプトの差分（両方UTF-8に揃えて作成）
- `差分_第1版_to_第2版r2.diff` … 第1版から第2版への差分（一式金額の3行のみ）
- `差分_label_layout_full_json.diff` … 設定JSONの差分
- `切替記録_20261013.json`／`切替記録_20261013_r2.json` … 本番切替の記録（第1版／第2版）
- `SHA256SUMS.txt` … 全ファイルのハッシュ

## 未検証
- 11明細以上（複数ページ）
