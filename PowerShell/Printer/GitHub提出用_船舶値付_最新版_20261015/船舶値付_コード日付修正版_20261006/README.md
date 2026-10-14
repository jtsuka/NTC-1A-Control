# 船舶値付用紙 PS版 コード・日付修正版（2026-10-06）

> **最新ではありません。** 最新は `00_最新_船舶値付_本番版_20261013_LayoutFix`。

| 項目 | 内容 |
|---|---|
| スクリプト | `Print-ValueTag_Ship_CsvCompatibilityTrial_20261006.ps1` |
| SHA256 | `3C84A96324D04CD6E0801BFF709066D11A1DB1533112C57724C35DACEA858731` |
| 使用期間 | 2026-10-07 の日次試験（本番） |
| 修正元 | `Print-ValueTag_Ship_Alignment_F8FIX_DetailOrderFIX_20261005.ps1` |
| 主な修正 | 得意先・受注先コードを原典VBAと同じく6・7桁のときだけ8桁へ補完。F8の「月－4桁年」形式をVBAと同じ表現へ |
| 作成 | Codex（Claude独立レビューで修正範囲PASS） |

収録：スクリプト、`修正仕様書_FIX_20261006.md`、`CSV参照元の調査と修正仕様.md`、`差分_20261005_to_20261006.diff`、`SHA256SUMS.txt`
