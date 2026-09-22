# Claudeコードレビュー依頼
## Format-KonyuCSV_RFC v3.2 OrderNoFilter REVIEW
### 2026-09-22

正式採用中の `Format-KonyuCSV_RFC_yyyyMMdd_v3.1.ps1` に、
`-ExcludeMissingOrderNo` スイッチを最小差分で追加したREVIEW版です。

## レビュー対象

- `Format-KonyuCSV_RFC_yyyyMMdd_v3.2_OrderNoFilter_REVIEW.ps1`
- 比較元 `Format-KonyuCSV_RFC_yyyyMMdd_v3.1_SOURCE.ps1`
- `DIFF_v3.1_to_v3.2_OrderNoFilter.txt`
- 改修仕様書

## 確認してほしい点

1. スイッチ未指定時にv3.1の既定動作が変わらないか。
2. `-ExcludeMissingOrderNo` 指定時だけ空欄行が除外されるか。
3. 「手配書番号」列が無い場合にFail Closedできるか。
4. 判定が `Repair-ColumnShift` の後で行われているか。
5. 半角／全角スペースのみの値を空欄として扱えるか。
6. `Repair-ColumnShift` 自体の業務ロジックを変更していないか。
7. CP932読込・CP932/BOMなし出力・RFC 4180出力を維持しているか。
8. `yyyy-MM-dd` の日付処理を巻き戻していないか。
9. `$processed` が「実際に出力したデータ行数」として矛盾しないか。
10. 途中エラー時のexit/出力ファイルの扱いに新たなリスクがないか。
11. MorningRoutineへ組み込む場合、既存I/F互換性に問題がないか。
12. TEST T01〜T10で不足する観点がないか。

## 特に確認したい設計判断

空欄判定は「列ずれ補正＋トリム後、RFC再構築直前」に置いています。
仕様上はこれを意図していますが、より安全な位置があれば指摘してください。

また、この機能は「手配書番号なし行を**除外**」するものであり、
「手配書番号なし行だけを抽出」する機能ではありません。

## 現時点の自己チェック

- v3.1を直接上書きしていない。
- PowerShellコードの `{}` / `()` 対応を機械確認済み。
- `Repair-ColumnShift` の本文は変更していない。
- `yyyy-MM-dd` 出力は維持。
- CP932で保存。
- REVIEW版のため、ClaudeレビューとTEST完了前は本番使用しない。
