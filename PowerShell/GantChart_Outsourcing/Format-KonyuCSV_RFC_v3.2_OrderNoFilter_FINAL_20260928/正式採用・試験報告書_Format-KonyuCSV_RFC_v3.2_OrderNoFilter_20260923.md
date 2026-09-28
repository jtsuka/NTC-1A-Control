# Format-KonyuCSV_RFC v3.2 OrderNoFilter 正式採用・試験報告書

## 1. 結論

`Format-KonyuCSV_RFC_yyyyMMdd_v3.2_OrderNoFilter_REVIEW.ps1` は、Claudeコードレビューおよび2026-09-23の単体・実データ試験を完了した。

全試験がPASSしたため、コード内容を変更せず、ファイル名のみ

`Format-KonyuCSV_RFC_yyyyMMdd_v3.2_OrderNoFilter_FINAL.ps1`

へ変更して正式採用する。

運用開始日は **2026-09-24** とする。

## 2. 採用方針

本版は当面 **単体運用** とする。

MorningRoutineへのマージは今回の正式採用条件に含めない。現行MorningRoutineには旧来構成との依存関係が残るため、単体運用で実績を積んだ後、統合の必要性を別案件として判断する。

## 3. Claudeレビュー結果

2026-09-22作成のREVIEW版について、以下12観点でレビューを実施しPASS判定となった。

1. スイッチ未指定時のv3.1互換性
2. 指定時だけ空欄除外
3. 手配書番号列なし時のFail Closed
4. `Repair-ColumnShift` 後の判定
5. 半角／全角スペース空欄判定
6. `Repair-ColumnShift` 本体無変更
7. CP932 / RFC4180維持
8. `yyyy-MM-dd`維持
9. `$processed`件数整合
10. 途中エラー時の扱い
11. MorningRoutine I/F互換
12. TEST項目妥当性

追加提案として「全角スペースのみ」のT11を採用した。

## 4. T01〜T11 実施結果

試験日時: 2026-09-23 09:20:12

対象入力:

`C:\HPDB\外注課納期管理_NoJOIN想定_全937件_App292_TEST_20260921.csv`

実測:

- SourceRows: 937
- OrderNoPresent: 327
- OrderNoMissing: 610

| Test | 結果 | 確認内容 |
|---|---|---|
| T01 | PASS | v3.1 Exit=0 / v3.2 Exit=0 / byte-identical=True |
| T02 | PASS | 入力20 / 出力20 / Exit=0 |
| T03 | PASS | 期待327 / 出力327 / 除外期待610 |
| T04 | PASS | 期待937 / 出力937 |
| T05 | PASS | Exit=1 / 出力未作成または0byte |
| T06 | PASS | 手配日 26-09-23 → 2026-09-23 |
| T07 | PASS | 商品名列ずれ修復、手配書番号維持 |
| T08 | PASS | NoBOM / CRLF / RFC全項目quote・roundtrip |
| T09 | PASS | Input=Outputを拒否し入力コピー不変 |
| T10 | PASS | 937件を再計数し327件出力 |
| T11 | PASS | 全角空白のみを除外 |

**PASS=11 / FAIL=0 / OVERALL=PASS**

## 5. 実データ RFC正規化試験

試験日時: 2026-09-23 09:36頃

対象:

`C:\HPDB\外注課納期管理_20260923.csv`

今回は `-ExcludeMissingOrderNo` を指定せず、RFC正規化機能のみを確認した。

実測:

- Input rows: 322
- Input columns: 141
- 手配書番号あり: 322
- 手配書番号空欄: 0
- 入力列数不一致: 0

結果:

| Test | 結果 | 確認内容 |
|---|---|---|
| T01_InputReadable | PASS | Rows=322 |
| T02_InputColumnConsistency | PASS | 列数不一致=0 |
| T03_FormatterExit | PASS | Exit=0 |
| T04_RowCountMaintained | PASS | Input=322 / Output=322 |
| T05_OrderNoCountMaintained | PASS | 322件維持 / 空欄0件維持 |
| T06_OutputColumnConsistency | PASS | 列数不一致=0 |
| T07_NoBOM | PASS | NoBOM=True |
| T08_CRLF | PASS | CRLF=True |
| T09_RFC4180_AllQuoted | PASS | quote/roundtrip=True |
| T10_InputUnchanged | PASS | SHA256 unchanged=True |

**PASS=10 / FAIL=0 / OVERALL=PASS**

## 6. 重要な確認事項

### v3.1互換
T01で、`-ExcludeMissingOrderNo` 未指定時のv3.2出力がv3.1とバイト単位で一致した。

### フィルタ機能
937件データで327件あり／610件空欄を独立再計数し、スイッチ指定時は327件のみ出力した。

### RFC正規化単体運用
2026-09-23の実データ322件をスイッチなしで処理し、件数・手配書番号・141列を維持したままRFC正規化できた。

### 元データ保護
InputPath=OutputPathは禁止され、実データ試験でも元CSVのSHA256が変化していないことを確認した。

## 7. FINAL化

REVIEW版からコード変更は行わない。

変更点はファイル名のみ:

```text
Format-KonyuCSV_RFC_yyyyMMdd_v3.2_OrderNoFilter_REVIEW.ps1
↓
Format-KonyuCSV_RFC_yyyyMMdd_v3.2_OrderNoFilter_FINAL.ps1
```

したがってFINAL PSのSHA256はREVIEW版と同一である。

## 8. 運用開始

2026-09-24から本FINAL版を正式運用する。

初期運用は単体実行とし、入力CSVを直接上書きせず、OutputPathは別名ファイルを指定する。

MorningRoutineへの統合は保留し、単体運用実績を確認してから改めて判断する。

## 9. 総合判定

**FINAL正式採用: PASS**
