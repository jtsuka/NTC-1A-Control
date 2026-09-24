# Format-KonyuCSV_RFC v3.2 OrderNoFilter 改修仕様書
## 手配書番号なし行の除外オプション追加（Claudeレビュー版）
### 2026-09-22

## 1. 目的

正式採用中の `Format-KonyuCSV_RFC_yyyyMMdd_v3.1.ps1` を母体とし、
RFC正規化・列ずれ補正・日付4桁年化・CP932出力等の既存ロジックを維持したまま、
**明示指定時だけ「手配書番号」が空欄の行を除外するオプション**を追加する。

新版候補:

`Format-KonyuCSV_RFC_yyyyMMdd_v3.2_OrderNoFilter_REVIEW.ps1`

この版は **REVIEW版**。ClaudeレビューおよびTEST完了までは本番採用しない。

## 2. 背景

NoJOIN想定CSVでは「手配書番号」が空欄の行が混在する。
2026-09-21の全937件想定CSVでは、手配書番号あり327件／空欄610件を確認している。

今回の改修は、必要な運用で明示的に空欄610件相当を除外し、
手配書番号が存在する行だけを出力できるようにするもの。

**注意:** このオプションは「手配書番号なし行だけを抽出する」機能ではない。
`-ExcludeMissingOrderNo` は「手配書番号なし行を除外する」機能である。

## 3. 基本方針

既定動作は変更しない。

```text
スイッチ指定なし:
  従来どおり全行を出力する。
  手配書番号が空欄でも除外しない。

-ExcludeMissingOrderNo 指定あり:
  「手配書番号」列が空欄の行だけを出力対象から除外する。
```

既存のMorningRoutine等へ無断でこのスイッチを付けない。
正式採用・運用組込みは別途レビューとTEST後に判断する。

## 4. 変更内容

### 4.1 パラメータ追加

```powershell
[switch]$ExcludeMissingOrderNo
```

### 4.2 対象列解決

ヘッダー行から完全一致で「手配書番号」を検索する。

```powershell
$orderNoIndex = [Array]::IndexOf($headers, "手配書番号")
```

`-ExcludeMissingOrderNo` 指定時に列が存在しない場合はFail Closedする。

### 4.3 判定タイミング

次の順序を守る。

```text
CSV 1行読込
↓
Parse-CsvLine
↓
必要なら Repair-ColumnShift
↓
日付変換・文字列トリム
↓
列数再確認
↓
-ExcludeMissingOrderNo 指定時だけ手配書番号空欄判定
↓
空欄なら出力せずカウント
空欄でなければRFC 4180形式へ再構築
```

列ずれ補正前の生配列で判定してはいけない。
補正後の正しい列位置で判定する。

### 4.4 空欄定義

以下を空欄として扱う。

- `""`
- 半角スペースのみ
- 全角スペースのみ
- 半角／全角スペース混在

### 4.5 ログ

スイッチ指定時のみ完了ログへ、

```text
手配書番号なし除外: N 行
```

を追加する。

起動時には除外機能の有効／無効も表示する。

## 5. 変更しないもの

v3.1から以下は変更しない。

- Windows PowerShell 5.1対応
- CP932読込
- CP932 / BOMなし出力
- CRLF出力
- RFC 4180形式の全項目ダブルクォート
- 内部ダブルクォートの二重化
- `Parse-CsvLine`
- `Repair-ColumnShift` の中身
- 商品名／商品情報に起因する既存列ずれ補正
- 日付列ヘッダー判定
- 2桁年 `yy-MM-dd` を `20yy-MM-dd` と明示解釈
- 出力日付 `yyyy-MM-dd`
- 前後スペース／全角スペースのトリム
- InputPath=OutputPath禁止
- 入力ファイル存在確認
- 出力先フォルダ存在確認
- ExitCodeの基本方針

## 6. 使用例

通常（従来互換）:

```powershell
.\Format-KonyuCSV_RFC_yyyyMMdd_v3.2_OrderNoFilter_REVIEW.ps1 `
  -InputPath "C:\HPDB\TMP\input.csv" `
  -OutputPath "C:\HPDB\TMP\output.csv"
```

手配書番号なし行を除外:

```powershell
.\Format-KonyuCSV_RFC_yyyyMMdd_v3.2_OrderNoFilter_REVIEW.ps1 `
  -InputPath "C:\HPDB\TMP\input.csv" `
  -OutputPath "C:\HPDB\TMP\output.csv" `
  -ExcludeMissingOrderNo
```

## 7. TEST項目

- T01: スイッチなし → v3.1と同じ件数・同じ内容を出力。
- T02: 全行に手配書番号あり + スイッチあり → 除外0件。
- T03: 手配書番号空欄混在 + スイッチあり → 空欄行のみ除外。
- T04: 手配書番号空欄混在 + スイッチなし → 空欄行も含め全件出力。
- T05: 「手配書番号」列なし + スイッチあり → 書込み前にエラー停止。
- T06: 日付出力が引き続き `yyyy-MM-dd`。
- T07: 列ずれ補正対象行でも、補正後の手配書番号列で判定。
- T08: CP932/BOMなし/RFC 4180形式がv3.1と一致。
- T09: 入力と出力が同一パスなら従来どおりSafetyStop。
- T10: 937件想定CSVを使う場合、スイッチありで手配書番号あり327件が出力対象となることを確認。
  ※ 実件数は入力CSVの内容を再計数してから判定し、固定値だけを信用しない。

## 8. 本番適用条件

ClaudeレビューPASS
→ TESTデータでT01〜T10
→ v3.1とのdiff確認
→ 必要ならMorningRoutineとのI/F確認
→ 正式版名を確定
→ PROD採用

レビュー前・TEST前にv3.1を上書きしない。
