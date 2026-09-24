# Format-KonyuCSV_RFC v3.2 OrderNoFilter 正式仕様書

- 正式版: `Format-KonyuCSV_RFC_yyyyMMdd_v3.2_OrderNoFilter_FINAL.ps1`
- 正式採用日: 2026-09-23
- 運用開始日: 2026-09-24
- 実行環境: Windows PowerShell 5.1
- 運用形態: **単体運用**
- MorningRoutine統合: **未実施。別案件として必要性を再検討する。**

## 1. 目的

正式採用中だった `Format-KonyuCSV_RFC_yyyyMMdd_v3.1.ps1` を母体とし、既存のRFC正規化・列ずれ補正・日付4桁年化・CP932出力等を維持したまま、明示指定時だけ「手配書番号」が空欄の行を除外できるようにする。

既定動作はv3.1互換とし、`-ExcludeMissingOrderNo` を指定しない場合は手配書番号の有無にかかわらず全行をRFC正規化して出力する。

## 2. 正式I/F

```powershell
param(
    [Parameter(Mandatory=$true)]
    [string]$InputPath,

    [Parameter(Mandatory=$false)]
    [string]$OutputPath = "",

    [Parameter(Mandatory=$false)]
    [switch]$ExcludeMissingOrderNo
)
```

### 通常運用（RFC正規化のみ）

```powershell
.\Format-KonyuCSV_RFC_yyyyMMdd_v3.2_OrderNoFilter_FINAL.ps1 `
  -InputPath  "C:\HPDB\input.csv" `
  -OutputPath "C:\HPDB\output.csv"
```

この場合、`-ExcludeMissingOrderNo` は付けない。

### 手配書番号空欄を除外する運用

```powershell
.\Format-KonyuCSV_RFC_yyyyMMdd_v3.2_OrderNoFilter_FINAL.ps1 `
  -InputPath  "C:\HPDB\input.csv" `
  -OutputPath "C:\HPDB\output.csv" `
  -ExcludeMissingOrderNo
```

## 3. `-ExcludeMissingOrderNo` の仕様

- 指定なし: 従来どおり全行を出力する。
- 指定あり: 「手配書番号」が空欄の行だけを出力対象から除外する。
- 「手配書番号なし行だけを抽出する」機能ではない。
- スイッチ指定時に「手配書番号」列が存在しなければFail Closedする。

### 空欄として扱う値

- 空文字
- 半角スペースのみ
- 全角スペースのみ
- 半角／全角スペースの混在

## 4. 判定順序

```text
CSV 1行読込
↓
Parse-CsvLine
↓
必要なら Repair-ColumnShift
↓
日付変換 / 文字列トリム
↓
列数再確認
↓
-ExcludeMissingOrderNo 指定時だけ手配書番号空欄判定
↓
空欄なら除外
空欄でなければRFC 4180形式へ再構築
```

列ずれ補正前の生配列では判定しない。必ず `Repair-ColumnShift` 後の正しい列位置で判定する。

## 5. v3.1から維持する仕様

- Windows PowerShell 5.1対応
- CP932読込
- CP932 / BOMなし出力
- CRLF出力
- RFC 4180形式の全項目ダブルクォート
- 内部ダブルクォートの二重化
- `Parse-CsvLine`
- `Repair-ColumnShift` の既存ロジック
- 商品名／商品情報に起因する既存列ずれ補正
- 日付列ヘッダー判定
- 2桁年 `yy-MM-dd` の4桁年化
- 出力日付 `yyyy-MM-dd`
- 前後スペース／全角スペースのトリム
- InputPath=OutputPath禁止
- 入力ファイル存在確認
- 出力先フォルダ存在確認
- ExitCodeの基本方針

## 6. 安全仕様

### InputPath = OutputPath
同一ファイル指定は禁止し、書込み前に停止する。元CSVの破損を防ぐ。

### 手配書番号列なし + 除外スイッチ
`-ExcludeMissingOrderNo` 指定時に対象列が見つからなければ、書込み前にFail Closedする。

### 元ファイル保護
正式運用でも原則として元CSVをOutputPathに指定しない。RFC正規化済みCSVは別名で保存する。

## 7. 正式採用試験

2026-09-23に以下を実施し、すべてPASSした。

### T01〜T11 単体回帰試験

- T01: スイッチなしでv3.1と出力がバイト単位一致
- T02: 全件手配書番号あり + switch → 除外0
- T03: 空欄混在 + switch → 空欄だけ除外
- T04: 空欄混在 + switchなし → 全件出力
- T05: 手配書番号列なし + switch → Fail Closed
- T06: `26-09-23` → `2026-09-23`
- T07: `Repair-ColumnShift` 後も手配書番号を正しく判定
- T08: CP932 / BOMなし / CRLF / RFC4180維持
- T09: InputPath=OutputPath SafetyStop
- T10: 実937件想定CSVで再計数し、327件を正しく出力
- T11: 全角スペースのみの手配書番号を空欄扱いして除外

結果: **PASS 11 / FAIL 0 / OVERALL PASS**

### 2026-09-23 実データ RFC正規化試験

対象: `外注課納期管理_20260923.csv`

- 入力: 322件
- 列数: 141列
- 手配書番号あり: 322件
- 手配書番号空欄: 0件
- `-ExcludeMissingOrderNo`: 未指定
- 出力: 322件
- 出力列数不一致: 0件
- BOMなし: PASS
- CRLF: PASS
- RFC4180全項目quote/roundtrip: PASS
- 元CSV SHA256不変: PASS

結果: **PASS 10 / FAIL 0 / OVERALL PASS**

## 8. 運用方針

2026-09-24から本FINAL版を単体Formatterとして運用開始する。

当面はMorningRoutineへマージしない。単体運用で安定性を確認した後、統合の必要性とリスクを別途評価する。

## 9. 正式採用判定

Claudeコードレビュー: PASS  
T01〜T11: PASS  
実データRFC正規化試験: PASS  
v3.1互換性: PASS  
安全停止: PASS

**総合判定: FINAL / 正式採用**
