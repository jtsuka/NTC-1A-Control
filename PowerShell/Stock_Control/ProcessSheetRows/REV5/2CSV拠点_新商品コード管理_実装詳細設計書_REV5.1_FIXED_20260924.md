# 2CSV拠点 新商品コード管理 実装詳細設計書 REV.5.1 FIXED

## Row範囲駆動I/F / r15.9 Rev3 Rebase

作成日: 2026-09-24\
状態: DESIGN FIXED / Claude GO WITH MINOR反映済み /
MainEndRow実Workbook確認済み\
実装状態: 未実装

------------------------------------------------------------------------

# 1. Baseline

本設計の唯一の実装Baseline:

``` text
ProcessSheetRows_20260917_r15_9_Rev3_MFlagUniversal_Phase3State_DuplicateFailClosed.ps1
SHA256=df7a06b42064b5401c67ed1d5719bdce7bb8dfbaa5df02f7a1fca1c5caed2e98
```

同名別ハッシュのファイルから実装を開始してはならない。

------------------------------------------------------------------------

# 2. 目的

外注鉄および成果連絡表（色物）のバーコード貼付シートには、現在数字シートで管理されていない商品が存在する。

これらを機械が直接数字シートへ登録するのではなく、

``` text
バーコード貼付シート
→ 未管理候補検出
→ 追加新規商品コード staging
→ 人が移動先数字シートを指定
→ 既存Phase3
→ 数字シートへ登録
```

の経路で安全に管理対象へ追加する。

2CSV拠点では同一商品コードがMain/Sub双方に存在し得るため、候補IdentityはCode単独ではなくSourceTypeを含める。

------------------------------------------------------------------------

# 3. REV.5.1 FIXEDの設計原則

1.  変数表をDataset Row範囲の正本とする。
2.  外注鉄、成果連絡表（色物）という拠点名を2CSV分岐へハードコードしない。
3.  A4000/A4034等のRowをPS1へハードコードしない。
4.  DatasetCountを1CSV/2CSV判定の正本とする。
5.  SubSiteCodeは補助情報・整合性確認用とし、2CSV判定の主条件にしない。
6.  Main/SubはStartRow/EndRowを変数表へ明示する。
7.  Row範囲外、重複、欠損、不正値を推測補正しない。
8.  Unknown
    Rowが1件でも発生した場合、その実行の新商品コード機能全体をfail-closedとする。
9.  CandidateKeyへRow番号を含めない。
10. Row設定変更後も過去HistoryのCandidateKeyを書き換えない。
11. SourceLabelは表示専用でCandidateKeyへ使用しない。
12. 1CSV拠点は現行Rev3互換を維持する。
13. Rev3の既存安全機構を弱めない。

------------------------------------------------------------------------

# 4. 2026-09-24 実Workbook確認

確認ファイル:

``` text
外注鉄在庫帳(20260924-123602).xlsx
成果連絡表（色物）(20260924-123601).xlsx
```

バーコード貼付シートを実測した。

## 4.1 外注鉄

``` text
Sub実データ       = Row 3～3923
Sub予約範囲       = Row 3～3999
MainStartRow      = 4000
Main実データ      = Row 4000～7388
Main予約範囲末尾 = Row 7999
Main実データ行数 = 3389
```

## 4.2 成果連絡表（色物）

``` text
Sub実データ       = Row 3～3923
Sub予約範囲       = Row 3～4033
MainStartRow      = 4034
Main実データ      = Row 4034～7422
Main予約範囲末尾 = Row 8033
Main実データ行数 = 3389
```

## 4.3 実証から採用する設定値

``` text
外注鉄
SubStartRow  = 3
SubEndRow    = 3999
MainStartRow = 4000
MainEndRow   = 7999

成果連絡表（色物）
SubStartRow  = 3
SubEndRow    = 4033
MainStartRow = 4034
MainEndRow   = 8033
```

現在の実データ末尾7388/7422は日々変動するためEndRowとして使用しない。

## 4.4 既存「CSV範囲_終端行=4000」の扱い

最新変数表には両拠点とも:

``` text
CSV範囲_終端行=4000
```

が存在する。

MainStartRowへ4000行の予約長として適用すると:

``` text
外注鉄       4000 + 4000 - 1 = 7999
成果連絡表   4034 + 4000 - 1 = 8033
```

となり、実Workbookの予約末尾と一致する。

ただし、この一致だけから既存列の業務上の本来意味を断定しない。

REV.5.1
FIXEDでは、この旧列の意味解釈や計算へPS1を依存させず、MainEndRow/SubEndRowを変数表へ明示する。

------------------------------------------------------------------------

# 5. 変数表 正式I/F

新商品コード管理用の最終I/F:

``` text
拠点名
在庫帳ブック名
新規コード管理_実行
新規コード管理_備考

CSVデータセット数

MainCsvFileName
MainStartRow
MainEndRow

SubCsvFileName
SubStartRow
SubEndRow
```

必要に応じて既存の他設定列は維持する。

新商品コード機能は旧StartCell列へフォールバックしない。
移行は一括移行を原則とする。

------------------------------------------------------------------------

# 6. NewProductConfig

``` powershell
[PSCustomObject]@{
    SiteName          = ...
    WorkbookFileName  = ...
    Enabled           = ...
    Note              = ...

    DatasetCount      = ...

    MainCsvFileName   = ...
    MainStartRow      = ...
    MainEndRow        = ...

    SubCsvFileName    = ...
    SubStartRow       = ...
    SubEndRow         = ...

    Row               = ...
}
```

------------------------------------------------------------------------

# 7. DatasetCount

``` text
DatasetCount=1
```

従来1CSV。

``` text
DatasetCount=2
```

Main/Sub 2Dataset。

空欄、不正値、3以上は新商品コード機能fail-closed。

DatasetCountが正本。 SubSiteCodeだけからIsTwoDatasetを決めない。

------------------------------------------------------------------------

# 8. Config Preflight

`新規コード管理_実行=有` の場合、候補検出より前に実施する。

DatasetCount=1: - 2CSV Row設定を要求しない。 - 現行1CSV処理へ進む。

DatasetCount=2: - MainStartRow/MainEndRow/SubStartRow/SubEndRowが整数 -
各StartRow \>= 3 - MainEndRow \>= MainStartRow - SubEndRow \>=
SubStartRow - Main/Sub範囲が重複しない -
MainCsvFileName/SubCsvFileNameが空欄でない - 必須設定が欠損していない

異常時:

``` text
ERROR NewProductDatasetConfigInvalid
NewProductFeatureError=True
Phase1=SKIP
Phase3=SKIP
通常在庫処理=CONTINUE
```

------------------------------------------------------------------------

# 9. Row → SourceType

新関数:

``` text
Get-NewProductSourceForRow
```

入力:

``` text
SourceRow
NewProductConfig
```

判定:

``` text
MainStartRow <= SourceRow <= MainEndRow
    → SourceType=Main

SubStartRow <= SourceRow <= SubEndRow
    → SourceType=Sub
```

どちらにも属さない:

``` text
SourceType=Unknown
Reason=OutOfDatasetRange
```

二重該当:

``` text
SourceType=Unknown
Reason=OverlappingDatasetRange
```

UnknownをMain/Subへ丸めない。

------------------------------------------------------------------------

# 10. Unknown Row Fail-closed

Unknown
Rowが1件でも発生した場合、当該candidateだけを除外して継続してはならない。

理由:
境界設定自体が実データと食い違っている可能性があり、他candidateのSourceTypeも信用できないため。

動作:

``` text
ERROR NewProductSourceUnknown
NewProductFeatureError=True
その実行の新商品コードPhase1/Phase3を停止
通常在庫処理は継続
```

------------------------------------------------------------------------

# 11. CandidateKey

1CSV:

``` text
CandidateKey=Code
SourceType=""
SourceLabel=""
```

2CSV:

``` text
CandidateKey=Code + "|" + SourceType
```

例:

``` text
ABC123|Main
ABC123|Sub
```

Row番号をCandidateKeyへ含めない。

------------------------------------------------------------------------

# 12. SourceLabel

SourceLabelは人向け表示専用。

例:

``` text
Main / 外注在庫
Sub  / 倉庫在庫
```

SourceLabel変更でCandidateKeyは変化しない。

------------------------------------------------------------------------

# 13. Row設定変更とHistory

将来:

``` text
MainStartRow 4000 → 4500
```

等へ変更しても、既存History:

``` text
ABC123|Main
```

は変更しない。

Row範囲は実行時のSourceType分類にのみ使用する。
過去HistoryのCandidateKey再構築・一括書換は禁止。

------------------------------------------------------------------------

# 14. staging

A:Fは既存互換を維持し、2CSV情報をG/Hへ追加する。

``` text
A Code
B Name
C Stock
D Destination
E FirstDetected
F State
G SourceType
H SourceLabel
```

1CSVではG/H空欄を許可。

2CSVではPhase3対象行のSourceType欠落をBlock。

SourceRowはログ/CSV診断情報には保持してよいが、staging
Identityには使用しない。

------------------------------------------------------------------------

# 15. History

登録履歴:

``` text
新規商品コード登録履歴
```

最低限:

``` text
CandidateKey
Code
SourceType
SourceLabel
DestinationSheet
DestinationRow
RegisteredAt
State
Note
```

2CSVの既知判定はCandidateKeyで行う。

History同一CandidateKey重複は異常としてBlock。

------------------------------------------------------------------------

# 16. Baseline

2CSV初回有効化時、既に数字シートで管理されているCodeについてSourceTypeを数字シートから推測しない。

候補側のSourceTypeと現行管理Codeの存在を突合し、BaselineManagedとして人確認を経て初期化する。

Baseline未完了で2CSV通常Updateを開始しない。

------------------------------------------------------------------------

# 17. Phase3 Preflight

REV.5 FIXEDのCritical項目を維持する。

1CSV: - 現行Codeベース挙動を維持。

2CSV: - duplicate判定 = CandidateKey - History判定 = CandidateKey -
runtimeSeen = CandidateKey -
同Code/別Sourceはそれだけではエラーにしない -
ManagedCodes.Contains(Code)だけで即Blockしない - numeric
sheet上のCode存在は診断情報 -
SourceType欠落/CandidateKey生成不能はBlock - Baseline未完了はBlock

同Code/同Destination/別Sourceは、人が明示的に同じDestinationを指定した場合は許可しWARN/Historyへ残す。

------------------------------------------------------------------------

# 18. staging削除ガード

1CSV: 既存ガードを維持。

2CSV:

``` text
Code
Destination
SourceType
```

3点一致を必須とする。

------------------------------------------------------------------------

# 19. DuplicateCandidateFailClosed

現行Rev3のLegacy M安全装置を変更しない。

実コード上:

``` text
Invoke-Phase3Move
→ Remove-Phase3StagingRows
→ Invoke-LegacyGapRepair
→ Invoke-LegacyMFlagGapRepair
→ Workbook.Save()
```

`DuplicateCandidateFailClosed` は `Invoke-LegacyMFlagGapRepair`
内のworksheet単位の安全skipであり、Phase3 MoveやSaveを停止しない。

同Code/同Destination/別Sourceを同一数字シートへ登録した場合、当日はphase3BySheet除外によりLegacy
M重複カウントから除外されるが、翌日以降は通常行として重複検出され、その数字シートのLegacy
M repairがskipされ続ける可能性がある。

これは既知の運用副作用として記録するが、安全装置を弱めない。

------------------------------------------------------------------------

# 20. 1CSV回帰

DatasetCount=1では2CSV Row範囲SourceTypeロジックを通さない。

``` text
CandidateKey=Code
SourceType=""
SourceLabel=""
```

既存6列stagingでも従来Phase3を実行可能とする。

------------------------------------------------------------------------

# 21. 新規コード管理_実行=無

2026-09-24最新版変数表では:

``` text
外注鉄             = 無
成果連絡表（色物） = 無
```

実装・Audit・Baseline確認が完了するまで本番では「無」を維持する。

無の場合、新商品コード機能を完全SKIPし、Row範囲ロジックへ到達しない。

------------------------------------------------------------------------

# 22. 変数表移行

採用:

``` text
一括移行
```

旧StartCell互換フォールバックは原則実装しない。

理由: - 新旧どちらで動いたかという追加状態を持ち込まない -
設定経路を一本化する - 誤設定をPreflightで明示的に検出する -
保守範囲を局所化する

移行順:

1.  変数表へ新Row列を追加
2.  対象拠点へ実証済み値を設定
3.  新規コード管理_実行は「無」のまま
4.  PS1実装
5.  Audit
6.  Config Preflight確認
7.  DatasetBoundary/SourceType確認
8.  Baseline候補確認
9.  人確認
10. Baseline初期化
11. 少数Phase3
12. 翌営業日冪等性確認
13. 対象拠点を段階的に「有」へ

------------------------------------------------------------------------

# 23. ログ

最低限:

``` text
INFO NewProductDatasetConfig DatasetCount=2 MainRange=4000-7999 SubRange=3-3999
PASS NewProductDatasetConfigValidated
INFO NewProductSource Code=ABC123 Row=125 SourceType=Sub CandidateKey=ABC123|Sub
ERROR NewProductSourceUnknown Code=XYZ999 Row=8000 Reason=OutOfDatasetRange
ERROR NewProductDatasetConfigInvalid Reason=OverlappingDatasetRange
INFO CrossSourceSameCode Code=ABC123 Keys=ABC123|Main,ABC123|Sub
WARN SameSourceDuplicate CandidateKey=ABC123|Main Count=2
WARN LegacyManagedUnknownSource Code=ABC123 SourceType=Sub
WARN Phase3SameCodeSameDestination ...
```

------------------------------------------------------------------------

# 24. 実装変更対象

既存変更対象:

``` text
Load-VariableTableData
Get-NewProductConfig
Get-ExistingNewProductCodes
Get-NewProductCandidates
Export-NewProductCandidateCsv
Write-NewProductStaging
Get-Phase3StagingCandidates
Get-Phase3Preflight
Export-Phase3CandidateCsv
Invoke-Phase3Move
Remove-Phase3StagingRows
```

メイン処理:

``` text
duplicateCodes
runtimeSeen
Phase3Moved後処理
```

新設候補:

``` text
Test-NewProductDatasetConfig
Get-NewProductSourceForRow
Get-NewProductCandidateKey
Get-NewProductHistoryKeys
Write-NewProductRegistrationHistory
Test-NewProductHistorySchema
Get-NewProductBaselineCandidates
Initialize-NewProductBaseline
```

------------------------------------------------------------------------

# 25. 変更禁止領域

以下の現行Rev3ロジックを意図的に変更しない。

``` text
Universal M policy
Phase3 F/H/E initial state
Invoke-LegacyGapRepairの既存判定
Invoke-LegacyMFlagGapRepairの安全判定
DuplicateCandidateFailClosed
FormulaRestore
reorder point processing
single final Save
MaxNewProductMoves
Audit non-update
```

------------------------------------------------------------------------

# 26. テスト

## Config

``` text
C01 DatasetCount=1 → 従来1CSV
C02 DatasetCount=2 + 正常範囲 → PASS
C03 Main/Sub範囲重複 → 新商品機能fail-closed
C04 Start/End逆転 → fail-closed
C05 非数値 → fail-closed
C06 欠損 → fail-closed
C07 DatasetCount=3 → fail-closed
```

## Boundary

``` text
B01 Row=MainStartRow → Main
B02 Row=MainEndRow → Main
B03 Row=SubStartRow → Sub
B04 Row=SubEndRow → Sub
B05 範囲外Row → Unknown / 機能全体fail-closed
B06 二重該当 → Unknown / 機能全体fail-closed
```

## Identity

``` text
I01 同Code Main/Sub → 別CandidateKey
I02 同Code同Source複数 → duplicate/manual review
I03 Row変更後も同SourceならCandidateKey不変
I04 SourceLabel変更でもCandidateKey不変
```

## staging/history

``` text
S01 2CSV staging A:H
S02 既存A:F保持
S03 2CSV SourceType欠落 → Phase3 Block
S04 History同一CandidateKey二重 → Block
S05 Move失敗 → History確定なし
S06 Save失敗 → Workbook全変更未確定
```

## Phase3

``` text
P01 1CSV既存Code → 従来挙動
P02 2CSV Main/Sub同Code → 許可
P03 同CandidateKey → Block
P04 別Destination → 両方可
P05 同Destination → 両方可 + WARN
P06 numeric Code存在 + 別Source → History/Baseline条件が整えば許可
P07 SourceType欠落 → Block
```

## Regression

``` text
R01 1CSV候補件数がBaseline Rev3と一致
R02 LegacyGapRepair不変
R03 LegacyMFlagGapRepair安全判定不変
R04 FormulaRestore不変
R05 通常在庫候補件数不変
R06 Save位置1回不変
R07 Audit Workbook不変
R08 新規コード管理_実行=無 → 完全SKIP
```

------------------------------------------------------------------------

# 27. Claude REV.5.1レビュー反映

Claude判定:

``` text
REV.5.1 ROW RANGE DESIGN: GO WITH MINOR
```

PASS: - DatasetCount正本 - SubSiteCodeを主条件から外す - Main/Sub
Start/End明示の設計 - Config Preflight - Row範囲重複fail-closed -
Row範囲外fail-closed - Unknownを推測補正しない -
RowをCandidateKeyへ含めない - History維持 - SourceLabel非Identity -
1CSV回帰 - staging A:H - Phase3 Preflight整合 -
duplicateCodes/runtimeSeen整合 - Baseline/History整合 -
DuplicateCandidateFailClosed不変 - 新規コード管理_実行=無の完全SKIP

MINOR: - MainEndRowの実値が変数表だけでは明確でない -
StartCell→StartRow/EndRow移行方法

上記MINORは2026-09-24の実Workbook確認により、本設計では明示Rowとして解消した。

------------------------------------------------------------------------

# 28. 実装前Gate

``` text
[x] REV5 FIXED DesignReview PASS
[x] Claude REV5.1 GO WITH MINOR
[x] 本番Rev3実体取得
[x] Baseline SHA256確定
[x] 最新変数表確認
[x] 外注鉄 DatasetCount=2確認
[x] 色物 DatasetCount=2確認
[x] 両拠点 新規コード管理_実行=無確認
[x] 外注鉄 実Workbook Row境界確認
[x] 色物 実Workbook Row境界確認
[x] MainEndRow実証値確定
[x] StartRow/EndRow一括移行方針確定
[ ] Baseline PS1保全コピー
[ ] 変数表へ新Row列を実際に追加・値設定
[ ] 実装コード作成
```

------------------------------------------------------------------------

# 29. 最終状態

``` text
DesignReview=PASS
REV5.1RowRangeReview=GO WITH MINOR → MINOR resolved by actual workbook inspection
CriticalDesignIssueRemaining=0
DatasetBoundarySource=VariableTable explicit StartRow/EndRow
UnknownRowPolicy=FailClosedNewProductFeature
CandidateKeyPolicy=Code|SourceType for 2CSV
HistoryPolicy=StableAcrossRowChanges
LegacySafetyGuard=KEEP
ImplementationStatus=NOT STARTED
```

実装は、本設計とBaseline SHA256を固定した後に開始する。
