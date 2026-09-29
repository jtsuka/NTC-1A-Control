# 2CSV拠点 新商品コード管理 実装詳細設計書 REV.5
作成日: 2026-09-24  
基準実装: `ProcessSheetRows_20260917_r15_9_Rev3_MFlagUniversal_Phase3State_DuplicateFailClosed.ps1`  
前身設計: `2CSV拠点_新商品コード管理_実装詳細設計書_REV4_STATIC_REVIEW_PASS_20260914.md`  
状態: **設計クロスレビュー依頼版 / 実装未着手**

---

## 0. REV.5の目的

REV.4はr15.7/r15.8系を基準に2CSV拠点の新商品コード管理を設計した。
その後、本番系はr15.9 Rev3へ進み、以下が正式な安定機能として追加・実証された。

- J>0管理対象行のM=1一律化
- Phase3新規行のJ>0 → M=1 / J<=0 → M空欄
- Phase3新規行のG=1かつF空欄 → F=当日
- Phase3新規行のI=1かつH空欄 → E=E+1、H=当日
- Legacy M補修の `DuplicateCandidateFailClosed`
- 2026-09-17本社本番相当試験で上記を実証
- 2026-09-24通常運用でも全対象が完走し、Phase3実移動を含め正常動作を確認

REV.5は、REV.4の2CSV設計をこのr15.9 Rev3へ再ベースする。
目的は、**外注鉄・成果連絡表（色物）のバーコード貼付シートに存在するが、数字シートで未管理の商品を「追加新規商品コード」へ安全に待避し、既存Phase3へ引き渡すこと**である。

最重要原則:

> **Rev3の安定機能を壊さない。新商品コード関連だけを局所変更する。**

---

# 1. 設計原則

1. 機械は移動先数字シートを推測しない。
2. Main/Sub双方に同じ商品コードが存在しても、それ自体を異常重複としない。
3. 同一Source内の同一商品コード重複は人確認とする。
4. 候補の識別と、数字シート上の既存登録判定を同じキー概念に押し込まない。
5. SourceTypeを数字シートから推測・復元しない。
6. 1CSV拠点は現行Rev3の挙動を完全維持する。
7. 不整合時は新商品コード機能だけをfail-closedし、既存通常在庫処理への影響を最小化する。
8. Phase3実行途中の整合性破壊はRev3既存方針どおりSave前throwとし、Workbook変更を確定しない。

---

# 2. 用語とキーの分離

REV.5では以下を明確に分離する。

## 2.1 Code

商品コードそのもの。

例:

```text
ABC123
```

数字シートは基本的にCodeを保持し、SourceTypeは保持しない。

## 2.2 CandidateKey

バーコード貼付／staging／履歴上で「どのソース由来の候補か」を識別するキー。

1CSV:

```text
CandidateKey = Code
```

2CSV:

```text
CandidateKey = Code + "|" + SourceType
```

例:

```text
ABC123|Main
ABC123|Sub
```

## 2.3 DestinationIdentity

数字シート側にはSourceTypeが存在しないため、CandidateKeyをそのまま数字シートの一意キーにはできない。

Phase3で数字シートへの配置を確認する際は少なくとも、

```text
Code + DestinationSheet
```

を別概念として扱う。

ただし、2CSVで同一Code・同一DestinationへMain/Sub双方を配置することを許可する場合、これだけでも一意ではない。
したがってSourceTypeの永続的な識別は登録履歴で担保する。

**CandidateKeyとDestinationIdentityを混同しないこと。**

---

# 3. Rev3で変更禁止とする領域

今回の実装では以下の既存ロジックを原則変更しない。

- J>0管理対象行のM=1一律化
- Phase3新規行のM初期化
- Phase3新規行のF/H状態初期化
- `Invoke-LegacyMFlagGapRepair` の既存補修ロジック
- `DuplicateCandidateFailClosed` の安全装置
- LegacyGapRepair
- FormulaRestore
- 発注点関連処理
- Phase3の数式テンプレート復元
- 最終Save責任一本化
- Move成功後のみstaging行削除
- `MaxNewProductMoves`
- Audit非更新原則

変更が必要になった場合は、REV.5設計変更として再レビューする。

---

# 4. 対象拠点

対象候補:

- 外注鉄
- 成果連絡表（色物）

ただし拠点名によるハードコード分岐は行わない。

正本は拠点別変数表の2CSV設定とする。

2CSV判定の第一候補:

```text
副拠点コードが空欄ではない
```

`CSVデータセット数=2` は整合性確認に使用する。

実装前に最新の拠点別変数表とRK-10実運用を照合し、外注鉄・色物双方の現行設定を確定する。

---

# 5. 待避シート互換性

シート名:

```text
追加新規商品コード
```

A:FはRev3互換として固定する。

```text
A 商品コード
B 商品名
C 在庫数
D 移動先シート
E 初回検知日
F 状態
G SourceType
H SourceLabel
```

A:Fの意味・位置を変更しない。

1CSV拠点ではG/Hが存在しなくても従来動作可能とする。
2CSVで新規作成または拡張する場合のみG/Hを使用する。

SourceType:

```text
Main
Sub
```

SourceLabelは表示専用であり、キー生成には使用しない。

---

# 6. 2CSV設定読込

`Load-VariableTableData` / `Get-NewProductConfig` 相当へ、最低限以下を保持させる。

```text
SiteName
WorkbookFileName
Enabled
Note
DatasetCount
SubSiteCode
MainCsvFileName
SubCsvFileName
MainCsvStartCell
SubCsvStartCell
SubCsvEndRow
```

Main/Subと、シート上のUpper/Lowerを混同しない。

開始セルは固定行をコードへハードコードせず、変数表から取得する。

---

# 7. DatasetBoundary

REV.4のFixReorderPoint r18.1互換設計を維持する。

関数候補:

```text
ConvertTo-NewProductStartRow
Get-NewProductDatasetBoundary
Get-NewProductSourceTypeForRow
```

原則:

- 小さい開始行 = Upper
- 大きい開始行 = Lower
- Main/Subは開始位置から別途対応付ける
- MainStart == SubStart → Invalid
- Start < 3 → Invalid
- Start > LastBarcodeRow → Invalid
- A列以外の開始セル → Invalid
- SourceTypeを決定できない行 → 自動候補化しない

2CSV境界異常時:

```text
NewProductFeatureError=True
Phase1 SKIP
Phase3 SKIP
通常在庫処理は継続
```

---

# 8. Phase1候補検出

現行Rev3のCode単独集約を、2CSV時だけCandidateKey単位へ変更する。

1CSV:

```text
key = Code
```

2CSV:

```text
key = Code|SourceType
```

候補オブジェクト:

```text
Code
Name
Stock
SourceRow
SourceType
SourceLabel
CandidateKey
DuplicateCount
State
```

同一CandidateKey内の複数行:

```text
SameSourceDuplicate
ManualReview
```

cross-source:

```text
ABC123|Main
ABC123|Sub
```

は別候補として正常。

---

# 9. ExistingNewCodesの改修【Claude指摘 #3】

現行 `Get-ExistingNewProductCodes` はA列CodeのみのHashSetである。

REV.5では責務を分離する。

```text
Get-ExistingNewProductKeysFromStaging
Get-NewProductHistoryKeys
```

1CSV:

```text
StagingKey = Code
```

2CSV:

```text
StagingKey = Code|SourceType
```

2CSVの既知候補判定:

```text
KnownKeys = StagingKeys ∪ HistoryKeys
```

**現行のCode単独ExistingNewCodesを2CSV候補抑止へ流用しない。**

---

# 10. ManagedCodesの扱い【Claude指摘 #3/#4】

ここはCandidateKeyと分離する。

## 10.1 1CSV

現行Rev3を維持。

```text
ManagedCodes.Contains(Code)
```

なら新商品候補から除外し、Phase3でも既存登録としてブロックする。

## 10.2 2CSV

`ManagedCodes` は数字シート由来のCode集合でありSourceTypeを持たない。

したがって、

```text
ManagedCodes.Contains(Code)
```

をそのままCandidateKeyの登録済み判定には使用しない。

また、ManagedCodes自体を疑似的に `Code|SourceType` 化してはならない。

2CSVでは、

- staging/history上のCandidateKey
- 数字シート上のCode存在
- DestinationSheet
- Baseline状態

を別々に評価する。

数字シートにCodeがあるが、該当CandidateKeyの履歴がない場合は自動でMain/Subを推測しない。

状態:

```text
LegacyManagedUnknownSource
```

として確認対象へ送る。

---

# 11. CandidateKey永続化

REV.4の判断を維持する。

数字シートにはSourceTypeがないため、Phase3成功後にstaging行を削除すると、翌日以降にどのSourceTypeを登録済みか復元できない。

よって同一Workbook内に、

```text
新規商品コード登録履歴
```

を持つ。

列:

```text
A CandidateKey
B 商品コード
C SourceType
D SourceLabel
E 移動先シート
F 移動先行
G 登録日
H 状態
I 備考
```

状態:

```text
Registered
BaselineManaged
```

最低限この2種。

---

# 12. Baseline

初回2CSV導入時、既存数字シートのCodeがMain/Subどちら由来か機械判断してはならない。

Baseline候補条件:

```text
ManagedCodes.Contains(Code)
AND NOT HistoryKeys.Contains(CandidateKey)
AND NOT StagingKeys.Contains(CandidateKey)
```

Stage B0 AuditでCSV出力のみ。
人確認後の専用Baseline Initializeでのみ `BaselineManaged` を登録する。

通常Phase1が勝手にBaselineを作成してはならない。

---

# 13. Baseline状態

REV.4の専用セル方式を維持する。

`新規商品コード登録履歴`:

```text
K1 = BaselineStatus
L1 = Initialized / NotInitialized

K2 = BaselineInitializedAt
L2 = yyyy/mm/dd hh:mm:ss

K3 = BaselineInitializedSite
L3 = SiteName
```

2CSV通常Update開始条件:

```text
History sheet exists
AND L1 == "Initialized"
AND L3 == current SiteName
```

Baseline対象0件でもInitializedは成立可能。

---

# 14. Phase3 Preflight【Critical #1】

Claude実コード確認により、現行Rev3は以下をCode単独で判定している。

```powershell
DuplicateCodes.Contains($Candidate.Code)
ManagedCodes.Contains($Candidate.Code)
```

これは2CSV要件と直接衝突するため、**実装必須のCritical変更点**とする。

## 14.1 1CSV

Rev3現行契約を完全維持。

- staging内duplicate Code → ERROR
- ManagedCodes.Contains(Code) → ERROR
- numeric sheet Code存在 → ERROR
- runtimeSeen Code → ERROR

## 14.2 2CSV

- staging内duplicate CandidateKey → ERROR
- HistoryKeys.Contains(CandidateKey) → ERROR
- runtimeSeen CandidateKey → ERROR
- same Code / different SourceType → それだけではERRORにしない
- ManagedCodes.Contains(Code)だけでは即ERRORにしない
- numeric sheetにCodeが存在するだけでは即ERRORにしない
- SourceType欠落 → ERROR
- CandidateKey生成不能 → ERROR
- Baseline未Initialized → Phase3禁止

ただし既存Code存在は必ず診断ログへ残す。

例:

```text
INFO Phase3ExistingCodeObserved
Code=ABC123
CandidateKey=ABC123|Sub
Action=NotBlockedByCodeAlone
```

---

# 15. Test-Phase3CodeExistsInNumericSheets

関数自体をCandidateKey対応へ無理に変更しない。

理由:

**数字シートにはSourceTypeがないためCandidateKeyを検査できない。**

1CSV:
従来どおりブロック条件。

2CSV:
診断目的で使用可能。
結果だけをCode単独のブロック条件には使用しない。

---

# 16. runtimeSeen【再確認項目】

Claudeレビューでは現行Rev3に2CSVロジックが存在しないことは確認された。

しかしREV.5実装後は明示的に以下へ分岐する必要がある。

1CSV:

```text
runtimeSeen = Code
```

2CSV:

```text
runtimeSeen = CandidateKey
```

よって「現状影響なし」と「REV.5で変更不要」は同義ではない。

**REV.5では変更対象とする。**

---

# 17. 同一Code・同一Destination

REV.4の最終設計を維持する。

2CSVでSourceTypeが異なれば、

```text
ABC123|Main → Sheet5
ABC123|Sub  → Sheet5
```

も人が明示的にDestinationを指定した結果として許可する。

ただし診断ログを出す。

```text
WARN Phase3SameCodeSameDestination
Action=AllowedByHumanDecision
```

SourceTypeの識別は登録履歴で保持する。

---

# 18. staging削除ガード【Claude指摘 #7】

現行Rev3の

```text
A = Code
D = Destination
```

照合を維持する。

2CSVでは追加で、

```text
G = SourceType
```

を照合する。

削除条件:

```text
Code一致
AND Destination一致
AND SourceType一致
```

1CSVでは現行ガードを変更しない。

---

# 19. Phase3 Move後の履歴

推奨順:

```text
1. 全Phase3 Move成功
2. Rev3既存のPhase3新規行初期状態処理
   - M
   - F/H/E
   - 数式/テンプレート処理
3. 登録履歴へMovedItems追加
4. 履歴追加内容を検証
5. staging成功行削除
6. Rev3既存後続処理
7. FullCalculation
8. Workbook.Save 1回
```

実際のRev3関数呼出順と照合し、既存順序を不必要に変更しない。

履歴検証:

```text
HistoryAdded == Phase3Moved
CandidateKey一致
Code一致
SourceType一致
DestinationSheet一致
DestinationRow一致
```

不一致ならSave前throw。

---

# 20. DuplicateCandidateFailClosed【Critical再評価対象】

Rev3の `DuplicateCandidateFailClosed` はLegacy M補修用の安全装置である。

Phase3 PreflightのCode単独重複ガードとは責務が異なる。

したがってREV.5では、これを安易に2CSV例外化・削除・CandidateKey化しない。

クロスレビューで以下を実コードフローから確認する。

1. `DuplicateCandidateFailClosed` のCodeCountsは「同一数字シート内」か。
2. Phase3でMain/Sub同一Codeを同一Destinationへ許可した場合、同一数字シート内Code重複として検出されるか。
3. 検出された場合、停止するのはLegacy M補修だけか。
4. Phase3 MoveそのものやSave全体を停止する経路があるか。
5. M補修がfail-closedになった場合、Phase3新規行自身のM初期化は既に完了しているか。
6. 翌日以降、その数字シートのLegacy M補修が継続的にskipされることを業務上許容できるか。

### 暫定方針

**Rev3の安全装置を弱めない。**

もし2CSVの正当な同一Code配置が `DuplicateCandidateFailClosed` を発火させる場合でも、まずは安全停止を維持する。

その影響をログ・テストで評価した後に、必要なら別REVで「正当な2CSV由来重複を識別可能にする方法」を設計する。

SourceTypeが数字シートに存在しない現状で、Code重複だけを見て安全装置を迂回させる実装は禁止する。

よってClaudeの「Critical」判定は、**干渉可能性としてCritical扱いを維持するが、現時点で安全装置変更を要求するCriticalとは確定しない。**

---

# 21. 1CSV回帰

1CSVでは以下を保証する。

- CandidateKey=Code
- Existing判定はCode
- ManagedCodes判定はCode
- duplicate判定はCode
- runtimeSeenはCode
- numeric global guardはCode
- SourceType不要
- G/H不要
- History/Baseline機能を要求しない
- Rev3 M/F/H/LegacyGap/FormulaRestore/Save挙動不変
- `新規コード管理_実行=無` なら新規機能完全SKIP

---

# 22. Fail-closed条件

2CSV新商品コード機能のみ停止:

- 2CSV設定不足
- 開始セル形式不正
- Main/Sub開始行同一
- SourceType判定不能
- staging SourceType欠落
- CandidateKey生成不能
- History schema不正
- History CandidateKey重複異常
- Baseline未Initialized
- Baseline Site不一致
- staging/history整合性異常

Workbook全体Save前throw:

- Phase3 Move途中の整合性破壊
- History追加失敗
- History検証失敗
- staging削除ガード不一致
- Rev3既存の重大Phase3エラー

---

# 23. ログ仕様

最低限追加:

```text
INFO NewProductDatasetMode IsTwoDataset=True
INFO DatasetBoundary ...
INFO NewProductCandidate Code=... SourceType=... CandidateKey=...
INFO CrossSourceSameCode ...
WARN SameSourceDuplicate ...
WARN LegacyManagedUnknownSource ...
INFO BaselineCandidateCount ...
PASS BaselineInitialized ...
INFO Phase3ExistingCodeObserved ...
WARN Phase3SameCodeSameDestination ...
UPDATE NewProductHistoryAdded ...
PASS NewProductHistoryVerified ...
WARN DuplicateCandidateFailClosed ...
```

ログから「正常なcross-source」と「危険なsame-source」を区別できること。

---

# 24. CSV監査出力

## NewProductCandidates

```text
Code
Name
Stock
SourceRow
SourceType
SourceLabel
CandidateKey
DuplicateCount
KnownInStaging
KnownInHistory
ManagedCodeExists
State
```

## Phase3MoveCandidates

```text
Code
SourceType
SourceLabel
CandidateKey
DestinationSheet
HistoryKeyExists
ManagedCodeExists
ValidationStatus
Reason
```

## BaselineCandidates

```text
Code
SourceType
SourceLabel
CandidateKey
SourceRow
ManagedCodeExists
KnownInStaging
KnownInHistory
Action
```

---

# 25. 実装変更対象

既存関数候補:

```text
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

```text
duplicateCodes生成
runtimeSeen
global duplicate guard
Phase3Moved後処理
```

新設候補:

```text
ConvertTo-NewProductStartRow
Get-NewProductDatasetBoundary
Get-NewProductSourceTypeForRow
Get-NewProductCandidateKey
Get-NewProductHistoryKeys
Write-NewProductRegistrationHistory
Test-NewProductHistorySchema
Get-NewProductBaselineCandidates
Initialize-NewProductBaseline
```

`Invoke-LegacyMFlagGapRepair` は原則変更対象外。

---

# 26. テスト仕様

## A. 1CSV回帰

```text
A01 Rev3通常候補件数が基準値と一致
A02 Phase3既存動作一致
A03 M列一律化一致
A04 F/H初期化一致
A05 DuplicateCandidateFailClosed一致
A06 LegacyGapRepair一致
A07 FormulaRestore一致
A08 AuditでWorkbook不変
A09 Save責任1回
```

## B. DatasetBoundary

```text
B01 Main=A3 Sub=A4000
B02 Main=A4000 Sub=A3
B03 Main=A3 Sub=A4034
B04 Main=Sub → Invalid
B05 A列以外 → Invalid
B06 Start<3 → Invalid
B07 Start>LastRow → Invalid
```

## C. Candidate

```text
C01 Mainのみ新Code → 1候補
C02 Subのみ新Code → 1候補
C03 双方同一Code → 2候補
C04 Main内重複 → ManualReview
C05 Sub内重複 → ManualReview
C06 UnknownSource → 候補化しない
```

## D. Staging

```text
D01 A:F互換維持
D02 2CSVでG/H追加
D03 SourceType空欄 → Phase3 Block
D04 cross-source同Code 2行保持
D05 same-source同CandidateKey → Block
```

## E. Baseline/History

```text
E01 Baseline未Initialized → 通常2CSV Update禁止
E02 Audit候補のみ出力
E03 Baseline Initialize成功
E04 CandidateKey履歴保存
E05 翌日同CandidateKey再候補化なし
E06 後日別Source出現 → 新候補
E07 History重複 → Block
```

## F. Phase3 Critical

```text
F01 ABC123|Main + ABC123|Sub → staging duplicate扱いしない
F02 片方登録済みHistory → そのCandidateKeyだけBlock
F03 ManagedCodesにABC123あり → Code単独理由だけでは2CSVを即Blockしない
F04 runtimeSeenはCandidateKey
F05 同Code別Destination → 許可
F06 同Code同Destination別Source → 許可 + WARN
F07 SourceType欠落 → Block
F08 staging削除はCode+Destination+SourceType照合
```

## G. DuplicateCandidateFailClosed干渉

```text
G01 Main/Sub同Codeを別数字シートへ配置
G02 Main/Sub同Codeを同一数字シートへ配置
G03 G02後のLegacy M補修挙動
G04 G02でPhase3 Move自体が完了するか
G05 G02でSave全体への影響がないか
G06 翌日再実行時のLegacy M補修skip範囲
G07 Phase3新規行のM初期化がLegacy M補修前に確定しているか
```

G系は実コードフロー確認後、テストコピーでのみ実施する。

---

# 27. 段階導入

1. REV.5静的設計クロスレビュー
2. r15.9 Rev3実コードとの関数単位差分確定
3. PowerShell 5.1 Parser
4. 1CSV Audit回帰
5. 2CSV Audit
6. Baseline Audit
7. Baseline Initialize（テストコピー）
8. Phase1 staging少数試験
9. Phase3 Main/Sub少数試験
10. DuplicateCandidateFailClosed干渉試験
11. 外注鉄または色物の1拠点RK-10先行
12. 翌営業日冪等性確認
13. 2拠点目
14. 本番正式化

2拠点同時有効化は禁止。

---

# 28. 実装開始条件

以下がすべて成立するまでコード変更しない。

```text
[ ] REV.5 Claudeクロスレビュー完了
[ ] CandidateKeyとManagedCodesの責務分離 PASS
[ ] Phase3 Preflight Critical設計 PASS
[ ] runtimeSeen設計 PASS
[ ] staging削除ガード PASS
[ ] DuplicateCandidateFailClosed干渉評価 PASS
[ ] 最新変数表の2CSV設定確認
[ ] 外注鉄/色物の新規コード機能は本番無効のまま
[ ] r15.9 Rev3原本ハッシュ/差分の扱いを記録
```

---

# 29. Claudeクロスレビュー依頼事項

以下を現行r15.9 Rev3実コードと照合して判定すること。

判定:

```text
PASS
要修正
Critical
未確認
```

確認項目:

1. REV.5でRev3既存安定ロジックを壊す箇所がないか。
2. CandidateKeyを候補/staging/historyの識別子に限定する設計は妥当か。
3. ManagedCodesをCandidateKey化しない判断は妥当か。
4. 数字シートにSourceTypeがない前提で、登録履歴による永続化が必要十分か。
5. `Get-ExistingNewProductCodes` の置換/分離設計に漏れがないか。
6. `Get-Phase3Preflight` の2CSV分岐にCode単独ブロックが残らないか。
7. `duplicateCodes` は2CSV時CandidateKeyになっているか。
8. `runtimeSeen` は2CSV時CandidateKeyにする必要があるか。現行実コードの全参照箇所を列挙すること。
9. `Test-Phase3CodeExistsInNumericSheets` を2CSVでは診断用途だけにする設計は安全か。
10. staging削除をCode+Destination+SourceTypeで照合する設計は十分か。
11. 同一Code・同一Destination・別Sourceを許可した場合の副作用は何か。
12. `DuplicateCandidateFailClosed` の実際の呼出順と影響範囲を確認し、Criticalの意味を再評価すること。
13. `DuplicateCandidateFailClosed` を変更せず維持した場合に、正当な2CSV重複がどの範囲で継続skipを生むか。
14. Phase3新規行のM/F/H初期化とLegacy M補修の順序に問題がないか。
15. Baseline条件 `Managedあり AND Historyなし AND Stagingなし` は妥当か。
16. Baseline専用セルK:L方式に不足がないか。
17. 1CSV拠点でRev3と完全互換にできるか。
18. `新規コード管理_実行=無` の拠点で新機能が完全SKIPされるか。
19. fail-closed不足がないか。
20. 実装変更対象関数の過不足を、実コードの関数名・行番号付きで指摘すること。

### 特に再確認してほしい点

前回レビューでは、

- #4 Phase3 Preflight = Critical
- #8 DuplicateCandidateFailClosed干渉 = Critical

とされた。

#4はREV.5でもCriticalとして採用する。

#8については、`DuplicateCandidateFailClosed` がLegacy M補修用であり、Phase3 Preflightとは別レイヤーであるため、

**「Phase3機能を成立させるため安全装置そのものを変更する必要があるCritical」なのか、  
「正当な重複によってLegacy M補修がfail-closedする影響を評価すべきCritical」なのか**

を実コードの呼出順に基づいて明確に区別して回答してほしい。

安全装置を弱める提案は、必要性が実証されるまで採用しない。

---

# 30. REV.5結論

REV.4の中心思想は維持する。

```text
same-source duplicate = ManualReview
cross-source same Code = 正常な別Candidate
```

ただしr15.9 Rev3への再ベースにより、最重要点を以下へ更新する。

1. Phase3 PreflightのCode単独判定は2CSVではCriticalな衝突点。
2. CandidateKeyとManagedCodesは別概念。
3. 数字シートにSourceTypeがないためCandidateKeyは登録履歴で永続化する。
4. runtimeSeenも2CSV時はCandidateKey化が必要。
5. staging削除ガードへSourceTypeを追加する。
6. Rev3のM/F/H初期化を変更しない。
7. `DuplicateCandidateFailClosed` は安全装置として維持し、干渉の実態を先に検証する。
8. 1CSVはRev3完全互換を必須とする。
9. 実装はClaudeクロスレビューPASS後に開始する。

**REV.5レビュー完了まではコード実装へ進まない。**


---

# 31. Claudeクロスレビュー結果反映（2026-09-24）

Claudeによる現行r15.9 Rev3実コードとのクロスレビュー結果:

```text
REV.5 DESIGN: GO WITH MINOR
```

REV.5の中心設計は妥当と判定された。
以下をFIX事項として正式反映する。

## 31.1 Phase3 Preflight

現行Rev3の以下のCode単独判定は、2CSV対応におけるCritical実装箇所である。

```powershell
DuplicateCodes.Contains($Candidate.Code)
ManagedCodes.Contains($Candidate.Code)
```

設計方針はREV.5 14章のままFIXする。

2CSV時:

```text
duplicate判定 = CandidateKey
history判定 = CandidateKey
runtimeSeen = CandidateKey
ManagedCodes Code単独 = 即時ブロック条件にしない
numeric Code存在 = 即時ブロック条件にしない
```

1CSV時は現行Rev3を完全維持する。

## 31.2 duplicateCodes

2CSV時の `duplicateCodes` はCandidateKey単位とする。

```text
ABC123|Main
ABC123|Sub
```

は重複ではない。

```text
ABC123|Main
ABC123|Main
```

は重複でありBlock/ManualReview対象。

これは実装必須事項とする。

## 31.3 runtimeSeen

2CSV時の `runtimeSeen` はCandidateKey単位とする。

1CSV:

```text
runtimeSeen = Code
```

2CSV:

```text
runtimeSeen = CandidateKey
```

実装時には現行Rev3の `runtimeSeen` 全参照箇所を機械検索し、取りこぼしがないことを確認する。

## 31.4 DuplicateCandidateFailClosed 再評価

現行r15.9 Rev3実コードの呼出順は以下。

```text
Invoke-Phase3Move
→ Remove-Phase3StagingRows
→ Invoke-LegacyGapRepair
→ Invoke-LegacyMFlagGapRepair
→ Workbook.Save
```

`DuplicateCandidateFailClosed` は `Invoke-LegacyMFlagGapRepair` 内で発生し、
動作は `continue` である。

したがって以下を正式仕様としてFIXする。

- Phase3 Moveを停止しない
- staging削除を停止しない
- K/O LegacyGapRepairを停止しない
- Workbook.Saveを停止しない
- 該当数字シートのLegacy M補修だけをskipする
- CodeCountsは数字シート単位
- 別数字シートの同一Codeには影響しない

また、当日Phase3移動した行は `$phase3BySheet` によりLegacy M解析対象から除外される。

したがって、

```text
ABC123|Main
ABC123|Sub
```

を同一数字シートへ当日登録した場合、その当日は
`DuplicateCandidateFailClosed` の対象にならない。

翌日以降、両行が通常既存行として解析されるようになると、
同一Code重複として検出される。

その2行が同一数字シートに存在し続ける限り、

**翌日以降、その数字シート全体のLegacy M欠損自動補修が継続的にskipされる。**

これは既知の副作用として仕様化する。

## 31.5 DuplicateCandidateFailClosed の安全装置は変更しない

上記副作用は存在するが、Phase3機能そのものを阻害しない。

また数字シートにはSourceTypeが存在しないため、
Legacy M補修側だけではそのCode重複が正当なMain/Sub由来か、
異常重複かを安全に判定できない。

よって現段階では、

**`DuplicateCandidateFailClosed` の判定ロジックを緩和・例外化しない。**

判定ロジックはRev3のまま維持する。

分類:

```text
旧: Critical
新: 要修正 / 既知の運用副作用
```

「要修正」は安全装置そのものの変更を意味しない。
仕様書・ログ・テストで影響を可視化することを意味する。

## 31.6 Legacy Mログ強化

`Invoke-LegacyMFlagGapRepair` の判定ロジックは変更禁止。

ただし運用診断性向上のため、
`DuplicateCandidateFailClosed` 発生時ログへ以下を追加できるか実装レビューする。

```text
Sheet
DuplicateCode
Count
Phase3HistoryObserved
Reason=DuplicateCode
Action=LegacyMRepairSkipped
```

SourceTypeを数字シートから推測してログへ書いてはならない。

履歴との照合を行う場合も、
「履歴上同一Codeの複数CandidateKeyが存在する」という観測情報に留め、
Legacy安全判定の解除条件には使用しない。

---

# 32. 同一Code・同一Destinationの正式扱い

2CSVで人が明示的に、

```text
ABC123|Main → Sheet5
ABC123|Sub  → Sheet5
```

を指定した場合、Phase3は許可する。

当日:

```text
Phase3 Move = 許可
M/F/H初期化 = 正常実施
History = 2 CandidateKeyを保存
Staging = 成功行削除
Save = 実施
```

翌日以降:

```text
Sheet5内に同一Code 2行
→ Legacy M補修 DuplicateCandidateFailClosed
→ Sheet5のLegacy M補修のみskip
```

したがってPhase3候補CSV/ログには、同一Code・同一Destination・別Sourceの場合、

```text
WARN Phase3SameCodeSameDestination
Action=AllowedByHumanDecision
LegacyMRepairImpact=PossibleFromNextRun
```

相当の警告を残す。

---

# 33. 実装時の必須検索

コード変更前後に少なくとも以下を機械検索する。

```text
Get-ExistingNewProductCodes
ManagedCodes.Contains
DuplicateCodes.Contains
duplicateCodes
runtimeSeen
Test-Phase3CodeExistsInNumericSheets
Get-Phase3Preflight
Remove-Phase3StagingRows
Invoke-LegacyMFlagGapRepair
DuplicateCandidateFailClosed
phase3BySheet
```

目的:

- Code単独判定の取り残し防止
- 1CSV/2CSV分岐漏れ防止
- Rev3安全装置への意図しない変更防止

---

# 34. 実装ベースラインSHA256ゲート

2026-09-24のクロスレビューで、
同一ファイル名のr15.9 Rev3について過去レビュー時のハッシュと
現在確認した実体のハッシュが一致しないことが判明した。

主要機能は実コード上同等であることを確認したが、
実装派生元は曖昧にしない。

コード実装開始直前に、
**現在本番で実際に採用しているPS1実体**について以下を記録する。

```text
BaselineFileName
BaselineSHA256
BaselineFileSize
BaselineLastWriteTime
BaselineCapturedAt
```

以後の実装版はこのBaseline実体からのみ派生させる。

過去に同名で存在した別ハッシュ版から派生させてはならない。

現在の作業環境にはr15.9 Rev3 PS1実体が存在しないため、
本REV.5 FIXED作成時点ではBaselineSHA256は未確定。

状態:

```text
BaselineSHA256 = PENDING
ImplementationGate = CLOSED
```

---

# 35. 実装開始条件（FIXED）

以下すべて成立後にコード実装へ進む。

```text
[x] REV.5 Claudeクロスレビュー完了
[x] REV.5 DESIGN = GO WITH MINOR
[x] CandidateKeyとManagedCodesの責務分離 PASS
[x] Phase3 Preflight Critical設計 PASS
[x] duplicateCodes CandidateKey化方針 FIX
[x] runtimeSeen CandidateKey化方針 FIX
[x] staging削除ガード設計 PASS
[x] DuplicateCandidateFailClosed影響範囲確定
[x] DuplicateCandidateFailClosed安全装置維持 FIX
[x] Baseline/History方式 PASS
[ ] 最新変数表の2CSV設定を実装直前に再確認
[ ] 現在本番Rev3 PS1実体を取得
[ ] BaselineSHA256を確定
[ ] Baseline PS1を保全コピー
[ ] 外注鉄/色物の新規コード機能が本番無効であることを再確認
```

上記未完了項目がある間:

```text
ImplementationGate = CLOSED
```

---

# 36. REV.5 FIXED 最終判定

```text
DesignReview = PASS
ClaudeCrossReview = GO WITH MINOR
CriticalDesignIssueRemaining = 0
KnownImplementationCritical = Phase3 Preflight 2CSV branch
KnownOperationalSideEffect = SameCodeSameDestination causes next-run Legacy M repair skip for that sheet
LegacySafetyGuard = KEEP
ImplementationGate = CLOSED until BaselineSHA256 and current settings are confirmed
```

REV.5の設計を正式FIXする。

次工程はコード作成ではなく、

1. 現在本番Rev3 PS1実体のBaselineSHA256確定
2. 最新拠点別変数表の2CSV設定確認
3. 本番Rev3保全コピー
4. そのBaselineから次期実装版を派生

の順とする。
