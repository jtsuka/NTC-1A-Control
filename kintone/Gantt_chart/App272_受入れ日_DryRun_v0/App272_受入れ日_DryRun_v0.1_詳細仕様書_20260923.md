# App272 受入れ日 自動更新
## DryRun v0.1 詳細仕様書
### 2026-09-23

---

## 0. 位置づけ

本仕様は、`受け入れ情報3か月.xlsx` の受入実績と、本番 kintone App272 の現状を
**書込みなしで突合する DryRun v0.1** の詳細仕様である。

この段階では App272 への PUT / POST / DELETE / PATCH は一切行わない。
本番 App272 は **参照先** として使用する。

入力実績に基づいて「将来自動反映するならどの受入日が候補になるか」を一覧化し、
業務ルールを確定するための証拠を作ることを目的とする。

---

# 1. 最重要覚書：100%自動化を保証しない

## 1.1 基幹側の受入入力は完全ではない

現場運用上、実際には受入済みであっても、
担当者が基幹システムで受入入力を行っていないケースが一定数存在する。

したがって、

```text
受入実績データに行が存在しない
        ≠
実際に未受入である
```

とする。

## 1.2 本機能の責務

本機能は、

> 基幹システムへ入力済みの受入実績を App272 へ自動転記・突合する

ための補助機能である。

次のことは保証しない。

```text
- 現実の受入実績を100%捕捉すること
- 基幹未入力の受入を自動検知すること
- App272の全案件について受入れ日を100%埋めること
- 受入実績に存在しない案件を「未受入」と断定すること
```

今後 Execute 版を作成しても、この制約は解消されない。
別の現物確認・担当者確認・倉庫記録等を連携しない限り、
基幹未入力分は自動処理の対象外となる。

## 1.3 DryRun上の表記

App272に存在するが今回の受入実績Excelに存在しない案件は、

```text
NO_SOURCE_DATA_UNKNOWN
```

と分類する。

**`NOT_RECEIVED` という名称は使用しない。**

理由は、
- 直近3ヶ月の受入実績抽出範囲外
- 基幹入力漏れ
- 本当に未受入
- その他の抽出条件

を v0.1 では区別できないためである。

---

# 2. 根拠データ

## 2.1 受入実績

入力例:

`受け入れ情報3か月.xlsx`

今回確認した列構成は106列、データ行は1,998行。

主要列:

| Excel列名 | 用途 |
|---|---|
| `発注番号` | App272突合キー |
| `受入日` | 受入れ日候補 |
| `受入数` | 分納・訂正等の参考 |
| `受入番号` | 受入イベント追跡用 |
| `検査合格数` | 参考 |
| `受入検収日` | 参考 |
| `手配書番号` | クロスチェック参考 |
| `注文伝票番号` | クロスチェック参考 |
| `注文伝票行番号` | クロスチェック参考 |
| `伝票区分` | 異常・訂正系の参考 |

実ファイルには `受入数` が負数の行も確認されている。
v0.1では負数行を自動更新候補として無条件に信用せず、
レビュー要フラグを立てる。

## 2.2 既存分析

既存分析では、

```text
App272系発注データ 937件
受入実績           1,998行
発注番号一致       78件
```

を確認済み。

受入実績は受入日基準の直近3ヶ月データであり、
古い注文であっても直近に受入が発生すれば含まれる。

取得頻度は月1回以上、可能なら週1回が候補。

---

# 3. 対象環境

```text
BaseUrl : https://bcurbkixz609.cybozu.com
AppId   : 272
環境    : PROD
処理    : READ ONLY / DRYRUN
```

## 3.1 App272の使用フィールド

| 意味 | フィールドコード | 必須 |
|---|---|---|
| レコードID | `$id` | 必須 |
| revision | `$revision` | 必須 |
| 発注番号 | `数値` | 必須 |
| 受入れ日 | `受入れ日` | 任意（v0.1では未作成でも分析継続可） |
| 手配書番号 | `ODERNO` | 参考 |
| 注文書番号 | `数値_44` | 参考 |

`発注番号` は App272 の一意キーとして扱う。

## 3.2 「受入れ日」フィールド未作成時

本番App272に `受入れ日` がまだ存在しない場合でも、
DryRun v0.1 は候補日分析を継続できる。

その場合、

```text
AcceptanceFieldStatus = MISSING
AppAcceptanceDate     = (空欄)
```

として出力する。

将来の Execute 版では `受入れ日` DATEフィールドの存在を必須とし、
未作成なら Fail Closed とする。

---

# 4. v0.1で禁止する処理

スクリプトには次を実装しない。

```text
- App272 records PUT
- App272 records POST
- DELETE
- PATCH
- Form更新
- Dropdown更新
- App270参照
- 旧Lookup参照
- MorningRoutine呼出
- Formatter呼出
- K加工着手日/K完成検査日の更新
- 外注入荷実績日の更新
```

kintone REST APIは GET のみ使用する。

---

# 5. 入力ファイル検証

## 5.1 必須列

```text
発注番号
受入日
```

どちらかが存在しない場合は即時 SafetyStop。

## 5.2 参考列

存在する場合だけ利用する。

```text
受入数
受入番号
検査合格数
受入検収日
手配書番号
注文伝票番号
注文伝票行番号
伝票区分
```

## 5.3 行単位検証

次を不正行として集計する。

```text
発注番号空欄
受入日空欄
受入日が日付に変換できない
```

不正行は候補日決定には使用しない。

---

# 6. App272取得

App272全レコードを GET する。

```text
limit 500
offset 0, 500, 1000 ...
```

v0.1想定件数は1,000件前後であるため OFFSET方式で十分。

GET対象は必要最小限のフィールドだけとする。

## 6.1 App272側キー重複

`数値`（発注番号）が2件以上存在した場合は、

```text
SafetyStop: App272 duplicate order number
```

とする。

App272主キーの前提が崩れているため、
将来の自動書込みを想定した突合結果として使用しない。

---

# 7. 受入実績のグルーピング

有効な受入実績行を `発注番号` 単位で集約する。

グループごとに次を算出する。

```text
SourceRowCount
DistinctReceiptDateCount
FirstReceiptDate
LastReceiptDate
CandidateReceiptDate
ReceiptQtySum
NegativeQtyRowCount
ReceiptNumbers
SourceOrderNumbers
SourceOrderSlipNumbers
```

## 7.1 v0.1の候補日

候補日は暫定的に、

```text
CandidateReceiptDate = 最大の受入日
```

とする。

これは**正式な完納日定義ではない**。

複数受入の場合は必ず、

```text
MULTI_RECEIPT
```

フラグを付ける。

## 7.2 将来の完納判定

将来版では、

```text
発注数
vs
累計受入数
```

を比較し、発注数に達した受入イベントを完納日とする方式を検討する。

ただし v0.1 では実装しない。

---

# 8. 数量・訂正系の安全扱い

## 8.1 負数の受入数

同じ発注番号グループに `受入数 < 0` が1行でも存在する場合、

```text
SOURCE_REVIEW_REQUIRED
NegativeQtyRowCount > 0
```

とする。

候補日は表示するが、

```text
FutureAutoEligible = NO
```

とする。

理由:
- 返品
- 訂正
- 取消
- 数量調整

等の意味を v0.1 では確定できないため。

## 8.2 複数受入

複数受入そのものは異常とはしない。

ただし、

```text
SourceRowCount > 1
または
DistinctReceiptDateCount > 1
```

なら `MULTI_RECEIPT` を付ける。

---

# 9. 突合分類

v0.1は次の分類を出力する。

## 9.1 `CANDIDATE_EMPTY_SINGLE`

```text
App272あり
受入実績あり
App272受入れ日空欄
受入実績1件
負数等のレビュー要因なし
```

将来の自動更新候補。

## 9.2 `CANDIDATE_EMPTY_MULTI`

```text
App272あり
受入実績あり
App272受入れ日空欄
複数受入
負数等のレビュー要因なし
```

候補日は最新日。
v0.1では書込みなし。
将来の業務ルール確認対象。

## 9.3 `ALREADY_SAME`

```text
App272受入れ日 == CandidateReceiptDate
```

変更不要。

## 9.4 `EXISTING_DIFFERENT`

```text
App272受入れ日あり
App272受入れ日 != CandidateReceiptDate
```

手入力・過去自動処理・基幹訂正等の可能性があるため、
自動上書き候補にはしない。

```text
FutureAutoEligible = NO
```

とする。

## 9.5 `SOURCE_REVIEW_REQUIRED`

```text
負数受入数あり
日付矛盾
その他ソース側にレビュー要因
```

自動更新禁止候補。

## 9.6 `SOURCE_ONLY_NO_APP272`

受入実績に発注番号があるが、
App272に該当発注番号が存在しない。

書込み対象外。
抽出期間差・App272対象外・古い案件等を確認するためのログ。

## 9.7 `NO_SOURCE_DATA_UNKNOWN`

App272には存在するが、
今回の受入実績ファイルには該当発注番号が無い。

**未受入とは判定しない。**

基幹未入力の可能性を含むため、
この分類名を固定する。

---

# 10. 手入力保護の暫定方針

v0.1は書込みをしないが、
将来版に向けて次を暫定方針とする。

```text
App272 受入れ日が空欄
  → 自動候補になり得る

App272 受入れ日に既存値あり
  → 自動上書きしない
```

既存値と候補日が違えば `EXISTING_DIFFERENT` として人確認へ回す。

これにより、ポップアップで人が入力した値を
基幹データが後から無条件上書きする事故を防ぐ。

---

# 11. 出力

## 11.1 詳細CSV

ファイル例:

```text
App272_AcceptanceDryRun_20260923_184500.csv
```

主列:

```text
Class
FutureAutoEligible
OrderNo
AppRecordId
AppRevision
AppAcceptanceDate
CandidateReceiptDate
SourceRowCount
DistinctReceiptDateCount
FirstReceiptDate
LastReceiptDate
ReceiptQtySum
NegativeQtyRowCount
AppOrderNo
SourceOrderNos
AppOrderSlipNo
SourceOrderSlipNos
ReceiptNumbers
AcceptanceFieldStatus
Note
```

UTF-8 BOM CSV。

## 11.2 サマリTXT

```text
App272_AcceptanceDryRun_20260923_184500_summary.txt
```

内容:

```text
InputRows
ValidSourceRows
InvalidSourceRows
UniqueSourcePO
App272Rows
UniqueApp272PO

CANDIDATE_EMPTY_SINGLE
CANDIDATE_EMPTY_MULTI
ALREADY_SAME
EXISTING_DIFFERENT
SOURCE_REVIEW_REQUIRED
SOURCE_ONLY_NO_APP272
NO_SOURCE_DATA_UNKNOWN

FutureAutoEligibleYes
FutureAutoEligibleNo
NegativeQtyGroups
MultiReceiptGroups

RESULT=DRYRUN_ONLY
WRITES=0
```

---

# 12. 実行例

```powershell
.\Compare-App272AcceptanceDryRun_v0.1.ps1 `
  -InputXlsx "C:\HPDB\受け入れ情報3か月.xlsx" `
  -TokenPath "C:\HPDB\Secrets\App272_Prod.token"
```

APIトークンファイルが無い場合は、
画面非表示のSecureString入力で受け取る。

---

# 13. SafetyStop

次の場合は停止する。

```text
AppId != 272
入力Excelなし
必須列なし
有効な受入実績が0件
App272 GET失敗
App272 発注番号重複
発注番号フィールド `数値` が存在しない
```

`受入れ日`フィールド未作成だけでは v0.1 を停止しない。
候補分析は継続する。

---

# 14. 受入判定

## PASS

```text
App272 GET成功
受入実績読込成功
発注番号単位の集約成功
分類CSV出力成功
サマリ出力成功
kintone write 0件
```

## FAIL

```text
PUT/POST/DELETE/PATCHが実行された
App272キー重複を無視した
受入実績が無い案件を未受入と断定した
負数行を無警告で自動候補化した
既存受入れ日を将来上書き可として分類した
```

---

# 15. v0.1 実施後に確認する業務判断

DryRun結果を見て次を確定する。

1. 複数受入時は「最新受入日」でよいか
2. 発注数と累計受入数による完納判定へ進むか
3. 負数受入の意味と扱い
4. 既存手入力受入れ日との差異処理
5. 受入実績取得頻度（週1 / 月1）
6. `NO_SOURCE_DATA_UNKNOWN` が多い場合の基幹入力漏れ実態
7. Execute版を作るか
8. 単独TEST PASS後にMorningRoutineへ統合するか

---

# 16. 今後の段階

```text
v0.1  PROD App272 READ ONLY DryRun
  ↓
実データ結果レビュー
  ↓
業務ルール確定
  ↓
v0.2 App292 Execute TEST
  ↓
revision付きPUT / PostVerify
  ↓
PROD単独運用
  ↓
必要ならMorningRoutine統合
```

**v0.1から直接PROD書込み版へ進めない。**
