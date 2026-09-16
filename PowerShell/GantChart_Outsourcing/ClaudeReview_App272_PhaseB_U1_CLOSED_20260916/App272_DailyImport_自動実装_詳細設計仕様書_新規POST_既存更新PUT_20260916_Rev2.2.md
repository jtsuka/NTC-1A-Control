# App272 Daily Import 自動実装 詳細設計仕様書 Rev.2.2

**対象**: 外注課納期管理 App272 日次CSV自動取込  
**作成日**: 2026-09-16  
**対象環境**: Windows PowerShell 5.1 / kintone  
**本番アプリ**: App272  
**参照元アプリ**: App270（READ ONLY）  
**試験アプリ**: App292  
**現行実証済みスクリプト**: `Import-App272_Phase1C_New_v0.7.4_MasterAuto.ps1`  
**上位自動化試作**: `Run-App272-DailyImport_v0.1.ps1`

**Rev.2反映事項（Claudeレビュー 2026-09-16）**:
- POST候補・PUT候補を横断したVendor/Staff option必要集合の一括算出を明記
- Manual保護＋未知Vendor変更の複合ケースを正式仕様化
- ExpectedUpdatePo集合ガードに加え、revision **および** 変更内容ハッシュを必須化
- 複数レコードPUTの原子性確認をPhase B着手前の必須前提試験へ前倒し
- Daily Wrapperの標準出力解析依存を廃止し、構造化Manifest（JSON）を正式I/F化
- POST→PUTの実行順を初期版の標準順序として採用
- Test U11 / U12を追加

- **2026-09-16 U12実測完了**: App292で2件の複数PUTに対し、1件だけ意図的にrevision競合を発生させた結果、APIはHTTP 409 Conflictとなり、競合していないもう1件も更新されなかった。再GETにより **ALL-OR-NOTHING（全件原子的失敗）** を実測確認。
- U12結果に基づき、Phase Bでは **revision指定付きの複数PUTを使用可能** とする。ただし大量一括更新は避け、少数バッチ＋各バッチ後再GET検証を維持する。


---

## 1. 目的

外注課納期管理CSVを日次で取り込み、App272を安全に同期する。

現行 v0.7.4 では、新規発注番号に対するPOSTは本番実証済みである。一方、既存発注番号についてCSV側の納期・手配日・外注先・担当者等が変更された場合、差分は検出するが既存App272へのPUTは行わない。

今後、注文書の納期変更等を正しく運用する場合、同一発注番号の既存App272レコードもCSVを正として同期できる必要がある。

本仕様では以下を一体化する。

1. 新規発注番号の自動POST
2. 既存発注番号の基幹値変更の自動PUT
3. 日程再計算またはフォールバック
4. Vendor / Staffドロップダウンマスタ自動補完
5. DryRunによる実行対象集合の確定
6. 実行直前の再確認とrevisionガード
7. POST / PUT後の再GET検証
8. 異常時の安全停止

---

## 2. 現在までに本番確認済みの事項

### 2.1 2026-09-16 本番App272

当日CSV 315件に対してDryRunを行い、以下を確認した。

- CSVRows: 315
- App272既存: 858
- NOCHANGE: 304
- 新規: 11
- 新規11件を本番POST
- App272: 858 → 869
- POST後の基幹9項目＋対象K日程の再GET検証: PASS
- Record errors: 0
- Post verification errors: 0
- Warnings: 0

### 2.2 新規日程のルール

通常計算可能な場合:

- `K加工着手日` = マスタに基づく計算値
- `K完成検査日` = マスタに基づく計算値
- `K日程種別` = `Batch`
- `K暫定設定` = `ON`
- `K計算根拠` = `Company` または `Provisional`

計算不能が次の理由の場合:

- `ShortLeadTime`
- `ScheduleMasterNotFound`

フォールバック:

- `K加工着手日` = CSV手配日
- `K完成検査日` = CSV納期
- `K日程種別` = `Batch`
- `K暫定設定` = `ON`
- `K計算根拠` = 空欄

2026-09-16本番では、通常計算3件、フォールバック8件（ShortLeadTime 7件、MasterNotFound 1件）を実証済み。

### 2.3 Vendor / Staffマスタ自動補完

App292で以下のE2Eを確認済み。

`未知Vendor/Staff検出 → Preview Form PUT → 意図差分照合 → Deploy → Deploy status SUCCESS → Live確認 → POST → 再GET検証`

Deploy status GETは以下を使用する。

`GET /k/v1/preview/app/deploy.json?apps[0]=<AppId>`

GETにJSON Bodyを付与しない。

---

## 3. 現行v0.7.4の制約

v0.7.4は既存App272レコードについて以下の基幹差分を検出する。

- ODERNO
- VENDOR_TEXT
- VENDOR_DD
- 手配日（`日付`）
- 納期（`日付_1`）

差分がある場合は `Update(BaseOnly)` と分類するが、Phase1Cでは既存レコードPUTを禁止しているため `BLOCKED_EXISTING_WRITE` となる。

したがって、同一発注番号でCSVの納期が変更されても、現状ではApp272の納期は更新されない。

これは日次自動化前に解消する。

---

## 4. 正本・キー・同期原則

### 4.1 正本

日次同期対象の基幹値は、当日CSVを正とする。

### 4.2 一意キー

App272同期キーは `発注番号（数値）` とする。

同一発注番号がCSV内で複数存在する場合は自動実行を停止する。

### 4.3 同期原則

次を維持する。

`CSV business values = App272 business values = App272 form dropdown options`

Vendor / Staffについては、CSV値がApp272フォームの選択肢に存在しない場合、レコードPOST/PUTより先にフォームマスタを補完・Deployする。

### 4.4 文字列正規化

過剰な正規化・曖昧一致は行わない。

- 前後Trimのみ
- Vendor名・担当者名の表記揺れを自動統合しない
- 全角/半角・旧字体等の推測変換を行わない

---

## 5. レコード分類

各CSV行を発注番号でApp272と照合し、次のいずれかに分類する。

### 5.1 NEW

App272に同一発注番号が存在しない。

→ `POST:NEW`

### 5.2 NOCHANGE

App272に存在し、同期対象基幹値がCSVと一致する。

→ 書込みなし

### 5.3 UPDATE_BASE_ONLY

App272に存在し、担当者等の基幹値だけが変化し、日程再計算条件には該当しない。

→ 基幹項目PUT

### 5.4 UPDATE_RECALC

App272に存在し、日程に影響する基幹値が変化する。

例:

- 手配日変更
- 納期変更
- 外注先変更

→ 基幹項目PUT＋K日程再計算/フォールバック

### 5.5 MANUAL_SCHEDULE_PROTECTED

既存 `K日程種別=Manual` で日程関連基幹値が変更された場合。

→ CSV基幹値は更新対象とするが、ManualのK日程は上書きしない。  
→ `ManualScheduleDueDateChanged` 等の警告を残す。  
→ K日程と新しい納期の矛盾がある場合もログに明示する。

**重要**: ManualだからCSVの納期更新自体を拒否するのではなく、「基幹値はCSVに同期、K日程は人の設定を保護」とする。

また、Manualレコードで外注先または担当者がCSV上で未知の新値へ変更された場合も、
そのレコードをPUT候補から除外するのではなく、POST候補・PUT候補を横断して
Vendor/Staff option必要集合を算出し、フォームoptionを先に補完・Deployした後で基幹値だけをPUTする。
この場合もManualのK日程はpayloadへ含めない。

### 5.6 PROTECTED / ERROR

以下は自動更新しない。

- 未知のK日程種別
- マスタ使用区分が許可値外
- 必須項目欠落
- CSV発注番号重複
- 日付形式不正
- revision競合
- 実行対象集合不一致

---

## 6. 既存レコードの同期対象フィールド

### 6.1 基幹同期対象

将来版では少なくとも以下を比較・同期する。

| CSV | App272 field | 用途 |
|---|---|---|
| 発注番号 | 数値 | 一意キー。原則変更対象外 |
| ODERNO | ODERNO | 基幹同期 |
| 指示先名 | VENDOR_TEXT | 基幹同期 |
| 指示先名 | VENDOR_DD | 日程・表示用 |
| 手配担当者名 | STAFF_TEXT | 基幹同期 |
| 手配担当者名 | STAFF_DD | 表示・運用用 |
| 手配日 | 日付 | 基幹同期 |
| 納期 | 日付_1 | 基幹同期 |
| ODERNO | ルックアップ | lookup同期 |

新規POST時の9項目と、既存PUT時の業務値を同一思想にする。

### 6.2 K日程同期対象

必要時のみ:

- K加工着手日
- K完成検査日
- K日程種別
- K暫定設定
- K計算根拠

---

## 7. 日程再計算トリガー

既存レコードで以下のいずれかが変わった場合、K日程の再評価対象とする。

1. 手配日
2. 納期
3. 外注先

担当者だけの変更では日程再計算しない。

ODERNO変更は原則基幹同期のみとし、日程ルールに影響しない。ただし将来要件でODERNOに日程意味が加わった場合は再検討する。

---

## 8. K日程種別ごとの既存更新ルール

### 8.1 Manual

優先度最高。

- 基幹値変更: PUT可
- K加工着手日: 保護
- K完成検査日: 保護
- K日程種別: Manual維持
- 納期変更等は警告ログ

### 8.2 Batch

CSV側で日程関連基幹値が変更された場合:

- 最新CSV値で再計算
- 正常計算できれば新しいBatch日程へPUT
- `ShortLeadTime` / `ScheduleMasterNotFound` はフォールバック

### 8.3 Auto

既存データを無条件に一括変換しない。

**日次CSVで基幹差分が発生したAutoのみ**今回の自動同期対象とし、PowerShellが再設定した時点で `K日程種別=Batch` とする。

差分がない既存Autoは、別の移行工程として扱い、Daily Importでは触らない。

### 8.4 空欄

基幹差分があり日程再計算が必要な場合、Batchとして設定する。

### 8.5 未知値

自動PUT禁止。安全停止または当該レコードを保護し、警告する。

---

## 9. 既存更新時のフォールバック

新規POSTと同じルールを既存のBatch/Auto/空欄更新にも適用する。

### 9.1 通常計算成功

- 計算値を使用
- K日程種別=Batch
- K暫定設定=ON
- K計算根拠=Company/Provisional

### 9.2 ShortLeadTime

- K加工着手日 = CSV手配日
- K完成検査日 = CSV納期
- K日程種別 = Batch
- K暫定設定 = ON
- K計算根拠 = 空欄
- ScheduleDecisionにFallback理由を保存

### 9.3 ScheduleMasterNotFound

同上。

### 9.4 要確認 / 使用しない / 不正マスタ

自動フォールバックしない。

既存仕様どおり安全側で保護する。

---

## 10. DryRun仕様

日次自動実行では、必ず先にPROD DryRunを行う。

DryRunは書込みを一切行わない。

### 10.1 出力対象集合

DryRunでは、人間向けDetailLogとは別に**機械可読な構造化Manifest（JSON）**を必ず出力する。
Wrapperは標準出力や人間向けログ文言を解析してはならず、このManifestだけを正式I/Fとして使用する。

Manifestには少なくとも以下を含める。

- `POST:NEW` 発注番号集合
- `PUT:BASE` 発注番号集合
- `PUT:RECALC` 発注番号集合
- `PROTECTED:MANUAL` 集合
- `ERROR/BLOCKED` 集合
- 各対象POのApp272 record id
- 各PUT候補の期待revision
- 各PUT候補のDryRun時変更前値
- 各PUT候補の変更後値
- 各PUT候補の変更フィールド集合
- 各PUT候補の変更内容ハッシュ
- Manifest全体の生成日時、対象App、入力CSVパス、入力CSVハッシュ

人間向けCSV/テキストログは監査・レビュー用に残すが、自動運転の制御判断には使用しない。

### 10.2 変更前後ログ

既存PUT候補はフィールド単位で、変更前→変更後を出力する。

例:

```text
PO=2609249755
日付_1: 2026-11-02 -> 2026-11-06
K完成検査日: 2026-10-30 -> 2026-11-05
K加工着手日: 2026-10-26 -> 2026-11-02
K日程種別: Batch -> Batch
```

### 10.3 集計

最低限:

- CSVRows
- NOCHANGE
- New POST count
- Existing PUT count
  - BaseOnly
  - Recalc
  - Manual protected schedule
- Fallback count
  - ShortLeadTime
  - MasterNotFound
- Missing Vendor options
- Missing Staff options
- Record errors
- Warnings

---

## 11. 自動生成する実行マニフェスト

現行 `Run-App272-DailyImport_v0.1.ps1` はDryRun detailから `POST:NEW` を抽出し、`ExpectedNewPo` と `MaxCreates` を自動生成する。

将来版 `Run-App272-DailyImport_v0.2.ps1` では以下へ拡張する。

### 11.1 New manifest

- `ExpectedNewPo[]`
- `MaxCreates = NewCount`

### 11.2 Update manifest

- `ExpectedUpdatePo[]`
- `MaxUpdates = UpdateCount`
- 各POの期待record id
- 各POの期待revision
- 各POの期待変更フィールド集合
- 各POの変更前値
- 各POの変更後値
- 各POの変更内容ハッシュ

**revision と変更内容ハッシュは「または」ではなく両方を必須とする。**

DryRunで確定した集合、record id、revision、変更前値、変更フィールド集合、変更内容ハッシュのいずれかが
Execute直前確認と1件でも異なる場合は停止する。

---

## 12. 実行直前安全確認

Execute前にApp272を再GETする。

### 12.1 新規POST

- 発注番号がまだ存在しないこと
- ExpectedNewPo集合が現在のNEW集合と完全一致すること

### 12.2 既存PUT

- 発注番号が存在すること
- レコードIDが同一であること
- `$revision` がDryRun時の期待値と一致すること
- 現値がDryRun時の変更前値と一致すること
- Execute直前に同じ変更フィールド集合・変更後値を再計算できること
- Execute直前に再計算した変更内容ハッシュがDryRun Manifestの期待ハッシュと一致すること

revisionは「レコードが変化したか」を見る粗い防御、
変更内容ハッシュは「今回適用しようとしている業務値が同一か」を見る細かい防御として、
**両方を必須**とする。

不一致なら、他ユーザー・別処理・CSV差替え・計算条件変化の可能性があるためPUTしない。

---

## 13. PUT方式

既存更新はkintone Records APIの複数レコード更新を使用する想定。

各更新レコードに:

- id
- revision
- record（変更フィールドのみ）

を指定する。

### 13.1 原則

- 不要なフィールドをPUTしない
- Manual K日程はpayloadに含めない
- 変更前後が同値のフィールドをpayloadに含めない
- revision指定を必須とする

### 13.2 バッチ原子性【U12実測済み / CLOSED】

2026-09-16、App292で以下の条件によりU12を実施した。

- Record A: 有効revisionのまま
- Record B: 事前に単独PUTしてrevisionを1段進め、複数PUT時には古いrevisionを指定
- A/B 2件を同一 `PUT /k/v1/records.json` に投入
- API応答と再GETの双方で実状態を確認

実測結果:

```text
MultiPutApiSuccess : False
HTTP                : 409 Conflict
Conclusion          : ATOMIC_ALL_OR_NOTHING
A                   : 未変更
B                   : 競合作成用の事前更新値のまま
Cleanup A           : ALREADY_ORIGINAL
Cleanup B           : RESTORED
```

**結論: revision競合を含む複数PUTは、今回のApp292実測では全件原子的に失敗した。**
競合していないAだけが更新される部分成功は発生しなかった。

Phase Bの実装方針を以下で確定する。

- revision指定付きの複数PUTを使用可能とする。
- ただし「原子的だから大量一括でも安全」とは扱わない。
- `MaxAutoUpdates` は全実行上限、`PutBatchSize` は1API要求あたりの件数として分離する。
- 初期試験では `PutBatchSize=10` を既定値とし、App292のU1〜U11で検証する。
- 各バッチPUTの直前にrevision・変更前値・changeHashを再確認する。
- 各バッチPUTの直後に再GETし、対象業務値を全件照合する。
- 1バッチでも409/400/検証不一致が発生した場合、後続バッチへ進まない。
- 再開時は新たにDryRun/Manifestを作り直し、古いManifestを再利用しない。

U12ログ:
`C:\HPDB\logs\Test-App292-U12_Result_20260916_104303.json`

---

## 14. Vendor / Staffマスタライフサイクル

POSTだけでなくPUTでも、CSVのVendor/Staff値がフォームoptionに存在することを先に保証する。

必要option集合は、**POST候補だけ・PUT候補だけを別々に見るのではなく、
今回の実行対象であるPOST候補＋PUT候補（Manual保護レコードの基幹PUTを含む）を横断して一括算出する。**
これにより「Manual保護レコードの外注先が未知Vendorへ変更された」場合でも、
K日程を保護したままVendor option追加→基幹PUTを実行できる。

手順:

1. Live / Preview Form GET
2. 差分なし確認
3. POST候補＋PUT候補から必要option集合算出
4. 上限チェック
5. PUT直前再GET
6. Preview Form PUT
7. PUT後/Deploy前に意図差分のみであること確認
8. Deploy POST
9. Status GETでSUCCESS待ち
10. Live / Preview再GET
11. option存在・差分なし確認
12. Record POST/PUTへ進む

---

## 15. POST/PUT後検証

処理後にApp272を再GETし、実際値を照合する。

### 15.1 新規POST

- 基幹9項目
- 日程対象ならK日程項目

### 15.2 既存PUT

- PUT対象基幹項目
- 再計算対象ならK日程項目
- Manualの場合はK日程が変更されていないことも検証

不一致が1件でもあれば終了コードを非0とする。

---

## 16. Daily Wrapper構成

将来の上位スクリプト:

`Run-App272-DailyImport_v0.2.ps1`

想定フロー:

```text
NAS CSV取得
  ↓
ローカルコピー
  ↓
CSV整形/構文検査
  ↓
PROD DryRun
  ↓
New / Update / Blocked / Error集合確定
  ↓
ExpectedNewPo / ExpectedUpdatePo 自動生成
  ↓
安全条件判定
  ↓
Form master必要時のみ更新/Deploy
  ↓
Execute直前App272再GET
  ↓
新規POST
  ↓
POST後再GET検証
  ↓
既存PUT
  ↓
PUT後再GET検証
  ↓
完了ログ/終了コード
```

### 16.1 無人実行での原則

- DryRun失敗 → Executeしない
- Record errors > 0 → Executeしない
- 未知K日程種別 → 対象保護。件数やポリシーにより全体停止を検討
- Preview/Live差分あり → Executeしない
- option追加上限超過 → Executeしない
- New/Update件数上限超過 → Executeしない
- candidate集合変化 → Executeしない
- revision競合 → PUTしない
- Post/Put verification error > 0 → 異常終了

### 16.2 初期版の書込み順序

Form option補完・Deployを必要時に最初に完了させた後、初期版では次の順序を標準とする。

1. 新規POST
2. POST後再GET検証
3. 既存PUT
4. PUT後再GET検証

POST対象とPUT対象は発注番号が重複しない独立集合だが、途中停止時の影響とログ追跡を単純化するため、
「新規未作成」という比較的被害の小さい処理を先に完了させる。

ただしPOST後検証が1件でも失敗した場合はPUTへ進まない。


### 16.3 構造化Manifest JSONの最低スキーマ

初期案:

```json
{
  "schemaVersion": "1.0",
  "runAt": "2026-09-16T09:00:00+09:00",
  "environment": "PROD",
  "targetApp": 272,
  "sourceApp": 270,
  "inputCsvPath": "C:\\HPDB\\外注課納期管理_YYYYMMDD_整形済み.csv",
  "inputCsvSha256": "...",
  "formBaselineSha256": "...",
  "new": [
    {
      "po": "2609xxxxxx",
      "classification": "POST:NEW"
    }
  ],
  "updates": [
    {
      "po": "2609xxxxxx",
      "recordId": "1234",
      "revision": "56",
      "classification": "PUT:RECALC",
      "changedFields": ["日付_1", "K加工着手日", "K完成検査日"],
      "before": {},
      "after": {},
      "changeHash": "..."
    }
  ],
  "protected": [],
  "errors": [],
  "counts": {
    "csvRows": 0,
    "new": 0,
    "updates": 0,
    "protected": 0,
    "errors": 0
  }
}
```

`changeHash`は、PO・recordId・期待revision・変更フィールド名・変更前値・変更後値を
**固定順序で正規化したJSON**からSHA-256を算出する。
PowerShellのHashtable列挙順や人間向け表示順に依存しないこと。

Wrapperは`schemaVersion`不一致、JSON parse失敗、必要キー欠落時に必ず停止する。

---

## 17. 上限値

初期運用案:

- `MaxAutoCreates`: 100
- `MaxAutoUpdates`: 50
- `PutBatchSize`: 10（U12結果反映後のApp292初期試験値）
- `MaxNewVendorOptions`: 20
- `MaxNewStaffOptions`: 10

`MaxAutoUpdates` は1回のDaily Import全体で許可する更新総数、
`PutBatchSize` は1回の `PUT /k/v1/records.json` に含める更新件数とする。

更新は通常レアケースと想定するためPOSTより低い上限を設定し、
U12で原子性を確認済みであっても初期版では少数バッチを維持する。

更新件数が異常に多い場合、CSV/基幹側の大規模変更やマッピング異常の可能性として自動停止させる。

---

## 18. App292試験計画

既存PUT機能は、必ずApp292で実証後にApp272へ反映する。

### Test U1: 納期変更 / Batch / 通常再計算

1. App292へ既存テストレコードを作る
2. CSVで同一発注番号の納期だけ変更
3. DryRunで `PUT:RECALC` を確認
4. 旧→新納期とK日程候補を確認
5. Execute
6. App292画面と再GETで一致確認

期待:

- 日付_1更新
- K加工着手日再計算
- K完成検査日再計算
- Batch維持

### Test U2: 納期変更 / Batch / ShortLeadTime

変更後納期を短くし通常計算不可にする。

期待:

- 日付_1更新
- K加工着手日 = 手配日
- K完成検査日 = 新納期
- Batch / ON
- K計算根拠空欄

### Test U3: Vendor変更 / 未登録Vendor

同一POのVendorを未知値へ変更。

期待:

- Missing Vendor検出
- Preview option追加
- Deploy SUCCESS
- VENDOR_TEXT/VENDOR_DD更新
- 新VendorルールでK日程再計算またはFallback

### Test U4: Staff変更 / 未登録Staff

期待:

- STAFF_TEXT/STAFF_DD更新
- K日程は変更しない
- 必要ならStaff optionだけ追加・Deploy

### Test U5: Manual＋納期変更

期待:

- 基幹納期はCSV値へ更新
- K加工着手日/K完成検査日は変更しない
- K日程種別Manual維持
- ManualScheduleDueDateChangedを警告
- K日程と新納期矛盾があれば明示

### Test U6: revision競合

DryRun後、App292画面から対象レコードを人為的に編集してrevisionを上げる。

期待:

- Execute直前またはPUTで競合検出
- 当該レコードを上書きしない
- 安全停止

### Test U7: ExpectedUpdatePo不一致

DryRunで確定した更新集合とExecute指定集合を意図的に変える。

期待:

- PUT前停止
- 書込みなし

### Test U8: 更新件数上限

`MaxUpdates` を候補数未満に設定。

期待:

- PUT前停止

### Test U9: NOCHANGE

同一CSVを再投入。

期待:

- POST=0
- PUT=0
- 冪等性PASS

### Test U10: 混在E2E

1つのCSV内に:

- NEW 1件
- UPDATE_RECALC 1件
- UPDATE_BASE_ONLY 1件
- Manual変更 1件
- NOCHANGE 1件

を含める。

DryRun→Execute→再GETまで一連で確認する。

### Test U11: Manual保護＋未知Vendor変更

ManualレコードのCSV側Vendorを、App292フォームに存在しない未知Vendorへ変更する。

期待:

- PUT候補として分類される
- 必要option集合に未知Vendorが含まれる
- Preview option追加
- Deploy SUCCESS
- VENDOR_TEXT / VENDOR_DD はCSV値へ更新
- ManualのK加工着手日 / K完成検査日は変更されない
- K日程種別=Manual維持
- POST候補＋PUT候補を横断したoption集合算出がPASS

### Test U12: 複数PUT原子性【PASS / CLOSED】

実施日時: 2026-09-16 10:43 JST  
対象App: 292  
対象:
- A: PO `2609249313`, RecordId=2
- B: PO `2609249755`, RecordId=3
- テストフィールド: `STAFF_TEXT`

手順:
1. A/B baseline revision=1を確認
2. Bだけ単独PUTしrevision 1→2
3. Aはrevision=1（有効）、Bはrevision=1（意図的に古い）で2件複数PUT
4. HTTP 409 Conflictを確認
5. 再GETでAが未変更、Bが競合作成時の値のままであることを確認
6. Bを元値へ復元

結果:

```text
MultiPutApiSuccess : False
Conclusion         : ATOMIC_ALL_OR_NOTHING
CleanupA           : ALREADY_ORIGINAL
CleanupB           : RESTORED
```

判定: **PASS**

Phase B決定:
- revision指定付き複数PUTを採用可能
- 初期 `PutBatchSize=10`
- バッチ直前ガード＋バッチ直後再GET検証を必須
- エラー時は後続バッチ停止

---

## 19. 本番移行条件

以下をすべて満たすまでApp272既存PUTを有効化しない。

1. App292でU12（複数PUT原子性）を最初にPASSし、PUT方式を確定
2. App292でU1〜U11 PASS
3. DryRunとExecuteの対象集合・revision・変更内容ハッシュ完全一致ガード確認
4. revision競合試験PASS
5. Manual保護試験PASS
6. Manual＋未知Vendor複合試験PASS
7. Vendor/Staff unknown option試験PASS
8. フォールバック試験PASS
9. 冪等性試験PASS
10. POST/PUT後検証PASS
11. 構造化Manifest JSONをWrapperが正式I/Fとして利用し、標準出力解析に依存しないこと
12. Claudeレビュー指摘反映済み
13. 本番初回は少数件の更新候補で人手確認して実施

---

## 20. 実装フェーズ案

### Phase A — 現行POST自動化の安全確認

- `Run-App272-DailyImport_v0.1.ps1` をPLAN ONLYで確認
- 新規0件時の安全終了確認
- 新規発生時のExpectedNewPo自動生成確認

### Phase A0 — 複数PUT原子性の先行検証【CLOSED】

- 2026-09-16 App292でU12実施済み
- HTTP 409時に競合していないレコードも未更新であることを再GET確認
- `ATOMIC_ALL_OR_NOTHING` を実測
- Phase Bは **複数PUT＋少数バッチ** 方式で着手可
- 初期 `PutBatchSize=10`

### Phase B — 既存PUT実装

新しい下位スクリプト案:

`Import-App272_DailySync_v0.8.0.ps1`

または現行系譜を維持する場合:

`Import-App272_Phase1C_v0.8.0_DailySync.ps1`

追加予定パラメータ:

```powershell
[string[]]$ExpectedNewPo
[string[]]$ExpectedUpdatePo
[int]$MaxCreates
[int]$MaxUpdates
```

### Phase C — App292回帰試験

U1〜U11を実施（U12はPhase A0で先行済み）。

### Phase D — Wrapper統合

`Run-App272-DailyImport_v0.2.ps1`

DryRunが出力する専用Manifest JSONからNew/Update両マニフェストを読み込む。

**禁止**:
- 標準出力の文言パース
- Summaryテキストの正規表現抽出
- 人間向けCSVの列位置依存

Wrapperは構造化Manifestのschema versionを確認し、不一致なら停止する。

### Phase E — 本番初回

- DryRun
- 人手レビュー
- Execute
- 画面目視
- ログ保管

### Phase F — 定時自動化

安定後にタスクスケジューラ/RK-10等から定時起動する。

---

## 21. Claudeレビュー結果とRev.2反映

2026-09-16のレビュー結果を以下の通り反映した。

1. **Manual＋未知Vendor変更の複合ケース**  
   → POST候補＋PUT候補を横断した必要option集合算出を14章へ明記。U11追加。

2. **Manual保護しつつ基幹納期を更新**  
   → 妥当として維持。

3. **Batch/Auto再計算トリガー**  
   → 妥当として維持。

4. **Autoは差分がある時だけBatch化**  
   → 妥当として維持。差分なし既存AutoはDaily Importで触らない。

5. **Fallbackの既存PUT適用**  
   → v0.7.4実証済みロジックを再利用する方針を維持。

6. **revision/TOCTOU**  
   → revision一致＋変更前値一致＋変更内容ハッシュ一致を必須化。

7. **ExpectedUpdatePoと変更内容ハッシュ**  
   → 11.2章を「revisionまたはhash」から「revisionおよびhash」に修正。

8. **複数レコードPUT原子性**  
   → Phase B着手前の最優先試験U12へ前倒し。

9. **POST/PUT順序**  
   → 初期版は Form option準備 → POST → POST検証 → PUT → PUT検証 を標準順序とする。

10. **option DeployとRecord write境界**  
    → 既存のM1〜M6系安全設計を維持し、Record write前にLive反映を確認。

11. **Wrapperのログ解析依存**  
    → 標準出力解析を廃止し、専用Manifest JSONを正式I/Fとする。

12. **追加異常系テスト**  
    → U11（Manual＋未知Vendor）とU12（複数PUT原子性）を追加。

Claudeレビューで示された「仕様骨格・安全思想に重大な問題なし」という評価を踏まえ、
Rev.2では上記の構造的リスク2点（PUT原子性、ログ解析依存）を実装前提条件として明文化した。

---

## 22. 今回の方針決定

2026-09-16時点では、App272本番への新規POST機能 v0.7.4 は実証済み。

既存レコード更新はまだ本番実装しない。

次工程は:

1. Rev.2確定
2. U12 PASS / Phase A0 CLOSED（2026-09-16）
3. Phase B: App292用の既存PUT実装（複数PUT＋PutBatchSize=10）
4. DryRun専用Manifest JSON出力を実装
5. U1〜U11試験
6. WrapperをManifest JSON正式I/Fへ変更
7. 本番App272でDryRunのみ
8. 更新候補が少数のタイミングで初回本番PUT
9. 安定後に定時自動化

以上。


---

## 23. Phase B U1 実測結果 — 既存Batch納期変更【PASS / CLOSED】

実施日: 2026-09-16  
対象: App292（TEST）  
PO: `2609249755`  
RecordId: `3`

### 23.1 目的

既存 `K日程種別=Batch` レコードについて、CSV側の納期だけが変更された場合に、

- 既存更新として検出する
- `UPDATE_RECALC` に分類する
- 基幹納期 `日付_1` を更新する
- `K加工着手日 / K完成検査日` を新納期から再計算する
- `Batch / ON / Company` を維持する
- Manifest JSONに revision / before / after / changedFields / changeHash を出力する
- Execute直前ガードを通した後にPUTする
- PUT後の再GETで実値検証する

ことを確認する。

### 23.2 テスト基準状態

U1前処理として App292 の PO `2609249755` を以下へ整えた。

```text
Revision        : 3 -> 4
VENDOR_TEXT     : ㈱吉川鉄工所
VENDOR_DD       : ㈱吉川鉄工所
STAFF_TEXT      : 木村　和也
STAFF_DD        : 木村　和也
手配日          : 2026-09-15（不変）
納期            : 2026-11-02（不変）
K加工着手日     : 2026-10-26
K完成検査日     : 2026-10-30
K日程種別       : Batch
K暫定設定       : ON
K計算根拠       : Company
```

基準状態設定後の再GET検証は全項目 PASS。

### 23.3 U1用CSV

元CSVから対象POを1行だけ抽出し、変更点を納期1項目だけに限定した。

```text
PO              : 2609249755
旧納期          : 2026-11-02
新納期          : 2026-11-06
Changed fields  : 納期
```

ファイル:
`C:\HPDB\App292_U1_2609249755_DueChanged_20260916.csv`

### 23.4 DryRun結果

使用スクリプト:
`Import-App272_PhaseB_Update_v0.8.0.3_TEST.ps1`

```text
Mode             : DRYRUN
App              : 292
CSV rows         : 1
Updates          : 1
Policy           : 2026-09-16-U12-1
U12              : ATOMIC_ALL_OR_NOTHING
PutBatchSize     : 10

PO=2609249755 ID=3 Rev=4 Class=UPDATE_RECALC
Changed : 日付_1,K加工着手日,K完成検査日
Due     : 2026-11-02 -> 2026-11-06
KStart  : 2026-10-26 -> 2026-11-02
KInspect: 2026-10-30 -> 2026-11-05
KType   : Batch -> Batch
KOnOff  : ON -> ON
KBasis  : 'Company' -> 'Company'
Hash    : 2a6a0416d8c969858a2bfd188e2395bb16492b5a5c68e4729571266638de9354

WRITE    : NONE
RESULT   : DRYRUN PASS
```

Manifest:
`C:\HPDB\logs\App292_PhaseB_Manifest_20260916_113829.json`

Detail:
`C:\HPDB\logs\App292_PhaseB_Detail_20260916_113829.csv`

### 23.5 Execute結果

実行条件:

- `-Execute`
- `-ExpectedUpdatePo "2609249755"`
- `-MaxUpdates 1`
- `-ConfirmExecute "APP292-PHASEB-EXECUTE"`

Execute前に以下を再GETして一致確認した。

- RecordId
- revision
- before値
- changeHash

PUT後、対象レコードを再GETし、変更対象フィールドを照合した。

```text
WRITE  : MULTI PUT EXECUTED
RESULT : EXECUTE PASS / post reGET verified
```

Manifest:
`C:\HPDB\logs\App292_PhaseB_Manifest_20260916_113924.json`

Detail:
`C:\HPDB\logs\App292_PhaseB_Detail_20260916_113924.csv`

### 23.6 判定

**U1 = PASS / CLOSED**

App292において、既存BatchレコードのCSV納期変更を検出し、基幹納期とK日程を再計算し、安全ガード付きでPUTし、再GETで実値検証する一連の経路を実証した。

次試験は `UPDATE_BASE_ONLY` 系を対象とする。
