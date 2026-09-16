# Claude クロスレビュー依頼 — App272 Phase B U1

## レビュー目的

App272 Daily Import の Phase B（既存レコードPUT）について、U12の原子性確認後にU1を実装・実測した。

添付一式を読んだうえで、**コード・仕様・テスト方法・実測結果の整合性を独立にレビュー**してほしい。

単に「PASS結果が出ている」ことではなく、実装上の抜け・危険な前提・PowerShell 5.1特有の問題・kintone REST API上の見落としがないかを重点確認してほしい。

## 今回の結論

- U12: `ATOMIC_ALL_OR_NOTHING` → CLOSED
- U1: 既存Batchの納期変更 → `UPDATE_RECALC` → PUT → post reGET verify → PASS / CLOSED
- 次は `UPDATE_BASE_ONLY` 系の試験へ進む予定

## 特に確認してほしい点

1. `Import-App272_PhaseB_Update_v0.8.0.3_TEST.ps1` の分類ロジック
   - Batch / Auto / Manual / blank の扱い
   - U1が `UPDATE_RECALC` になる条件
   - 実際のChangedFieldsだけをPUT対象にしている点

2. 日程再計算
   - 完成検査日 = 納期からマスタ日数を暦日減算後、前営業日補正
   - 加工着手日 = 完成検査日からマスタ日数を暦日減算後、次営業日補正
   - Company / Provisionalの判定
   - ShortLeadTime fallback
   - MasterNotFound fallback

3. Execute前ガード
   - RecordId
   - revision
   - before値
   - changeHash
   - ExpectedUpdatePo完全一致
   - MaxUpdates

4. changeHash
   - fixed-order相当のcanonical stringとして十分か
   - Hashtable列挙順に依存していないか
   - changedFields sortの扱いに問題がないか

5. PUT
   - U12で実測した原子性を前提に少数バッチ化した設計が妥当か
   - `PutBatchSize=10` の初期値が妥当か
   - バッチ後再GET検証で十分か
   - エラー時の停止・再開設計に不足がないか

6. Manifest
   - Rev.2.2が要求する機械可読Manifestとして不足項目がないか
   - 将来Wrapperが標準出力や人間用CSVを解析せずに動けるか

7. U1実測
   - DryRunのBefore/After
   - `ChangedFields = 日付_1,K加工着手日,K完成検査日`
   - Execute後再GET
   - U1を正式CLOSEDとしてよいか

8. Windows PowerShell 5.1互換
   - `[ordered]` parameter type誤用はv0.8.0.1で修正済み
   - Generic Listの `@($list)` 問題は `.ToArray()`へ修正済み
   - 他にPS5.1特有の潜在問題が残っていないか

## レビュー回答形式

以下の形式で回答してほしい。

### 1. 総合判定
- GO / CONDITIONAL GO / STOP

### 2. U1 CLOSED判定
- CLOSED可 / 追加試験必要

### 3. 重大問題
- あれば列挙
- なければ「重大問題なし」

### 4. 修正推奨
- Must
- Should
- Nice to have

### 5. 次試験への提言
`UPDATE_BASE_ONLY` をどういう入力で試すべきか、具体的なテストケースを提示。

## 添付主要ファイル

- `App272_DailyImport_..._Rev2.2.md`
- `App272-PhaseB-PutPolicy_v0.1.ps1`
- `Import-App272_PhaseB_Update_v0.8.0.3_TEST.ps1`
- `Test-App292-U12_MultiPutAtomicity_v0.1.2.ps1`
- `PhaseB_U1_CLOSED_実測結果_20260916.md`
- U1前処理・CSV生成スクリプト
