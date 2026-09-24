# Ver0.6.2.3-PROD-CANDIDATE1 詳細設計
## App293 → App272 受入実績同期

### 位置づけ
TEST7/TEST8 PASS後のApp272正式Execute候補。Claudeレビュー前のため、まだApp272へDeploy/Executeしない。

### Execute環境
- App272 (ID=272): Analysis + Controlled Execute
- App292 (ID=292): Analysis Only / Execute禁止
- App293 (ID=293): READ ONLY

### 対象
- CANDIDATE_EMPTY_SINGLE
- FutureAutoEligible=YES
- App272受入れ日が空欄
- SourceRowCount=1
- NegativeReceiptCount=0
- 有効CandidateReceiptDateあり

### 1回の書込み上限
MaxWrites=50。
候補総数が50を超えていても自動で先頭50件を選ばない。
人がチェックした件だけを実行し、1回最大50件。
98件なら最大50件＋残り48件の2回に分割可能。

これはTEST7の「候補総数>50なら全面SafetyStop」からのPROD運用上の変更点。
**MaxWritesを「1回の実PUT件数上限」として適用する。**
この意味変更をClaude重点レビュー対象とする。

### Safety
1. App272発注番号重複 → SafetyStop
2. App293受入番号空欄/重複 → SafetyStop
3. Form type不一致 → Fail Closed
4. 選択上限50
5. Class/FutureAutoEligible二重ガード
6. confirm
7. 最終確認文字列 `APP272-ACCEPTANCE-EXECUTE:<件数>`
8. Execute直前に全Analysis再実行
9. 選択POについてClass/FutureAutoEligible/targetId/targetRevision/CurrentAcceptanceDate/CandidateReceiptDate/SourceRowCount/NegativeReceiptCountの一致を要求
10. 1件でも変化 → WRITE 0 / Auto Retryなし
11. revision付きPUT
12. PUTフィールドは受入れ日のみ
13. PostVerify全件
14. mismatch → FAIL / Auto Retryなし

### PROD初回手順案
Claude PASS後:
1. App272へPROD候補版を適用
2. Analysisのみ
3. PowerShell PROD DryRunと再比較
4. まず1件だけPROD Execute
5. PostVerify / 再Analysis ALREADY_SAME確認
6. 問題なければ最大50件単位で段階実行
