# Claudeレビュー依頼
## Ver0.6.2.3-PROD-CANDIDATE1 App272受入実績同期
### 2026-09-24

TEST7/TEST8実機結果を同梱。

## 既にPASS済み
- App293 1,135件 / 発注番号空欄39件
- App272/App292 937件
- JS Analysis 1,868行
- PowerShell DryRunと発注番号単位100% MATCH
- TEST8 App292 1件PUT PASS
- TEST8 App292 2件同時PUT PASS
- PostVerify PASS
- 再AnalysisでALREADY_SAME遷移 PASS
- TEST8コードレビュー PASS

## PROD候補版の変更
1. Execute許可をApp292→App272へ切替
2. App292はAnalysis Only
3. PROD手動選択上限を3→50
4. 候補総数>50でも全候補をSafetyStopにはせず、**選択された実PUT件数が50以下なら実行可**
5. Execute直前に再Analysisし、選択POの状態一致を再検証
6. 最終確認文字列 `APP272-ACCEPTANCE-EXECUTE:<件数>` を追加

## 重点レビュー
R01 App272以外でExecute不能か
R02 App293 READ ONLYか
R03 書込み対象がSINGLE/YESのみか
R04 書込みフィールドが受入れ日のみか
R05 既存受入れ日保護が維持されているか
R06 MaxWrites=50を「1回の実PUT上限」と再定義してよいか
R07 候補98件を50+48に分割する設計に安全上の問題がないか
R08 自動で先頭50件選択せず、手動選択のみか
R09 Execute直前reAnalysis guardは十分か
R10 revision guardは維持されているか
R11 PostVerifyは十分か
R12 App272 duplicate PO SafetyStop維持か
R13 App293受入番号空欄/重複SafetyStop維持か
R14 NO_SOURCE_DATA_UNKNOWNを未受入と表現していないか
R15 自動再試行が無いか
R16 初回PRODは1件だけ→確認→段階拡大で妥当か

## 回答希望
A. 総合判定 PASS / CONDITIONAL PASS / FAIL
B. 必須修正
C. 推奨修正
D. MaxWrites意味変更の可否
E. 初回App272 1件Executeへ進んでよいか
