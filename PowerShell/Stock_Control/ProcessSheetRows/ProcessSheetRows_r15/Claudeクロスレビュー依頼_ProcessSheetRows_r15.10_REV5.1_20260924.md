# Claude クロスレビュー依頼

## ProcessSheetRows r15.10 REV5.1 Implementation Candidate

2026-09-24

REV.5.1 FIXED設計は `APPROVED` 済みです。
今回は同梱した実装コードをBaseline
Rev3と直接diffして静的レビューしてください。

Baseline SHA256:
`df7a06b42064b5401c67ed1d5719bdce7bb8dfbaa5df02f7a1fca1c5caed2e98`

Candidate SHA256:
`88f48ae1584ded90d249a9a30872611e94397364f93e8a9fba573ffeecefb04d`

## 重点確認

PASS / 要修正 / Critical / 未確認 で回答してください。

1.  VariableTable DatasetCount/Row4列読込
2.  DatasetCount=1回帰
3.  DatasetCount=2 Config Preflight
4.  Row→SourceType
5.  Unknown Row機能全体fail-closed
6.  CandidateKey=`Code|SourceType`
7.  Main/Sub同Code保持
8.  同一Source duplicate
9.  staging A:H
10. staging keys + History keysのKnownKeys統合
11. 2CSVでManagedCodes Code単独をPhase1除外しない
12. BaselineManaged候補抽出
13. `-InitializeNewProductBaseline`安全性
14. Baseline初期化とPhase3を同一実行で混在させない
15. Baseline未初期化時Phase3 fail-closed
16. Phase3 duplicateCodes CandidateKey化
17. runtimeSeen CandidateKey化
18. 2CSV numeric Code存在を診断へ降格
19. History重複CandidateKey Block
20. Phase3成功後History登録
21. staging削除guard SourceType
22. History失敗時Save前停止
23. MaxNewProductCandidates/MaxNewProductMoves
24. Audit non-update
25. single final Save
26. Invoke-LegacyMFlagGapRepair不変
27. DuplicateCandidateFailClosed不変
28. Invoke-LegacyGapRepair不変
29. FormulaRestore不変
30. Windows PowerShell 5.1構文

特にHistory
schema/marker、Baseline候補条件、初回Baseline後のKnown判定、History登録順序、1CSV
A:F互換、COM release、StrictMode 2.0を厳しく確認してください。

このCandidateはまだ実行しません。静的レビューのみです。

最終判定:

``` text
r15.10 REV5.1 IMPLEMENTATION: GO TO AUDIT
r15.10 REV5.1 IMPLEMENTATION: GO WITH MINOR
r15.10 REV5.1 IMPLEMENTATION: HOLD
```
