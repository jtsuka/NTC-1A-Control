# ProcessSheetRows r15.10 REV5.1 Implementation Candidate 静的監査

作成日: 2026-09-24

## 位置づけ

このPS1は **Claude静的クロスレビュー用 Implementation Candidate** です。
本番投入用ではありません。RK-10採用変数を変更してはいけません。

Baseline:

``` text
BASELINE_PRESERVED_ProcessSheetRows_20260917_r15_9_Rev3_MFlagUniversal_Phase3State_DuplicateFailClosed.ps1
SHA256=df7a06b42064b5401c67ed1d5719bdce7bb8dfbaa5df02f7a1fca1c5caed2e98
```

Implementation Candidate:

``` text
ProcessSheetRows_20260924_r15_10_REV5_1_2CSV_ImplementationCandidate.ps1
SHA256=88f48ae1584ded90d249a9a30872611e94397364f93e8a9fba573ffeecefb04d
```

## 実装した主要項目

-   DatasetCount=1/2とRow4列の読込
-   Config Preflight / Row→SourceType / Unknown Row fail-closed
-   2CSV CandidateKey=`Code|SourceType`
-   Main/Sub同Codeを別候補として保持
-   同一Source重複をCandidateKey単位で検出
-   staging A:H（G=SourceType, H=SourceLabel）
-   Phase3 duplicateCodes/runtimeSeenのCandidateKey化
-   2CSVでManagedCodes/数字シートCode単独を即Blockにしない
-   staging削除guardへSourceType追加
-   History sheet `新規商品コード登録履歴`
-   2CSV Baseline未初期化時Phase3 fail-closed
-   `-InitializeNewProductBaseline` 明示指定時のみBaseline初期化
-   Baseline初期化とPhase3 Moveを同一実行で混在させない
-   Phase3成功後CandidateKey単位History登録
-   History書込失敗時はSave前例外

## 変更禁止領域の関数単位照合

  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  Function                        Baseline SHA256                                                      Candidate SHA256                                                     Same
  ------------------------------- -------------------------------------------------------------------- -------------------------------------------------------------------- -----------------
  `Get-MFlagPolicyAnalysis`       `89706149f91ee0779e2bd69fd1d9bc7c7dc33874f07fcdf742ae51388c6b7e52`   `89706149f91ee0779e2bd69fd1d9bc7c7dc33874f07fcdf742ae51388c6b7e52`   **YES**

  `Invoke-LegacyMFlagGapRepair`   `520ce9b9bb77b2453145642429e7af365e56a33ce6d598b32dc0dc3717af4085`   `520ce9b9bb77b2453145642429e7af365e56a33ce6d598b32dc0dc3717af4085`   **YES**

  `Invoke-LegacyGapRepair`        `f49f89442882588f88c608d5bd0dd5628f852a590cb02ed482274f130816f477`   `f49f89442882588f88c608d5bd0dd5628f852a590cb02ed482274f130816f477`   **YES**

  `Invoke-RestorePlan`            `30d53a6e70fe0d48e820353c84b3a07ef93685fd922f60f7a2a7e804abccba35`   `30d53a6e70fe0d48e820353c84b3a07ef93685fd922f60f7a2a7e804abccba35`   **YES**
  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

4関数は全て完全一致。

## Diff概要

``` text
Unified diff lines=735
Added=351
Removed=126
```

## 未実機確認

PowerShell 5.1 parser、Excel COM、Audit、Baseline候補件数、staging
A:H、Phase3実移動、History書込、翌日冪等性は未確認。

``` text
Design=APPROVED
Static implementation=Candidate created
Production readiness=NO
RK-10 adoption=DO NOT CHANGE
新規コード管理_実行=無のまま
```
