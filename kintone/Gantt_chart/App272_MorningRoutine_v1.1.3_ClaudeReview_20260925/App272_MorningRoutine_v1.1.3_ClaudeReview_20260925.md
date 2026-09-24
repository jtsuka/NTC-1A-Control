# App272 MorningRoutine v1.1.3 Claude再レビュー依頼

## 判定依頼
v1.1.3候補を、v1.1.2に対する**最小PS5.1互換修正**としてレビューしてください。
本番Executeはまだ行いません。

## 2026-09-25 08:09 実機DryRun結果
- Phase1C: CSVRows=313 / App272 New=31 / App270 Add=16 / Planned POST=15
- New fallback=4（ShortLeadTime=3 / MasterNotFound=1）
- Record errors=0 / Warnings=0
- PhaseB: Blocked New App272=31 / Planned PUT=0 / Record errors=0 / Warnings=0
- F2後の比較処理で停止:
  `このオブジェクトにプロパティ 'Value' が見つかりません。`
- App272/App270へのWRITEはなし。TMPは保持された。

## 原因
`Get-PhaseBPlan` の `ExpectedBlockedNew` は `[Nullable[int]]` 宣言だが、
Windows PowerShell 5.1で実引数31を受けると比較時の実体が `System.Int32` となり、
`$ExpectedBlockedNew.Value` が存在せず停止した。

## 修正方針
業務ロジック、分類、Fail Closed条件、PO集合照合、Phase1C、PhaseBには変更を加えない。
`$ExpectedBlockedNew.Value` の2参照だけを `[int]$ExpectedBlockedNew` に変更する。
表示上の版名のみ v1.1.2 -> v1.1.3 とする。

## SHA256
- 修正前 v1.1.2: `c4ccdc73835c4bd2029a6cac3d2b87a5ba72f0892a88cc97bd9a3272f4e86356`
- 修正後 v1.1.3: `68246cd637a3bc1a778c62874ca3fed9f64e5fe4be7f3bc4ec8353bd73fb380b`

## 完全差分
```diff
--- Run-App272-MorningRoutine_v1.1.2_NoLookup_CANDIDATE_REVIEW(1).ps1
+++ Run-App272-MorningRoutine_v1.1.3_NoLookup_CANDIDATE_REVIEW.ps1
@@ -1,7 +1,7 @@
 #requires -Version 5.1
 <#
 .SYNOPSIS
-  App272 Morning Routine v1.1.2 NoLookup
+  App272 Morning Routine v1.1.3 NoLookup
 
 .DESCRIPTION
   2026-09-17時点の本番早朝処理用ラッパー。
@@ -277,8 +277,8 @@
             Stop-Safety "NoLookup sees unexpected Blocked New App272=$blockedNew."
         }
     }
-    elseif($blockedNew -ne $ExpectedBlockedNew.Value){
-        Stop-Safety "NoLookup BlockedNew mismatch. Expected=$($ExpectedBlockedNew.Value) Actual=$blockedNew"
+    elseif($blockedNew -ne [int]$ExpectedBlockedNew){
+        Stop-Safety "NoLookup BlockedNew mismatch. Expected=$([int]$ExpectedBlockedNew) Actual=$blockedNew"
     }
 
     return [pscustomobject]@{
@@ -314,7 +314,7 @@
     Require-File $ExistingPutScript 'PhaseB PROD script'
 
     Write-Host '================================================================='
-    Write-Host 'App272 Morning Routine v1.1.2 NoLookup'
+    Write-Host 'App272 Morning Routine v1.1.3 NoLookup'
     Write-Host '================================================================='
     Write-Host ("Mode           : {0}" -f $Mode)
     Write-Host ("DateTag        : {0}" -f $DateTag)
```

## レビューしてほしい点
1. `.Value` -> `[int]$ExpectedBlockedNew` がWindows PowerShell 5.1で妥当か。
2. `$null -eq $ExpectedBlockedNew` 分岐との組合せで、未指定時と指定時を正しく区別できるか。
3. Expected=31 / Actual=31 がPASSし、不一致時は従来どおりSafetyStopするか。
4. F1の New=31 / POST eligible=15 / Deferred=16 の分類・上限20の意味を変えていないか。
5. Execute時のExpectedNewPoがPOST eligible 15件だけである安全条件を変えていないか。
6. E2/V2でExpectedBlockedNew=Deferred 16を要求する設計を変えていないか。
7. V1のDeferred PO集合完全一致を変えていないか。
8. PS5.1構文・型変換上の新たな懸念がないか。
9. 今回の修正以外に本番Execute前のブロッカーがあるか。

## 次の試験
ClaudeレビューPASS後、同じTMPを `-ReusePrepared` でDryRun。
期待値:
- New=31
- POST eligible=15
- Deferred(App270)=16
- Unexpected blocked=0
- F2 BlockedNew=31
- PUT=0 / Errors=0 / Warnings=0
- `MORNING ROUTINE DRYRUN PASS`
