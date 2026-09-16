#requires -Version 5.1
<#
.SYNOPSIS
  App272 DailySync Phase B policy v0.1

.DESCRIPTION
  2026-09-16 App292 U12 result (ATOMIC_ALL_OR_NOTHING) をコード側へ固定するための
  ポリシー定義。今後の Import-App272_Phase1C_v0.8.0_DailySync.ps1 から読み込む前提。

  U12実測:
    - 2件複数PUT
    - 1件だけstale revision
    - API: HTTP 409
    - 非競合レコードも未更新
    - Conclusion: ATOMIC_ALL_OR_NOTHING

  注意:
    原子性確認済みでも大量一括PUTを許可する根拠にはしない。
    初期実装は少数バッチとし、各バッチ前後の安全確認を必須とする。
#>

Set-StrictMode -Version 2.0

$script:App272PhaseBPolicy = [ordered]@{
    PolicyVersion = '2026-09-16-U12-1'
    U12Result = 'ATOMIC_ALL_OR_NOTHING'
    U12VerifiedApp = 292
    U12VerifiedAt = '2026-09-16T10:43:05+09:00'

    MultiPutAllowed = $true

    # 全Daily Importで許可する既存更新総数
    MaxAutoUpdates = 50

    # 1回の PUT /k/v1/records.json に含める初期件数
    PutBatchSize = 10

    # Execute直前ガード
    RequireExpectedUpdatePoExactSet = $true
    RequireRecordIdMatch = $true
    RequireRevisionMatch = $true
    RequireBeforeValueMatch = $true
    RequireChangeHashMatch = $true

    # バッチ実行後
    RequirePostBatchReGetVerification = $true
    StopFollowingBatchesOnAnyApiError = $true
    StopFollowingBatchesOnAnyVerificationError = $true

    # 競合/異常時
    ReuseOldManifestAfterConflict = $false
}

function Get-App272PhaseBPolicy {
    [CmdletBinding()]
    param()
    return [pscustomobject]$script:App272PhaseBPolicy
}
