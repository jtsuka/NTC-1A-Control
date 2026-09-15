<#
.SYNOPSIS
    船舶値付用紙 - Apeos C3570 ジョブ履歴API 単独確認スクリプト（Rev.4仕様準拠）

.DESCRIPTION
    Print-ValueTag.ps1 本体にはまだ組み込まない、単独の実機テスト用ツール。
    次の4点を1回の実行でまとめて確認する。

      1. PrintDocument.DocumentName に設定した識別子が、Apeos側の NetInFilename に
         そのまま反映されるか
      2. Windows側の投入時刻／Apeos画面表示時刻／API Created／API Completed の
         4点比較材料をログに残す
         ※注意：本スクリプト自体は印刷を行わない。ここで記録する時刻は
         「本スクリプトの確認処理を開始した時刻」であり、実際のPrint()呼び出し時刻
         とは厳密には別物。正確な投入時刻は Test-PrintApeosDocumentName.ps1 側で
         記録し、そちらと突き合わせること（画面表示時刻は引き続き手動確認）。
      3. ポーリング中に観測された State の遷移（PROCESSING → PRINTING → COMPLETED
         のように途中状態を経由するのか、最初から COMPLETED しか出ないのか）を
         タイムスタンプ付きで記録する
      4. 最終的に State が COMPLETED になるか

    詳細仕様は「船舶値付用紙－FUJIFILM複合機ジョブ履歴API連携_詳細仕様書.md」Rev.4を参照。

.PARAMETER ApeosIP
    Apeos複合機のIPアドレス。既定値は船舶機械部のC3570。

.PARAMETER ExpectedPCName
    RK-10実行PC名。job-list APIの UserName / UserID と一致させる。

.PARAMETER ExpectedDocumentName
    PrintDocument.DocumentName に設定した一意な識別子。
    例: SHIP_VALUETAG_TEST_20260916_060000_A7F3
    PowerShellのワイルドカード文字（* ? [ ]）を含めないこと（-like のエスケープを省略しているため）。

.PARAMETER ExpectedCopies
    期待する印刷部数。job-list APIの CopiesRequested と一致させる。

.PARAMETER PollingIntervalSec
    ポーリング間隔（秒）。仕様書の初期値は2秒。

.PARAMETER TimeoutSec
    タイムアウト（秒）。仕様書の初期値は60秒。

.PARAMETER MaxConsecutiveApiErrors
    API通信エラーを何回連続まで許容してポーリングを継続するか。
    チャッピーのRev.4レビュー指摘（一時的なLAN応答遅延への耐性）を反映。
    既定値3（＝3回連続失敗で APEOS_API_ERROR として終了）。

.PARAMETER KnownFailureStates
    確認済みの異常終了State一覧。実機テストで判明するまでは空のままにしておくこと。
    空のままなら、このスクリプトは COMPLETED か タイムアウト でしか終了しない
    （＝正常進行中のジョブを誤って異常と判定しない、Rev.4の設計方針どおり）。

.PARAMETER LogPath
    実行ログ（State遷移・タイムスタンプ）の出力先CSVパス。省略時は
    カレントディレクトリに ApeosJobCheck_<実行開始時刻>.csv として出力する。

.EXAMPLE
    .\Check-ApeosPrintJob.ps1 -ExpectedPCName "RPA-5F" `
        -ExpectedDocumentName "SHIP_VALUETAG_TEST_20260916_060000_A7F3" `
        -ExpectedCopies 1

.NOTES
    このスクリプトは「確認専用」であり、印刷そのものは行わない。
    テスト印刷は別途 Print-ValueTag.ps1（またはテスト用の単発印刷）で
    このスクリプトの実行と同時か直前に実行しておくこと。
#>

[CmdletBinding()]
param(
    [string]$ApeosIP = "192.168.0.209",

    [Parameter(Mandatory = $true)]
    [string]$ExpectedPCName,

    [Parameter(Mandatory = $true)]
    [string]$ExpectedDocumentName,

    [Parameter(Mandatory = $true)]
    [int]$ExpectedCopies,

    [int]$PollingIntervalSec = 2,

    [int]$TimeoutSec = 60,

    [int]$MaxConsecutiveApiErrors = 3,

    # 8章のテストで判明した異常終了Stateをここに追加していく。実機確認までは空のまま。
    [string[]]$KnownFailureStates = @(),

    [string]$LogPath
)

# ---------------------------------------------------------------------------
# 準備
# ---------------------------------------------------------------------------

$runStartLocal = Get-Date
if (-not $LogPath) {
    $stamp = $runStartLocal.ToString("yyyyMMdd_HHmmss")
    $LogPath = Join-Path -Path (Get-Location) -ChildPath "ApeosJobCheck_$stamp.csv"
}

$uri = "http://$ApeosIP/jobs/api/job-list?methodName=GET&limit=25&typeFilter=PRINT"

# State遷移ログ（項目③④）。同じStateが連続しても、観測できたことに意味があるので
# 「前回と値が変わった時」だけでなく毎回のポーリングを1行として全件残す。
$stateLog = [System.Collections.Generic.List[object]]::new()

Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host " Apeos ジョブ履歴API 単独確認スクリプト" -ForegroundColor Cyan
Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host "実行開始（確認スクリプト開始時刻・Local。実際のPrint()時刻ではない点に注意）: $($runStartLocal.ToString('yyyy-MM-dd HH:mm:ss.fff'))"
Write-Host "対象複合機       : $ApeosIP"
Write-Host "期待PC名         : $ExpectedPCName"
Write-Host "期待DocumentName : $ExpectedDocumentName"
Write-Host "期待部数         : $ExpectedCopies"
Write-Host "ポーリング間隔    : $PollingIntervalSec 秒 / タイムアウト: $TimeoutSec 秒"
Write-Host "既知の異常終了State: $(if ($KnownFailureStates.Count -eq 0) { '(未設定)' } else { $KnownFailureStates -join ', ' })"
Write-Host ""
Write-Host "※項目②（Apeos画面表示時刻）はこのスクリプトでは取得できません。"
Write-Host "  実行と並行してブラウザーで該当ジョブの詳細モーダルを開き、表示時刻を控えておいてください。"
Write-Host ""

# ---------------------------------------------------------------------------
# ポーリングループ
# ---------------------------------------------------------------------------

$deadline = $runStartLocal.AddSeconds($TimeoutSec)
$consecutiveErrors = 0
$foundJobSnapshot = $null
$lastLoggedState = $null
$finalResult = $null
$pollCount = 0

while ((Get-Date) -lt $deadline) {
    $pollCount++
    $pollLocalTime = Get-Date

    try {
        $result = Invoke-RestMethod -Uri $uri -Method Post -ContentType "application/json" -TimeoutSec 15
        $consecutiveErrors = 0
    } catch {
        $consecutiveErrors++
        Write-Warning "API通信エラー（連続 $consecutiveErrors 回目）: $($_.Exception.Message)"

        $stateLog.Add([PSCustomObject]@{
            PollCount        = $pollCount
            LocalCheckTime   = $pollLocalTime.ToString("yyyy-MM-dd HH:mm:ss.fff")
            JobID            = $null
            State            = "(API_ERROR)"
            CreatedRaw       = $null
            CompletedRaw     = $null
            NetInFilename    = $null
            CopiesRequested  = $null
            Note             = $_.Exception.Message
        })

        if ($consecutiveErrors -ge $MaxConsecutiveApiErrors) {
            $finalResult = [PSCustomObject]@{
                Result = "APEOS_API_ERROR"
                Detail = "連続 $MaxConsecutiveApiErrors 回のAPI通信エラーによりポーリングを中断"
            }
            break
        }
        Start-Sleep -Seconds $PollingIntervalSec
        continue
    }

    # 対象ジョブを探す（4.2の照合条件：ジョブ種別・PC名・DocumentName・部数）
    $matchedInfo = $null
    foreach ($job in $result.Jobs) {
        $info = $job.JobInfo
        if ($info.UserJobType -eq "PRINT" `
            -and $info.UserName -eq $ExpectedPCName `
            -and $info.NetInFilename -like "*$ExpectedDocumentName*" `
            -and ([int]$info.CopiesRequested -eq $ExpectedCopies)) {
            $matchedInfo = $info
            break
        }
    }

    if ($matchedInfo) {
        $foundJobSnapshot = $matchedInfo

        # State変化があったとき、または初回発見時にコンソールへも表示する
        if ($matchedInfo.State -ne $lastLoggedState) {
            Write-Host "[$($pollLocalTime.ToString('HH:mm:ss'))] State変化を検知: $lastLoggedState → $($matchedInfo.State)" -ForegroundColor Yellow
            $lastLoggedState = $matchedInfo.State
        }

        $stateLog.Add([PSCustomObject]@{
            PollCount        = $pollCount
            LocalCheckTime   = $pollLocalTime.ToString("yyyy-MM-dd HH:mm:ss.fff")
            JobID            = $matchedInfo.JobID
            State            = $matchedInfo.State
            CreatedRaw       = $matchedInfo.Created     # 文字列のまま保持（[datetime]キャストしない、Rev.4方針）
            CompletedRaw     = $matchedInfo.Completed   # 同上
            NetInFilename    = $matchedInfo.NetInFilename
            CopiesRequested  = $matchedInfo.CopiesRequested
            Note             = ""
        })

        if ($matchedInfo.State -eq "COMPLETED") {
            $finalResult = [PSCustomObject]@{
                Result = "APEOS_JOB_COMPLETED"
                Detail = "JobID $($matchedInfo.JobID) が COMPLETED で完了"
            }
            break
        } elseif ($KnownFailureStates -contains $matchedInfo.State) {
            $finalResult = [PSCustomObject]@{
                Result = "APEOS_JOB_FAILED"
                Detail = "JobID $($matchedInfo.JobID) が確認済みの異常終了State [$($matchedInfo.State)] で終了"
            }
            break
        }
        # それ以外＝途中状態とみなし、ポーリングを継続する
    } else {
        $stateLog.Add([PSCustomObject]@{
            PollCount        = $pollCount
            LocalCheckTime   = $pollLocalTime.ToString("yyyy-MM-dd HH:mm:ss.fff")
            JobID            = $null
            State            = "(NOT_FOUND)"
            CreatedRaw       = $null
            CompletedRaw     = $null
            NetInFilename    = $null
            CopiesRequested  = $null
            Note             = ""
        })
    }

    Start-Sleep -Seconds $PollingIntervalSec
}

# ---------------------------------------------------------------------------
# タイムアウト時の最終判定（ループを break せずに while が終了した場合）
# ---------------------------------------------------------------------------

if (-not $finalResult) {
    if ($foundJobSnapshot) {
        $finalResult = [PSCustomObject]@{
            Result = "APEOS_JOB_STATE_TIMEOUT"
            Detail = "JobID $($foundJobSnapshot.JobID) は発見したが、タイムアウトまでにCOMPLETEDにも確認済みの異常終了Stateにもならなかった（最終観測State: $($foundJobSnapshot.State)）"
        }
    } else {
        $finalResult = [PSCustomObject]@{
            Result = "APEOS_JOB_NOT_FOUND_TIMEOUT"
            Detail = "タイムアウトまで対象ジョブ（DocumentName: $ExpectedDocumentName）自体が見つからなかった"
        }
    }
}

$runEndLocal = Get-Date

# ---------------------------------------------------------------------------
# ログ出力
# ---------------------------------------------------------------------------

$stateLog | Export-Csv -Path $LogPath -NoTypeInformation -Encoding UTF8
Write-Host ""
Write-Host "State遷移ログを出力しました: $LogPath"

# ---------------------------------------------------------------------------
# 結果レポート
# ---------------------------------------------------------------------------

Write-Host ""
Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host " 確認結果" -ForegroundColor Cyan
Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host "Result           : $($finalResult.Result)"
Write-Host "Detail           : $($finalResult.Detail)"
Write-Host "ポーリング回数     : $pollCount"
Write-Host "実行開始（Local） : $($runStartLocal.ToString('yyyy-MM-dd HH:mm:ss.fff'))"
Write-Host "実行終了（Local） : $($runEndLocal.ToString('yyyy-MM-dd HH:mm:ss.fff'))"

if ($foundJobSnapshot) {
    Write-Host ""
    Write-Host "--- 項目①：DocumentNameの継承確認 ---" -ForegroundColor Green
    Write-Host "期待した識別子    : $ExpectedDocumentName"
    Write-Host "実際のNetInFilename: $($foundJobSnapshot.NetInFilename)"
    if ($foundJobSnapshot.NetInFilename -like "*$ExpectedDocumentName*") {
        Write-Host "→ 一致：PrintDocument.DocumentName は NetInFilename に継承されている" -ForegroundColor Green
    } else {
        Write-Host "→ 不一致：継承されていない可能性がある。要確認" -ForegroundColor Red
    }

    Write-Host ""
    Write-Host "--- 項目②：時刻比較材料（Test-PrintApeosDocumentName.ps1の出力・Apeos画面表示時刻と突き合わせること） ---" -ForegroundColor Green
    Write-Host "確認スクリプト開始時刻（Local、参考値。実際のPrint()時刻はTest-PrintApeosDocumentName.ps1側の出力を使用）: $($runStartLocal.ToString('yyyy-MM-dd HH:mm:ss'))"
    Write-Host "API Created（生値）        : $($foundJobSnapshot.Created)"
    Write-Host "API Completed（生値）      : $($foundJobSnapshot.Completed)"
    Write-Host "Apeosブラウザー画面の表示時刻: ※目視で控えた値と上記を比較してください"

    Write-Host ""
    Write-Host "--- 項目③：観測されたState遷移 ---" -ForegroundColor Green
    $stateLog | Where-Object { $_.JobID } | Select-Object PollCount, LocalCheckTime, State | Format-Table -AutoSize
} else {
    Write-Host ""
    Write-Host "対象ジョブが一度も見つかりませんでした。DocumentName／PC名／部数の条件を確認してください。" -ForegroundColor Red
}

Write-Host "=================================================================" -ForegroundColor Cyan
