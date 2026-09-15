<#
.SYNOPSIS
    船舶値付用紙 - Apeos C3570 ジョブ履歴 遡及取得スクリプト（監査・障害調査用）

.DESCRIPTION
    仕様書Rev.5で確定したページング仕様（offsetJobID / offsetJobIDType による
    カーソル方式）を使い、job-list APIを複数回呼び出して過去のジョブ履歴を
    まとめて取得する。

    Check-ApeosPrintJob.ps1（印刷直後の即時確認用）とは目的が異なり、
    こちらは「後から特定期間のジョブを洗い出す」監査・障害調査用のツール。
    現時点では Print-ValueTag.ps1 本体・RK-10のフローには組み込まれていない。

.PARAMETER ApeosIP
    Apeos複合機のIPアドレス。既定値は船舶機械部のC3570。

.PARAMETER MaxPages
    取得するページ数の上限（1ページ＝最大25件）。既定値10（＝最大250件相当）。
    無制限にすると複合機側の負荷やジョブ履歴の保持上限との兼ね合いが読めないため、
    上限を必ず設けること。

.PARAMETER TypeFilter
    ジョブ種別フィルタ。既定値 "PRINT"。
    注意：typeFilter=PRINT を指定した状態でのページング動作（offsetJobIDの
    挙動）は、仕様書Rev.5時点ではtypeFilter=ALLでのみ実機確認済みで、
    PRINT指定時は未検証（8章参照）。本番の監査用途で使う前に、
    -TypeFilter ALL で取得してPowerShell側で UserJobType -eq "PRINT" を
    フィルタする方式と結果が一致するか、一度突き合わせておくことを推奨する。

.PARAMETER SinceLocal
    この日時（ローカル）以降のジョブに絞り込む。省略時は絞り込みなし。
    ※ API側のCreated/Completedはタイムゾーン仕様が未確定（仕様書3.7参照）のため、
    このフィルタは「生値の文字列比較」ではなく、取得した全件を対象に
    参考情報として表示するのみで、確定的な絞り込みには使わないこと。

.PARAMETER OutCsvPath
    結果を出力するCSVパス。省略時はカレントディレクトリに
    ApeosJobHistory_<実行時刻>.csv として出力する。

.EXAMPLE
    .\Get-ApeosJobHistory.ps1 -MaxPages 5

.EXAMPLE
    .\Get-ApeosJobHistory.ps1 -MaxPages 20 -TypeFilter ALL -OutCsvPath "C:\temp\history.csv"

.NOTES
    このスクリプトはCookie等の認証なしでAPIへアクセスする（仕様書3.1参照）。
    社内LAN内での利用に留め、取得したジョブ履歴（文書名・PC名を含む）の
    取り扱いには注意すること。
#>

[CmdletBinding()]
param(
    [string]$ApeosIP = "192.168.0.209",

    [int]$MaxPages = 10,

    [string]$TypeFilter = "PRINT",

    [Nullable[datetime]]$SinceLocal = $null,

    [string]$OutCsvPath
)

$runStart = Get-Date
if (-not $OutCsvPath) {
    $stamp = $runStart.ToString("yyyyMMdd_HHmmss")
    $OutCsvPath = Join-Path -Path (Get-Location) -ChildPath "ApeosJobHistory_$stamp.csv"
}

Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host " Apeos ジョブ履歴 遡及取得（監査・障害調査用）" -ForegroundColor Cyan
Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host "対象複合機   : $ApeosIP"
Write-Host "取得ページ上限: $MaxPages（最大 $($MaxPages * 25) 件相当）"
Write-Host "typeFilter   : $TypeFilter"
if ($TypeFilter -ne "ALL") {
    Write-Warning "typeFilter=$TypeFilter でのページング動作は仕様書Rev.5時点で未検証です（8章参照）。結果に抜けがないか、必要に応じて -TypeFilter ALL でも突き合わせてください。"
}
Write-Host ""

$allJobs = New-Object System.Collections.Generic.List[object]
$offsetJobID = $null
$offsetJobIDType = $null
$pagesFetched = 0

for ($i = 0; $i -lt $MaxPages; $i++) {
    $uri = "http://$ApeosIP/jobs/api/job-list?methodName=GET&limit=25&typeFilter=$TypeFilter"
    if ($offsetJobID) {
        $uri += "&offsetJobID=$offsetJobID&offsetJobIDType=$offsetJobIDType"
    }

    Write-Host "[ページ $($i + 1)] 取得中... offsetJobID=$offsetJobID"

    try {
        $result = Invoke-RestMethod -Uri $uri -Method Post -ContentType "application/json" -TimeoutSec 15
    } catch {
        Write-Warning "ページ $($i + 1) の取得に失敗しました: $($_.Exception.Message)"
        break
    }

    $pagesFetched++

    if (-not $result.Jobs -or $result.Jobs.Count -eq 0) {
        Write-Host "  → ジョブなし。終了します。"
        break
    }

    foreach ($job in $result.Jobs) {
        $info = $job.JobInfo
        $allJobs.Add([PSCustomObject]@{
            JobID           = $info.JobID
            UserJobType     = $info.UserJobType
            UserName        = $info.UserName
            State           = $info.State
            CreatedRaw      = $info.Created      # 文字列のまま保持（[datetime]キャストしない、仕様書3.7方針）
            CompletedRaw    = $info.Completed    # 同上
            NetInFilename   = $info.NetInFilename
            CopiesRequested = $info.CopiesRequested
            DocumentNumber  = $info.DocumentNumber
        })
    }

    Write-Host "  → $($result.Jobs.Count) 件取得（累計 $($allJobs.Count) 件）"

    if (-not $result.Next) {
        Write-Host "  → Next=false のため、これ以上古いジョブは存在しません。"
        break
    }

    $lastInfo = $result.Jobs[-1].JobInfo
    $offsetJobID = $lastInfo.JobID
    $offsetJobIDType = $lastInfo.State
}

# ---------------------------------------------------------------------------
# 出力
# ---------------------------------------------------------------------------

$allJobs | Export-Csv -Path $OutCsvPath -NoTypeInformation -Encoding UTF8

Write-Host ""
Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host " 完了" -ForegroundColor Cyan
Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host "取得ページ数: $pagesFetched / $MaxPages"
Write-Host "取得ジョブ件数: $($allJobs.Count)"
Write-Host "出力先CSV: $OutCsvPath"

if ($SinceLocal) {
    Write-Host ""
    Write-Host "※ -SinceLocal が指定されていますが、Created/Completedのタイムゾーン仕様が" -ForegroundColor Yellow
    Write-Host "  まだ確定していないため（仕様書3.7参照）、このスクリプトでは自動フィルタを" -ForegroundColor Yellow
    Write-Host "  行っていません。出力されたCSVのCreatedRaw列を目視で確認してください。" -ForegroundColor Yellow
}

if ($allJobs.Count -gt 0) {
    Write-Host ""
    Write-Host "--- 取得結果（先頭10件） ---"
    $allJobs | Select-Object -First 10 | Format-Table JobID, UserName, State, CreatedRaw, NetInFilename -AutoSize
}
