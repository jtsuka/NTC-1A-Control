#requires -Version 5.1
<#
.SYNOPSIS
  App272 Morning Routine v1.1.0 NoLookup

.DESCRIPTION
  2026-09-17時点の本番早朝処理用ラッパー。
  CSV整形の受け渡し領域を C:\HPDB\TMP に固定し、当日の日付付きファイルで連携する。

  正常終了時:
    - TMP内の当日Raw/整形済みCSVを削除（-KeepTemp 指定時を除く）
  異常終了時:
    - TMPファイルを削除せず証拠として残す

  Formatter正式I/F:
    -InputPath <raw csv>
    -OutputPath <formatted csv>

  Executeモードでは次の順で処理する:
    1. Raw CSVをNASからTMPへコピー
    2. FormatterでTMPへ整形済みCSV出力
    3. Phase1C DryRun
    4. PhaseB DryRun（事前監査）
    5. 安全条件確認
    6. Phase1C Execute（新規がある場合のみ）
    7. PhaseB DryRunを再実行（POST後の最新状態で再計画）
    8. PhaseB Execute（既存更新がある場合のみ）
    9. Phase1C最終DryRun = 新規0件を確認
   10. NoLookup最終DryRun = Updates=0 / Errors=0 / Warnings=0 / BlockedNew=0を確認
   11. 正常時のみTMP削除

  注意:
    - App270はREAD ONLY。
    - App272 PRODのみ更新。
    - App292 TEST tokenは使用しない。
    - 1回のPhaseB Executeは最大50件。50件超はSafetyStop。
    - NoLookup版では既存Manual/Batch保護は正常状態として扱い、PUT候補・Errors・Warnings・BlockedNewを安全判定する。
#>

[CmdletBinding()]
param(
    [ValidateSet('DryRun','Execute')]
    [string]$Mode = 'DryRun',

    [string]$ConfirmExecute = '',

    [string]$DateTag = (Get-Date -Format 'yyyyMMdd'),
    [string]$WorkDir = 'C:\HPDB',
    [string]$TmpDir = 'C:\HPDB\TMP',
    [string]$LogDir = 'C:\HPDB\logs',
    [string]$SecretDir = 'C:\HPDB\Secrets',

    [string]$SourceCsv = '',
    [string]$FormatterScript = 'C:\HPDB\Format-KonyuCSV_RFC_yyyyMMdd_v3.1.ps1',
    [string]$NewPostScript = 'C:\HPDB\Import-App272_Phase1C_New_v0.7.4_MasterAuto.ps1',
    [string]$ExistingPutScript = 'C:\HPDB\Import-App272_Execute_v0.2.2_NoLookup_DPAPI.ps1',

    [ValidateRange(1,100)]
    [int]$MaxNew = 20,

    [ValidateRange(1,50)]
    [int]$MaxUpdates = 50,

    [switch]$ReusePrepared,
    [switch]$KeepTemp,
    [switch]$ForcePromptToken
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

$script:RoutineSucceeded = $false
$script:ExitCode = 1
$script:RoutineError = $null

function Stop-Safety {
    param([string]$Message)
    throw "SafetyStop: $Message"
}

function Require-File {
    param([string]$Path,[string]$Label)
    if(-not (Test-Path -LiteralPath $Path -PathType Leaf)){
        Stop-Safety "$Label not found: $Path"
    }
}

function Ensure-Directory {
    param([string]$Path)
    if(-not (Test-Path -LiteralPath $Path -PathType Container)){
        New-Item -ItemType Directory -Path $Path -Force | Out-Null
    }
}

function ConvertFrom-SecureStringPlain {
    param([Security.SecureString]$Secure)
    $ptr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($Secure)
    try { return [Runtime.InteropServices.Marshal]::PtrToStringBSTR($ptr) }
    finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($ptr) }
}

function Get-NewestFileAfter {
    param(
        [string]$Directory,
        [string]$Filter,
        [datetime]$After
    )
    $f = Get-ChildItem -LiteralPath $Directory -Filter $Filter -File |
        Where-Object { $_.LastWriteTime -ge $After.AddSeconds(-2) } |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if($null -eq $f){
        Stop-Safety "expected log not found. Filter=$Filter After=$After"
    }
    return $f.FullName
}

function Assert-PathUnderTmp {
    param([string]$Path,[string]$TmpRoot)
    $fullPath = [IO.Path]::GetFullPath($Path)
    $fullRoot = [IO.Path]::GetFullPath($TmpRoot).TrimEnd('\') + '\'
    if(-not $fullPath.StartsWith($fullRoot,[StringComparison]::OrdinalIgnoreCase)){
        Stop-Safety "cleanup target is outside TMP. Path=$fullPath TmpRoot=$fullRoot"
    }
}

function Remove-TempFileSafe {
    param([string]$Path,[string]$TmpRoot)
    if(Test-Path -LiteralPath $Path -PathType Leaf){
        Assert-PathUnderTmp -Path $Path -TmpRoot $TmpRoot
        Remove-Item -LiteralPath $Path -Force
        Write-Host ("  removed: {0}" -f $Path)
    }
}

function Get-Phase1CPlan {
    param(
        [string]$CsvPath,
        [string]$ScriptPath,
        [string]$Token270,
        [string]$Token272,
        [int]$MaxCreates,
        [string]$DateTag,
        [string]$LogDir
    )

    $start = Get-Date
    & $ScriptPath `
        -CsvPath $CsvPath `
        -Environment PROD `
        -App272 272 `
        -App270 270 `
        -ApiToken270 $Token270 `
        -ApiToken272 $Token272 `
        -DryRun `
        -MaxCreates $MaxCreates

    if(-not $?){ Stop-Safety 'Phase1C DryRun failed.' }

    $detailPath = Get-NewestFileAfter -Directory $LogDir `
        -Filter "Import-App272_Phase1C_DryRun_Detail_${DateTag}_*.csv" `
        -After $start

    $newRows = @(
        Import-Csv -LiteralPath $detailPath |
        Where-Object { $_.App272 -eq 'New' }
    )
    $newPo = @(
        $newRows |
        ForEach-Object { ([string]$_.発注番号).Trim() } |
        Where-Object { $_ } |
        Sort-Object -Unique
    )

    if($newPo.Count -ne $newRows.Count){
        Stop-Safety "Phase1C new PO set is not unique. Rows=$($newRows.Count) UniquePO=$($newPo.Count)"
    }
    if($newPo.Count -gt $MaxCreates){
        Stop-Safety "Phase1C new count=$($newPo.Count) exceeds MaxNew=$MaxCreates"
    }

    return [pscustomobject]@{
        DetailPath = $detailPath
        NewPo = $newPo
        NewCount = $newPo.Count
    }
}

function Get-PhaseBPlan {
    param(
        [string]$CsvPath,
        [string]$ScriptPath,
        [string]$Token270,
        [string]$Token272,
        [int]$MaxPlan,
        [string]$DateTag,
        [string]$LogDir
    )

    $start = Get-Date

    & $ScriptPath `
        -CsvPath $CsvPath `
        -App272 272 `
        -App270 270 `
        -ApiToken270 $Token270 `
        -ApiToken272 $Token272 `
        -DryRun `
        -MaxWrites $MaxPlan

    if(-not $?){ Stop-Safety 'NoLookup existing-update DryRun failed.' }

    $summaryPath = Get-NewestFileAfter -Directory $LogDir `
        -Filter "Import-App272_DryRun_Summary_${DateTag}_*.txt" `
        -After $start

    $detailPath = Get-NewestFileAfter -Directory $LogDir `
        -Filter "Import-App272_DryRun_Detail_${DateTag}_*.csv" `
        -After $start

    $summaryText = Get-Content -LiteralPath $summaryPath -Raw

    function Get-SummaryInt {
        param([string]$Label)
        $m = [regex]::Match(
            $summaryText,
            ('(?m)^\s*' + [regex]::Escape($Label) + '\s*:\s*(\d+)\s*$')
        )
        if(-not $m.Success){
            Stop-Safety "NoLookup summary field not found: $Label ($summaryPath)"
        }
        return [int]$m.Groups[1].Value
    }

    $planned = Get-SummaryInt 'Planned PUT writes'
    $errors = Get-SummaryInt 'Record errors'
    $warnings = Get-SummaryInt 'Warnings'
    $blockedNew = Get-SummaryInt 'Blocked New App272'

    $rows = @(Import-Csv -LiteralPath $detailPath)
    $updates = @($rows | Where-Object { ([string]$_.WRITE) -like 'PUT:*' })
    $updatePo = @(
        $updates |
        ForEach-Object { ([string]$_.発注番号).Trim() } |
        Where-Object { $_ } |
        Sort-Object -Unique
    )

    if($planned -ne $updates.Count){
        Stop-Safety "NoLookup plan mismatch. Summary=$planned Detail=$($updates.Count)"
    }
    if($updatePo.Count -ne $updates.Count){
        Stop-Safety "NoLookup update PO set is not unique. Rows=$($updates.Count) UniquePO=$($updatePo.Count)"
    }
    if($errors -ne 0 -or $warnings -ne 0){
        Stop-Safety "NoLookup preflight has errors/warnings. Errors=$errors Warnings=$warnings"
    }
    if($blockedNew -ne 0){
        Stop-Safety "NoLookup sees Blocked New App272=$blockedNew. Phase1C/new-record flow must resolve these before existing PUT."
    }

    return [pscustomobject]@{
        SummaryPath = $summaryPath
        DetailPath = $detailPath
        Updates = $updates
        UpdatePo = $updatePo
        UpdateCount = $updates.Count
        ErrorCount = $errors
        WarningCount = $warnings
        BlockedNewCount = $blockedNew
    }
}

try {
    if($Mode -eq 'Execute' -and $ConfirmExecute -ne 'APP272-MORNING-EXECUTE'){
        Stop-Safety "Execute requires -ConfirmExecute 'APP272-MORNING-EXECUTE'"
    }

    Ensure-Directory $WorkDir
    Ensure-Directory $TmpDir
    Ensure-Directory $LogDir
    Ensure-Directory $SecretDir

    if([string]::IsNullOrWhiteSpace($SourceCsv)){
        $SourceCsv = "\\192.168.0.35\disk\RPA-DATA\外注課\外注課納期管理_$DateTag.csv"
    }

    $tmpRaw = Join-Path $TmpDir "外注課納期管理_${DateTag}_RAW.csv"
    $formatted = Join-Path $TmpDir "外注課納期管理_${DateTag}_整形済み.csv"

    Require-File $NewPostScript 'Phase1C script'
    Require-File $ExistingPutScript 'PhaseB PROD script'

    Write-Host '================================================================='
    Write-Host 'App272 Morning Routine v1.1.0 NoLookup'
    Write-Host '================================================================='
    Write-Host ("Mode           : {0}" -f $Mode)
    Write-Host ("DateTag        : {0}" -f $DateTag)
    Write-Host ("Source CSV     : {0}" -f $SourceCsv)
    Write-Host ("TMP Raw        : {0}" -f $tmpRaw)
    Write-Host ("TMP Formatted  : {0}" -f $formatted)
    Write-Host ("Phase1C        : {0}" -f $NewPostScript)
    Write-Host ("PhaseB         : {0}" -f $ExistingPutScript)
    Write-Host ("KeepTemp       : {0}" -f [bool]$KeepTemp)
    Write-Host ''

    if(-not $ReusePrepared){
        Require-File $SourceCsv 'NAS source CSV'
        Require-File $FormatterScript 'formatter script'

        if((Test-Path -LiteralPath $tmpRaw) -or (Test-Path -LiteralPath $formatted)){
            Stop-Safety "same-date TMP file already exists. Evidence is preserved. Use -ReusePrepared only after confirming the files are the intended failed-run artifacts. Raw=$tmpRaw Formatted=$formatted"
        }

        # Formatter v3.1 is a reviewed/frozen artifact.
        # Windows PowerShell 5.1 may return $null for ExternalScriptInfo.Parameters
        # for this CP932 script, so do not use Get-Command.Parameters for the I/F guard.
        # Pin the approved formatter by SHA256 instead.
        $expectedFormatterSha256 = 'a731fc263c538fccfb3942edb584c80ad8ee5ac10a5079a82253c632d5b5b9fd'
        $actualFormatterSha256 = (Get-FileHash -LiteralPath $FormatterScript -Algorithm SHA256).Hash.ToLowerInvariant()
        if($actualFormatterSha256 -ne $expectedFormatterSha256){
            Stop-Safety "formatter SHA256 mismatch. Expected=$expectedFormatterSha256 Actual=$actualFormatterSha256 Script=$FormatterScript"
        }

        Write-Host '[P1] NAS -> TMP raw copy...'
        Copy-Item -LiteralPath $SourceCsv -Destination $tmpRaw
        Require-File $tmpRaw 'TMP raw CSV'

        Write-Host '[P2] CSV formatting -> TMP...'
        & $FormatterScript -InputPath $tmpRaw -OutputPath $formatted
        if(-not $?){ Stop-Safety 'formatter returned failure.' }
        Require-File $formatted 'formatted CSV'
    }
    else {
        Write-Host '[P1/P2] ReusePrepared specified. No NAS copy / formatter execution.'
        Require-File $tmpRaw 'existing TMP raw CSV'
        Require-File $formatted 'existing formatted CSV'
    }

    Write-Host ''
    Write-Host '[AUTH] DPAPI API token loading...'
    $token270Path = Join-Path $SecretDir 'App270_ReadOnly.token'
    $token272Path = Join-Path $SecretDir 'App272_Prod.token'

    $useStored = (-not $ForcePromptToken) -and
                 (Test-Path -LiteralPath $token270Path -PathType Leaf) -and
                 (Test-Path -LiteralPath $token272Path -PathType Leaf)

    if($useStored){
        try {
            $enc270 = (Get-Content -LiteralPath $token270Path -Raw).Trim()
            $enc272 = (Get-Content -LiteralPath $token272Path -Raw).Trim()
            if([string]::IsNullOrWhiteSpace($enc270) -or [string]::IsNullOrWhiteSpace($enc272)){
                Stop-Safety 'encrypted token file is empty.'
            }

            $token270 = ConvertFrom-SecureStringPlain ($enc270 | ConvertTo-SecureString)
            $token272 = ConvertFrom-SecureStringPlain ($enc272 | ConvertTo-SecureString)

            Write-Host ("  App270 READ ONLY: DPAPI stored token ({0})" -f $token270Path)
            Write-Host ("  App272 PROD     : DPAPI stored token ({0})" -f $token272Path)
        }
        catch {
            Stop-Safety "DPAPI token load failed. Same Windows user/PC? Detail=$($_.Exception.Message)"
        }
    }
    else {
        Write-Host '  Stored token not used. Prompting interactively.'
        $token270 = ConvertFrom-SecureStringPlain (Read-Host 'App270 READ ONLY API Token' -AsSecureString)
        $token272 = ConvertFrom-SecureStringPlain (Read-Host 'App272 PROD API Token' -AsSecureString)
    }

    if([string]::IsNullOrWhiteSpace($token270) -or [string]::IsNullOrWhiteSpace($token272)){
        Stop-Safety 'API token is blank.'
    }

    # ------------------------------------------------------------
    # PRE-FLIGHT
    # ------------------------------------------------------------
    Write-Host ''
    Write-Host '[F1] Phase1C PRE-FLIGHT DryRun...'
    $newPlan = Get-Phase1CPlan `
        -CsvPath $formatted `
        -ScriptPath $NewPostScript `
        -Token270 $token270 `
        -Token272 $token272 `
        -MaxCreates $MaxNew `
        -DateTag $DateTag `
        -LogDir $LogDir

    Write-Host ''
    Write-Host '[F2] NoLookup existing-update PRE-FLIGHT DryRun...'
    $putPlan = Get-PhaseBPlan `
        -CsvPath $formatted `
        -ScriptPath $ExistingPutScript `
        -Token270 $token270 `
        -Token272 $token272 `
        -MaxPlan $MaxUpdates `
        -DateTag $DateTag `
        -LogDir $LogDir

    Write-Host ''
    Write-Host '=== PRE-FLIGHT SUMMARY ==='
    Write-Host ("NEW candidates   : {0}" -f $newPlan.NewCount)
    Write-Host ("PUT candidates   : {0}" -f $putPlan.UpdateCount)
    Write-Host ("Phase1C Detail   : {0}" -f $newPlan.DetailPath)
    Write-Host ("NoLookup Summary : {0}" -f $putPlan.SummaryPath)
    Write-Host ("NoLookup Detail  : {0}" -f $putPlan.DetailPath)
    if($putPlan.UpdateCount -gt $MaxUpdates){
        Stop-Safety "NoLookup Updates=$($putPlan.UpdateCount) exceeds MaxUpdates=$MaxUpdates. Manual review/split required."
    }

    if($Mode -eq 'DryRun'){
        Write-Host ''
        Write-Host 'WRITE  : NONE'
        Write-Host 'RESULT : MORNING ROUTINE DRYRUN PASS'
        $script:RoutineSucceeded = $true
        $script:ExitCode = 0
    }
    else {

    # ------------------------------------------------------------
    # EXECUTE NEW POST
    # ------------------------------------------------------------
    if($newPlan.NewCount -gt 0){
        Write-Host ''
        Write-Host ("[E1] Phase1C Execute NEW={0}..." -f $newPlan.NewCount)
        & $NewPostScript `
            -CsvPath $formatted `
            -Environment PROD `
            -App272 272 `
            -App270 270 `
            -ApiToken270 $token270 `
            -ApiToken272 $token272 `
            -Execute `
            -ExpectedNewPo $newPlan.NewPo `
            -MaxCreates $newPlan.NewCount `
            -ConfirmExecute 'APP272-NEW-EXECUTE'

        if(-not $?){ Stop-Safety 'Phase1C Execute failed.' }
    }
    else {
        Write-Host ''
        Write-Host '[E1] Phase1C Execute skipped: NEW=0'
    }

    # ------------------------------------------------------------
    # RE-PLAN PUT AFTER POST
    # ------------------------------------------------------------
    Write-Host ''
    Write-Host '[E2] NoLookup existing-update re-DryRun after Phase1C Execute...'
    $putPlan2 = Get-PhaseBPlan `
        -CsvPath $formatted `
        -ScriptPath $ExistingPutScript `
        -Token270 $token270 `
        -Token272 $token272 `
        -MaxPlan $MaxUpdates `
        -DateTag $DateTag `
        -LogDir $LogDir

    Write-Host ("  PUT candidates: {0}" -f $putPlan2.UpdateCount)
    if($putPlan2.UpdateCount -gt $MaxUpdates){
        Stop-Safety "PhaseB re-plan Updates=$($putPlan2.UpdateCount) exceeds MaxUpdates=$MaxUpdates."
    }

    if($putPlan2.UpdateCount -gt 0){
        Write-Host ''
        Write-Host ("[E3] NoLookup existing-update Execute PUT={0}..." -f $putPlan2.UpdateCount)
        & $ExistingPutScript `
            -CsvPath $formatted `
            -App272 272 `
            -App270 270 `
            -ApiToken270 $token270 `
            -ApiToken272 $token272 `
            -Execute `
            -MaxWrites $putPlan2.UpdateCount `
            -ConfirmExecute 'APP272-EXECUTE'

        if(-not $?){ Stop-Safety 'PhaseB Execute failed.' }
    }
    else {
        Write-Host ''
        Write-Host '[E3] NoLookup existing-update Execute skipped: Updates=0'
    }

    # ------------------------------------------------------------
    # FINAL VERIFY
    # ------------------------------------------------------------
    Write-Host ''
    Write-Host '[V1] Final Phase1C DryRun...'
    $finalNew = Get-Phase1CPlan `
        -CsvPath $formatted `
        -ScriptPath $NewPostScript `
        -Token270 $token270 `
        -Token272 $token272 `
        -MaxCreates $MaxNew `
        -DateTag $DateTag `
        -LogDir $LogDir

    if($finalNew.NewCount -ne 0){
        Stop-Safety "Final verification failed: NEW remains $($finalNew.NewCount)"
    }

    Write-Host ''
    Write-Host '[V2] Final NoLookup existing-update DryRun...'
    $finalPut = Get-PhaseBPlan `
        -CsvPath $formatted `
        -ScriptPath $ExistingPutScript `
        -Token270 $token270 `
        -Token272 $token272 `
        -MaxPlan $MaxUpdates `
        -DateTag $DateTag `
        -LogDir $LogDir

    if($finalPut.UpdateCount -ne 0 -or $finalPut.ErrorCount -ne 0 -or $finalPut.WarningCount -ne 0 -or $finalPut.BlockedNewCount -ne 0){
        Stop-Safety "Final verification failed: Updates=$($finalPut.UpdateCount) Errors=$($finalPut.ErrorCount) Warnings=$($finalPut.WarningCount) BlockedNew=$($finalPut.BlockedNewCount)"
    }

    Write-Host ''
    Write-Host '================================================================='
    Write-Host 'FINAL RESULT'
    Write-Host '  Phase1C NEW     : 0 remaining'
    Write-Host '  NoLookup Updates : 0 remaining'
    Write-Host '  NoLookup Errors   : 0'
    Write-Host '  NoLookup Warnings : 0'
    Write-Host '  RESULT          : EXECUTE PASS / FINAL VERIFY PASS'
    Write-Host '================================================================='

    $script:RoutineSucceeded = $true
    $script:ExitCode = 0
    }
}
catch {
    $script:RoutineError = $_
    $script:RoutineSucceeded = $false
    $script:ExitCode = 1
    Write-Error ("MORNING ROUTINE FAILED: {0}" -f $_.Exception.Message)
}
finally {
    Write-Host ''
    if($script:RoutineSucceeded -and -not $KeepTemp){
        Write-Host '[CLEANUP] Successful routine. Removing TMP handoff files...'
        try {
            if($null -ne $tmpRaw){ Remove-TempFileSafe -Path $tmpRaw -TmpRoot $TmpDir }
            if($null -ne $formatted){ Remove-TempFileSafe -Path $formatted -TmpRoot $TmpDir }
            Write-Host '[CLEANUP] PASS'
        }
        catch {
            $script:ExitCode = 1
            Write-Warning ("TMP cleanup failed. Business processing succeeded, but routine exit code is set to 1. Detail={0}" -f $_.Exception.Message)
        }
    }
    elseif($KeepTemp){
        Write-Host '[CLEANUP] KeepTemp specified. TMP files retained.'
    }
    else {
        Write-Host '[CLEANUP] Routine did not complete successfully. TMP files retained for investigation.'
    }
}

Write-Host ("EXIT CODE: {0}" -f $script:ExitCode)
exit $script:ExitCode
