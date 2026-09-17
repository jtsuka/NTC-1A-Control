#requires -Version 5.1
<#
.SYNOPSIS
  App272 morning routine interim wrapper v0.1
.DESCRIPTION
  2026-09-17時点の安全運用ラッパー。
  1) NASの当日CSVをC:\HPDBへコピー
  2) 既存整形スクリプトを実行
  3) Phase1C 新規POST DryRun
  4) DryRun DetailからExpectedNewPoを機械的に確定
  5) -ExecuteNew 指定時のみ新規POST Execute
  6) PhaseB 既存PUT DryRun
  7) Manifestから更新候補を表示して停止

  既存PUTの本番自動Executeは、Phase B全ケース検証が完了するまでこのv0.1では行わない。
#>

[CmdletBinding()]
param(
    [string]$DateTag = (Get-Date -Format 'yyyyMMdd'),
    [string]$WorkDir = 'C:\HPDB',
    [string]$LogDir = 'C:\HPDB\logs',

    [string]$SourceCsv = '',
    [string]$FormatterScript = 'C:\HPDB\Format-KonyuCSV_RFC_yyyyMMdd_v2.ps1',
    [string]$NewPostScript = 'C:\HPDB\Import-App272_Phase1C_New_v0.7.4_MasterAuto.ps1',
    [string]$ExistingPutScript = 'C:\HPDB\Import-App272_PhaseB_Update_v0.8.0.5_PROD.ps1',

    [switch]$SkipPrepare,
    [switch]$ExecuteNew,

    [ValidateRange(1,100)]
    [int]$MaxNew = 20,

    [ValidateRange(1,50)]
    [int]$MaxUpdatesPlan = 50
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

function ConvertFrom-SecureStringPlain {
    param([Security.SecureString]$Secure)
    $ptr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($Secure)
    try { return [Runtime.InteropServices.Marshal]::PtrToStringBSTR($ptr) }
    finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($ptr) }
}

function Require-File {
    param([string]$Path,[string]$Label)
    if(-not (Test-Path -LiteralPath $Path -PathType Leaf)){
        throw "SafetyStop: $Label not found: $Path"
    }
}

function Get-NewestFileAfter {
    param([string]$Directory,[string]$Filter,[datetime]$After)
    $f = Get-ChildItem -LiteralPath $Directory -Filter $Filter -File |
        Where-Object { $_.LastWriteTime -ge $After.AddSeconds(-2) } |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if($null -eq $f){ throw "SafetyStop: expected log not found. Filter=$Filter After=$After" }
    return $f.FullName
}

if([string]::IsNullOrWhiteSpace($SourceCsv)){
    $SourceCsv = "\\192.168.0.35\disk\RPA-DATA\外注課\外注課納期管理_$DateTag.csv"
}

$localRaw = Join-Path $WorkDir "外注課納期管理_$DateTag.csv"
$formatted = Join-Path $WorkDir "外注課納期管理_${DateTag}_整形済み.csv"

Require-File $NewPostScript 'Phase1C script'
Require-File $ExistingPutScript 'PhaseB PROD script'

Write-Host '================================================================='
Write-Host 'App272 Morning Routine v0.1 (interim safe wrapper)'
Write-Host '================================================================='
Write-Host ("DateTag       : {0}" -f $DateTag)
Write-Host ("Source CSV    : {0}" -f $SourceCsv)
Write-Host ("Formatted CSV : {0}" -f $formatted)
Write-Host ("ExecuteNew    : {0}" -f [bool]$ExecuteNew)
Write-Host 'Existing PUT  : DRYRUN ONLY in v0.1'
Write-Host ''

if(-not $SkipPrepare){
    Require-File $SourceCsv 'NAS source CSV'
    Require-File $FormatterScript 'formatter script'

    # Current wrapper expects the confirmed formatter contract:
    # -InputPath / -OutputPath
    $formatterText = Get-Content -LiteralPath $FormatterScript -Raw
    if($formatterText -notmatch '\bInputPath\b' -or $formatterText -notmatch '\bOutputPath\b'){
        throw "SafetyStop: formatter does not expose expected InputPath/OutputPath contract: $FormatterScript"
    }

    Write-Host '[P1] NAS -> local copy...'
    Copy-Item -LiteralPath $SourceCsv -Destination $localRaw -Force

    Write-Host '[P2] CSV formatting...'
    & $FormatterScript -InputPath $localRaw -OutputPath $formatted
    if(-not $?) { throw 'SafetyStop: formatter returned failure.' }
    Require-File $formatted 'formatted CSV'
}
else {
    Write-Host '[P1/P2] SKIP prepare. Existing formatted CSV will be used.'
    Require-File $formatted 'formatted CSV'
}

Write-Host ''
Write-Host '[AUTH] API tokens are requested once and reused inside this PowerShell process.'
$token270 = ConvertFrom-SecureStringPlain (Read-Host 'App270 READ ONLY API Token' -AsSecureString)
$token272 = ConvertFrom-SecureStringPlain (Read-Host 'App272 PROD API Token' -AsSecureString)
if([string]::IsNullOrWhiteSpace($token270) -or [string]::IsNullOrWhiteSpace($token272)){
    throw 'SafetyStop: API token is blank.'
}

# ---- Phase1C DryRun ----
Write-Host ''
Write-Host '[N1] Phase1C NEW POST DryRun...'
$newDryStart = Get-Date
& $NewPostScript `
    -CsvPath $formatted `
    -Environment PROD `
    -App272 272 `
    -App270 270 `
    -ApiToken270 $token270 `
    -ApiToken272 $token272 `
    -DryRun `
    -MaxCreates $MaxNew

if(-not $?) { throw 'SafetyStop: Phase1C DryRun failed.' }

$detailPath = Get-NewestFileAfter -Directory $LogDir `
    -Filter "Import-App272_Phase1C_DryRun_Detail_${DateTag}_*.csv" `
    -After $newDryStart

$newRows = @(
    Import-Csv -LiteralPath $detailPath |
    Where-Object { $_.App272 -eq 'New' }
)
$newPo = @($newRows | ForEach-Object { ([string]$_.発注番号).Trim() } | Where-Object { $_ } | Sort-Object -Unique)

if($newPo.Count -ne $newRows.Count){
    throw "SafetyStop: new PO set is not unique. Rows=$($newRows.Count) UniquePO=$($newPo.Count)"
}
if($newPo.Count -gt $MaxNew){
    throw "SafetyStop: New count $($newPo.Count) exceeds MaxNew=$MaxNew"
}

Write-Host ''
Write-Host ("[N2] New candidates = {0}" -f $newPo.Count)
foreach($po in $newPo){ Write-Host ("  {0}" -f $po) }

if($ExecuteNew -and $newPo.Count -gt 0){
    Write-Host ''
    Write-Host '[N3] Phase1C NEW POST Execute with exact ExpectedNewPo set...'
    & $NewPostScript `
        -CsvPath $formatted `
        -Environment PROD `
        -App272 272 `
        -App270 270 `
        -ApiToken270 $token270 `
        -ApiToken272 $token272 `
        -Execute `
        -ExpectedNewPo $newPo `
        -MaxCreates $newPo.Count `
        -ConfirmExecute 'APP272-NEW-EXECUTE'

    if(-not $?) { throw 'SafetyStop: Phase1C Execute failed.' }
}
elseif($ExecuteNew){
    Write-Host '[N3] No new records. Execute skipped.'
}
else {
    Write-Host '[N3] ExecuteNew not specified. NEW POST write skipped.'
}

# ---- PhaseB DryRun only ----
Write-Host ''
Write-Host '[U1] PhaseB existing PUT DryRun...'
$updDryStart = Get-Date
& $ExistingPutScript `
    -CsvPath $formatted `
    -App272 272 `
    -App270 270 `
    -ApiToken270 $token270 `
    -ApiToken272 $token272 `
    -DryRun `
    -MaxUpdates $MaxUpdatesPlan

if(-not $?) { throw 'SafetyStop: PhaseB DryRun failed.' }

$manifestPath = Get-NewestFileAfter -Directory $LogDir `
    -Filter "App272_PhaseB_Manifest_${DateTag}_*.json" `
    -After $updDryStart

$manifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
$updates = @($manifest.updates)
$protected = @($manifest.protected)

Write-Host ''
Write-Host '=== MORNING ROUTINE SUMMARY ==='
Write-Host ("Formatted CSV : {0}" -f $formatted)
Write-Host ("New candidates: {0}" -f $newPo.Count)
Write-Host ("New executed  : {0}" -f ([bool]$ExecuteNew))
Write-Host ("PUT candidates: {0}" -f $updates.Count)
Write-Host ("PUT protected : {0}" -f $protected.Count)
Write-Host ("PUT Manifest  : {0}" -f $manifestPath)

if($updates.Count -gt 0){
    Write-Host ''
    Write-Host 'Existing PUT candidates (PLAN ONLY):'
    foreach($u in $updates){
        Write-Host ("  PO={0} Class={1} Changed={2}" -f `
            $u.po,$u.classification,(@($u.changedFields)-join ','))
    }
}

Write-Host ''
Write-Host 'RESULT: ROUTINE v0.1 COMPLETE'
Write-Host 'NOTE  : Existing PUT is intentionally NOT executed by this wrapper yet.'
Write-Host '================================================================='
