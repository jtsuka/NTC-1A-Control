#requires -Version 5.1
<#
.SYNOPSIS
  App292 NoJOIN MasterAuto Windowsプリフライト v0.3
.DESCRIPTION
  書込みなし。C:\HPDB の必要ファイルだけ確認する。
#>

[CmdletBinding()]
param([string]$Hpdb='C:\HPDB')

Set-StrictMode -Version 2.0
$ErrorActionPreference='Stop'

$required = @(
    'Clear-App292-TestData_v0.2.ps1',
    'Prepare-App292-NoJoinMasterOptions_v0.2.ps1',
    'Import-App292-NoJoin_v0.3_MasterAuto.ps1',
    '外注課納期管理_NoJOIN想定_全937件_App292_TEST_20260921.csv',
    '加工先別_日程暫定シート_20260910.xlsx',
    'カレンダー(2023).xlsx',
    'JapanHolidays_2026_2027.csv',
    'Secrets\App292_Test.token'
)

Write-Host '================================================================='
Write-Host 'App292 NoJOIN MasterAuto Preflight v0.3'
Write-Host '================================================================='
Write-Host "PowerShellVersion : $($PSVersionTable.PSVersion)"
Write-Host "HPDB              : $Hpdb"

$missing=@()
foreach($rel in $required) {
    $p=Join-Path $Hpdb $rel
    if(Test-Path -LiteralPath $p) { Write-Host "[OK]      $p" }
    else { Write-Host "[MISSING] $p"; $missing += $p }
}
if($missing.Count -gt 0) { throw "SafetyStop: required files missing=$($missing.Count)" }

Write-Host ''
Write-Host '本ラッパーは書込みを行いません。'
Write-Host 'Claudeレビュー後の再試験では、現在の937件をClearしてからv0.3 DryRun→Executeの順です。'
Write-Host 'RESULT: PREFLIGHT PASS'
