#requires -Version 5.1
<#
.SYNOPSIS
  App292 NoJOIN Windowsテスト補助ランナー v0.2
.DESCRIPTION
  C:\HPDB 配下で実行するための案内兼プリフライト。
  書込みは行わない。必要ファイルの存在とPowerShell環境だけ確認する。
#>

[CmdletBinding()]
param(
    [string]$Hpdb = 'C:\HPDB'
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

$required = @(
    'Clear-App292-TestData_v0.2.ps1',
    'Prepare-App292-NoJoinMasterOptions_v0.2.ps1',
    'Import-App292-NoJoin_v0.2.ps1',
    '外注課納期管理_NoJOIN想定_全937件_App292_TEST_20260921.csv',
    'Secrets\App292_Test.token'
)

Write-Host '================================================================='
Write-Host 'App292 NoJOIN Windows Preflight v0.2'
Write-Host '================================================================='
Write-Host "PowerShellVersion : $($PSVersionTable.PSVersion)"
Write-Host "HPDB              : $Hpdb"

if ($PSVersionTable.PSVersion.Major -ne 5) {
    Write-Warning 'Windows PowerShell 5.1を前提にしています。'
}

$missing = @()

foreach ($rel in $required) {
    $path = Join-Path $Hpdb $rel

    if (Test-Path -LiteralPath $path) {
        Write-Host "[OK]      $path"
    } else {
        Write-Host "[MISSING] $path"
        $missing += $path
    }
}

if ($missing.Count -gt 0) {
    throw "SafetyStop: required files missing=$($missing.Count)"
}

Write-Host ''
Write-Host '次の順序で実行してください。'
Write-Host ''
Write-Host '1) マスタoption確認（書込みなし）'
Write-Host '   .\Prepare-App292-NoJoinMasterOptions_v0.2.ps1 -DryRun'
Write-Host ''
Write-Host '2) App292削除確認（書込みなし）'
Write-Host '   .\Clear-App292-TestData_v0.2.ps1 -DryRun'
Write-Host ''
Write-Host '3) NoJOIN 937件取込確認（書込みなし）'
Write-Host '   .\Import-App292-NoJoin_v0.2.ps1 -DryRun'
Write-Host ''
Write-Host 'Claude再レビュー後にExecuteへ進んでください。'
Write-Host 'RESULT: PREFLIGHT PASS'
