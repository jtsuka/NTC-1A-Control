#requires -Version 5.1
<#
.SYNOPSIS
  App292 TEST 全レコード削除 v0.2
.DESCRIPTION
  Windows PowerShell 5.1用。
  App292(TEST)だけを対象に全レコードを削除する。
  既定はDryRun。App272/App270にはアクセスしない。
#>

[CmdletBinding()]
param(
    [string]$BaseUrl = 'https://bcurbkixz609.cybozu.com',
    [int]$AppId = 292,
    [string]$TokenPath = 'C:\HPDB\Secrets\App292_Test.token',
    [switch]$DryRun,
    [switch]$Execute,
    [string]$ConfirmExecute = '',
    [ValidateRange(1,5000)][int]$MaxDeletes = 2000
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

if ($AppId -ne 292) { throw "SafetyStop: AppId must be 292. Actual=$AppId" }
if ($DryRun -and $Execute) { throw 'SafetyStop: -DryRun and -Execute cannot be combined.' }
if (-not $DryRun -and -not $Execute) { $DryRun = $true }
if ($Execute -and $ConfirmExecute -ne 'APP292-CLEAR-ALL') {
    throw "SafetyStop: Execute requires -ConfirmExecute 'APP292-CLEAR-ALL'"
}

function Get-PlainToken {
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "SafetyStop: token file not found: $Path"
    }

    $enc = (Get-Content -LiteralPath $Path -Raw).Trim()
    if ([string]::IsNullOrWhiteSpace($enc)) { throw 'SafetyStop: encrypted token file is blank.' }

    $sec = $enc | ConvertTo-SecureString
    $ptr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($sec)
    try { return [Runtime.InteropServices.Marshal]::PtrToStringBSTR($ptr) }
    finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($ptr) }
}

function Get-KValue {
    param($Record, [string]$Field)
    $p = $Record.PSObject.Properties[$Field]
    if ($null -eq $p -or $null -eq $p.Value) { return '' }
    $v = $p.Value.PSObject.Properties['value']
    if ($null -eq $v -or $null -eq $v.Value) { return '' }
    return ([string]$v.Value).Trim()
}

function Get-AllRecords {
    param([string]$Token)

    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $all = New-Object System.Collections.Generic.List[object]
    $lastId = 0

    while ($true) {
        $query = '$id > {0} order by $id asc limit 500' -f $lastId
        $uri = '{0}/k/v1/records.json?app={1}&query={2}' -f `
            $BaseUrl.TrimEnd('/'), $AppId, [uri]::EscapeDataString($query)

        $resp = Invoke-RestMethod -Method Get -Uri $uri -Headers $headers

        foreach ($r in $resp.records) { $all.Add($r) }
        if ($resp.records.Count -lt 500) { break }

        $lastId = [int64](Get-KValue $resp.records[$resp.records.Count - 1] '$id')
    }

    return $all.ToArray()
}

function Remove-RecordBatch {
    param([string]$Token, [string[]]$Ids)

    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $body = [ordered]@{
        app = $AppId
        ids = @($Ids | ForEach-Object { [int64]$_ })
    } | ConvertTo-Json -Depth 5

    $uri = $BaseUrl.TrimEnd('/') + '/k/v1/records.json'

    Invoke-RestMethod -Method Delete -Uri $uri -Headers $headers `
        -ContentType 'application/json; charset=utf-8' `
        -Body ([Text.Encoding]::UTF8.GetBytes($body)) | Out-Null
}

$token = Get-PlainToken -Path $TokenPath
$before = @(Get-AllRecords -Token $token)
$count = $before.Count

Write-Host '================================================================='
Write-Host 'Clear App292 TEST Data v0.2'
Write-Host '================================================================='
Write-Host "AppId        : $AppId (TEST fixed)"
Write-Host "CurrentCount : $count"
Write-Host ("Mode         : {0}" -f $(if ($Execute) { 'EXECUTE' } else { 'DRYRUN' }))
Write-Host 'App270/App272: NOT USED'

if ($count -gt $MaxDeletes) {
    throw "SafetyStop: CurrentCount=$count exceeds MaxDeletes=$MaxDeletes"
}

if (-not $Execute) {
    Write-Host 'WRITE        : NONE'
    Write-Host 'RESULT       : DRYRUN PASS'
    exit 0
}

if ($count -eq 0) {
    Write-Host 'Already empty.'
    Write-Host 'RESULT       : PASS'
    exit 0
}

$ids = @($before | ForEach-Object { Get-KValue $_ '$id' })

for ($i = 0; $i -lt $ids.Count; $i += 100) {
    $end = [Math]::Min($i + 99, $ids.Count - 1)
    $batch = @($ids[$i..$end])

    Write-Host ("DELETE {0}-{1} / {2}" -f ($i + 1), ($end + 1), $ids.Count)
    Remove-RecordBatch -Token $token -Ids $batch
}

$after = @(Get-AllRecords -Token $token)
if ($after.Count -ne 0) {
    throw "PostVerifyFailed: App292 count after delete=$($after.Count)"
}

Write-Host 'PostVerify   : 0 records'
Write-Host 'RESULT       : PASS'
