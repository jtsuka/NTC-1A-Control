#requires -Version 5.1
<#
.SYNOPSIS
  App292 U1前処理: 既存レコードをBatch日程の基準状態へ整える v0.1

.DESCRIPTION
  U1（既存Batchの納期変更 -> 基幹納期更新 + K日程再計算）のため、
  App292の指定POについて、U1を「納期だけの差分」に限定できるよう、
  Vendor/StaffとK日程を既知の基準値へ整える。

  既定はREAD ONLY。
  -Execute と -ConfirmExecute 'APP292-U1-BASELINE' が揃った時だけApp292へPUTする。

  発注番号/ODERNO/手配日/納期は変更しない。
#>

[CmdletBinding()]
param(
    [string]$BaseUrl = 'https://bcurbkixz609.cybozu.com',
    [int]$App292 = 292,
    [string]$ApiToken292 = '',
    [string]$Po = '2609249755',
    [string]$ExpectedOrderDate = '2026-09-15',
    [string]$ExpectedDueDate = '2026-11-02',
    [string]$ExpectedVendor = '㈱吉川鉄工所',
    [string]$ExpectedStaff = '木村　和也',
    [string]$BaselineStart = '2026-10-26',
    [string]$BaselineInspection = '2026-10-30',
    [switch]$Execute,
    [string]$ConfirmExecute = ''
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

function ConvertFrom-SecureStringPlain {
    param([Security.SecureString]$Secure)
    $bstr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($Secure)
    try { [Runtime.InteropServices.Marshal]::PtrToStringBSTR($bstr) }
    finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($bstr) }
}

function Get-Token {
    if (-not [string]::IsNullOrWhiteSpace($ApiToken292)) { return $ApiToken292 }
    return ConvertFrom-SecureStringPlain (Read-Host "App292 API Token" -AsSecureString)
}

function Get-KValue {
    param($Record,[string]$FieldCode)
    $p = $Record.PSObject.Properties[$FieldCode]
    if ($null -eq $p -or $null -eq $p.Value) { return '' }
    $v = $p.Value.PSObject.Properties['value']
    if ($null -eq $v) { return '' }
    return [string]$v.Value
}

function Get-ByPo {
    param([string]$Token,[string]$PoValue)
    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $query = '数値 = "{0}" limit 100' -f ($PoValue -replace '"','\"')
    $uri = '{0}/k/v1/records.json?app={1}&query={2}' -f `
        $BaseUrl.TrimEnd('/'), $App292, [Uri]::EscapeDataString($query)
    $resp = Invoke-RestMethod -Method Get -Uri $uri -Headers $headers
    $m = @($resp.records)
    if ($m.Count -ne 1) {
        throw "SafetyStop: PO=$PoValue matched=$($m.Count), expected exactly 1."
    }
    return $m[0]
}

if ($App292 -ne 292) { throw "SafetyStop: App292 fixed to 292." }
if ($Execute -and $ConfirmExecute -ne 'APP292-U1-BASELINE') {
    throw "SafetyStop: -Execute requires -ConfirmExecute 'APP292-U1-BASELINE'"
}

$token = Get-Token
$r = Get-ByPo -Token $token -PoValue $Po

$id = Get-KValue $r '$id'
$rev = Get-KValue $r '$revision'
$orderDate = Get-KValue $r '日付'
$dueDate = Get-KValue $r '日付_1'

if ($orderDate -ne $ExpectedOrderDate) {
    throw "SafetyStop: 手配日 unexpected. actual=$orderDate expected=$ExpectedOrderDate"
}
if ($dueDate -ne $ExpectedDueDate) {
    throw "SafetyStop: 納期 unexpected. actual=$dueDate expected=$ExpectedDueDate"
}

Write-Host "=== App292 U1 Batch Baseline v0.1 ==="
Write-Host "Mode             : $(if ($Execute) {'EXECUTE'} else {'READ ONLY'})"
Write-Host "App              : $App292"
Write-Host "PO               : $Po"
Write-Host "RecordId         : $id"
Write-Host "Revision         : $rev"
Write-Host "手配日           : $orderDate"
Write-Host "納期             : $dueDate"
Write-Host ""
Write-Host "Current VENDOR_TEXT : $(Get-KValue $r 'VENDOR_TEXT')"
Write-Host "Current VENDOR_DD   : $(Get-KValue $r 'VENDOR_DD')"
Write-Host "Current STAFF_TEXT  : $(Get-KValue $r 'STAFF_TEXT')"
Write-Host "Current STAFF_DD    : $(Get-KValue $r 'STAFF_DD')"
Write-Host "Current K加工着手日 : $(Get-KValue $r 'K加工着手日')"
Write-Host "Current K完成検査日 : $(Get-KValue $r 'K完成検査日')"
Write-Host "Current K日程種別   : $(Get-KValue $r 'K日程種別')"
Write-Host "Current K暫定設定   : $(Get-KValue $r 'K暫定設定')"
Write-Host "Current K計算根拠   : $(Get-KValue $r 'K計算根拠')"
Write-Host ""
Write-Host "Target VENDOR_TEXT  : $ExpectedVendor"
Write-Host "Target VENDOR_DD    : $ExpectedVendor"
Write-Host "Target STAFF_TEXT   : $ExpectedStaff"
Write-Host "Target STAFF_DD     : $ExpectedStaff"
Write-Host "Target K加工着手日  : $BaselineStart"
Write-Host "Target K完成検査日  : $BaselineInspection"
Write-Host "Target K日程種別    : Batch"
Write-Host "Target K暫定設定    : ON"
Write-Host "Target K計算根拠    : Company"

if (-not $Execute) {
    Write-Host ""
    Write-Host "RESULT : BASELINE PLAN PASS"
    Write-Host "WRITE  : NONE"
    exit 0
}

# TOCTOU
$pre = Get-ByPo -Token $token -PoValue $Po
if ((Get-KValue $pre '$id') -ne $id -or
    (Get-KValue $pre '$revision') -ne $rev -or
    (Get-KValue $pre '日付') -ne $orderDate -or
    (Get-KValue $pre '日付_1') -ne $dueDate) {
    throw "TOCTOU stop: record changed after baseline GET."
}

$headers = @{ 'X-Cybozu-API-Token' = $token }
$uri = $BaseUrl.TrimEnd('/') + '/k/v1/record.json'
$bodyObj = [ordered]@{
    app = $App292
    id = [int64]$id
    revision = [int64]$rev
    record = [ordered]@{
        'VENDOR_TEXT'  = @{ value = $ExpectedVendor }
        'VENDOR_DD'    = @{ value = $ExpectedVendor }
        'STAFF_TEXT'   = @{ value = $ExpectedStaff }
        'STAFF_DD'     = @{ value = $ExpectedStaff }
        'K加工着手日' = @{ value = $BaselineStart }
        'K完成検査日' = @{ value = $BaselineInspection }
        'K日程種別'   = @{ value = 'Batch' }
        'K暫定設定'   = @{ value = 'ON' }
        'K計算根拠'   = @{ value = 'Company' }
    }
}
$body = $bodyObj | ConvertTo-Json -Depth 10
$resp = Invoke-RestMethod -Method Put -Uri $uri -Headers $headers `
    -ContentType 'application/json; charset=utf-8' -Body $body

$after = Get-ByPo -Token $token -PoValue $Po
$checks = [ordered]@{
    VENDOR_TEXT = ((Get-KValue $after 'VENDOR_TEXT') -eq $ExpectedVendor)
    VENDOR_DD   = ((Get-KValue $after 'VENDOR_DD') -eq $ExpectedVendor)
    STAFF_TEXT  = ((Get-KValue $after 'STAFF_TEXT') -eq $ExpectedStaff)
    STAFF_DD    = ((Get-KValue $after 'STAFF_DD') -eq $ExpectedStaff)
    K加工着手日 = ((Get-KValue $after 'K加工着手日') -eq $BaselineStart)
    K完成検査日 = ((Get-KValue $after 'K完成検査日') -eq $BaselineInspection)
    K日程種別   = ((Get-KValue $after 'K日程種別') -eq 'Batch')
    K暫定設定   = ((Get-KValue $after 'K暫定設定') -eq 'ON')
    K計算根拠   = ((Get-KValue $after 'K計算根拠') -eq 'Company')
    手配日不変   = ((Get-KValue $after '日付') -eq $ExpectedOrderDate)
    納期不変     = ((Get-KValue $after '日付_1') -eq $ExpectedDueDate)
}

$failed = @($checks.GetEnumerator() | Where-Object { -not $_.Value })
Write-Host ""
Write-Host "PUT revision     : $($resp.revision)"
foreach ($c in $checks.GetEnumerator()) {
    Write-Host ("{0,-16}: {1}" -f $c.Key, $(if ($c.Value) {'PASS'} else {'FAIL'}))
}
if ($failed.Count -gt 0) { throw "Baseline verification failed." }

Write-Host ""
Write-Host "RESULT : U1 BASELINE PASS"
