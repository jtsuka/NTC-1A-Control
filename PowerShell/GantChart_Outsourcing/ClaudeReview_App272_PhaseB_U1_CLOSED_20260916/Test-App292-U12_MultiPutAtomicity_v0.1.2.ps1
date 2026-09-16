#requires -Version 5.1
<#
.SYNOPSIS
  App292 U12専用: 複数PUT原子性テスト v0.1.2

.DESCRIPTION
  App272 Daily Import 詳細設計 Rev.2 の Test U12 をApp292で単独実証する。
  2件の既存レコードを使い、片方だけ意図的にrevision競合させた複数PUTを送信し、
  「全件失敗」か「部分成功」かを再GETで判定する。

  既定はREAD ONLY。-Execute と ConfirmExecute='APP292-U12-EXECUTE' が揃った場合だけ書込みする。

  書込み時の流れ:
    1) 2レコードの基準値/revisionを再GET
    2) Record Bだけ単独PUTしrevisionを進める（競合作成）
    3) Record A=正しい旧revision / Record B=古いrevision の2件を同一PUT /records.jsonで送る
    4) API応答と再GETで原子性を判定
    5) 判定結果を保存
    6) 個別PUTで両レコードのテスト対象フィールドを元値へ復元

  重要:
    - TEST App292以外では動作しない。
    - 既定テストフィールド STAFF_TEXT は文字列フィールドとして存在することを事前確認する。
    - テスト前後の業務値を復元するが、revision番号自体は更新により進む。
    - 本スクリプトは「原子性を観測する」ためのテストであり、本番App272では使用しない。

.PARAMETER PoA
  テスト対象Aの発注番号（App292の「数値」）。

.PARAMETER PoB
  テスト対象Bの発注番号（App292の「数値」）。

.PARAMETER Execute
  実書込みを行う。未指定時はREAD ONLYで候補確認だけ。

.PARAMETER ConfirmExecute
  Execute時は APP292-U12-EXECUTE を必須とする。

.EXAMPLE
  # 候補一覧だけ確認
  .\Test-App292-U12_MultiPutAtomicity_v0.1.ps1

.EXAMPLE
  # 2件指定してDryRun
  .\Test-App292-U12_MultiPutAtomicity_v0.1.ps1 `
    -PoA "2609249313" `
    -PoB "2609249755"

.EXAMPLE
  # U12実行
  .\Test-App292-U12_MultiPutAtomicity_v0.1.ps1 `
    -PoA "2609249313" `
    -PoB "2609249755" `
    -Execute `
    -ConfirmExecute "APP292-U12-EXECUTE"
#>

[CmdletBinding()]
param(
    [string]$BaseUrl = 'https://bcurbkixz609.cybozu.com',
    [int]$App292 = 292,
    [string]$ApiToken292 = '',
    [string]$PoA = '',
    [string]$PoB = '',
    [string]$TestFieldCode = 'STAFF_TEXT',
    [string]$LogDirectory = 'C:\HPDB\logs',
    [switch]$Execute,
    [string]$ConfirmExecute = ''
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

function ConvertFrom-SecureStringPlain {
    param([Security.SecureString]$Secure)
    $bstr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($Secure)
    try { return [Runtime.InteropServices.Marshal]::PtrToStringBSTR($bstr) }
    finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($bstr) }
}

function Get-Token {
    if (-not [string]::IsNullOrWhiteSpace($ApiToken292)) { return $ApiToken292 }
    $s = Read-Host "App292 API Token" -AsSecureString
    return ConvertFrom-SecureStringPlain $s
}

function Get-KintoneErrorText {
    param([Parameter(Mandatory=$true)]$ErrorRecord)
    $parts = New-Object System.Collections.Generic.List[string]
    try {
        $respProp = $ErrorRecord.Exception.PSObject.Properties['Response']
        if ($null -ne $respProp -and $null -ne $respProp.Value) {
            $resp = $respProp.Value
            try { $parts.Add("HTTP=$([int]$resp.StatusCode) $($resp.StatusDescription)") } catch {}
            try {
                $stream = $resp.GetResponseStream()
                if ($null -ne $stream) {
                    $reader = New-Object IO.StreamReader($stream)
                    try {
                        $body = $reader.ReadToEnd()
                        if ($body) { $parts.Add("Body=$body") }
                    } finally { $reader.Dispose() }
                }
            } catch {}
        }
    } catch {}
    try {
        if ($ErrorRecord.Exception.Message) { $parts.Add("Exception=$($ErrorRecord.Exception.Message)") }
    } catch {}
    if ($parts.Count -eq 0) { return [string]$ErrorRecord }
    return ($parts -join ' | ')
}

function Get-KValue {
    param($Record, [string]$FieldCode)
    if ($null -eq $Record) { return '' }
    $p = $Record.PSObject.Properties[$FieldCode]
    if ($null -eq $p -or $null -eq $p.Value) { return '' }
    $v = $p.Value.PSObject.Properties['value']
    if ($null -eq $v) { return '' }
    return [string]$v.Value
}

function Invoke-App292GetAll {
    param([string]$Token)
    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $all = New-Object System.Collections.Generic.List[object]
    $lastId = 0

    while ($true) {
        $query = '$id > {0} order by $id asc limit 500' -f $lastId
        $uri = '{0}/k/v1/records.json?app={1}&query={2}' -f `
            $BaseUrl.TrimEnd('/'), `
            $App292, `
            [Uri]::EscapeDataString($query)

        $resp = Invoke-RestMethod -Method Get -Uri $uri -Headers $headers
        $records = @($resp.records)
        foreach ($r in $records) { $all.Add($r) }
        if ($records.Count -lt 500) { break }
        $lastId = [int64](Get-KValue $records[-1] '$id')
    }
    # Windows PowerShell 5.1では @($GenericList) が
    # '引数の型が一致しません' (ArgumentException) になる場合があるため、
    # 作業実績のあるv0.7.4と同じく ToArray() を明示する。
    return $all.ToArray()
}

function Get-LiveFieldDefinition {
    param([string]$Token, [string]$FieldCode)
    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $uri = '{0}/k/v1/app/form/fields.json?app={1}&lang=ja' -f $BaseUrl.TrimEnd('/'), $App292
    $resp = Invoke-RestMethod -Method Get -Uri $uri -Headers $headers
    $p = $resp.properties.PSObject.Properties[$FieldCode]
    if ($null -eq $p) { return $null }
    return $p.Value
}

function Invoke-SingleUpdate {
    param(
        [string]$Token,
        [string]$RecordId,
        [string]$Revision,
        [string]$FieldCode,
        [AllowEmptyString()][string]$Value
    )
    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $uri = $BaseUrl.TrimEnd('/') + '/k/v1/record.json'
    $bodyObj = [ordered]@{
        app = $App292
        id = [int64]$RecordId
        revision = [int64]$Revision
        record = [ordered]@{
            $FieldCode = @{ value = $Value }
        }
    }
    $body = $bodyObj | ConvertTo-Json -Depth 10
    return Invoke-RestMethod -Method Put -Uri $uri -Headers $headers `
        -ContentType 'application/json; charset=utf-8' -Body $body
}

function Invoke-MultiUpdateRaw {
    param(
        [string]$Token,
        [object[]]$Items
    )
    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $uri = $BaseUrl.TrimEnd('/') + '/k/v1/records.json'
    $bodyObj = [ordered]@{
        app = $App292
        records = @($Items)
    }
    $body = $bodyObj | ConvertTo-Json -Depth 20
    try {
        $resp = Invoke-RestMethod -Method Put -Uri $uri -Headers $headers `
            -ContentType 'application/json; charset=utf-8' -Body $body
        return [pscustomobject]@{
            Succeeded = $true
            Response = $resp
            ErrorText = ''
        }
    }
    catch {
        return [pscustomobject]@{
            Succeeded = $false
            Response = $null
            ErrorText = (Get-KintoneErrorText $_)
        }
    }
}

function Find-RecordByPo {
    param([object[]]$Records, [string]$Po)
    $m = @($Records | Where-Object { (Get-KValue $_ '数値') -eq $Po })
    if ($m.Count -ne 1) {
        throw "PO lookup failed: PO=$Po matched=$($m.Count) (expected exactly 1)"
    }
    return $m[0]
}

function New-TestMarker {
    param([string]$Label)
    return ('U12_{0}_{1}' -f $Label, (Get-Date -Format 'yyyyMMdd_HHmmssfff'))
}

function Restore-RecordField {
    param(
        [string]$Token,
        [string]$Po,
        [string]$OriginalValue
    )
    $now = Invoke-App292GetAll -Token $Token
    $r = Find-RecordByPo -Records $now -Po $Po
    $id = Get-KValue $r '$id'
    $rev = Get-KValue $r '$revision'
    [void](Invoke-SingleUpdate -Token $Token -RecordId $id -Revision $rev `
        -FieldCode $TestFieldCode -Value $OriginalValue)
}

if ($App292 -ne 292) {
    throw "SafetyStop: U12 test is fixed to TEST App292. App292=$App292"
}

if ($Execute -and $ConfirmExecute -ne 'APP292-U12-EXECUTE') {
    throw "SafetyStop: -Execute requires -ConfirmExecute 'APP292-U12-EXECUTE'"
}

if (-not (Test-Path -LiteralPath $LogDirectory)) {
    New-Item -ItemType Directory -Path $LogDirectory -Force | Out-Null
}

$token = Get-Token
$records = @(Invoke-App292GetAll -Token $token)

Write-Host "=== App292 U12 複数PUT原子性テスト v0.1.2 ==="
Write-Host "App               : $App292"
Write-Host "Mode              : $(if ($Execute) { 'EXECUTE' } else { 'READ ONLY / PLAN' })"
Write-Host "Test field        : $TestFieldCode"
Write-Host "App292 records    : $($records.Count)"
Write-Host ""

$fieldDef = Get-LiveFieldDefinition -Token $token -FieldCode $TestFieldCode
if ($null -eq $fieldDef) {
    throw "SafetyStop: TestFieldCode '$TestFieldCode' not found in App292 form."
}
$fieldType = [string]$fieldDef.type
if ($fieldType -notin @('SINGLE_LINE_TEXT','MULTI_LINE_TEXT')) {
    throw "SafetyStop: TestFieldCode '$TestFieldCode' type=$fieldType. Use a text field only."
}
Write-Host "Field type        : $fieldType"

if ([string]::IsNullOrWhiteSpace($PoA) -or [string]::IsNullOrWhiteSpace($PoB)) {
    Write-Host ""
    Write-Host "PoA / PoB が未指定です。候補一覧を表示します（WRITE: NONE）。"
    Write-Host ""
    $records |
        Select-Object -First 20 |
        ForEach-Object {
            [pscustomobject]@{
                PO       = Get-KValue $_ '数値'
                RecordId = Get-KValue $_ '$id'
                Revision = Get-KValue $_ '$revision'
                Vendor   = Get-KValue $_ 'VENDOR_TEXT'
                Staff    = Get-KValue $_ 'STAFF_TEXT'
                KType    = Get-KValue $_ 'K日程種別'
            }
        } | Format-Table -AutoSize

    Write-Host ""
    Write-Host "RESULT : CANDIDATE LIST ONLY"
    Write-Host "WRITE  : NONE"
    exit 0
}

if ($PoA -eq $PoB) {
    throw "SafetyStop: PoA and PoB must be different."
}

$recA = Find-RecordByPo -Records $records -Po $PoA
$recB = Find-RecordByPo -Records $records -Po $PoB

$baseA = [pscustomobject]@{
    Po = $PoA
    Id = Get-KValue $recA '$id'
    Revision = Get-KValue $recA '$revision'
    OriginalValue = Get-KValue $recA $TestFieldCode
}
$baseB = [pscustomobject]@{
    Po = $PoB
    Id = Get-KValue $recB '$id'
    Revision = Get-KValue $recB '$revision'
    OriginalValue = Get-KValue $recB $TestFieldCode
}

Write-Host ""
Write-Host "=== BASELINE ==="
Write-Host ("A: PO={0} ID={1} revision={2} {3}='{4}'" -f $baseA.Po,$baseA.Id,$baseA.Revision,$TestFieldCode,$baseA.OriginalValue)
Write-Host ("B: PO={0} ID={1} revision={2} {3}='{4}'" -f $baseB.Po,$baseB.Id,$baseB.Revision,$TestFieldCode,$baseB.OriginalValue)

if (-not $Execute) {
    Write-Host ""
    Write-Host "=== U12 PLAN ==="
    Write-Host "1. Bだけ単独PUTしてrevisionを進める"
    Write-Host "2. A=baseline revision / B=古いbaseline revision で2件PUT"
    Write-Host "3. 再GETし、Aが未変更なら全件原子的失敗、Aだけ変更なら部分成功と判定"
    Write-Host "4. A/Bのテストフィールドを個別PUTで元値へ復元"
    Write-Host ""
    Write-Host "RESULT : PLAN PASS"
    Write-Host "WRITE  : NONE"
    exit 0
}

$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$jsonLog = Join-Path $LogDirectory ("Test-App292-U12_Result_{0}.json" -f $stamp)
$txtLog  = Join-Path $LogDirectory ("Test-App292-U12_Summary_{0}.txt" -f $stamp)

$markerA = New-TestMarker 'A_TARGET'
$markerBConflict = New-TestMarker 'B_CONFLICT'
$markerBTarget = New-TestMarker 'B_TARGET'

$testResult = $null
$cleanupA = 'NOT_NEEDED'
$cleanupB = 'NOT_NEEDED'
$multiResult = $null

try {
    Write-Host ""
    Write-Host "=== U12 EXECUTE START ===" -ForegroundColor Yellow

    # TOCTOU: Execute直前に再GETしてbaseline revision/value一致確認
    $pre = @(Invoke-App292GetAll -Token $token)
    $preA = Find-RecordByPo -Records $pre -Po $PoA
    $preB = Find-RecordByPo -Records $pre -Po $PoB

    if ((Get-KValue $preA '$id') -ne $baseA.Id -or
        (Get-KValue $preA '$revision') -ne $baseA.Revision -or
        (Get-KValue $preA $TestFieldCode) -ne $baseA.OriginalValue) {
        throw "TOCTOU stop: Record A changed after baseline."
    }
    if ((Get-KValue $preB '$id') -ne $baseB.Id -or
        (Get-KValue $preB '$revision') -ne $baseB.Revision -or
        (Get-KValue $preB $TestFieldCode) -ne $baseB.OriginalValue) {
        throw "TOCTOU stop: Record B changed after baseline."
    }

    Write-Host "[U12-1] Record Bだけ単独PUTしてrevision競合作成..."
    $bConflictResp = Invoke-SingleUpdate -Token $token -RecordId $baseB.Id `
        -Revision $baseB.Revision -FieldCode $TestFieldCode -Value $markerBConflict
    Write-Host ("        B revision {0} -> {1}" -f $baseB.Revision, $bConflictResp.revision)

    Write-Host "[U12-2] A=有効revision / B=古いrevision で複数PUT..."
    $items = @(
        [ordered]@{
            id = [int64]$baseA.Id
            revision = [int64]$baseA.Revision
            record = [ordered]@{
                $TestFieldCode = @{ value = $markerA }
            }
        },
        [ordered]@{
            id = [int64]$baseB.Id
            revision = [int64]$baseB.Revision   # intentionally stale
            record = [ordered]@{
                $TestFieldCode = @{ value = $markerBTarget }
            }
        }
    )

    $multiResult = Invoke-MultiUpdateRaw -Token $token -Items $items
    Write-Host ("        API success = {0}" -f $multiResult.Succeeded)
    if (-not $multiResult.Succeeded) {
        Write-Host ("        API error   = {0}" -f $multiResult.ErrorText)
    }

    Write-Host "[U12-3] 再GETして実状態を判定..."
    $after = @(Invoke-App292GetAll -Token $token)
    $afterA = Find-RecordByPo -Records $after -Po $PoA
    $afterB = Find-RecordByPo -Records $after -Po $PoB

    $afterAValue = Get-KValue $afterA $TestFieldCode
    $afterBValue = Get-KValue $afterB $TestFieldCode

    if ($afterAValue -eq $baseA.OriginalValue -and $afterBValue -eq $markerBConflict) {
        $testResult = 'ATOMIC_ALL_OR_NOTHING'
    }
    elseif ($afterAValue -eq $markerA -and $afterBValue -eq $markerBConflict) {
        $testResult = 'PARTIAL_SUCCESS_OBSERVED'
    }
    else {
        $testResult = 'UNEXPECTED_STATE'
    }

    Write-Host ("        A after = '{0}'" -f $afterAValue)
    Write-Host ("        B after = '{0}'" -f $afterBValue)
    Write-Host ("        RESULT  = {0}" -f $testResult) -ForegroundColor Cyan

    $resultObject = [ordered]@{
        runAt = (Get-Date).ToString('o')
        app = $App292
        test = 'U12-MultiPutAtomicity'
        testField = $TestFieldCode
        baseline = [ordered]@{
            A = [ordered]@{ po=$baseA.Po; id=$baseA.Id; revision=$baseA.Revision; value=$baseA.OriginalValue }
            B = [ordered]@{ po=$baseB.Po; id=$baseB.Id; revision=$baseB.Revision; value=$baseB.OriginalValue }
        }
        conflict = [ordered]@{
            BValue = $markerBConflict
            BRevisionAfterConflict = [string]$bConflictResp.revision
        }
        multiPut = [ordered]@{
            apiSucceeded = [bool]$multiResult.Succeeded
            errorText = [string]$multiResult.ErrorText
            attemptedAValue = $markerA
            attemptedBValue = $markerBTarget
            staleBRevision = $baseB.Revision
        }
        observed = [ordered]@{
            AValue = $afterAValue
            ARevision = Get-KValue $afterA '$revision'
            BValue = $afterBValue
            BRevision = Get-KValue $afterB '$revision'
        }
        conclusion = $testResult
        phaseBPolicy = [ordered]@{
            multiPutAllowed = ($testResult -eq 'ATOMIC_ALL_OR_NOTHING')
            initialPutBatchSize = 10
            requireRevision = $true
            requireBeforeValueMatch = $true
            requireChangeHash = $true
            requirePostBatchReGetVerification = $true
            stopFollowingBatchesOnAnyError = $true
        }
    }

    $resultObject | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath $jsonLog -Encoding UTF8
}
finally {
    Write-Host ""
    Write-Host "[U12-4] Cleanup: A/Bのテストフィールドを元値へ復元..."
    try {
        $nowA = Find-RecordByPo -Records @(Invoke-App292GetAll -Token $token) -Po $PoA
        if ((Get-KValue $nowA $TestFieldCode) -ne $baseA.OriginalValue) {
            Restore-RecordField -Token $token -Po $PoA -OriginalValue $baseA.OriginalValue
            $cleanupA = 'RESTORED'
        } else {
            $cleanupA = 'ALREADY_ORIGINAL'
        }
    }
    catch {
        $cleanupA = 'FAILED: ' + (Get-KintoneErrorText $_)
    }

    try {
        $nowB = Find-RecordByPo -Records @(Invoke-App292GetAll -Token $token) -Po $PoB
        if ((Get-KValue $nowB $TestFieldCode) -ne $baseB.OriginalValue) {
            Restore-RecordField -Token $token -Po $PoB -OriginalValue $baseB.OriginalValue
            $cleanupB = 'RESTORED'
        } else {
            $cleanupB = 'ALREADY_ORIGINAL'
        }
    }
    catch {
        $cleanupB = 'FAILED: ' + (Get-KintoneErrorText $_)
    }

    Write-Host ("        Cleanup A = {0}" -f $cleanupA)
    Write-Host ("        Cleanup B = {0}" -f $cleanupB)
}

$summary = @"
=== App292 U12 Result ===
RunAt              : $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')
App                : $App292
PoA                : $PoA
PoB                : $PoB
TestField          : $TestFieldCode
MultiPutApiSuccess : $($multiResult.Succeeded)
Conclusion         : $testResult
PhaseBPolicy       : $(if ($testResult -eq 'ATOMIC_ALL_OR_NOTHING') { 'MULTI_PUT_ALLOWED / PutBatchSize=10 / PreGuard+PostVerify required' } else { 'SINGLE_OR_MICRO_BATCH_REQUIRED' })
CleanupA           : $cleanupA
CleanupB           : $cleanupB
JSONLog            : $jsonLog
"@
$summary | Set-Content -LiteralPath $txtLog -Encoding UTF8
Write-Host ""
Write-Host $summary

if ($cleanupA -like 'FAILED:*' -or $cleanupB -like 'FAILED:*') {
    throw "U12 cleanup failed. Check App292 immediately. Summary=$txtLog"
}

switch ($testResult) {
    'ATOMIC_ALL_OR_NOTHING' {
        Write-Host "U12 PASS: 複数PUTは今回の実測では全件原子的に失敗しました。" -ForegroundColor Green
        Write-Host "Phase B policy: MULTI_PUT_ALLOWED / initial PutBatchSize=10 / revision+before+changeHash guard / post-batch reGET verify" -ForegroundColor Green
        exit 0
    }
    'PARTIAL_SUCCESS_OBSERVED' {
        Write-Host "U12 IMPORTANT: 部分成功を確認しました。Phase Bは1件PUT/極小バッチ＋チェックポイント設計が必須です。" -ForegroundColor Yellow
        exit 2
    }
    default {
        throw "U12 unexpected state. Check JSON log: $jsonLog"
    }
}
