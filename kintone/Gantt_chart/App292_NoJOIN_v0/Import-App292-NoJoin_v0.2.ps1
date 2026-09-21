#requires -Version 5.1
<#
.SYNOPSIS
  App292 NoJOIN 全937件取込総合試験 v0.2
.DESCRIPTION
  Windows PowerShell 5.1用。
  C:\HPDB のNoJOIN想定CSVをApp292(TEST)へ直接POSTする。
  App270およびルックアップは使用しない。
  主キーは発注番号（field code: 数値）。
  既定はDryRun。
#>

[CmdletBinding()]
param(
    [string]$CsvPath = 'C:\HPDB\外注課納期管理_NoJOIN想定_全937件_App292_TEST_20260921.csv',
    [string]$BaseUrl = 'https://bcurbkixz609.cybozu.com',
    [int]$AppId = 292,
    [string]$TokenPath = 'C:\HPDB\Secrets\App292_Test.token',
    [string]$LogDirectory = 'C:\HPDB\logs',
    [switch]$DryRun,
    [switch]$Execute,
    [string]$ConfirmExecute = '',
    [ValidateRange(1,2000)][int]$ExpectedCsvRows = 937,
    [ValidateRange(0,2000)][int]$ExpectedBlankOrderNo = 610,
    [ValidateRange(1,2000)][int]$MaxCreates = 937
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

if ($AppId -ne 292) { throw 'SafetyStop: AppId must be 292.' }
if ($DryRun -and $Execute) { throw 'SafetyStop: DryRun/Execute conflict.' }
if (-not $DryRun -and -not $Execute) { $DryRun = $true }
if ($Execute -and $ConfirmExecute -ne 'APP292-NOJOIN-937-EXECUTE') {
    throw "SafetyStop: Execute requires -ConfirmExecute 'APP292-NOJOIN-937-EXECUTE'"
}

if (-not (Test-Path -LiteralPath $CsvPath -PathType Leaf)) { throw "CSV not found: $CsvPath" }
if (-not (Test-Path -LiteralPath $LogDirectory -PathType Container)) {
    New-Item -ItemType Directory -Path $LogDirectory -Force | Out-Null
}

function Import-CsvAllowDuplicateHeaders {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)][string]$Path,
        [System.Text.Encoding]$Encoding = ([System.Text.Encoding]::GetEncoding(932))
    )

    Add-Type -AssemblyName Microsoft.VisualBasic -ErrorAction Stop
    $parser = New-Object Microsoft.VisualBasic.FileIO.TextFieldParser($Path, $Encoding, $true)

    try {
        $parser.TextFieldType = [Microsoft.VisualBasic.FileIO.FieldType]::Delimited
        $parser.SetDelimiters(',')
        $parser.HasFieldsEnclosedInQuotes = $true
        $parser.TrimWhiteSpace = $false

        if ($parser.EndOfData) { return @() }

        $rawHeaders = @($parser.ReadFields())
        if ($rawHeaders.Count -eq 0) { return @() }

        $seen = @{}
        $headers = New-Object System.Collections.Generic.List[string]

        foreach ($raw in $rawHeaders) {
            $name = [string]$raw
            if ($headers.Count -eq 0) { $name = $name.TrimStart([char]0xFEFF) }
            if ([string]::IsNullOrWhiteSpace($name)) { $name = '__BLANK_HEADER' }

            if ($seen.ContainsKey($name)) {
                $seen[$name]++
                $headers.Add(('{0}__DUP{1}' -f $name, $seen[$name]))
            } else {
                $seen[$name] = 1
                $headers.Add($name)
            }
        }

        $rows = New-Object System.Collections.Generic.List[object]
        $lineNo = 1

        while (-not $parser.EndOfData) {
            $lineNo++
            try {
                $fields = @($parser.ReadFields())
            } catch {
                throw "CSV解析エラー: 行 $lineNo : $($_.Exception.Message)"
            }

            if ($fields.Count -ne $headers.Count) {
                throw "CSV列数不一致: 行 $lineNo / header=$($headers.Count) / row=$($fields.Count)"
            }

            $obj = [ordered]@{}
            for ($i = 0; $i -lt $headers.Count; $i++) {
                $obj[$headers[$i]] = $fields[$i]
            }
            $rows.Add([pscustomobject]$obj)
        }

        return $rows.ToArray()
    }
    finally {
        if ($null -ne $parser) { $parser.Close() }
    }
}

function Get-PlainToken {
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) { throw "Token not found: $Path" }

    $enc = (Get-Content -LiteralPath $Path -Raw).Trim()
    $sec = $enc | ConvertTo-SecureString
    $ptr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($sec)

    try { return [Runtime.InteropServices.Marshal]::PtrToStringBSTR($ptr) }
    finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($ptr) }
}

function Normalize-Date {
    param([AllowNull()][AllowEmptyString()][string]$Value)

    if ([string]::IsNullOrWhiteSpace($Value)) { return '' }
    $v = $Value.Trim()

    $formats = @(
        'yyyy-MM-dd HH:mm:ss',
        'yyyy/M/d HH:mm:ss',
        'yyyy/MM/dd HH:mm:ss',
        'yyyy-MM-dd',
        'yyyy/M/d',
        'yyyy/MM/dd'
    )

    foreach ($fmt in $formats) {
        $dt = [datetime]::MinValue

        if ([datetime]::TryParseExact(
            $v,
            $fmt,
            [Globalization.CultureInfo]::InvariantCulture,
            [Globalization.DateTimeStyles]::None,
            [ref]$dt
        )) {
            return $dt.ToString('yyyy-MM-dd')
        }
    }

    throw "Invalid date: '$Value'"
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

function Get-LiveForm {
    param([string]$Token)

    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $uri = $BaseUrl.TrimEnd('/') + '/k/v1/app/form/fields.json'

    return Invoke-RestMethod -Method Get -Uri $uri -Headers $headers -Body @{ app = $AppId; lang = 'ja' }
}

function Test-OptionExists {
    param($Form, [string]$Code, [string]$Value)

    $p = $Form.properties.PSObject.Properties[$Code]
    if ($null -eq $p) { return $false }

    return ($null -ne $p.Value.options.PSObject.Properties[$Value])
}

function Add-RecordBatch {
    param([string]$Token, [object[]]$Records)

    if ($Records.Count -lt 1 -or $Records.Count -gt 100) {
        throw "Invalid POST batch count=$($Records.Count)"
    }

    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $uri = $BaseUrl.TrimEnd('/') + '/k/v1/records.json'
    $body = [ordered]@{
        app = $AppId
        records = $Records
    } | ConvertTo-Json -Depth 20

    return Invoke-RestMethod -Method Post -Uri $uri -Headers $headers `
        -ContentType 'application/json; charset=utf-8' `
        -Body ([Text.Encoding]::UTF8.GetBytes($body))
}

$rows = @(Import-CsvAllowDuplicateHeaders -Path $CsvPath)

$required = @(
    '発注番号',
    '手配書番号',
    '注文伝票番号',
    '注文伝票行番号',
    '商品名',
    '発注数',
    '手配日',
    '納期',
    '指示先名',
    '手配担当者名'
)

if ($rows.Count -eq 0) { throw 'CSV empty.' }

$columns = @($rows[0].PSObject.Properties.Name)
foreach ($c in $required) {
    if ($columns -notcontains $c) { throw "Required column missing: $c" }
}

if ($rows.Count -ne $ExpectedCsvRows) {
    throw "SafetyStop: CSVRows=$($rows.Count) expected=$ExpectedCsvRows"
}

$poSeen = @{}
$blankOrderNo = 0
$blankOrderSlip = 0
$errors = New-Object System.Collections.Generic.List[string]
$normalized = New-Object System.Collections.Generic.List[object]

$rowNo = 1

foreach ($r in $rows) {
    $rowNo++

    $po = ([string]$r.発注番号).Trim()
    $orderNo = ([string]$r.手配書番号).Trim()
    $slipNo = ([string]$r.注文伝票番号).Trim()
    $slipLine = ([string]$r.注文伝票行番号).Trim()
    $vendor = ([string]$r.指示先名).Trim()
    $staff = ([string]$r.手配担当者名).Trim()
    $product = ([string]$r.商品名).Trim()
    $qty = ([string]$r.発注数).Trim()

    try { $orderDate = Normalize-Date ([string]$r.手配日) }
    catch { $errors.Add("Row=$rowNo PO=$po OrderDate $($_.Exception.Message)"); continue }

    try { $dueDate = Normalize-Date ([string]$r.納期) }
    catch { $errors.Add("Row=$rowNo PO=$po DueDate $($_.Exception.Message)"); continue }

    if (-not $po) {
        $errors.Add("Row=$rowNo blank PO")
        continue
    }

    if ($poSeen.ContainsKey($po)) {
        $errors.Add("Duplicate PO=$po rows=$($poSeen[$po]),$rowNo")
        continue
    }

    $poSeen[$po] = $rowNo

    if (-not $orderNo) { $blankOrderNo++ }
    if (-not $slipNo) { $blankOrderSlip++ }

    if (-not $slipNo -or -not $slipLine -or -not $vendor -or -not $staff -or -not $orderDate -or -not $dueDate) {
        $errors.Add("Row=$rowNo PO=$po required operational value blank")
        continue
    }

    $normalized.Add([pscustomobject]@{
        Row = $rowNo
        PO = $po
        OrderNo = $orderNo
        SlipNo = $slipNo
        SlipLine = $slipLine
        Product = $product
        Qty = $qty
        OrderDate = $orderDate
        DueDate = $dueDate
        Vendor = $vendor
        Staff = $staff
    })
}

if ($blankOrderNo -ne $ExpectedBlankOrderNo) {
    throw "SafetyStop: BlankOrderNo=$blankOrderNo expected=$ExpectedBlankOrderNo"
}

if ($blankOrderSlip -ne 0) {
    throw "SafetyStop: BlankOrderSlip=$blankOrderSlip expected=0"
}

if ($errors.Count -gt 0) {
    $errors | Select-Object -First 50 | ForEach-Object { Write-Error $_ }
    throw "SafetyStop: CSV validation errors=$($errors.Count)"
}

if ($normalized.Count -gt $MaxCreates) {
    throw "SafetyStop: creates=$($normalized.Count) MaxCreates=$MaxCreates"
}

$token = Get-PlainToken -Path $TokenPath
$existing = @(Get-AllRecords -Token $token)
$form = Get-LiveForm -Token $token

$missingVendors = @(
    $normalized |
    ForEach-Object { $_.Vendor } |
    Sort-Object -Unique |
    Where-Object { -not (Test-OptionExists -Form $form -Code 'VENDOR_DD' -Value $_) }
)

$missingStaff = @(
    $normalized |
    ForEach-Object { $_.Staff } |
    Sort-Object -Unique |
    Where-Object { -not (Test-OptionExists -Form $form -Code 'STAFF_DD' -Value $_) }
)

$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$planPath = Join-Path $LogDirectory "App292_NoJOIN_Plan_$stamp.csv"

$normalized |
    Select-Object Row,PO,OrderNo,SlipNo,SlipLine,Product,Qty,OrderDate,DueDate,Vendor,Staff |
    Export-Csv -LiteralPath $planPath -NoTypeInformation -Encoding UTF8

Write-Host '================================================================='
Write-Host 'Import App292 NoJOIN v0.2'
Write-Host '================================================================='
Write-Host "CSVRows            : $($rows.Count)"
Write-Host "UniquePO           : $($poSeen.Count)"
Write-Host "HandOrderPresent   : $($rows.Count - $blankOrderNo)"
Write-Host "HandOrderBlank     : $blankOrderNo"
Write-Host "OrderSlipPresent   : $($rows.Count - $blankOrderSlip)"
Write-Host "Existing App292    : $($existing.Count)"
Write-Host "Missing VENDOR_DD  : $($missingVendors.Count)"
Write-Host "Missing STAFF_DD   : $($missingStaff.Count)"
Write-Host "Plan               : $planPath"
Write-Host 'App270              : NOT USED'
Write-Host 'ルックアップ        : NOT WRITTEN'
Write-Host 'Key                 : 発注番号 / field 数値'

if ($missingVendors.Count -gt 0 -or $missingStaff.Count -gt 0) {
    foreach ($x in $missingVendors) { Write-Host "  [MISSING VENDOR] $x" }
    foreach ($x in $missingStaff) { Write-Host "  [MISSING STAFF] $x" }

    throw 'SafetyStop: dropdown option missing. Run Prepare-App292-NoJoinMasterOptions_v0.2.ps1 first.'
}

if ($Execute -and $existing.Count -ne 0) {
    throw ("SafetyStop: Execute requires empty App292. Current={0}. " +
           "Recovery: run Clear-App292-TestData_v0.2.ps1 -DryRun, then -Execute " +
           "-ConfirmExecute 'APP292-CLEAR-ALL', verify 0 records, and rerun this import from the beginning.") -f $existing.Count
}

if (-not $Execute) {
    Write-Host 'WRITE               : NONE'
    Write-Host 'RESULT              : DRYRUN PASS'
    exit 0
}

$payloads = New-Object System.Collections.Generic.List[object]

foreach ($x in $normalized) {
    $record = [ordered]@{
        '数値' = @{ value = $x.PO }
        'ODERNO' = @{ value = $x.OrderNo }
        '数値_44' = @{ value = $x.SlipNo }
        '数値_45' = @{ value = $x.SlipLine }
        '商品名' = @{ value = $x.Product }
        '数値_10' = @{ value = $x.Qty }
        '日付' = @{ value = $x.OrderDate }
        '日付_1' = @{ value = $x.DueDate }
        'VENDOR_TEXT' = @{ value = $x.Vendor }
        'VENDOR_DD' = @{ value = $x.Vendor }
        'STAFF_TEXT' = @{ value = $x.Staff }
        'STAFF_DD' = @{ value = $x.Staff }
        'K加工着手日' = @{ value = $x.OrderDate }
        'K完成検査日' = @{ value = $x.DueDate }
        'K日程種別' = @{ value = 'Batch' }
        'K暫定設定' = @{ value = 'ON' }
        'K計算根拠' = @{ value = '' }
    }

    $payloads.Add($record)
}

for ($i = 0; $i -lt $payloads.Count; $i += 100) {
    $end = [Math]::Min($i + 99, $payloads.Count - 1)
    $batch = @($payloads[$i..$end])

    Write-Host ("POST {0}-{1} / {2}" -f ($i + 1), ($end + 1), $payloads.Count)

    [void](Add-RecordBatch -Token $token -Records $batch)
}

# 途中バッチで失敗した場合、本v0.2は部分再開しない。
# App292を全削除→0件確認→937件を先頭から再実行する。

$after = @(Get-AllRecords -Token $token)

if ($after.Count -ne $ExpectedCsvRows) {
    throw "PostVerifyFailed: App292 count=$($after.Count) expected=$ExpectedCsvRows"
}

$byPo = @{}

foreach ($r in $after) {
    $po = Get-KValue $r '数値'

    if ($byPo.ContainsKey($po)) {
        throw "PostVerifyFailed: duplicate App292 PO=$po"
    }

    $byPo[$po] = $r
}

$verifyErrors = New-Object System.Collections.Generic.List[string]

foreach ($x in $normalized) {
    if (-not $byPo.ContainsKey($x.PO)) {
        $verifyErrors.Add("Missing PO=$($x.PO)")
        continue
    }

    $r = $byPo[$x.PO]

    $pairs = @(
        @('ODERNO', $x.OrderNo),
        @('数値_44', $x.SlipNo),
        @('数値_45', $x.SlipLine),
        @('商品名', $x.Product),
        @('数値_10', $x.Qty),
        @('日付', $x.OrderDate),
        @('日付_1', $x.DueDate),
        @('VENDOR_TEXT', $x.Vendor),
        @('VENDOR_DD', $x.Vendor),
        @('STAFF_TEXT', $x.Staff),
        @('STAFF_DD', $x.Staff),
        @('K加工着手日', $x.OrderDate),
        @('K完成検査日', $x.DueDate),
        @('K日程種別', 'Batch'),
        @('K暫定設定', 'ON'),
        @('K計算根拠', '')
    )

    foreach ($pair in $pairs) {
        $actual = Get-KValue $r $pair[0]
        $expected = [string]$pair[1]

        if ($actual -ne $expected) {
            $verifyErrors.Add(
                "PO=$($x.PO) Field=$($pair[0]) Expected='$expected' Actual='$actual'"
            )
        }
    }
}

if ($verifyErrors.Count -gt 0) {
    $verifyErrors | Select-Object -First 50 | ForEach-Object { Write-Error $_ }
    throw "PostVerifyFailed: mismatches=$($verifyErrors.Count)"
}

Write-Host "PostVerify App292    : $($after.Count)"
Write-Host 'PostVerifyMismatch   : 0'
Write-Host 'RESULT               : PASS'
