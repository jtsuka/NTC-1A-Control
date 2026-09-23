#requires -Version 5.1
<#
.SYNOPSIS
  App272 受入実績突合 DryRun v0.1

.DESCRIPTION
  本番 App272(PROD) を READ ONLY で参照し、
  受入実績Excelの受入日候補と突合して分類CSV/TXTを出力する。

  重要:
  - kintoneへの書込みは一切行わない。
  - Invoke-RestMethod は GET のみ。
  - 受入実績に行がないことを「未受入」とは判定しない。
  - 基幹システムの受入入力漏れが一定数あり得るため、
    本処理は受入実績100%捕捉を保証しない。

.NOTES
  Windows PowerShell 5.1
  Excel COMを使用するため Microsoft Excel が必要。
#>

[CmdletBinding()]
param(
    [string]$InputXlsx = 'C:\HPDB\受け入れ情報3か月.xlsx',
    [string]$BaseUrl = 'https://bcurbkixz609.cybozu.com',
    [int]$AppId = 272,
    [string]$TokenPath = 'C:\HPDB\Secrets\App272_Prod.token',
    [string]$OutputDirectory = 'C:\HPDB\Logs',
    [ValidateRange(1,200000)][int]$MaxSourceRows = 100000,
    [ValidateRange(1,10000)][int]$MaxAppRows = 5000
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

# ----------------------------------------------------------------------
# Safety
# ----------------------------------------------------------------------
if ($AppId -ne 272) {
    throw 'SafetyStop: AppId must be 272(PROD).'
}
if (-not (Test-Path -LiteralPath $InputXlsx -PathType Leaf)) {
    throw "Input Excel not found: $InputXlsx"
}
if (-not (Test-Path -LiteralPath $OutputDirectory)) {
    New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
}

$timestamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$detailPath = Join-Path $OutputDirectory ("App272_AcceptanceDryRun_{0}.csv" -f $timestamp)
$summaryPath = Join-Path $OutputDirectory ("App272_AcceptanceDryRun_{0}_summary.txt" -f $timestamp)

Write-Host '============================================================'
Write-Host 'App272 受入実績突合 DryRun v0.1'
Write-Host '============================================================'
Write-Host 'Mode       : READ ONLY / DRYRUN'
Write-Host "AppId      : $AppId (PROD)"
Write-Host "Input      : $InputXlsx"
Write-Host 'IMPORTANT  : 基幹未入力の受入は本処理では検知できません。'
Write-Host '             受入実績なし = 未受入 とは判定しません。'
Write-Host 'WRITES     : 0'
Write-Host ''

# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------
function Get-ApiToken {
    param([string]$Path)

    if ($Path -and (Test-Path -LiteralPath $Path -PathType Leaf)) {
        $token = ([System.IO.File]::ReadAllText($Path)).Trim()
        if ([string]::IsNullOrWhiteSpace($token)) {
            throw "API token file is empty: $Path"
        }
        return $token
    }

    $secure = Read-Host 'App272 READ用 API Token' -AsSecureString
    $bstr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($secure)
    try {
        return [Runtime.InteropServices.Marshal]::PtrToStringBSTR($bstr)
    }
    finally {
        if ($bstr -ne [IntPtr]::Zero) {
            [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($bstr)
        }
    }
}

function Convert-ToDateText {
    param($Value)

    if ($null -eq $Value) { return $null }

    if ($Value -is [datetime]) {
        return $Value.ToString('yyyy-MM-dd')
    }

    if ($Value -is [double] -or $Value -is [int] -or $Value -is [decimal]) {
        try {
            return ([datetime]::FromOADate([double]$Value)).ToString('yyyy-MM-dd')
        } catch {
            return $null
        }
    }

    $s = ([string]$Value).Trim()
    if ([string]::IsNullOrWhiteSpace($s)) { return $null }

    $dt = [datetime]::MinValue
    $formats = @('yyyy/MM/dd','yyyy-MM-dd','yyyy/M/d','yyyy-M-d','MM/dd/yyyy')
    foreach ($fmt in $formats) {
        if ([datetime]::TryParseExact(
            $s,
            $fmt,
            [Globalization.CultureInfo]::InvariantCulture,
            [Globalization.DateTimeStyles]::None,
            [ref]$dt
        )) {
            return $dt.ToString('yyyy-MM-dd')
        }
    }

    if ([datetime]::TryParse($s, [ref]$dt)) {
        return $dt.ToString('yyyy-MM-dd')
    }

    return $null
}

function Convert-ToNullableDecimal {
    param($Value)
    if ($null -eq $Value) { return $null }

    $s = ([string]$Value).Trim()
    if ([string]::IsNullOrWhiteSpace($s)) { return $null }

    $d = [decimal]0
    if ([decimal]::TryParse(
        $s,
        [Globalization.NumberStyles]::Any,
        [Globalization.CultureInfo]::InvariantCulture,
        [ref]$d
    )) { return $d }

    if ([decimal]::TryParse($s, [ref]$d)) { return $d }
    return $null
}

function Join-Unique {
    param([object[]]$Values)
    $items = @(
        $Values |
        ForEach-Object { if ($null -ne $_) { ([string]$_).Trim() } } |
        Where-Object { -not [string]::IsNullOrWhiteSpace($_) } |
        Sort-Object -Unique
    )
    return ($items -join '|')
}

function Read-AcceptanceExcel {
    param(
        [string]$Path,
        [int]$MaxRows
    )

    $excel = $null
    $book = $null
    $sheet = $null
    try {
        $excel = New-Object -ComObject Excel.Application
        $excel.Visible = $false
        $excel.DisplayAlerts = $false

        $book = $excel.Workbooks.Open($Path, 0, $true)
        $sheet = $book.Worksheets.Item(1)
        $used = $sheet.UsedRange

        $rowCount = [int]$used.Rows.Count
        $colCount = [int]$used.Columns.Count

        if ($rowCount -lt 2) {
            throw 'Input Excel has no data rows.'
        }
        if (($rowCount - 1) -gt $MaxRows) {
            throw "SafetyStop: source rows=$($rowCount-1) exceeds MaxSourceRows=$MaxRows"
        }

        $values = $used.Value2
        $headerMap = @{}
        for ($c = 1; $c -le $colCount; $c++) {
            $h = ([string]$values[1,$c]).Trim()
            if (-not [string]::IsNullOrWhiteSpace($h) -and -not $headerMap.ContainsKey($h)) {
                $headerMap[$h] = $c
            }
        }

        foreach ($required in @('発注番号','受入日')) {
            if (-not $headerMap.ContainsKey($required)) {
                throw "SafetyStop: required column missing: $required"
            }
        }

        $optional = @(
            '受入数','受入番号','検査合格数','受入検収日',
            '手配書番号','注文伝票番号','注文伝票行番号','伝票区分'
        )

        $rows = New-Object System.Collections.Generic.List[object]
        $invalid = New-Object System.Collections.Generic.List[object]

        for ($r = 2; $r -le $rowCount; $r++) {
            $po = ([string]$values[$r,$headerMap['発注番号']]).Trim()
            $dateRaw = $values[$r,$headerMap['受入日']]
            $dateText = Convert-ToDateText $dateRaw

            if ([string]::IsNullOrWhiteSpace($po) -or [string]::IsNullOrWhiteSpace($dateText)) {
                $invalid.Add([pscustomobject]@{
                    ExcelRow = $r
                    OrderNo = $po
                    ReceiptDateRaw = [string]$dateRaw
                    Reason = if ([string]::IsNullOrWhiteSpace($po)) {
                        'BLANK_ORDER_NO'
                    } else {
                        'INVALID_OR_BLANK_RECEIPT_DATE'
                    }
                })
                continue
            }

            $obj = [ordered]@{
                ExcelRow = $r
                OrderNo = $po
                ReceiptDate = $dateText
                ReceiptQty = $null
                ReceiptNo = ''
                PassedQty = $null
                AcceptanceInspectionDate = ''
                SourceOrderNo = ''
                SourceOrderSlipNo = ''
                SourceOrderSlipLineNo = ''
                SlipType = ''
            }

            if ($headerMap.ContainsKey('受入数')) {
                $obj.ReceiptQty = Convert-ToNullableDecimal $values[$r,$headerMap['受入数']]
            }
            if ($headerMap.ContainsKey('受入番号')) {
                $obj.ReceiptNo = ([string]$values[$r,$headerMap['受入番号']]).Trim()
            }
            if ($headerMap.ContainsKey('検査合格数')) {
                $obj.PassedQty = Convert-ToNullableDecimal $values[$r,$headerMap['検査合格数']]
            }
            if ($headerMap.ContainsKey('受入検収日')) {
                $obj.AcceptanceInspectionDate = Convert-ToDateText $values[$r,$headerMap['受入検収日']]
            }
            if ($headerMap.ContainsKey('手配書番号')) {
                $obj.SourceOrderNo = ([string]$values[$r,$headerMap['手配書番号']]).Trim()
            }
            if ($headerMap.ContainsKey('注文伝票番号')) {
                $obj.SourceOrderSlipNo = ([string]$values[$r,$headerMap['注文伝票番号']]).Trim()
            }
            if ($headerMap.ContainsKey('注文伝票行番号')) {
                $obj.SourceOrderSlipLineNo = ([string]$values[$r,$headerMap['注文伝票行番号']]).Trim()
            }
            if ($headerMap.ContainsKey('伝票区分')) {
                $obj.SlipType = ([string]$values[$r,$headerMap['伝票区分']]).Trim()
            }

            $rows.Add([pscustomobject]$obj)
        }

        return [pscustomobject]@{
            Rows = @($rows)
            InvalidRows = @($invalid)
            InputRows = $rowCount - 1
            ColumnCount = $colCount
            HeaderMap = $headerMap
        }
    }
    finally {
        if ($book) { $book.Close($false) | Out-Null }
        if ($excel) { $excel.Quit() }

        foreach ($com in @($sheet,$book,$excel)) {
            if ($com) {
                try { [void][Runtime.InteropServices.Marshal]::ReleaseComObject($com) } catch {}
            }
        }
        [GC]::Collect()
        [GC]::WaitForPendingFinalizers()
    }
}

function Get-AppFormProperties {
    param(
        [string]$Base,
        [int]$App,
        [hashtable]$Headers
    )

    $uri = '{0}/k/v1/app/form/fields.json?app={1}' -f $Base.TrimEnd('/'), $App
    return Invoke-RestMethod -Method Get -Uri $uri -Headers $Headers
}

function Get-App272Records {
    param(
        [string]$Base,
        [int]$App,
        [hashtable]$Headers,
        [string[]]$FieldCodes,
        [int]$MaxRows
    )

    $all = New-Object System.Collections.Generic.List[object]
    $offset = 0
    $limit = 500

    while ($true) {
        $parts = New-Object System.Collections.Generic.List[string]
        $parts.Add('app=' + [uri]::EscapeDataString([string]$App))
        for ($i=0; $i -lt $FieldCodes.Count; $i++) {
            $parts.Add(
                ('fields[{0}]={1}' -f $i, [uri]::EscapeDataString($FieldCodes[$i]))
            )
        }

        $query = 'order by $id asc limit {0} offset {1}' -f $limit,$offset
        $parts.Add('query=' + [uri]::EscapeDataString($query))

        $uri = '{0}/k/v1/records.json?{1}' -f $Base.TrimEnd('/'), ($parts -join '&')
        $resp = Invoke-RestMethod -Method Get -Uri $uri -Headers $Headers
        $batch = @($resp.records)

        foreach ($record in $batch) {
            $all.Add($record)
            if ($all.Count -gt $MaxRows) {
                throw "SafetyStop: App272 rows exceed MaxAppRows=$MaxRows"
            }
        }

        if ($batch.Count -lt $limit) { break }
        $offset += $limit
    }

    return @($all)
}

function Get-FieldValue {
    param($Record,[string]$Code)
    if ($null -eq $Record) { return $null }
    if (-not ($Record.PSObject.Properties.Name -contains $Code)) { return $null }
    $field = $Record.$Code
    if ($null -eq $field) { return $null }
    return $field.value
}

# ----------------------------------------------------------------------
# Read source Excel
# ----------------------------------------------------------------------
Write-Host '[1/6] 受入実績Excelを読込中...'
$source = Read-AcceptanceExcel -Path $InputXlsx -MaxRows $MaxSourceRows
$sourceRows = @($source.Rows)
$invalidRows = @($source.InvalidRows)

if ($sourceRows.Count -eq 0) {
    throw 'SafetyStop: valid source receipt rows = 0'
}

Write-Host ("      InputRows       : {0}" -f $source.InputRows)
Write-Host ("      ValidSourceRows : {0}" -f $sourceRows.Count)
Write-Host ("      InvalidRows     : {0}" -f $invalidRows.Count)

# ----------------------------------------------------------------------
# Group source rows
# ----------------------------------------------------------------------
Write-Host '[2/6] 発注番号単位へ集約中...'
$sourceGroups = @{}
foreach ($g in ($sourceRows | Group-Object OrderNo)) {
    $rows = @($g.Group)
    $dates = @($rows | Select-Object -ExpandProperty ReceiptDate | Sort-Object -Unique)
    $candidate = $dates[-1]

    $qtyValues = @($rows | Where-Object { $null -ne $_.ReceiptQty } | Select-Object -ExpandProperty ReceiptQty)
    $qtySum = $null
    if ($qtyValues.Count -gt 0) {
        $qtySum = [decimal]0
        foreach ($q in $qtyValues) { $qtySum += [decimal]$q }
    }

    $negativeCount = @($rows | Where-Object { $null -ne $_.ReceiptQty -and $_.ReceiptQty -lt 0 }).Count

    $sourceGroups[$g.Name] = [pscustomobject]@{
        OrderNo = $g.Name
        Rows = $rows
        SourceRowCount = $rows.Count
        DistinctReceiptDateCount = $dates.Count
        FirstReceiptDate = $dates[0]
        LastReceiptDate = $dates[-1]
        CandidateReceiptDate = $candidate
        ReceiptQtySum = $qtySum
        NegativeQtyRowCount = $negativeCount
        ReceiptNumbers = Join-Unique ($rows | Select-Object -ExpandProperty ReceiptNo)
        SourceOrderNos = Join-Unique ($rows | Select-Object -ExpandProperty SourceOrderNo)
        SourceOrderSlipNos = Join-Unique ($rows | Select-Object -ExpandProperty SourceOrderSlipNo)
        SourceSlipTypes = Join-Unique ($rows | Select-Object -ExpandProperty SlipType)
    }
}
Write-Host ("      UniqueSourcePO   : {0}" -f $sourceGroups.Count)

# ----------------------------------------------------------------------
# kintone READ ONLY
# ----------------------------------------------------------------------
Write-Host '[3/6] App272フォーム定義をGET中...'
$token = Get-ApiToken -Path $TokenPath
$headers = @{ 'X-Cybozu-API-Token' = $token }

$form = Get-AppFormProperties -Base $BaseUrl -App $AppId -Headers $headers
$props = $form.properties

if (-not $props.'数値') {
    throw 'SafetyStop: App272 field code 数値 (発注番号) not found.'
}

$acceptanceFieldStatus = 'MISSING'
if ($props.'受入れ日') {
    if ($props.'受入れ日'.type -eq 'DATE') {
        $acceptanceFieldStatus = 'DATE'
    } else {
        $acceptanceFieldStatus = 'WRONG_TYPE:' + [string]$props.'受入れ日'.type
    }
}

$fieldCodes = @('$id','$revision','数値')
if ($props.'受入れ日') { $fieldCodes += '受入れ日' }
if ($props.'ODERNO') { $fieldCodes += 'ODERNO' }
if ($props.'数値_44') { $fieldCodes += '数値_44' }

Write-Host ("      AcceptanceField : {0}" -f $acceptanceFieldStatus)

Write-Host '[4/6] App272(PROD)レコードをGET中...'
$appRecords = @(Get-App272Records `
    -Base $BaseUrl `
    -App $AppId `
    -Headers $headers `
    -FieldCodes $fieldCodes `
    -MaxRows $MaxAppRows)

Write-Host ("      App272Rows       : {0}" -f $appRecords.Count)

# token should not be retained longer than necessary
$token = $null
$headers = $null

# ----------------------------------------------------------------------
# Build App map + duplicate safety
# ----------------------------------------------------------------------
$appMap = @{}
$duplicateKeys = New-Object System.Collections.Generic.List[string]

foreach ($r in $appRecords) {
    $po = ([string](Get-FieldValue $r '数値')).Trim()
    if ([string]::IsNullOrWhiteSpace($po)) { continue }

    if ($appMap.ContainsKey($po)) {
        $duplicateKeys.Add($po)
        continue
    }

    $appMap[$po] = $r
}

if ($duplicateKeys.Count -gt 0) {
    $dups = ($duplicateKeys | Sort-Object -Unique) -join ', '
    throw "SafetyStop: App272 duplicate order number(s): $dups"
}

# ----------------------------------------------------------------------
# Compare
# ----------------------------------------------------------------------
Write-Host '[5/6] 突合・分類中...'
$results = New-Object System.Collections.Generic.List[object]

# App272-centered rows
foreach ($po in ($appMap.Keys | Sort-Object)) {
    $app = $appMap[$po]
    $appAcceptance = ''
    if ($props.'受入れ日') {
        $appAcceptance = ([string](Get-FieldValue $app '受入れ日')).Trim()
    }

    $common = [ordered]@{
        Class = ''
        FutureAutoEligible = 'NO'
        OrderNo = $po
        AppRecordId = [string](Get-FieldValue $app '$id')
        AppRevision = [string](Get-FieldValue $app '$revision')
        AppAcceptanceDate = $appAcceptance
        CandidateReceiptDate = ''
        SourceRowCount = 0
        DistinctReceiptDateCount = 0
        FirstReceiptDate = ''
        LastReceiptDate = ''
        ReceiptQtySum = ''
        NegativeQtyRowCount = 0
        AppOrderNo = if ($props.'ODERNO') { [string](Get-FieldValue $app 'ODERNO') } else { '' }
        SourceOrderNos = ''
        AppOrderSlipNo = if ($props.'数値_44') { [string](Get-FieldValue $app '数値_44') } else { '' }
        SourceOrderSlipNos = ''
        ReceiptNumbers = ''
        AcceptanceFieldStatus = $acceptanceFieldStatus
        Note = ''
    }

    if (-not $sourceGroups.ContainsKey($po)) {
        $common.Class = 'NO_SOURCE_DATA_UNKNOWN'
        $common.Note = '受入実績ファイルに該当なし。未受入とは判定しない。抽出期間外・基幹未入力等を含み得る。'
        $results.Add([pscustomobject]$common)
        continue
    }

    $sg = $sourceGroups[$po]
    $common.CandidateReceiptDate = $sg.CandidateReceiptDate
    $common.SourceRowCount = $sg.SourceRowCount
    $common.DistinctReceiptDateCount = $sg.DistinctReceiptDateCount
    $common.FirstReceiptDate = $sg.FirstReceiptDate
    $common.LastReceiptDate = $sg.LastReceiptDate
    $common.ReceiptQtySum = $sg.ReceiptQtySum
    $common.NegativeQtyRowCount = $sg.NegativeQtyRowCount
    $common.SourceOrderNos = $sg.SourceOrderNos
    $common.SourceOrderSlipNos = $sg.SourceOrderSlipNos
    $common.ReceiptNumbers = $sg.ReceiptNumbers

    if ($sg.NegativeQtyRowCount -gt 0) {
        $common.Class = 'SOURCE_REVIEW_REQUIRED'
        $common.FutureAutoEligible = 'NO'
        $common.Note = '受入数が負数の行を含むため、返品/訂正/取消等の業務意味確認が必要。'
    }
    elseif (-not [string]::IsNullOrWhiteSpace($appAcceptance)) {
        if ($appAcceptance -eq $sg.CandidateReceiptDate) {
            $common.Class = 'ALREADY_SAME'
            $common.FutureAutoEligible = 'NO'
            $common.Note = 'App272既存値と候補日が一致。変更不要。'
        }
        else {
            $common.Class = 'EXISTING_DIFFERENT'
            $common.FutureAutoEligible = 'NO'
            $common.Note = '既存受入れ日を自動上書きしない。人確認対象。'
        }
    }
    elseif ($sg.SourceRowCount -gt 1 -or $sg.DistinctReceiptDateCount -gt 1) {
        $common.Class = 'CANDIDATE_EMPTY_MULTI'
        $common.FutureAutoEligible = if ($acceptanceFieldStatus -eq 'DATE') { 'REVIEW' } else { 'NO' }
        $common.Note = '複数受入。v0.1候補日は最新日。正式ルール確定待ち。'
    }
    else {
        $common.Class = 'CANDIDATE_EMPTY_SINGLE'
        $common.FutureAutoEligible = if ($acceptanceFieldStatus -eq 'DATE') { 'YES' } else { 'NO' }
        $common.Note = '単一受入・App272空欄。将来自動更新候補。'
    }

    $results.Add([pscustomobject]$common)
}

# Source-only rows
foreach ($po in ($sourceGroups.Keys | Sort-Object)) {
    if ($appMap.ContainsKey($po)) { continue }

    $sg = $sourceGroups[$po]
    $results.Add([pscustomobject][ordered]@{
        Class = 'SOURCE_ONLY_NO_APP272'
        FutureAutoEligible = 'NO'
        OrderNo = $po
        AppRecordId = ''
        AppRevision = ''
        AppAcceptanceDate = ''
        CandidateReceiptDate = $sg.CandidateReceiptDate
        SourceRowCount = $sg.SourceRowCount
        DistinctReceiptDateCount = $sg.DistinctReceiptDateCount
        FirstReceiptDate = $sg.FirstReceiptDate
        LastReceiptDate = $sg.LastReceiptDate
        ReceiptQtySum = $sg.ReceiptQtySum
        NegativeQtyRowCount = $sg.NegativeQtyRowCount
        AppOrderNo = ''
        SourceOrderNos = $sg.SourceOrderNos
        AppOrderSlipNo = ''
        SourceOrderSlipNos = $sg.SourceOrderSlipNos
        ReceiptNumbers = $sg.ReceiptNumbers
        AcceptanceFieldStatus = $acceptanceFieldStatus
        Note = '受入実績には存在するがApp272に該当発注番号なし。書込み対象外。'
    })
}

# ----------------------------------------------------------------------
# Output
# ----------------------------------------------------------------------
Write-Host '[6/6] DryRun結果を出力中...'

$results |
    Sort-Object Class,OrderNo |
    Export-Csv -LiteralPath $detailPath -NoTypeInformation -Encoding UTF8

$classCounts = @{}
foreach ($r in $results) {
    if (-not $classCounts.ContainsKey($r.Class)) { $classCounts[$r.Class] = 0 }
    $classCounts[$r.Class]++
}

function Count-Class([string]$Name) {
    if ($classCounts.ContainsKey($Name)) { return $classCounts[$Name] }
    return 0
}

$futureYes = @($results | Where-Object { $_.FutureAutoEligible -eq 'YES' }).Count
$futureReview = @($results | Where-Object { $_.FutureAutoEligible -eq 'REVIEW' }).Count
$negativeGroups = @($results | Where-Object { $_.NegativeQtyRowCount -gt 0 }).Count
$multiGroups = @($results | Where-Object {
    $_.SourceRowCount -gt 1 -or $_.DistinctReceiptDateCount -gt 1
}).Count

$summary = @(
    'App272 Acceptance DryRun v0.1'
    ('Timestamp=' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'))
    'Mode=READ_ONLY'
    'AppId=272'
    'Environment=PROD'
    'WRITES=0'
    ''
    ('InputRows=' + $source.InputRows)
    ('ValidSourceRows=' + $sourceRows.Count)
    ('InvalidSourceRows=' + $invalidRows.Count)
    ('UniqueSourcePO=' + $sourceGroups.Count)
    ('App272Rows=' + $appRecords.Count)
    ('UniqueApp272PO=' + $appMap.Count)
    ('AcceptanceFieldStatus=' + $acceptanceFieldStatus)
    ''
    ('CANDIDATE_EMPTY_SINGLE=' + (Count-Class 'CANDIDATE_EMPTY_SINGLE'))
    ('CANDIDATE_EMPTY_MULTI=' + (Count-Class 'CANDIDATE_EMPTY_MULTI'))
    ('ALREADY_SAME=' + (Count-Class 'ALREADY_SAME'))
    ('EXISTING_DIFFERENT=' + (Count-Class 'EXISTING_DIFFERENT'))
    ('SOURCE_REVIEW_REQUIRED=' + (Count-Class 'SOURCE_REVIEW_REQUIRED'))
    ('SOURCE_ONLY_NO_APP272=' + (Count-Class 'SOURCE_ONLY_NO_APP272'))
    ('NO_SOURCE_DATA_UNKNOWN=' + (Count-Class 'NO_SOURCE_DATA_UNKNOWN'))
    ''
    ('FutureAutoEligibleYES=' + $futureYes)
    ('FutureAutoEligibleREVIEW=' + $futureReview)
    ('NegativeQtyGroups=' + $negativeGroups)
    ('MultiReceiptGroups=' + $multiGroups)
    ''
    'IMPORTANT=受入実績なしは未受入を意味しない。基幹未入力を含み得る。'
    'RESULT=DRYRUN_ONLY'
    'WRITES=0'
)

[System.IO.File]::WriteAllLines(
    $summaryPath,
    $summary,
    (New-Object System.Text.UTF8Encoding($true))
)

Write-Host ''
Write-Host '============================================================'
Write-Host 'DRYRUN COMPLETED'
Write-Host '============================================================'
Write-Host "Detail : $detailPath"
Write-Host "Summary: $summaryPath"
Write-Host 'RESULT : DRYRUN_ONLY'
Write-Host 'WRITES : 0'
Write-Host ''
Write-Host '※ 受入実績なし = 未受入 とは判定していません。'
Write-Host '※ 基幹未入力の受入は、本処理だけでは検知できません。'
