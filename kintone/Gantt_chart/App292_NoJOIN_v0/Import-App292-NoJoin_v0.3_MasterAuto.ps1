#requires -Version 5.1
<#
.SYNOPSIS
  App292 NoJOIN 全937件取込総合試験 v0.3 MasterAuto

.DESCRIPTION
  Windows PowerShell 5.1用。
  NoJOIN想定CSV 937件を App292(TEST) へ直接POSTする。
  App270およびルックアップは使用しない。主キーは発注番号（field code: 数値）。

  v0.3では、当初仕様どおり加工先別日程暫定シートを取込時から適用する。
  日程計算ロジックは Import-App272_Phase1C_New_v0.7.4_MasterAuto.ps1 の
  Import-ScheduleMasterExcel / Import-CompanyCalendar / Import-HolidaySet /
  Get-DayState / Move-ToWorkingDay / Calculate-ScheduleCompat / Get-MasterKey
  を流用する。

  判定:
    使用       -> マスタ前倒し日数で計算し、K日程へ書込
    要確認     -> 計算値はログに出すが、K日程はベースライン（手配日/納期）
    使用しない -> ベースライン
    マスタなし -> ベースライン
    ShortLeadTime -> ベースラインへフォールバック
    不正使用区分 / 不正日数 / InvalidDateRelation -> SafetyStop

  K計算根拠:
    計算適用時のみ Company / Provisional。
    ベースライン時は、計算根拠を偽装しないため空欄。

  既定はDryRun。
#>

[CmdletBinding()]
param(
    [string]$CsvPath = 'C:\HPDB\外注課納期管理_NoJOIN想定_全937件_App292_TEST_20260921.csv',
    [string]$ScheduleMasterPath = 'C:\HPDB\加工先別_日程暫定シート_20260910.xlsx',
    [string]$CalendarPath = 'C:\HPDB\カレンダー(2023).xlsx',
    [string]$JapanHolidayPath = 'C:\HPDB\JapanHolidays_2026_2027.csv',
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
if ($Execute -and $ConfirmExecute -ne 'APP292-NOJOIN-937-MASTERAUTO-EXECUTE') {
    throw "SafetyStop: Execute requires -ConfirmExecute 'APP292-NOJOIN-937-MASTERAUTO-EXECUTE'"
}

foreach ($requiredPath in @($CsvPath,$ScheduleMasterPath,$CalendarPath,$JapanHolidayPath,$TokenPath)) {
    if (-not (Test-Path -LiteralPath $requiredPath -PathType Leaf)) {
        throw "SafetyStop: required file not found: $requiredPath"
    }
}
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
    $enc = (Get-Content -LiteralPath $Path -Raw).Trim()
    if ([string]::IsNullOrWhiteSpace($enc)) { throw 'SafetyStop: encrypted token file is blank.' }
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
        'yyyy-MM-dd HH:mm:ss','yyyy/M/d HH:mm:ss','yyyy/MM/dd HH:mm:ss',
        'yyyy-MM-dd','yyyy/M/d','yyyy/MM/dd'
    )
    foreach ($fmt in $formats) {
        $dt = [datetime]::MinValue
        if ([datetime]::TryParseExact(
            $v,$fmt,[Globalization.CultureInfo]::InvariantCulture,
            [Globalization.DateTimeStyles]::None,[ref]$dt
        )) {
            return $dt.ToString('yyyy-MM-dd')
        }
    }
    throw "Invalid date: '$Value'"
}

# Phase1C v0.7.4 の Import-HolidaySet が参照する名前を維持する。
function Normalize-DateStrict {
    param([AllowNull()][AllowEmptyString()][string]$Value)
    $result = Normalize-Date $Value
    if ([string]::IsNullOrWhiteSpace($result)) { return $null }
    return $result
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
            $BaseUrl.TrimEnd('/'),$AppId,[uri]::EscapeDataString($query)
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
    return Invoke-RestMethod -Method Get -Uri $uri -Headers $headers -Body @{ app=$AppId; lang='ja' }
}

function Test-OptionExists {
    param($Form,[string]$Code,[string]$Value)
    $p = $Form.properties.PSObject.Properties[$Code]
    if ($null -eq $p) { return $false }
    return ($null -ne $p.Value.options.PSObject.Properties[$Value])
}

function Add-RecordBatch {
    param([string]$Token,[object[]]$Records)
    if ($Records.Count -lt 1 -or $Records.Count -gt 100) {
        throw "Invalid POST batch count=$($Records.Count)"
    }
    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $uri = $BaseUrl.TrimEnd('/') + '/k/v1/records.json'
    $body = [ordered]@{ app=$AppId; records=$Records } | ConvertTo-Json -Depth 20
    return Invoke-RestMethod -Method Post -Uri $uri -Headers $headers `
        -ContentType 'application/json; charset=utf-8' `
        -Body ([Text.Encoding]::UTF8.GetBytes($body))
}

function Import-CompanyCalendar {
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path)) {
        throw "会社カレンダーが見つかりません: $Path"
    }

    $excel = $null
    $book = $null
    $sheet = $null
    $map = @{}
    try {
        $excel = New-Object -ComObject Excel.Application
        $excel.Visible = $false
        $excel.DisplayAlerts = $false
        $book = $excel.Workbooks.Open($Path, $null, $true)
        $sheet = $book.Worksheets.Item('カレンダー')
        $used = $sheet.UsedRange
        $lastRow = $used.Rows.Count

        for ($r = 2; $r -le $lastRow; $r++) {
            $rawDate = $sheet.Cells.Item($r, 2).Value2
            $status = [string]$sheet.Cells.Item($r, 4).Text
            if ($null -eq $rawDate -or [string]::IsNullOrWhiteSpace($status)) { continue }

            $dt = $null
            if ($rawDate -is [double] -or $rawDate -is [int]) {
                $dt = [datetime]::FromOADate([double]$rawDate)
            } else {
                $tmp = New-Object datetime
                if ([datetime]::TryParse([string]$rawDate, [ref]$tmp)) { $dt = $tmp }
            }
            if ($null -eq $dt) { continue }
            $map[$dt.ToString('yyyy-MM-dd')] = $status.Trim()
        }
    }
    finally {
        if ($null -ne $book) { $book.Close($false) | Out-Null }
        if ($null -ne $excel) { $excel.Quit() }
        foreach ($obj in @($sheet,$book,$excel)) {
            if ($null -ne $obj) {
                try { [void][Runtime.InteropServices.Marshal]::ReleaseComObject($obj) } catch {}
            }
        }
        [gc]::Collect()
        [gc]::WaitForPendingFinalizers()
    }

    if ($map.Count -eq 0) { throw '会社カレンダーから営業日データを取得できませんでした。' }
    return $map
}

function Import-ScheduleMasterExcel {
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path)) {
        throw "日程マスタExcelが見つかりません: $Path"
    }

    $ext = [System.IO.Path]::GetExtension($Path).ToLowerInvariant()
    if ($ext -notin @('.xlsx','.xlsm','.xls')) {
        throw "日程マスタはExcelファイルを指定してください (.xlsx/.xlsm/.xls): $Path"
    }

    $excel = $null
    $book = $null
    $sheet = $null
    $used = $null
    $rows = @()

    try {
        $excel = New-Object -ComObject Excel.Application
        $excel.Visible = $false
        $excel.DisplayAlerts = $false
        $book = $excel.Workbooks.Open($Path, $null, $true)
        $sheet = $book.Worksheets.Item('加工先別設定')
        $used = $sheet.UsedRange
        $lastRow = $used.Rows.Count

        $headerRow = 0
        for ($r = 1; $r -le [Math]::Min($lastRow, 20); $r++) {
            $headerA = (([string]$sheet.Cells.Item($r, 1).Text) -replace '\s','').Trim()
            if ($headerA -eq '加工先（指示先名）') {
                $headerRow = $r
                break
            }
        }
        if ($headerRow -eq 0) {
            throw "日程マスタExcelに見出し「加工先（指示先名）」が見つかりません。"
        }

        $expectedHeaders = @{
            1 = '加工先（指示先名）'
            3 = '完成検査日（納期の○日前）'
            4 = '加工着手日（完成検査日の○日前）'
            5 = '標準ルールを使用'
            6 = '備考'
        }
        foreach ($col in $expectedHeaders.Keys) {
            $actual = (([string]$sheet.Cells.Item($headerRow, [int]$col).Text) -replace '\s','').Trim()
            if ($actual -ne $expectedHeaders[$col]) {
                throw "日程マスタExcelの列見出しが想定外です。列=$col 期待='$($expectedHeaders[$col])' 実際='$actual'"
            }
        }

        $dataStarted = $false
        for ($r = $headerRow + 1; $r -le $lastRow; $r++) {
            $vendor = ([string]$sheet.Cells.Item($r, 1).Text).Trim()

            if ([string]::IsNullOrWhiteSpace($vendor)) {
                if ($dataStarted) { break }
                continue
            }

            $dataStarted = $true
            $rows += [pscustomobject]@{
                指示先名 = $vendor
                完成検査日前倒し日数 = ([string]$sheet.Cells.Item($r, 3).Text).Trim()
                加工着手日前倒し日数 = ([string]$sheet.Cells.Item($r, 4).Text).Trim()
                使用区分 = ([string]$sheet.Cells.Item($r, 5).Text).Trim()
                備考 = ([string]$sheet.Cells.Item($r, 6).Text).Trim()
            }
        }
    }
    finally {
        if ($null -ne $book) { $book.Close($false) | Out-Null }
        if ($null -ne $excel) { $excel.Quit() }
        foreach ($obj in @($used,$sheet,$book,$excel)) {
            if ($null -ne $obj) {
                try { [void][Runtime.InteropServices.Marshal]::ReleaseComObject($obj) } catch {}
            }
        }
        [gc]::Collect()
        [gc]::WaitForPendingFinalizers()
    }

    if ($rows.Count -eq 0) {
        throw '日程マスタExcelから加工先データを取得できませんでした。'
    }

    return @($rows)
}

function Import-HolidaySet {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "日本祝日CSVが見つかりません: $Path"
    }
    $set = @{}
    $rows = Import-Csv -LiteralPath $Path -Encoding UTF8
    foreach ($r in $rows) {
        if ([string]::IsNullOrWhiteSpace($r.Date)) { continue }
        $d = Normalize-DateStrict $r.Date
        $set[$d] = $true
    }
    return $set
}

function Get-DayState {
    param(
        [datetime]$Date,
        [hashtable]$CompanyCalendar,
        [hashtable]$JapanHolidays
    )

    $key = $Date.ToString('yyyy-MM-dd')
    if ($CompanyCalendar.ContainsKey($key)) {
        $status = [string]$CompanyCalendar[$key]
        return [pscustomobject]@{
            IsWorking = ($status -eq '営業日')
            Source = 'Company'
        }
    }

    $dow = $Date.DayOfWeek
    $isWeekend = ($dow -eq [DayOfWeek]::Saturday -or $dow -eq [DayOfWeek]::Sunday)
    $isHoliday = $JapanHolidays.ContainsKey($key)
    return [pscustomobject]@{
        IsWorking = (-not $isWeekend -and -not $isHoliday)
        Source = 'Provisional'
    }
}

function Move-ToWorkingDay {
    param(
        [datetime]$Date,
        [int]$Direction,
        [hashtable]$CompanyCalendar,
        [hashtable]$JapanHolidays
    )

    $d = $Date
    $usedProvisional = $false
    for ($guard = 0; $guard -lt 40; $guard++) {
        $state = Get-DayState -Date $d -CompanyCalendar $CompanyCalendar -JapanHolidays $JapanHolidays
        if ($state.Source -eq 'Provisional') { $usedProvisional = $true }
        if ($state.IsWorking) {
            return [pscustomobject]@{
                Date = $d
                UsedProvisional = $usedProvisional
            }
        }
        $d = $d.AddDays($Direction)
    }
    throw "40日以内に営業日を検出できません: $($Date.ToString('yyyy-MM-dd'))"
}

function Calculate-ScheduleCompat {
    param(
        [string]$DueDate,
        [AllowNull()][string]$OrderDate,
        [int]$InspectionDaysBeforeDue,
        [int]$StartDaysBeforeInspection,
        [hashtable]$CompanyCalendar,
        [hashtable]$JapanHolidays
    )

    $due = [datetime]::ParseExact($DueDate, 'yyyy-MM-dd', [Globalization.CultureInfo]::InvariantCulture)
    $order = $null
    if (-not [string]::IsNullOrWhiteSpace($OrderDate)) {
        $order = [datetime]::ParseExact($OrderDate, 'yyyy-MM-dd', [Globalization.CultureInfo]::InvariantCulture)
    }

    # 現行JS互換: まず暦日で引き、休日に当たった場合だけ方向補正する。
    $inspectionBase = $due.AddDays(-1 * $InspectionDaysBeforeDue)
    $inspectionMoved = Move-ToWorkingDay -Date $inspectionBase -Direction -1 -CompanyCalendar $CompanyCalendar -JapanHolidays $JapanHolidays

    $startBase = $inspectionMoved.Date.AddDays(-1 * $StartDaysBeforeInspection)
    $startMoved = Move-ToWorkingDay -Date $startBase -Direction 1 -CompanyCalendar $CompanyCalendar -JapanHolidays $JapanHolidays

    if ($startMoved.Date -gt $inspectionMoved.Date -or $inspectionMoved.Date -gt $due) {
        return [pscustomobject]@{ Ok=$false; Reason='InvalidDateRelation' }
    }
    if ($null -ne $order -and $startMoved.Date -lt $order) {
        return [pscustomobject]@{ Ok=$false; Reason='ShortLeadTime' }
    }

    $provisional = ($inspectionMoved.UsedProvisional -or $startMoved.UsedProvisional)
    return [pscustomobject]@{
        Ok = $true
        Reason = ''
        InspectionBase = $inspectionBase.ToString('yyyy-MM-dd')
        Inspection = $inspectionMoved.Date.ToString('yyyy-MM-dd')
        StartBase = $startBase.ToString('yyyy-MM-dd')
        Start = $startMoved.Date.ToString('yyyy-MM-dd')
        KScheduleType = 'Batch'
        KProvisionalEnabled = 'ON'
        KCalculationBasis = $(if ($provisional) { 'Provisional' } else { 'Company' })
        InspectionAdjustment = '休日の場合は前営業日'
        StartAdjustment = '休日の場合は翌営業日'
    }
}

function Get-MasterKey {
    param([string]$Value)
    if ($null -eq $Value) { return '' }
    return $Value.Trim()
}

# -------------------- CSV / master / calendar preflight --------------------

$rows = @(Import-CsvAllowDuplicateHeaders -Path $CsvPath)
$requiredColumns = @(
    '発注番号','手配書番号','注文伝票番号','注文伝票行番号',
    '商品名','発注数','手配日','納期','指示先名','手配担当者名'
)
if ($rows.Count -eq 0) { throw 'CSV empty.' }
$columns = @($rows[0].PSObject.Properties.Name)
foreach ($c in $requiredColumns) {
    if ($columns -notcontains $c) { throw "Required column missing: $c" }
}
if ($rows.Count -ne $ExpectedCsvRows) {
    throw "SafetyStop: CSVRows=$($rows.Count) expected=$ExpectedCsvRows"
}

$masterRows = @(Import-ScheduleMasterExcel -Path $ScheduleMasterPath)
$master = @{}
foreach ($m in $masterRows) {
    $key = Get-MasterKey $m.指示先名
    if (-not $key) { continue }
    if ($master.ContainsKey($key)) {
        throw "SafetyStop: duplicate vendor in schedule master: '$key'"
    }
    $master[$key] = $m
}

$companyCalendar = Import-CompanyCalendar -Path $CalendarPath
$calendarDates = @($companyCalendar.Keys | Sort-Object)
if ($calendarDates.Count -eq 0) { throw 'SafetyStop: company calendar is empty.' }
$calendarMin = $calendarDates[0]
$calendarMax = $calendarDates[$calendarDates.Count - 1]
$holidays = Import-HolidaySet -Path $JapanHolidayPath

# -------------------- normalize + schedule plan --------------------

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

    if (-not $po) { $errors.Add("Row=$rowNo blank PO"); continue }
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

    # Actual write defaults to baseline.
    $kStart = $orderDate
    $kInspection = $dueDate
    $kType = 'Batch'
    $kOnOff = 'ON'
    $kBasis = ''
    $decision = 'Baseline-MasterNotFound'
    $ruleStatus = ''
    $inspectionDays = ''
    $startDays = ''
    $previewStart = ''
    $previewInspection = ''
    $previewBasis = ''
    $scheduleNote = 'Schedule master not found -> baseline'

    if ($master.ContainsKey($vendor)) {
        $m = $master[$vendor]
        $ruleStatus = ([string]$m.使用区分).Trim()
        $scheduleNote = ([string]$m.備考).Trim()

        if ($ruleStatus -eq '使用しない') {
            $decision = 'Baseline-RuleDisabled'
            $scheduleNote = 'Rule disabled -> baseline'
        }
        elseif ($ruleStatus -eq '使用' -or $ruleStatus -eq '要確認') {
            $tmpInspection = 0
            $tmpStart = 0
            $okI = [int]::TryParse(([string]$m.完成検査日前倒し日数).Trim(), [ref]$tmpInspection)
            $okS = [int]::TryParse(([string]$m.加工着手日前倒し日数).Trim(), [ref]$tmpStart)

            if (-not $okI -or -not $okS -or
                $tmpInspection -lt 0 -or $tmpStart -lt 0 -or
                $tmpInspection -gt 365 -or $tmpStart -gt 365) {
                $errors.Add("Row=$rowNo PO=$po Vendor='$vendor' ScheduleRuleInvalid")
                continue
            }

            $inspectionDays = $tmpInspection
            $startDays = $tmpStart

            $calc = Calculate-ScheduleCompat `
                -DueDate $dueDate `
                -OrderDate $orderDate `
                -InspectionDaysBeforeDue $tmpInspection `
                -StartDaysBeforeInspection $tmpStart `
                -CompanyCalendar $companyCalendar `
                -JapanHolidays $holidays

            if ($calc.Ok) {
                $previewStart = $calc.Start
                $previewInspection = $calc.Inspection
                $previewBasis = $calc.KCalculationBasis

                if ($ruleStatus -eq '使用') {
                    $kStart = $calc.Start
                    $kInspection = $calc.Inspection
                    $kBasis = $calc.KCalculationBasis
                    $decision = 'Calculated-MasterApplied'
                    $scheduleNote = 'Master rule applied'
                }
                else {
                    # 要確認: 計算結果はPlanログに残すが、書込値はbaseline。
                    $decision = 'Baseline-ReviewOnly'
                    $scheduleNote = 'ReviewOnly: calculated values logged; baseline written'
                }
            }
            elseif ($calc.Reason -eq 'ShortLeadTime') {
                $decision = $(if ($ruleStatus -eq '要確認') { 'Baseline-ReviewOnly-ShortLeadTime' } else { 'Fallback-ShortLeadTime' })
                $scheduleNote = 'ShortLeadTime -> baseline'
            }
            else {
                $errors.Add("Row=$rowNo PO=$po Vendor='$vendor' ScheduleCalcFailed=$($calc.Reason)")
                continue
            }
        }
        else {
            $errors.Add("Row=$rowNo PO=$po Vendor='$vendor' UnknownRuleStatus='$ruleStatus'")
            continue
        }
    }

    $normalized.Add([pscustomobject][ordered]@{
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
        RuleStatus = $ruleStatus
        InspectionDays = $inspectionDays
        StartDays = $startDays
        PreviewKStart = $previewStart
        PreviewKInspection = $previewInspection
        PreviewKBasis = $previewBasis
        KStart = $kStart
        KInspection = $kInspection
        KType = $kType
        KOnOff = $kOnOff
        KBasis = $kBasis
        ScheduleDecision = $decision
        ScheduleNote = $scheduleNote
    })
}

if ($blankOrderNo -ne $ExpectedBlankOrderNo) {
    throw "SafetyStop: BlankOrderNo=$blankOrderNo expected=$ExpectedBlankOrderNo"
}
if ($blankOrderSlip -ne 0) {
    throw "SafetyStop: BlankOrderSlip=$blankOrderSlip expected=0"
}
if ($errors.Count -gt 0) {
    $errors | Select-Object -First 80 | ForEach-Object { Write-Error $_ }
    throw "SafetyStop: validation/schedule errors=$($errors.Count)"
}
if ($normalized.Count -gt $MaxCreates) {
    throw "SafetyStop: creates=$($normalized.Count) MaxCreates=$MaxCreates"
}

# -------------------- App292 / dropdown preflight --------------------

$token = Get-PlainToken -Path $TokenPath
$existing = @(Get-AllRecords -Token $token)
$form = Get-LiveForm -Token $token

$missingVendors = @(
    $normalized | ForEach-Object { $_.Vendor } | Sort-Object -Unique |
    Where-Object { -not (Test-OptionExists -Form $form -Code 'VENDOR_DD' -Value $_) }
)
$missingStaff = @(
    $normalized | ForEach-Object { $_.Staff } | Sort-Object -Unique |
    Where-Object { -not (Test-OptionExists -Form $form -Code 'STAFF_DD' -Value $_) }
)

$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$planPath = Join-Path $LogDirectory "App292_NoJOIN_MasterAuto_Plan_$stamp.csv"

$normalized |
    Select-Object Row,PO,OrderNo,SlipNo,SlipLine,Product,Qty,OrderDate,DueDate,Vendor,Staff,
                  RuleStatus,InspectionDays,StartDays,PreviewKStart,PreviewKInspection,PreviewKBasis,
                  KStart,KInspection,KType,KOnOff,KBasis,ScheduleDecision,ScheduleNote |
    Export-Csv -LiteralPath $planPath -NoTypeInformation -Encoding UTF8

$countApplied = @($normalized | Where-Object ScheduleDecision -eq 'Calculated-MasterApplied').Count
$countReview = @($normalized | Where-Object { $_.ScheduleDecision -like 'Baseline-ReviewOnly*' }).Count
$countDisabled = @($normalized | Where-Object ScheduleDecision -eq 'Baseline-RuleDisabled').Count
$countMissing = @($normalized | Where-Object ScheduleDecision -eq 'Baseline-MasterNotFound').Count
$countShort = @($normalized | Where-Object ScheduleDecision -eq 'Fallback-ShortLeadTime').Count
$countCompany = @($normalized | Where-Object KBasis -eq 'Company').Count
$countProvisional = @($normalized | Where-Object KBasis -eq 'Provisional').Count

Write-Host '================================================================='
Write-Host 'Import App292 NoJOIN v0.3 MasterAuto'
Write-Host '================================================================='
Write-Host "CSVRows              : $($rows.Count)"
Write-Host "UniquePO             : $($poSeen.Count)"
Write-Host "HandOrderPresent     : $($rows.Count - $blankOrderNo)"
Write-Host "HandOrderBlank       : $blankOrderNo"
Write-Host "OrderSlipPresent     : $($rows.Count - $blankOrderSlip)"
Write-Host "Existing App292      : $($existing.Count)"
Write-Host "ScheduleMasterRows   : $($master.Count)"
Write-Host "CalendarCoverage     : $calendarMin .. $calendarMax"
Write-Host "Calculated Applied   : $countApplied"
Write-Host "ReviewOnly Baseline  : $countReview"
Write-Host "RuleDisabled Baseline: $countDisabled"
Write-Host "MasterMissingBaseline: $countMissing"
Write-Host "ShortLeadTimeFallback: $countShort"
Write-Host "K Basis Company      : $countCompany"
Write-Host "K Basis Provisional  : $countProvisional"
Write-Host "Missing VENDOR_DD    : $($missingVendors.Count)"
Write-Host "Missing STAFF_DD     : $($missingStaff.Count)"
Write-Host "Plan                 : $planPath"
Write-Host 'App270                : NOT USED'
Write-Host 'ルックアップ          : NOT WRITTEN'
Write-Host 'Key                   : 発注番号 / field 数値'

if ($missingVendors.Count -gt 0 -or $missingStaff.Count -gt 0) {
    foreach ($x in $missingVendors) { Write-Host "  [MISSING VENDOR] $x" }
    foreach ($x in $missingStaff) { Write-Host "  [MISSING STAFF] $x" }
    throw 'SafetyStop: dropdown option missing. Run Prepare-App292-NoJoinMasterOptions_v0.2.ps1 first.'
}

if ($Execute -and $existing.Count -ne 0) {
    throw ("SafetyStop: Execute requires empty App292. Current={0}. " +
           "Recovery/retest: run Clear-App292-TestData_v0.2.ps1 -DryRun, then -Execute " +
           "-ConfirmExecute 'APP292-CLEAR-ALL', verify 0 records, then rerun v0.3 from the beginning.") -f $existing.Count
}

if (-not $Execute) {
    Write-Host 'WRITE                 : NONE'
    Write-Host 'RESULT                : DRYRUN PASS'
    exit 0
}

# -------------------- POST --------------------

$payloads = New-Object System.Collections.Generic.List[object]
foreach ($x in $normalized) {
    $record = [ordered]@{
        '数値' = @{ value=$x.PO }
        'ODERNO' = @{ value=$x.OrderNo }
        '数値_44' = @{ value=$x.SlipNo }
        '数値_45' = @{ value=$x.SlipLine }
        '商品名' = @{ value=$x.Product }
        '数値_10' = @{ value=$x.Qty }
        '日付' = @{ value=$x.OrderDate }
        '日付_1' = @{ value=$x.DueDate }
        'VENDOR_TEXT' = @{ value=$x.Vendor }
        'VENDOR_DD' = @{ value=$x.Vendor }
        'STAFF_TEXT' = @{ value=$x.Staff }
        'STAFF_DD' = @{ value=$x.Staff }
        'K加工着手日' = @{ value=$x.KStart }
        'K完成検査日' = @{ value=$x.KInspection }
        'K日程種別' = @{ value=$x.KType }
        'K暫定設定' = @{ value=$x.KOnOff }
        'K計算根拠' = @{ value=$x.KBasis }
    }
    $payloads.Add($record)
}

for ($i=0; $i -lt $payloads.Count; $i+=100) {
    $end = [Math]::Min($i+99,$payloads.Count-1)
    $batch = @($payloads[$i..$end])
    Write-Host ("POST {0}-{1} / {2}" -f ($i+1),($end+1),$payloads.Count)
    [void](Add-RecordBatch -Token $token -Records $batch)
}

# 部分POST失敗時は部分再開しない。
# Clear-App292-TestData_v0.2.ps1 で0件化後、v0.3を先頭から再実行する。

# -------------------- PostVerify --------------------

$after = @(Get-AllRecords -Token $token)
if ($after.Count -ne $ExpectedCsvRows) {
    throw "PostVerifyFailed: App292 count=$($after.Count) expected=$ExpectedCsvRows"
}

$byPo = @{}
foreach ($r in $after) {
    $po = Get-KValue $r '数値'
    if ($byPo.ContainsKey($po)) { throw "PostVerifyFailed: duplicate App292 PO=$po" }
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
        @('ODERNO',$x.OrderNo),@('数値_44',$x.SlipNo),@('数値_45',$x.SlipLine),
        @('商品名',$x.Product),@('数値_10',$x.Qty),@('日付',$x.OrderDate),@('日付_1',$x.DueDate),
        @('VENDOR_TEXT',$x.Vendor),@('VENDOR_DD',$x.Vendor),
        @('STAFF_TEXT',$x.Staff),@('STAFF_DD',$x.Staff),
        @('K加工着手日',$x.KStart),@('K完成検査日',$x.KInspection),
        @('K日程種別',$x.KType),@('K暫定設定',$x.KOnOff),@('K計算根拠',$x.KBasis)
    )

    foreach ($pair in $pairs) {
        $actual = Get-KValue $r $pair[0]
        $expected = [string]$pair[1]
        if ($actual -ne $expected) {
            $verifyErrors.Add("PO=$($x.PO) Field=$($pair[0]) Expected='$expected' Actual='$actual'")
        }
    }
}

if ($verifyErrors.Count -gt 0) {
    $verifyErrors | Select-Object -First 80 | ForEach-Object { Write-Error $_ }
    throw "PostVerifyFailed: mismatches=$($verifyErrors.Count)"
}

Write-Host "PostVerify App292      : $($after.Count)"
Write-Host 'PostVerifyMismatch     : 0'
Write-Host 'RESULT                 : PASS'
