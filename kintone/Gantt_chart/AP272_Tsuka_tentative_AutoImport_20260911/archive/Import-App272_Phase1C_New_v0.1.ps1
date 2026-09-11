#requires -Version 5.1
<#
.SYNOPSIS
  kintone App272 API自動取込 Phase 1C 新規1件POST版 v0.1

.DESCRIPTION
  - 外注課納期管理CSVを検証する。
  - App270 / App272 をGETし、Phase 1Cでは新規App272だけを安全に作成する。
  - 新規App272候補について、加工先別マスタから暫定日程を計算する。
  - 日程計算は schedule_preview_vendor_v3.0.3_rev3_5.js 互換:
      完成検査日 = 納期 - N暦日 → 休日なら前営業日へ
      加工着手日 = 完成検査日 - N暦日 → 休日なら次営業日へ
  - 会社カレンダーに日付が存在する範囲は会社判定を優先し、未定義日は土日＋日本祝日で暫定判定する。
  - K日程種別は設定主体を表し、Manual / Batch / Auto の3区分とする。
      優先度は Manual > Batch > Auto。
      新規App272の日程を本PowerShellが算出した場合は Batch を候補値とする。
  - K暫定設定は ON / OFF の既存意味を保持し、計算根拠は K計算根拠（未作成でもDryRun可）としてログ上に分離する。
  - 既存App272は確定済み基幹5項目（ODERNO/VENDOR_TEXT/VENDOR_DD/日付/日付_1）を比較し、差分なしは NOCHANGE、差分ありは Update(BaseOnly) とする。
  - CSV「指示先名」は旧来互換フィールド VENDOR_TEXT と日程用 VENDOR_DD の2か所へ同期する。新規App272でも両方へ同値を書込む前提でDryRun判定する。
  - K日程種別=Manual かつ納期差分ありの場合は ManualScheduleDueDateChanged を警告ログへ残す。
  - 既存K日程種別=Auto / Batch の日付関係を検査し、矛盾があれば各警告ログへ残す（DryRunでは書換えなし）。
  - 優先度ルールを実判定へ反映する。Manual / Batch は保護、Auto / 空欄はBatch再計算候補。
  - 未知のK日程種別は安全側で保護する。
  - 加工先マスタの使用区分は 使用 / 要確認 / 使用しない の3値のみ許可し、それ以外は安全側で保護・警告する。
  - Execute対象は新規App272のみ。既存App272のPUTはPhase 1Cでは禁止する。
  - App270追加はv0.1では禁止する。
  - 新規App272は 数値(発注番号) / ODERNO / VENDOR_TEXT / VENDOR_DD / 日付 / 日付_1 / ルックアップ のみ作成する。
  - K日程は新規時に書かない（要確認・ShortLeadTime等の判定結果を尊重）。

.NOTES
  既定はDryRun。書込みには -Execute と -ConfirmExecute 'APP272-NEW-EXECUTE' が必要。
  Execute時は -ExpectedNewPo で新規対象の発注番号を明示する。
  v0.1はPOSTのみ実装し、PUT/DELETEは実行しない。
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)]
    [string]$CsvPath,

    [string]$ScheduleMasterPath = 'C:\HPDB\App272_ScheduleMaster.csv',
    [string]$CalendarPath       = 'C:\HPDB\カレンダー(2023).xlsx',
    [string]$JapanHolidayPath   = 'C:\HPDB\JapanHolidays_2026_2027.csv',
    [string]$LogDirectory       = 'C:\HPDB\logs',

    [string]$BaseUrl = 'https://bcurbkixz609.cybozu.com',
    [int]$App270 = 270,
    [int]$App272 = 272,

    [string]$ApiToken270 = '',
    [string]$ApiToken272 = '',

    [switch]$SkipKintone,
    [switch]$DryRun,
    [switch]$Execute,
    [string]$ConfirmExecute = '',
    [string]$ExpectedNewPo = '',
    [int]$MaxCreates = 1
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

$script:ExitCode = 0
$script:Warnings = New-Object System.Collections.Generic.List[string]

function Add-Warning {
    param([string]$Message)
    $script:Warnings.Add($Message)
    Write-Warning $Message
}

function ConvertFrom-SecureStringPlain {
    param([Security.SecureString]$Secure)
    $ptr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($Secure)
    try { return [Runtime.InteropServices.Marshal]::PtrToStringBSTR($ptr) }
    finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($ptr) }
}

function Get-ApiTokenInteractive {
    param([string]$Label)
    $secure = Read-Host "$Label API Token" -AsSecureString
    return ConvertFrom-SecureStringPlain $secure
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
            }
            else {
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
            }
            catch {
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

function Normalize-DateStrict {
    param([AllowNull()][AllowEmptyString()][string]$Value)

    if ([string]::IsNullOrWhiteSpace($Value)) { return $null }
    $v = $Value.Trim()

    # 2桁年は推測補正しない。
    if ($v -match '^\d{2}[-/]\d{1,2}[-/]\d{1,2}$') {
        throw "2桁年は受理しません: '$v'"
    }

    $formats = @('yyyy/M/d','yyyy/MM/dd','yyyy-M-d','yyyy-MM-dd')
    foreach ($format in $formats) {
        $dt = [datetime]::MinValue
        $ok = [datetime]::TryParseExact(
            $v,
            $format,
            [Globalization.CultureInfo]::InvariantCulture,
            [Globalization.DateTimeStyles]::None,
            [ref]$dt
        )
        if ($ok) {
            return $dt.ToString('yyyy-MM-dd')
        }
    }
    throw "日付形式が不正です: '$v'"
}

function Get-KValue {
    param($Record, [string]$FieldCode)
    if ($null -eq $Record) { return '' }
    $p = $Record.PSObject.Properties[$FieldCode]
    if ($null -eq $p -or $null -eq $p.Value) { return '' }
    $vp = $p.Value.PSObject.Properties['value']
    if ($null -eq $vp -or $null -eq $vp.Value) { return '' }
    return ([string]$vp.Value).Trim()
}

function Test-KFieldExists {
    param($Record, [string]$FieldCode)
    if ($null -eq $Record) { return $false }
    return ($null -ne $Record.PSObject.Properties[$FieldCode])
}

function Get-KintoneHttpErrorDetail {
    param(
        [Parameter(Mandatory=$true)]$ErrorRecord,
        [string]$RequestUri,
        [string]$QueryText,
        [int]$AppId
    )

    $ex = $ErrorRecord.Exception
    $statusCode = ''
    $statusDescription = ''
    $responseBody = ''
    $errorDetailsBody = ''
    $responseHeadersText = ''

    # Windows PowerShell 5.1 の Invoke-RestMethod は、HTTPエラー本文を
    # ErrorDetails.Message に保持する場合がある。まずこちらを優先する。
    try {
        if ($null -ne $ErrorRecord.ErrorDetails -and
            -not [string]::IsNullOrWhiteSpace([string]$ErrorRecord.ErrorDetails.Message)) {
            $errorDetailsBody = [string]$ErrorRecord.ErrorDetails.Message
        }
    } catch {}

    if ($null -ne $ex.Response) {
        try { $statusCode = [int]$ex.Response.StatusCode } catch {}
        try { $statusDescription = [string]$ex.Response.StatusDescription } catch {}
        try {
            if ($null -ne $ex.Response.Headers) {
                $pairs = New-Object System.Collections.Generic.List[string]
                foreach ($key in $ex.Response.Headers.AllKeys) {
                    $pairs.Add(('{0}={1}' -f $key, $ex.Response.Headers[$key]))
                }
                $responseHeadersText = $pairs -join '; '
            }
        } catch {}
        try {
            $stream = $ex.Response.GetResponseStream()
            if ($null -ne $stream) {
                $reader = New-Object System.IO.StreamReader($stream, [System.Text.Encoding]::UTF8)
                try { $responseBody = $reader.ReadToEnd() }
                finally { $reader.Dispose() }
            }
        } catch {}
    }

    # GetResponseStream() が既に消費済みでも ErrorDetails.Message が残る場合がある。
    $rawBody = $responseBody
    if ([string]::IsNullOrWhiteSpace($rawBody)) { $rawBody = $errorDetailsBody }

    $kintoneCode = ''
    $kintoneMessage = ''
    $kintoneId = ''
    if (-not [string]::IsNullOrWhiteSpace($rawBody)) {
        try {
            $json = $rawBody | ConvertFrom-Json -ErrorAction Stop
            if ($null -ne $json.code) { $kintoneCode = [string]$json.code }
            if ($null -ne $json.message) { $kintoneMessage = [string]$json.message }
            if ($null -ne $json.id) { $kintoneId = [string]$json.id }
        } catch {}
    }

    $lines = New-Object System.Collections.Generic.List[string]
    $lines.Add('kintone GET failed.')
    $lines.Add(('AppId          : {0}' -f $AppId))
    if ($statusCode) { $lines.Add(('HTTP Status    : {0} {1}' -f $statusCode, $statusDescription)) }
    else { $lines.Add(('HTTP Exception : {0}' -f $ex.Message)) }
    $lines.Add(('Exception Type : {0}' -f $ex.GetType().FullName))
    if ($kintoneCode) { $lines.Add(('kintone code   : {0}' -f $kintoneCode)) }
    if ($kintoneMessage) { $lines.Add(('kintone message: {0}' -f $kintoneMessage)) }
    if ($kintoneId) { $lines.Add(('kintone id     : {0}' -f $kintoneId)) }
    $lines.Add(('Query          : {0}' -f $QueryText))
    $lines.Add(('Request URI    : {0}' -f $RequestUri))
    if (-not [string]::IsNullOrWhiteSpace($rawBody)) {
        $lines.Add(('Raw Response   : {0}' -f $rawBody))
    } else {
        $lines.Add('Raw Response   : <empty>')
    }
    if (-not [string]::IsNullOrWhiteSpace($responseHeadersText)) {
        $lines.Add(('Resp. Headers  : {0}' -f $responseHeadersText))
    }
    $lines.Add('API token is sent only in the request header and is not shown above.')
    return ($lines -join [Environment]::NewLine)
}

function Invoke-KintoneGetAll {
    param(
        [string]$BaseUrl,
        [int]$AppId,
        [string]$Token
    )

    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $all = New-Object System.Collections.Generic.List[object]
    $lastId = 0
    $page = 0

    while ($true) {
        $page++
        $query = '$id > {0} order by $id asc limit 500' -f $lastId
        $uri = '{0}/k/v1/records.json?app={1}&query={2}' -f (
            $BaseUrl.TrimEnd('/'),
            $AppId,
            [uri]::EscapeDataString($query)
        )

        Write-Host ('      GET App{0} page={1} lastId={2}' -f $AppId, $page, $lastId)
        try {
            $resp = Invoke-RestMethod -Method Get -Uri $uri -Headers $headers
        }
        catch {
            $detail = Get-KintoneHttpErrorDetail -ErrorRecord $_ -RequestUri $uri -QueryText $query -AppId $AppId
            throw $detail
        }

        foreach ($r in $resp.records) { $all.Add($r) }
        Write-Host ('      -> {0} records received (total={1})' -f $resp.records.Count, $all.Count)
        if ($resp.records.Count -lt 500) { break }
        $lastId = [int](Get-KValue $resp.records[$resp.records.Count - 1] '$id')
    }
    return $all.ToArray()
}



function Invoke-KintoneCreateRecord {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)][string]$BaseUrl,
        [Parameter(Mandatory=$true)][int]$AppId,
        [Parameter(Mandatory=$true)][string]$Token,
        [Parameter(Mandatory=$true)][hashtable]$RecordFields
    )

    $uri = $BaseUrl.TrimEnd('/') + '/k/v1/record.json'
    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $bodyObj = [ordered]@{
        app = $AppId
        record = $RecordFields
    }
    $body = $bodyObj | ConvertTo-Json -Depth 10

    try {
        return Invoke-RestMethod -Method Post -Uri $uri -Headers $headers `
            -ContentType 'application/json; charset=utf-8' -Body $body
    }
    catch {
        $detail = Get-KintoneHttpErrorDetail $_
        throw "App$AppId POST failed: $detail"
    }
}

function Invoke-KintoneUpdateRecord {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)][string]$BaseUrl,
        [Parameter(Mandatory=$true)][int]$AppId,
        [Parameter(Mandatory=$true)][string]$Token,
        [Parameter(Mandatory=$true)][string]$RecordId,
        [Parameter(Mandatory=$true)][string]$Revision,
        [Parameter(Mandatory=$true)][hashtable]$RecordFields
    )

    $uri = $BaseUrl.TrimEnd('/') + '/k/v1/record.json'
    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $bodyObj = [ordered]@{
        app = $AppId
        id = [int64]$RecordId
        revision = [int64]$Revision
        record = $RecordFields
    }
    $body = $bodyObj | ConvertTo-Json -Depth 10

    try {
        return Invoke-RestMethod -Method Put -Uri $uri -Headers $headers `
            -ContentType 'application/json; charset=utf-8' -Body $body
    }
    catch {
        $detail = Get-KintoneHttpErrorDetail $_
        throw "App$AppId PUT failed: recordId=$RecordId revision=$Revision : $detail"
    }
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

# -------------------- main --------------------
$started = Get-Date
Write-Host '=== App272 API自動取込 Phase 1C New POST v0.1 ===' -ForegroundColor Cyan

if ($DryRun -and $Execute) {
    Write-Error '-DryRun と -Execute は同時指定できません。'
    exit 30
}
if (-not $DryRun -and -not $Execute) {
    Write-Error '必ず -DryRun または -Execute のどちらかを指定してください。'
    exit 30
}
if ($Execute -and $ConfirmExecute -ne 'APP272-NEW-EXECUTE') {
    Write-Error "新規作成には -ConfirmExecute 'APP272-NEW-EXECUTE' が必要です。"
    exit 31
}
if ($Execute -and [string]::IsNullOrWhiteSpace($ExpectedNewPo)) {
    Write-Error '-Execute時は -ExpectedNewPo で対象の発注番号を明示してください。'
    exit 34
}
if ($Execute -and $SkipKintone) {
    Write-Error '-Execute と -SkipKintone は同時指定できません。'
    exit 32
}
if ($MaxCreates -ne 1) {
    Write-Error '-MaxCreates は安全上 1 固定です。'
    exit 33
}

try {
    if (-not (Test-Path -LiteralPath $CsvPath)) { throw "入力CSVが見つかりません: $CsvPath" }
    if (-not (Test-Path -LiteralPath $ScheduleMasterPath)) { throw "日程マスタが見つかりません: $ScheduleMasterPath" }

    if (-not (Test-Path -LiteralPath $LogDirectory)) {
        New-Item -ItemType Directory -Path $LogDirectory -Force | Out-Null
    }

    Write-Host "CSV              : $CsvPath"
    Write-Host "Schedule Master  : $ScheduleMasterPath"
    Write-Host "Company Calendar : $CalendarPath"
    Write-Host "Japan Holidays   : $JapanHolidayPath"
    if ($DryRun) { Write-Host 'WRITE             : NONE (DryRun / GET only)' }
    else { Write-Host "WRITE             : ENABLED (POST only / MaxCreates=$MaxCreates)" -ForegroundColor Yellow }

    # 実運用CSVはShift-JIS/CP932。Windows PowerShell 5.1では Default がCP932相当。
    $csvRows = @(Import-CsvAllowDuplicateHeaders -Path $CsvPath)
    if ($csvRows.Count -eq 0) { throw '入力CSVが0件です。' }

    $required = @('発注番号','手配書番号','手配日','納期','指示先名')
    $props = @($csvRows[0].PSObject.Properties.Name)
    foreach ($name in $required) {
        if ($props -notcontains $name) { throw "必須列がありません: $name" }
    }

    $masterRows = @(Import-Csv -LiteralPath $ScheduleMasterPath -Encoding UTF8)
    $master = @{}
    foreach ($m in $masterRows) {
        $key = Get-MasterKey $m.指示先名
        if (-not $key) { continue }
        $master[$key] = $m
    }

    Write-Host '[1/5] 会社カレンダー読込...'
    $companyCalendar = Import-CompanyCalendar -Path $CalendarPath
    $calendarDates = @($companyCalendar.Keys | Sort-Object)
    $calendarMin = $calendarDates[0]
    $calendarMax = $calendarDates[$calendarDates.Count - 1]
    Write-Host "      Coverage: $calendarMin .. $calendarMax ($($companyCalendar.Count) days)"
    if (-not ($companyCalendar.Keys | Where-Object { $_ -like '2026-*' } | Select-Object -First 1)) {
        throw '会社カレンダーに2026年データが存在しません。'
    }

    Write-Host '[2/5] 日本祝日CSV読込...'
    $holidays = Import-HolidaySet -Path $JapanHolidayPath

    $app270Records = @()
    $app272Records = @()
    $app270ByOrder = @{}
    $app272ByPo = @{}
    $kCalculationBasisFieldExists = $false

    if (-not $SkipKintone) {
        if ([string]::IsNullOrWhiteSpace($ApiToken270)) { $ApiToken270 = Get-ApiTokenInteractive 'App270' }
        if ([string]::IsNullOrWhiteSpace($ApiToken272)) { $ApiToken272 = Get-ApiTokenInteractive 'App272' }

        Write-Host '[3/5] kintone App270 GET...'
        $app270Records = @(Invoke-KintoneGetAll -BaseUrl $BaseUrl -AppId $App270 -Token $ApiToken270)
        foreach ($r in $app270Records) {
            $key = Get-KValue $r '文字列__1行_'
            if ($key -and -not $app270ByOrder.ContainsKey($key)) { $app270ByOrder[$key] = $r }
        }

        Write-Host '[4/5] kintone App272 GET...'
        $app272Records = @(Invoke-KintoneGetAll -BaseUrl $BaseUrl -AppId $App272 -Token $ApiToken272)
        foreach ($r in $app272Records) {
            $key = Get-KValue $r '数値'
            if ($key -and -not $app272ByPo.ContainsKey($key)) { $app272ByPo[$key] = $r }
            if (Test-KFieldExists $r 'K計算根拠') { $kCalculationBasisFieldExists = $true }
        }
    } else {
        Add-Warning '-SkipKintone のため App270/App272 の存在判定は行いません。'
    }

    Write-Host '[5/5] CSV分類・暫定日程計算...'
    $details = New-Object System.Collections.Generic.List[object]
    $firstDueByOrder = @{}

    $index = 0
    foreach ($row in $csvRows) {
        $index++
        $po = ([string]$row.発注番号).Trim()
        $orderNo = ([string]$row.手配書番号).Trim()
        $vendor = ([string]$row.指示先名).Trim()
        $errors = New-Object System.Collections.Generic.List[string]

        $orderDate = $null
        $dueDate = $null
        try { $orderDate = Normalize-DateStrict ([string]$row.手配日) } catch { $errors.Add("手配日: $($_.Exception.Message)") }
        try { $dueDate = Normalize-DateStrict ([string]$row.納期) } catch { $errors.Add("納期: $($_.Exception.Message)") }

        if (-not $po) { $errors.Add('発注番号が空欄') }
        if ($orderNo -and $dueDate -and -not $firstDueByOrder.ContainsKey($orderNo)) {
            $firstDueByOrder[$orderNo] = $dueDate
        }

        $existing272 = $null
        $app272Action = 'Unknown(SkipKintone)'
        $baseDiffFields = New-Object System.Collections.Generic.List[string]
        $oldDueDate = ''
        $oldOrderDate = ''
        $oldOrderNo = ''
        $oldVendorText = ''
        $oldVendorDD = ''
        $currentKScheduleType = ''
        $currentKStart = ''
        $currentKInspection = ''
        $recordId272 = ''
        $revision272 = ''
        $currentLookup = ''
        $manualWarning = ''
        $existingAutoWarning = ''
        $existingAutoIssues = New-Object System.Collections.Generic.List[string]
        $existingBatchWarning = ''
        $existingBatchIssues = New-Object System.Collections.Generic.List[string]

        if (-not $SkipKintone) {
            if ($app272ByPo.ContainsKey($po)) {
                $existing272 = $app272ByPo[$po]

                # v1.3: CSV「指示先名」は旧来の文字列フィールド VENDOR_TEXT と
                # 日程管理用ドロップダウン VENDOR_DD の両方へ同期する。
                # その他基幹項目は正式マッピング確定後に比較対象へ追加する。
                $oldOrderNo = Get-KValue $existing272 'ODERNO'
                $oldVendorText = Get-KValue $existing272 'VENDOR_TEXT'
                $oldVendorDD = Get-KValue $existing272 'VENDOR_DD'
                $oldOrderDate = Get-KValue $existing272 '日付'
                $oldDueDate = Get-KValue $existing272 '日付_1'
                $currentKScheduleType = Get-KValue $existing272 'K日程種別'
                $currentKStart = Get-KValue $existing272 'K加工着手日'
                $currentKInspection = Get-KValue $existing272 'K完成検査日'
                $recordId272 = Get-KValue $existing272 '$id'
                $revision272 = Get-KValue $existing272 '$revision'
                $currentLookup = Get-KValue $existing272 'ルックアップ'

                if ($oldOrderNo -ne $orderNo) { $baseDiffFields.Add('ODERNO') }
                if ($oldVendorText -ne $vendor) { $baseDiffFields.Add('VENDOR_TEXT') }
                if ($oldVendorDD -ne $vendor) { $baseDiffFields.Add('VENDOR_DD') }
                if ($oldOrderDate -ne $orderDate) { $baseDiffFields.Add('日付') }
                if ($oldDueDate -ne $dueDate) { $baseDiffFields.Add('日付_1') }

                if ($baseDiffFields.Count -eq 0) {
                    $app272Action = 'NOCHANGE'
                } else {
                    $app272Action = 'Update(BaseOnly)'
                }

                if ($currentKScheduleType -eq 'Manual' -and $oldDueDate -ne $dueDate) {
                    $manualWarning = 'ManualScheduleDueDateChanged'
                }

                # 既存のシステム生成日程（Auto / Batch）は書き換えず、日付関係の矛盾だけを検出する。
                if ($currentKScheduleType -eq 'Auto' -or $currentKScheduleType -eq 'Batch') {
                    $kStartDate = $null
                    $kInspectionDate = $null
                    $dueForCheck = $null
                    # PowerShellは空のListをサブ式経由で代入すると$null化することがあるため、
                    # Auto/BatchそれぞれのListへ直接追加する。
                    if (-not [string]::IsNullOrWhiteSpace($currentKStart)) {
                        try { $kStartDate = [datetime]::ParseExact($currentKStart, 'yyyy-MM-dd', [Globalization.CultureInfo]::InvariantCulture) }
                        catch {
                            if ($currentKScheduleType -eq 'Batch') { $existingBatchIssues.Add('K加工着手日形式不正') }
                            else { $existingAutoIssues.Add('K加工着手日形式不正') }
                        }
                    }
                    if (-not [string]::IsNullOrWhiteSpace($currentKInspection)) {
                        try { $kInspectionDate = [datetime]::ParseExact($currentKInspection, 'yyyy-MM-dd', [Globalization.CultureInfo]::InvariantCulture) }
                        catch {
                            if ($currentKScheduleType -eq 'Batch') { $existingBatchIssues.Add('K完成検査日形式不正') }
                            else { $existingAutoIssues.Add('K完成検査日形式不正') }
                        }
                    }
                    if (-not [string]::IsNullOrWhiteSpace($dueDate)) {
                        try { $dueForCheck = [datetime]::ParseExact($dueDate, 'yyyy-MM-dd', [Globalization.CultureInfo]::InvariantCulture) }
                        catch { }
                    }

                    if ($null -ne $kStartDate -and $null -ne $kInspectionDate -and $kStartDate -gt $kInspectionDate) {
                        if ($currentKScheduleType -eq 'Batch') { $existingBatchIssues.Add('K加工着手日>K完成検査日') } else { $existingAutoIssues.Add('K加工着手日>K完成検査日') }
                    }
                    if ($null -ne $kInspectionDate -and $null -ne $dueForCheck -and $kInspectionDate -gt $dueForCheck) {
                        if ($currentKScheduleType -eq 'Batch') { $existingBatchIssues.Add('K完成検査日>納期') } else { $existingAutoIssues.Add('K完成検査日>納期') }
                    }
                    if ($null -ne $kStartDate -and $null -ne $dueForCheck -and $kStartDate -gt $dueForCheck) {
                        if ($currentKScheduleType -eq 'Batch') { $existingBatchIssues.Add('K加工着手日>納期') } else { $existingAutoIssues.Add('K加工着手日>納期') }
                    }

                    if ($currentKScheduleType -eq 'Batch' -and $existingBatchIssues.Count -gt 0) {
                        $existingBatchWarning = 'ExistingBatchScheduleInvalid'
                    }
                    elseif ($currentKScheduleType -eq 'Auto' -and $existingAutoIssues.Count -gt 0) {
                        $existingAutoWarning = 'ExistingAutoScheduleInvalid'
                    }
                }
            } else {
                $app272Action = 'New'
            }
        }

        $app270Action = 'Unknown(SkipKintone)'
        if (-not $SkipKintone) {
            if (-not $orderNo) { $app270Action = 'None(BlankODERNO)' }
            elseif ($app270ByOrder.ContainsKey($orderNo)) { $app270Action = 'Exists(NoUpdate)' }
            else { $app270Action = 'Add' }
        }

        $lookupAction = 'Unknown(SkipKintone)'
        if (-not $SkipKintone) {
            if (-not $orderNo) { $lookupAction = 'None(BlankODERNO)' }
            elseif ($null -eq $existing272) { $lookupAction = 'SyncAfterCreate' }
            else {
                if ($currentLookup -eq $orderNo) { $lookupAction = 'NoChange' }
                else { $lookupAction = 'Sync' }
            }
        }

        $scheduleStatus = 'NotApplicable'
        $ruleStatus = ''
        $inspectionDays = $null
        $startDays = $null
        $inspectionBase = ''
        $inspection = ''
        $startBase = ''
        $start = ''
        $calcBasis = ''
        $proposedKType = ''
        $proposedKOnOff = ''
        $scheduleNote = ''

        # v1.0: Manual > Batch > Auto の優先度を実判定へ反映。
        # Manual / Batch は保護。Auto / 空欄、およびNewはBatch計算候補。未知種別は安全側で保護。
        $scheduleTargetKind = ''
        $isScheduleCalculationTarget = $false

        if ($SkipKintone -or $app272Action -eq 'New') {
            $isScheduleCalculationTarget = $true
            $scheduleTargetKind = 'New'
        }
        elseif ($null -ne $existing272) {
            switch ($currentKScheduleType) {
                'Manual' { $scheduleStatus = 'ProtectedManual' }
                'Batch'  { $scheduleStatus = 'ProtectedBatch' }
                'Auto' {
                    $isScheduleCalculationTarget = $true
                    $scheduleTargetKind = 'AutoToBatch'
                }
                '' {
                    $isScheduleCalculationTarget = $true
                    $scheduleTargetKind = 'BlankToBatch'
                }
                default { $scheduleStatus = 'ProtectedUnknownType' }
            }
        }

        if ($isScheduleCalculationTarget -and $errors.Count -eq 0) {
            if (-not $master.ContainsKey($vendor)) {
                $scheduleStatus = 'ScheduleMasterNotFound'
            } else {
                $m = $master[$vendor]
                $ruleStatus = ([string]$m.使用区分).Trim()
                $scheduleNote = ([string]$m.備考).Trim()
                if ($ruleStatus -eq '使用しない') {
                    $scheduleStatus = 'ScheduleRuleDisabled'
                }
                elseif ($ruleStatus -ne '使用' -and $ruleStatus -ne '要確認') {
                    # 安全側: 許可された3値以外は正式使用扱いにしない。
                    $scheduleStatus = 'ProtectedUnknownRuleStatus'
                    Add-Warning ("InvalidRuleStatus: Vendor='{0}' Status='{1}' (allowed: 使用 / 要確認 / 使用しない)" -f $vendor, $ruleStatus)
                }
                else {
                    $tmpInspection = 0
                    $tmpStart = 0
                    $okI = [int]::TryParse(([string]$m.完成検査日前倒し日数).Trim(), [ref]$tmpInspection)
                    $okS = [int]::TryParse(([string]$m.加工着手日前倒し日数).Trim(), [ref]$tmpStart)
                    if (-not $okI -or -not $okS -or $tmpInspection -lt 0 -or $tmpStart -lt 0 -or $tmpInspection -gt 365 -or $tmpStart -gt 365) {
                        $scheduleStatus = 'ScheduleRuleInvalid'
                    } else {
                        $inspectionDays = $tmpInspection
                        $startDays = $tmpStart
                        $calc = Calculate-ScheduleCompat `
                            -DueDate $dueDate `
                            -OrderDate $orderDate `
                            -InspectionDaysBeforeDue $inspectionDays `
                            -StartDaysBeforeInspection $startDays `
                            -CompanyCalendar $companyCalendar `
                            -JapanHolidays $holidays

                        if ($calc.Ok) {
                            $inspectionBase = $calc.InspectionBase
                            $inspection = $calc.Inspection
                            $startBase = $calc.StartBase
                            $start = $calc.Start
                            $calcBasis = $calc.KCalculationBasis
                            $proposedKType = 'Batch'
                            $proposedKOnOff = $calc.KProvisionalEnabled
                            if ($ruleStatus -eq '要確認') {
                                $scheduleStatus = 'Calculated-ReviewOnly'
                            }
                            elseif ($scheduleTargetKind -eq 'New') {
                                $scheduleStatus = 'Calculated-NewBatchCandidate'
                            }
                            elseif ($scheduleTargetKind -eq 'AutoToBatch') {
                                $scheduleStatus = 'Calculated-BatchOverwriteCandidate(Auto)'
                            }
                            elseif ($scheduleTargetKind -eq 'BlankToBatch') {
                                $scheduleStatus = 'Calculated-BatchOverwriteCandidate(Blank)'
                            }
                            else {
                                $scheduleStatus = 'Calculated-Candidate'
                            }
                        } else {
                            $scheduleStatus = $calc.Reason
                        }
                    }
                }
            }
        }


        $plannedWrite = 'NONE'
        $isScheduleWriteCandidate = (
            $scheduleStatus -eq 'Calculated-BatchOverwriteCandidate(Auto)' -or
            $scheduleStatus -eq 'Calculated-BatchOverwriteCandidate(Blank)'
        )

        if ($app270Action -eq 'Add') {
            $plannedWrite = 'BLOCKED_APP270_ADD'
        }
        elseif ($app272Action -eq 'New') {
            if ($errors.Count -gt 0) {
                $plannedWrite = 'BLOCKED_NEW_ERRORS'
            }
            elseif (-not $orderNo) {
                $plannedWrite = 'BLOCKED_NEW_NO_ODERNO'
            }
            elseif ($app270Action -ne 'Exists(NoUpdate)') {
                $plannedWrite = 'BLOCKED_NEW_APP270_NOT_READY'
            }
            else {
                $plannedWrite = 'POST:NEW'
            }
        }
        else {
            # Phase1Cでは既存レコードは一切書かない。
            if ($app272Action -eq 'Update(BaseOnly)' -or
                $lookupAction -eq 'Sync' -or
                $isScheduleWriteCandidate) {
                $plannedWrite = 'BLOCKED_EXISTING_WRITE'
            }
        }

        $details.Add([pscustomobject][ordered]@{
            Row = $index
            発注番号 = $po
            ODERNO = $orderNo
            指示先名 = $vendor
            手配日 = $orderDate
            納期 = $dueDate
            App270 = $app270Action
            App272 = $app272Action
            Lookup = $lookupAction
            BaseDiffFields = ($baseDiffFields -join ',')
            旧ODERNO = $oldOrderNo
            旧指示先名_VENDORTEXT = $oldVendorText
            旧指示先名_VENDORDD = $oldVendorDD
            旧手配日 = $oldOrderDate
            旧納期 = $oldDueDate
            App272RecordId = $recordId272
            App272Revision = $revision272
            現Lookup = $currentLookup
            現K日程種別 = $currentKScheduleType
            K日程優先判定 = $(if ($scheduleTargetKind) { $scheduleTargetKind } else { $scheduleStatus })
            現K加工着手日 = $currentKStart
            現K完成検査日 = $currentKInspection
            ManualWarning = $manualWarning
            ExistingAutoWarning = $existingAutoWarning
            ExistingAutoIssues = ($existingAutoIssues -join ',')
            ExistingBatchWarning = $existingBatchWarning
            ExistingBatchIssues = ($existingBatchIssues -join ',')
            RuleStatus = $ruleStatus
            検日前倒し暦日 = $inspectionDays
            着日前倒し暦日 = $startDays
            完成検査日_基準日 = $inspectionBase
            K完成検査日_候補 = $inspection
            加工着手日_基準日 = $startBase
            K加工着手日_候補 = $start
            K日程種別_候補 = $proposedKType
            K暫定設定_候補 = $proposedKOnOff
            K計算根拠_候補 = $calcBasis
            ScheduleDecision = $scheduleStatus
            休日補正 = '検査=前営業日 / 着手=翌営業日'
            備考 = $scheduleNote
            Errors = ($errors -join ' | ')
            WRITE = $plannedWrite
        })
    }


    # -------------------- Phase 1C preflight / execute --------------------
    $createCandidates = @($details | Where-Object { $_.WRITE -eq 'POST:NEW' })
    $blockedApp270Add = @($details | Where-Object WRITE -eq 'BLOCKED_APP270_ADD')
    $blockedExistingWrite = @($details | Where-Object WRITE -eq 'BLOCKED_EXISTING_WRITE')
    $createCountPlanned = $createCandidates.Count

    if ($Execute) {
        if ($blockedApp270Add.Count -gt 0) {
            throw "安全停止: App270追加候補が $($blockedApp270Add.Count) 件あります。Phase1C v0.1ではApp270追加を禁止しています。"
        }
        if ($blockedExistingWrite.Count -gt 0) {
            throw "安全停止: 既存App272の書込み候補が $($blockedExistingWrite.Count) 件あります。Phase1Cでは既存PUTを禁止しています。"
        }
        if ($createCountPlanned -ne 1) {
            throw "安全停止: 新規App272候補は1件である必要があります。現在=$createCountPlanned 件"
        }
        if ($createCountPlanned -gt $MaxCreates) {
            throw "安全停止: POST予定件数 $createCountPlanned が MaxCreates=$MaxCreates を超えています。"
        }

        $d = $createCandidates[0]

        if ($d.発注番号 -ne $ExpectedNewPo) {
            throw "安全停止: 新規候補の発注番号 '$($d.発注番号)' が ExpectedNewPo '$ExpectedNewPo' と一致しません。"
        }
        if ($d.App270 -ne 'Exists(NoUpdate)') {
            throw "安全停止: App270未確認です。App270=$($d.App270)"
        }
        if ($d.Lookup -ne 'SyncAfterCreate') {
            throw "安全停止: Lookup判定が想定外です。Lookup=$($d.Lookup)"
        }
        if (-not [string]::IsNullOrWhiteSpace([string]$d.Errors)) {
            throw "安全停止: 新規候補にErrorsがあります: $($d.Errors)"
        }
        if ([string]::IsNullOrWhiteSpace([string]$d.ODERNO) -or
            [string]::IsNullOrWhiteSpace([string]$d.発注番号) -or
            [string]::IsNullOrWhiteSpace([string]$d.指示先名) -or
            [string]::IsNullOrWhiteSpace([string]$d.手配日) -or
            [string]::IsNullOrWhiteSpace([string]$d.納期)) {
            throw "安全停止: 新規作成に必要な基幹項目が空欄です。"
        }

        # 念のため、GET済みApp272に同一発注番号がないことをExecute直前にも確認。
        if ($app272ByPo.ContainsKey([string]$d.発注番号)) {
            throw "安全停止: 発注番号 $($d.発注番号) は既にApp272に存在します。"
        }

        # Phase1C v0.1ではK日程を新規レコードへ書かない。
        $recordFields = @{
            '数値'        = @{ value = [string]$d.発注番号 }
            'ODERNO'      = @{ value = [string]$d.ODERNO }
            'VENDOR_TEXT' = @{ value = [string]$d.指示先名 }
            'VENDOR_DD'   = @{ value = [string]$d.指示先名 }
            '日付'        = @{ value = [string]$d.手配日 }
            '日付_1'      = @{ value = [string]$d.納期 }
            'ルックアップ' = @{ value = [string]$d.ODERNO }
        }

        Write-Host ''
        Write-Host "=== EXECUTE START: App272 POST 1件 ===" -ForegroundColor Yellow
        Write-Host ("POST Row={0} PO={1} ODERNO={2} Vendor={3}" -f `
            $d.Row, $d.発注番号, $d.ODERNO, $d.指示先名)
        Write-Host ("Fields={0}" -f (($recordFields.Keys | Sort-Object) -join ','))

        $resp = Invoke-KintoneCreateRecord `
            -BaseUrl $BaseUrl `
            -AppId $App272 `
            -Token $ApiToken272 `
            -RecordFields $recordFields

        $d.WRITE = 'DONE:POST:NEW'
        Write-Host ("=== EXECUTE COMPLETE: App272新規1件 / id={0} revision={1} ===" -f `
            $resp.id, $resp.revision) -ForegroundColor Green
    }

    $stamp = (Get-Date).ToString('yyyyMMdd_HHmmss')
    $modeName = $(if ($Execute) { "Execute" } else { "DryRun" })
    $detailPath = Join-Path $LogDirectory "Import-App272_Phase1C_${modeName}_Detail_$stamp.csv"
    $summaryPath = Join-Path $LogDirectory "Import-App272_Phase1C_${modeName}_Summary_$stamp.txt"
    $details | Export-Csv -LiteralPath $detailPath -NoTypeInformation -Encoding UTF8

    $countApp270Add = @($details | Where-Object App270 -eq 'Add').Count
    $countApp272New = @($details | Where-Object App272 -eq 'New').Count
    $countApp272Update = @($details | Where-Object App272 -eq 'Update(BaseOnly)').Count
    $countApp272NoChange = @($details | Where-Object App272 -eq 'NOCHANGE').Count
    $countManualDueWarning = @($details | Where-Object ManualWarning -eq 'ManualScheduleDueDateChanged').Count
    $countExistingAutoWarning = @($details | Where-Object ExistingAutoWarning -eq 'ExistingAutoScheduleInvalid').Count
    $countExistingBatchWarning = @($details | Where-Object ExistingBatchWarning -eq 'ExistingBatchScheduleInvalid').Count
    $countExistingGeneratedWarning = $countExistingAutoWarning + $countExistingBatchWarning
    $countProtectedManual = @($details | Where-Object ScheduleDecision -eq 'ProtectedManual').Count
    $countProtectedBatch = @($details | Where-Object ScheduleDecision -eq 'ProtectedBatch').Count
    $countProtectedUnknown = @($details | Where-Object ScheduleDecision -eq 'ProtectedUnknownType').Count
    $countAutoToBatch = @($details | Where-Object ScheduleDecision -eq 'Calculated-BatchOverwriteCandidate(Auto)').Count
    $countBlankToBatch = @($details | Where-Object ScheduleDecision -eq 'Calculated-BatchOverwriteCandidate(Blank)').Count
    $countNewBatch = @($details | Where-Object ScheduleDecision -eq 'Calculated-NewBatchCandidate').Count
    $countLookup = @($details | Where-Object { $_.Lookup -eq 'Sync' -or $_.Lookup -eq 'SyncAfterCreate' }).Count
    $countSchedule = @($details | Where-Object { $_.ScheduleDecision -like 'Calculated-*' }).Count
    $countCompany = @($details | Where-Object K計算根拠_候補 -eq 'Company').Count
    $countProvisional = @($details | Where-Object K計算根拠_候補 -eq 'Provisional').Count
    $countReview = @($details | Where-Object ScheduleDecision -eq 'Calculated-ReviewOnly').Count
    $countMasterMissing = @($details | Where-Object ScheduleDecision -eq 'ScheduleMasterNotFound').Count
    $countInvalidMasterStatus = @($details | Where-Object ScheduleDecision -eq 'ProtectedUnknownRuleStatus').Count
    $countErrors = @($details | Where-Object { -not [string]::IsNullOrWhiteSpace($_.Errors) }).Count

    $summary = @"
=== Import-App272 Phase1C Summary ===
RunAt                         : $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')
Mode                          : $(if ($Execute) { 'EXECUTE' } else { 'DRYRUN' })
InputCSV                      : $CsvPath
CSVRows                       : $($csvRows.Count)
App270Records                 : $($app270Records.Count)
App272Records                 : $($app272Records.Count)
App270 Add candidates         : $countApp270Add
App272 New candidates         : $countApp272New
App272 Update(BaseOnly)       : $countApp272Update
App272 NOCHANGE               : $countApp272NoChange
Manual due-date warnings      : $countManualDueWarning
Existing Auto schedule warnings : $countExistingAutoWarning
Existing Batch schedule warnings: $countExistingBatchWarning
Existing generated warnings      : $countExistingGeneratedWarning
Protected Manual              : $countProtectedManual
Protected Batch               : $countProtectedBatch
Protected Unknown type        : $countProtectedUnknown
Auto -> Batch candidates      : $countAutoToBatch
Blank -> Batch candidates     : $countBlankToBatch
New Batch candidates          : $countNewBatch
Lookup sync candidates        : $countLookup
Schedule calculated           : $countSchedule
  Company                     : $countCompany
  Provisional                 : $countProvisional
  ReviewOnly(master要確認)     : $countReview
ScheduleMasterNotFound        : $countMasterMissing
Invalid master status         : $countInvalidMasterStatus
Record errors                 : $countErrors
K計算根拠 field exists        : $kCalculationBasisFieldExists
Calendar coverage             : $calendarMin .. $calendarMax
Base diff scope               : ODERNO / VENDOR_TEXT / VENDOR_DD / 日付 / 日付_1 (verified mapping only)
Vendor sync rule              : CSV 指示先名 -> VENDOR_TEXT + VENDOR_DD
Schedule type rule            : Manual > Batch > Auto (active in DryRun decision)
Existing Manual/Batch          : Protected
Existing Auto/blank            : Batch recalculation candidate
New schedule source            : Batch
Schedule algorithm            : 暦日差引 → 検査=前営業日 / 着手=翌営業日
Planned POST creates          : $createCountPlanned
Blocked existing writes       : $($blockedExistingWrite.Count)
MaxCreates                    : $MaxCreates
WRITE                         : $(if ($Execute) { 'POST executed for one verified new App272 only' } else { 'NONE' })
DetailLog                     : $detailPath
Warnings                      : $($script:Warnings.Count)
ElapsedSeconds                : $([math]::Round(((Get-Date)-$started).TotalSeconds,2))
"@

    $summary | Set-Content -LiteralPath $summaryPath -Encoding UTF8
    Write-Host ''
    Write-Host $summary

    if (-not $kCalculationBasisFieldExists -and -not $SkipKintone) {
        Add-Warning 'App272に K計算根拠 フィールドは確認できません。ScheduleのExecuteは安全停止します。'
    }

    if ($countManualDueWarning -gt 0) {
        Add-Warning "Manual日程のまま納期が変更される候補が $countManualDueWarning 件あります。詳細ログの ManualWarning / 旧納期 / 納期 / 現K日程を確認してください。"
    }

    if ($countExistingBatchWarning -gt 0) {
        Add-Warning "既存Batch日程に矛盾が $countExistingBatchWarning 件あります。書換えは行っていません。詳細ログの ExistingBatchWarning / ExistingBatchIssues を確認してください。"
    }

    if ($countProtectedUnknown -gt 0) {
        Add-Warning "未知のK日程種別を持つ既存レコードが $countProtectedUnknown 件あります。安全側で保護しました。"
    }

    if ($countErrors -gt 0) {
        Write-Warning "レコードエラー $countErrors 件。詳細CSVを確認してください。"
        exit 40
    }

    exit 0
}
catch {
    Write-Error $_.Exception.Message
    exit 10
}
