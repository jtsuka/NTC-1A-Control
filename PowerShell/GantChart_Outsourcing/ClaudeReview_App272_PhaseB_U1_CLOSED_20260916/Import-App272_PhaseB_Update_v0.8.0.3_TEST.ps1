#requires -Version 5.1
<#
.SYNOPSIS
  App292 Phase B 既存PUT TEST v0.8.0.3

.DESCRIPTION
  App272 Daily Import Rev.2.1 / Phase B のApp292専用実装。
  既存レコードのCSV差分を検出し、Batch/Auto/Manualの扱いに従って
  UPDATE_BASE_ONLY / UPDATE_RECALC / MANUAL_BASE_ONLY / NOCHANGE を分類する。

  v0.8.0 TESTの主目的はU1:
    既存Batchの納期変更
      -> 日付_1更新
      -> K加工着手日 / K完成検査日 再計算
      -> Batch / ON / Company(or Provisional) 維持
      -> Manifest JSON生成
      -> revision + before + changeHash ガード
      -> PUT後再GET検証

  安全条件:
    - App292固定。本番App272では動作しない。
    - 既定はDryRun。
    - Executeには -ConfirmExecute 'APP292-PHASEB-EXECUTE' 必須。
    - Execute時は -ExpectedUpdatePo で対象集合を全件明示。
    - 新規POSTは行わない。
    - 未知Vendor/Staff optionの自動追加はv0.8.0 TESTでは未実装。
      Vendor/Staff差分を検出した場合は安全停止（U11で後続実装）。
    - Policy moduleのU12Result=ATOMIC_ALL_OR_NOTHINGを確認する。
    - Manifest JSONを正式な機械可読出力として生成する。
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)]
    [string]$CsvPath,

    [string]$ScheduleMasterPath = 'C:\HPDB\加工先別_日程暫定シート_20260910.xlsx',
    [string]$CalendarPath       = 'C:\HPDB\カレンダー(2023).xlsx',
    [string]$JapanHolidayPath   = 'C:\HPDB\JapanHolidays_2026_2027.csv',
    [string]$PolicyPath         = 'C:\HPDB\App272-PhaseB-PutPolicy_v0.1.ps1',
    [string]$LogDirectory       = 'C:\HPDB\logs',

    [string]$BaseUrl = 'https://bcurbkixz609.cybozu.com',
    [int]$App292 = 292,
    [string]$ApiToken292 = '',

    [switch]$DryRun,
    [switch]$Execute,
    [string]$ConfirmExecute = '',
    [string[]]$ExpectedUpdatePo = @(),

    [ValidateRange(1,50)]
    [int]$MaxUpdates = 10
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

function ConvertFrom-SecureStringPlain {
    param([Security.SecureString]$Secure)
    $ptr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($Secure)
    try { return [Runtime.InteropServices.Marshal]::PtrToStringBSTR($ptr) }
    finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($ptr) }
}

function Get-ApiTokenInteractive {
    param([string]$Label)
    return ConvertFrom-SecureStringPlain (Read-Host "$Label API Token" -AsSecureString)
}

function Import-CsvAllowDuplicateHeaders {
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
        $seen = @{}
        $headers = New-Object System.Collections.Generic.List[string]
        foreach ($raw in $rawHeaders) {
            $name = [string]$raw
            if ($headers.Count -eq 0) { $name = $name.TrimStart([char]0xFEFF) }
            if ([string]::IsNullOrWhiteSpace($name)) { $name = '__BLANK_HEADER' }

            if ($seen.ContainsKey($name)) {
                $seen[$name]++
                $headers.Add(('{0}__DUP{1}' -f $name,$seen[$name]))
            } else {
                $seen[$name] = 1
                $headers.Add($name)
            }
        }

        $rows = New-Object System.Collections.Generic.List[object]
        $lineNo = 1
        while (-not $parser.EndOfData) {
            $lineNo++
            $fields = @($parser.ReadFields())
            if ($fields.Count -ne $headers.Count) {
                throw "CSV列数不一致: 行=$lineNo header=$($headers.Count) row=$($fields.Count)"
            }
            $o = [ordered]@{}
            for ($i=0; $i -lt $headers.Count; $i++) { $o[$headers[$i]] = [string]$fields[$i] }
            $rows.Add([pscustomobject]$o)
        }
        return $rows.ToArray()
    }
    finally {
        if ($null -ne $parser) { $parser.Close(); $parser.Dispose() }
    }
}

function Normalize-DateStrict {
    param([AllowNull()][AllowEmptyString()][string]$Value)
    if ([string]::IsNullOrWhiteSpace($Value)) { return $null }
    $v = $Value.Trim()
    if ($v -match '^\d{2}[-/]\d{1,2}[-/]\d{1,2}$') { throw "2桁年は受理しません: '$v'" }
    foreach ($f in @('yyyy/M/d','yyyy/MM/dd','yyyy-M-d','yyyy-MM-dd')) {
        $dt = [datetime]::MinValue
        if ([datetime]::TryParseExact($v,$f,[Globalization.CultureInfo]::InvariantCulture,[Globalization.DateTimeStyles]::None,[ref]$dt)) {
            return $dt.ToString('yyyy-MM-dd')
        }
    }
    throw "日付形式が不正です: '$v'"
}

function Get-KValue {
    param($Record,[string]$FieldCode)
    if ($null -eq $Record) { return '' }
    $p = $Record.PSObject.Properties[$FieldCode]
    if ($null -eq $p -or $null -eq $p.Value) { return '' }
    $vp = $p.Value.PSObject.Properties['value']
    if ($null -eq $vp -or $null -eq $vp.Value) { return '' }
    return ([string]$vp.Value).Trim()
}

function Invoke-KintoneGetAll {
    param([string]$Token)
    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $all = New-Object System.Collections.Generic.List[object]
    $lastId = 0
    while ($true) {
        $query = '$id > {0} order by $id asc limit 500' -f $lastId
        $uri = '{0}/k/v1/records.json?app={1}&query={2}' -f `
            $BaseUrl.TrimEnd('/'),$App292,[Uri]::EscapeDataString($query)
        $resp = Invoke-RestMethod -Method Get -Uri $uri -Headers $headers
        $r = @($resp.records)
        foreach ($x in $r) { $all.Add($x) }
        if ($r.Count -lt 500) { break }
        $lastId = [int64](Get-KValue $r[-1] '$id')
    }
    return $all.ToArray()
}

function Import-CompanyCalendar {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) { throw "会社カレンダーが見つかりません: $Path" }
    $excel=$null; $book=$null; $sheet=$null; $map=@{}
    try {
        $excel = New-Object -ComObject Excel.Application
        $excel.Visible=$false; $excel.DisplayAlerts=$false
        $book = $excel.Workbooks.Open($Path,$null,$true)
        $sheet = $book.Worksheets.Item('カレンダー')
        $lastRow = $sheet.UsedRange.Rows.Count
        for ($r=2; $r -le $lastRow; $r++) {
            $rawDate=$sheet.Cells.Item($r,2).Value2
            $status=[string]$sheet.Cells.Item($r,4).Text
            if ($null -eq $rawDate -or [string]::IsNullOrWhiteSpace($status)) { continue }
            $dt=$null
            if ($rawDate -is [double] -or $rawDate -is [int]) { $dt=[datetime]::FromOADate([double]$rawDate) }
            else {
                $tmp=[datetime]::MinValue
                if ([datetime]::TryParse([string]$rawDate,[ref]$tmp)) { $dt=$tmp }
            }
            if ($null -ne $dt) { $map[$dt.ToString('yyyy-MM-dd')]=$status.Trim() }
        }
    } finally {
        if ($null -ne $book) { $book.Close($false) | Out-Null }
        if ($null -ne $excel) { $excel.Quit() }
        foreach ($o in @($sheet,$book,$excel)) {
            if ($null -ne $o) { try { [void][Runtime.InteropServices.Marshal]::ReleaseComObject($o) } catch {} }
        }
        [gc]::Collect(); [gc]::WaitForPendingFinalizers()
    }
    if ($map.Count -eq 0) { throw '会社カレンダーから営業日データを取得できませんでした。' }
    return $map
}

function Import-ScheduleMasterExcel {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) { throw "日程マスタExcelが見つかりません: $Path" }

    $excel=$null; $book=$null; $sheet=$null; $used=$null
    $rows = New-Object System.Collections.Generic.List[object]
    try {
        $excel = New-Object -ComObject Excel.Application
        $excel.Visible=$false; $excel.DisplayAlerts=$false
        $book=$excel.Workbooks.Open($Path,$null,$true)
        $sheet=$book.Worksheets.Item('加工先別設定')
        $used=$sheet.UsedRange
        $lastRow=$used.Rows.Count

        $headerRow=0
        for ($r=1; $r -le [Math]::Min($lastRow,20); $r++) {
            if ((([string]$sheet.Cells.Item($r,1).Text) -replace '\s','').Trim() -eq '加工先（指示先名）') {
                $headerRow=$r; break
            }
        }
        if ($headerRow -eq 0) { throw '日程マスタの見出しが見つかりません。' }

        $started=$false
        for ($r=$headerRow+1; $r -le $lastRow; $r++) {
            $vendor=([string]$sheet.Cells.Item($r,1).Text).Trim()
            if ([string]::IsNullOrWhiteSpace($vendor)) {
                if ($started) { break } else { continue }
            }
            $started=$true
            $rows.Add([pscustomobject]@{
                指示先名=$vendor
                完成検査日前倒し日数=([string]$sheet.Cells.Item($r,3).Text).Trim()
                加工着手日前倒し日数=([string]$sheet.Cells.Item($r,4).Text).Trim()
                使用区分=([string]$sheet.Cells.Item($r,5).Text).Trim()
                備考=([string]$sheet.Cells.Item($r,6).Text).Trim()
            })
        }
    } finally {
        if ($null -ne $book) { $book.Close($false) | Out-Null }
        if ($null -ne $excel) { $excel.Quit() }
        foreach ($o in @($used,$sheet,$book,$excel)) {
            if ($null -ne $o) { try { [void][Runtime.InteropServices.Marshal]::ReleaseComObject($o) } catch {} }
        }
        [gc]::Collect(); [gc]::WaitForPendingFinalizers()
    }
    if ($rows.Count -eq 0) { throw '日程マスタExcelからデータを取得できませんでした。' }
    return $rows.ToArray()
}

function Import-HolidaySet {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) { throw "日本祝日CSVが見つかりません: $Path" }
    $set=@{}
    foreach ($r in @(Import-Csv -LiteralPath $Path -Encoding UTF8)) {
        if (-not [string]::IsNullOrWhiteSpace([string]$r.Date)) {
            $set[(Normalize-DateStrict ([string]$r.Date))]=$true
        }
    }
    return $set
}

function Get-DayState {
    param([datetime]$Date,[hashtable]$CompanyCalendar,[hashtable]$JapanHolidays)
    $key=$Date.ToString('yyyy-MM-dd')
    if ($CompanyCalendar.ContainsKey($key)) {
        return [pscustomobject]@{IsWorking=([string]$CompanyCalendar[$key] -eq '営業日'); Source='Company'}
    }
    $dow=$Date.DayOfWeek
    $weekend=($dow -eq [DayOfWeek]::Saturday -or $dow -eq [DayOfWeek]::Sunday)
    return [pscustomobject]@{IsWorking=(-not $weekend -and -not $JapanHolidays.ContainsKey($key)); Source='Provisional'}
}

function Move-ToWorkingDay {
    param([datetime]$Date,[int]$Direction,[hashtable]$CompanyCalendar,[hashtable]$JapanHolidays)
    $d=$Date; $usedProv=$false
    for ($i=0;$i -lt 40;$i++) {
        $s=Get-DayState -Date $d -CompanyCalendar $CompanyCalendar -JapanHolidays $JapanHolidays
        if ($s.Source -eq 'Provisional') { $usedProv=$true }
        if ($s.IsWorking) { return [pscustomobject]@{Date=$d;UsedProvisional=$usedProv} }
        $d=$d.AddDays($Direction)
    }
    throw "40日以内に営業日を検出できません: $($Date.ToString('yyyy-MM-dd'))"
}

function Calculate-ScheduleCompat {
    param(
        [string]$DueDate,[string]$OrderDate,
        [int]$InspectionDaysBeforeDue,[int]$StartDaysBeforeInspection,
        [hashtable]$CompanyCalendar,[hashtable]$JapanHolidays
    )
    $due=[datetime]::ParseExact($DueDate,'yyyy-MM-dd',[Globalization.CultureInfo]::InvariantCulture)
    $order=[datetime]::ParseExact($OrderDate,'yyyy-MM-dd',[Globalization.CultureInfo]::InvariantCulture)
    $inspectionBase=$due.AddDays(-1*$InspectionDaysBeforeDue)
    $inspectionMoved=Move-ToWorkingDay -Date $inspectionBase -Direction -1 -CompanyCalendar $CompanyCalendar -JapanHolidays $JapanHolidays
    $startBase=$inspectionMoved.Date.AddDays(-1*$StartDaysBeforeInspection)
    $startMoved=Move-ToWorkingDay -Date $startBase -Direction 1 -CompanyCalendar $CompanyCalendar -JapanHolidays $JapanHolidays

    if ($startMoved.Date -gt $inspectionMoved.Date -or $inspectionMoved.Date -gt $due) {
        return [pscustomobject]@{Ok=$false;Reason='InvalidDateRelation'}
    }
    if ($startMoved.Date -lt $order) {
        return [pscustomobject]@{Ok=$false;Reason='ShortLeadTime'}
    }
    $prov=($inspectionMoved.UsedProvisional -or $startMoved.UsedProvisional)
    return [pscustomobject]@{
        Ok=$true; Reason=''
        Start=$startMoved.Date.ToString('yyyy-MM-dd')
        Inspection=$inspectionMoved.Date.ToString('yyyy-MM-dd')
        Basis=$(if($prov){'Provisional'}else{'Company'})
    }
}

function Get-Sha256Hex {
    param([string]$Text)
    $sha=[Security.Cryptography.SHA256]::Create()
    try {
        $bytes=[Text.Encoding]::UTF8.GetBytes($Text)
        return (($sha.ComputeHash($bytes) | ForEach-Object { $_.ToString('x2') }) -join '')
    } finally { $sha.Dispose() }
}

function New-CanonicalChangeString {
    param(
        [string]$Po,[string]$RecordId,[string]$Revision,
        [System.Collections.IDictionary]$Before,
        [System.Collections.IDictionary]$After,
        [string[]]$ChangedFields
    )
    $parts = New-Object System.Collections.Generic.List[string]
    $parts.Add("PO=$Po")
    $parts.Add("RecordId=$RecordId")
    $parts.Add("Revision=$Revision")
    foreach ($f in @($ChangedFields | Sort-Object)) {
        $bv = if ($Before.Contains($f)) { [string]$Before[$f] } else { '' }
        $av = if ($After.Contains($f)) { [string]$After[$f] } else { '' }
        $parts.Add(("{0}|BEFORE={1}|AFTER={2}" -f $f,$bv,$av))
    }
    return ($parts -join "`n")
}

function Convert-OrderedToHashtable {
    param([System.Collections.IDictionary]$Dictionary)
    $h=@{}
    foreach($k in $Dictionary.Keys){$h[$k]=[string]$Dictionary[$k]}
    return $h
}

function Test-DictionaryEqual {
    param(
        [System.Collections.IDictionary]$Expected,
        [System.Collections.IDictionary]$Actual,
        [string[]]$Fields
    )
    foreach ($f in $Fields) {
        $a = if ($Expected.Contains($f)) {[string]$Expected[$f]} else {''}
        $b = if ($Actual.Contains($f)) {[string]$Actual[$f]} else {''}
        if ($a -ne $b) { return $false }
    }
    return $true
}

function Invoke-MultiPut {
    param([string]$Token,[object[]]$Items)
    $headers=@{'X-Cybozu-API-Token'=$Token}
    $uri=$BaseUrl.TrimEnd('/')+'/k/v1/records.json'
    $body=[ordered]@{app=$App292;records=@($Items)} | ConvertTo-Json -Depth 20
    return Invoke-RestMethod -Method Put -Uri $uri -Headers $headers -ContentType 'application/json; charset=utf-8' -Body $body
}

# ---------- safety / policy ----------
if ($App292 -ne 292) { throw "SafetyStop: Phase B TEST is fixed to App292." }
if ($DryRun -and $Execute) { throw 'SafetyStop: -DryRun and -Execute cannot be combined.' }
if (-not $DryRun -and -not $Execute) { $DryRun=$true }
if ($Execute -and $ConfirmExecute -ne 'APP292-PHASEB-EXECUTE') {
    throw "SafetyStop: Execute requires -ConfirmExecute 'APP292-PHASEB-EXECUTE'"
}
if (-not (Test-Path -LiteralPath $PolicyPath)) { throw "Policy module not found: $PolicyPath" }
. $PolicyPath
$policy=Get-App272PhaseBPolicy
if ($policy.U12Result -ne 'ATOMIC_ALL_OR_NOTHING' -or -not $policy.MultiPutAllowed) {
    throw "SafetyStop: Phase B policy does not allow multi PUT."
}
if ($MaxUpdates -gt [int]$policy.MaxAutoUpdates) {
    throw "SafetyStop: MaxUpdates=$MaxUpdates exceeds policy MaxAutoUpdates=$($policy.MaxAutoUpdates)"
}

if (-not (Test-Path -LiteralPath $LogDirectory)) { New-Item -ItemType Directory -Path $LogDirectory -Force | Out-Null }
if (-not (Test-Path -LiteralPath $CsvPath)) { throw "CSV not found: $CsvPath" }

# ---------- input ----------
Write-Host '[S1] CSV読込...'
$csvRows=@(Import-CsvAllowDuplicateHeaders -Path $CsvPath)
if ($csvRows.Count -eq 0) { throw '入力CSVが0件です。' }
$required=@('発注番号','手配書番号','手配日','納期','指示先名','手配担当者名')
$props=@($csvRows[0].PSObject.Properties.Name)
foreach($f in $required){if($props -notcontains $f){throw "必須列がありません: $f"}}

Write-Host '[S2] 日程マスタ・カレンダー読込...'
$master=@{}
foreach($m in @(Import-ScheduleMasterExcel -Path $ScheduleMasterPath)){ $master[[string]$m.指示先名.Trim()]=$m }
$company=Import-CompanyCalendar -Path $CalendarPath
$holidays=Import-HolidaySet -Path $JapanHolidayPath

if ([string]::IsNullOrWhiteSpace($ApiToken292)) { $ApiToken292=Get-ApiTokenInteractive 'App292' }
Write-Host '[S3] App292 GET...'
$appRecords=@(Invoke-KintoneGetAll -Token $ApiToken292)
$byPo=@{}
foreach($r in $appRecords){
    $po=Get-KValue $r '数値'
    if($po){
        if($byPo.ContainsKey($po)){throw "SafetyStop: App292 duplicate PO=$po"}
        $byPo[$po]=$r
    }
}

Write-Host '[S4] 差分分類・日程再計算...'
$updates=New-Object System.Collections.Generic.List[object]
$nonUpdates=New-Object System.Collections.Generic.List[object]

foreach($row in $csvRows){
    $po=([string]$row.発注番号).Trim()
    $vendor=([string]$row.指示先名).Trim()
    $staff=([string]$row.手配担当者名).Trim()
    $orderno=([string]$row.手配書番号).Trim()
    $orderDate=Normalize-DateStrict ([string]$row.手配日)
    $dueDate=Normalize-DateStrict ([string]$row.納期)

    if(-not $po){throw '発注番号が空欄です。'}
    if(-not $byPo.ContainsKey($po)){
        $nonUpdates.Add([pscustomobject]@{po=$po;classification='NOT_EXISTING';reason='Phase B does not POST'})
        continue
    }

    $r=$byPo[$po]
    $recordId=Get-KValue $r '$id'
    $revision=Get-KValue $r '$revision'
    $kType=Get-KValue $r 'K日程種別'

    $before=[ordered]@{
        ODERNO=Get-KValue $r 'ODERNO'
        VENDOR_TEXT=Get-KValue $r 'VENDOR_TEXT'
        VENDOR_DD=Get-KValue $r 'VENDOR_DD'
        STAFF_TEXT=Get-KValue $r 'STAFF_TEXT'
        STAFF_DD=Get-KValue $r 'STAFF_DD'
        日付=Get-KValue $r '日付'
        日付_1=Get-KValue $r '日付_1'
        ルックアップ=Get-KValue $r 'ルックアップ'
        K加工着手日=Get-KValue $r 'K加工着手日'
        K完成検査日=Get-KValue $r 'K完成検査日'
        K日程種別=$kType
        K暫定設定=Get-KValue $r 'K暫定設定'
        K計算根拠=Get-KValue $r 'K計算根拠'
    }

    $after=[ordered]@{}
    foreach($k in $before.Keys){$after[$k]=[string]$before[$k]}

    # 基幹CSV正本
    $after.ODERNO=$orderno
    $after.VENDOR_TEXT=$vendor
    $after.VENDOR_DD=$vendor
    $after.STAFF_TEXT=$staff
    $after.STAFF_DD=$staff
    $after.日付=$orderDate
    $after.日付_1=$dueDate
    $after.ルックアップ=$orderno

    $baseDiff=New-Object System.Collections.Generic.List[string]
    foreach($f in @('ODERNO','VENDOR_TEXT','VENDOR_DD','STAFF_TEXT','STAFF_DD','日付','日付_1','ルックアップ')){
        if([string]$before[$f] -ne [string]$after[$f]){$baseDiff.Add($f)}
    }

    if($baseDiff.Count -eq 0){
        $nonUpdates.Add([pscustomobject]@{po=$po;classification='NOCHANGE';reason='CSV business values equal'})
        continue
    }

    # v0.8.0 TESTでは未知option lifecycleはU11へ分離。Vendor/Staff差分は安全停止。
    if($baseDiff -contains 'VENDOR_TEXT' -or $baseDiff -contains 'VENDOR_DD' -or
       $baseDiff -contains 'STAFF_TEXT' -or $baseDiff -contains 'STAFF_DD'){
        throw "SafetyStop: v0.8.0 TEST does not update Vendor/Staff yet. PO=$po Diff=$($baseDiff -join ',')"
    }

    $classification=''
    $scheduleReason=''
    $recalc=$false

    switch($kType){
        'Manual' {
            $classification='MANUAL_BASE_ONLY'
            # K fields untouched
        }
        'Batch' {
            $classification='UPDATE_RECALC'
            $recalc=$true
        }
        'Auto' {
            $classification='UPDATE_RECALC'
            $recalc=$true
            $after.K日程種別='Batch'
        }
        '' {
            $classification='UPDATE_RECALC'
            $recalc=$true
            $after.K日程種別='Batch'
        }
        default {
            throw "SafetyStop: unknown K日程種別 '$kType' PO=$po"
        }
    }

    if($recalc){
        if(-not $master.ContainsKey($vendor)){
            $after.K加工着手日=$orderDate
            $after.K完成検査日=$dueDate
            $after.K日程種別='Batch'
            $after.K暫定設定='ON'
            $after.K計算根拠=''
            $scheduleReason='Fallback-MasterNotFound'
        } else {
            $m=$master[$vendor]
            $rule=([string]$m.使用区分).Trim()
            if($rule -ne '使用'){throw "SafetyStop: schedule master status=$rule PO=$po Vendor=$vendor"}
            $i=0;$s=0
            if(-not [int]::TryParse(([string]$m.完成検査日前倒し日数).Trim(),[ref]$i) -or
               -not [int]::TryParse(([string]$m.加工着手日前倒し日数).Trim(),[ref]$s)){
                throw "SafetyStop: schedule master days invalid PO=$po"
            }
            $calc=Calculate-ScheduleCompat -DueDate $dueDate -OrderDate $orderDate `
                -InspectionDaysBeforeDue $i -StartDaysBeforeInspection $s `
                -CompanyCalendar $company -JapanHolidays $holidays
            if($calc.Ok){
                $after.K加工着手日=$calc.Start
                $after.K完成検査日=$calc.Inspection
                $after.K日程種別='Batch'
                $after.K暫定設定='ON'
                $after.K計算根拠=$calc.Basis
                $scheduleReason='Calculated'
            } elseif($calc.Reason -eq 'ShortLeadTime'){
                $after.K加工着手日=$orderDate
                $after.K完成検査日=$dueDate
                $after.K日程種別='Batch'
                $after.K暫定設定='ON'
                $after.K計算根拠=''
                $scheduleReason='Fallback-ShortLeadTime'
            } else {
                throw "SafetyStop: schedule calculation failed Reason=$($calc.Reason) PO=$po"
            }
        }
    }

    $changed=New-Object System.Collections.Generic.List[string]
    foreach($f in $after.Keys){
        if([string]$before[$f] -ne [string]$after[$f]){$changed.Add([string]$f)}
    }
    if($changed.Count -eq 0){
        $nonUpdates.Add([pscustomobject]@{po=$po;classification='NOCHANGE_AFTER_RECALC';reason='No actual field change'})
        continue
    }

    $canonical=New-CanonicalChangeString -Po $po -RecordId $recordId -Revision $revision `
        -Before $before -After $after -ChangedFields $changed.ToArray()
    $hash=Get-Sha256Hex $canonical

    $updates.Add([pscustomobject][ordered]@{
        po=$po
        recordId=$recordId
        revision=$revision
        classification=$classification
        scheduleReason=$scheduleReason
        changedFields=$changed.ToArray()
        before=[pscustomobject]$before
        after=[pscustomobject]$after
        changeHash=$hash
    })
}

if($updates.Count -gt $MaxUpdates){throw "SafetyStop: update count=$($updates.Count) > MaxUpdates=$MaxUpdates"}

$actualSet=@($updates | ForEach-Object {$_.po} | Sort-Object -Unique)
if($Execute){
    if($ExpectedUpdatePo.Count -eq 0){throw 'SafetyStop: Execute requires -ExpectedUpdatePo.'}
    $expectedSet=@($ExpectedUpdatePo | ForEach-Object {[string]$_} | Sort-Object -Unique)
    if(($actualSet -join '|') -ne ($expectedSet -join '|')){
        throw "SafetyStop: ExpectedUpdatePo mismatch. actual=$($actualSet -join ',') expected=$($expectedSet -join ',')"
    }
}

Write-Host '[S5] Manifest/Detail生成...'
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'
$manifestPath=Join-Path $LogDirectory "App292_PhaseB_Manifest_$stamp.json"
$detailPath=Join-Path $LogDirectory "App292_PhaseB_Detail_$stamp.csv"

$manifest=[ordered]@{
    schemaVersion='1.0'
    runAt=(Get-Date).ToString('o')
    environment='TEST'
    targetApp=292
    inputCsvPath=$CsvPath
    policyVersion=[string]$policy.PolicyVersion
    u12Result=[string]$policy.U12Result
    putBatchSize=[int]$policy.PutBatchSize
    updates=$updates.ToArray()
    nonUpdates=$nonUpdates.ToArray()
    counts=[ordered]@{
        csvRows=$csvRows.Count
        updates=$updates.Count
        nonUpdates=$nonUpdates.Count
        errors=0
    }
}
$manifest | ConvertTo-Json -Depth 30 | Set-Content -LiteralPath $manifestPath -Encoding UTF8

$detailRows=@()
foreach($u in $updates){
    $detailRows += [pscustomobject]@{
        PO=$u.po
        RecordId=$u.recordId
        Revision=$u.revision
        Classification=$u.classification
        ScheduleReason=$u.scheduleReason
        ChangedFields=(@($u.changedFields)-join ',')
        OldDue=$u.before.日付_1
        NewDue=$u.after.日付_1
        OldKStart=$u.before.K加工着手日
        NewKStart=$u.after.K加工着手日
        OldKInspection=$u.before.K完成検査日
        NewKInspection=$u.after.K完成検査日
        KTypeAfter=$u.after.K日程種別
        KOnOffAfter=$u.after.K暫定設定
        KBasisAfter=$u.after.K計算根拠
        ChangeHash=$u.changeHash
    }
}
$detailRows | Export-Csv -LiteralPath $detailPath -NoTypeInformation -Encoding UTF8

Write-Host "=== App292 Phase B Update v0.8.0.3 TEST ==="
Write-Host "Mode             : $(if($Execute){'EXECUTE'}else{'DRYRUN'})"
Write-Host "App              : 292"
Write-Host "CSV rows         : $($csvRows.Count)"
Write-Host "Updates          : $($updates.Count)"
Write-Host "Policy           : $($policy.PolicyVersion)"
Write-Host "U12              : $($policy.U12Result)"
Write-Host "PutBatchSize     : $($policy.PutBatchSize)"
Write-Host "Manifest         : $manifestPath"
Write-Host "DetailLog        : $detailPath"
Write-Host ""

foreach($u in $updates){
    Write-Host ("PO={0} ID={1} Rev={2} Class={3}" -f $u.po,$u.recordId,$u.revision,$u.classification)
    Write-Host ("  Changed : {0}" -f (@($u.changedFields)-join ','))
    Write-Host ("  Due     : {0} -> {1}" -f $u.before.日付_1,$u.after.日付_1)
    Write-Host ("  KStart  : {0} -> {1}" -f $u.before.K加工着手日,$u.after.K加工着手日)
    Write-Host ("  KInspect: {0} -> {1}" -f $u.before.K完成検査日,$u.after.K完成検査日)
    Write-Host ("  KType   : {0} -> {1}" -f $u.before.K日程種別,$u.after.K日程種別)
    Write-Host ("  KOnOff  : {0} -> {1}" -f $u.before.K暫定設定,$u.after.K暫定設定)
    Write-Host ("  KBasis  : '{0}' -> '{1}'" -f $u.before.K計算根拠,$u.after.K計算根拠)
    Write-Host ("  Hash    : {0}" -f $u.changeHash)
}

if(-not $Execute){
    Write-Host ""
    Write-Host "WRITE            : NONE"
    Write-Host "RESULT           : DRYRUN PASS"
    exit 0
}

# ---------- Execute: exact pre-guard / multi PUT / post verify ----------
$latest=@(Invoke-KintoneGetAll -Token $ApiToken292)
$latestByPo=@{}
foreach($r in $latest){$p=Get-KValue $r '数値'; if($p){$latestByPo[$p]=$r}}

$putItems=New-Object System.Collections.Generic.List[object]
foreach($u in $updates){
    if(-not $latestByPo.ContainsKey($u.po)){throw "TOCTOU: PO missing $($u.po)"}
    $r=$latestByPo[$u.po]
    if((Get-KValue $r '$id') -ne $u.recordId){throw "TOCTOU: recordId mismatch PO=$($u.po)"}
    if((Get-KValue $r '$revision') -ne $u.revision){throw "TOCTOU: revision mismatch PO=$($u.po)"}

    $current=[ordered]@{}
    foreach($f in $u.before.PSObject.Properties.Name){$current[$f]=Get-KValue $r $f}
    $beforeDict=[ordered]@{}
    foreach($p in $u.before.PSObject.Properties){$beforeDict[$p.Name]=[string]$p.Value}
    if(-not (Test-DictionaryEqual -Expected $beforeDict -Actual $current -Fields @($beforeDict.Keys))){
        throw "TOCTOU: before values mismatch PO=$($u.po)"
    }

    $afterDict=[ordered]@{}
    foreach($p in $u.after.PSObject.Properties){$afterDict[$p.Name]=[string]$p.Value}
    $canonical=New-CanonicalChangeString -Po $u.po -RecordId $u.recordId -Revision $u.revision `
        -Before $beforeDict -After $afterDict -ChangedFields @($u.changedFields)
    $rehash=Get-Sha256Hex $canonical
    if($rehash -ne $u.changeHash){throw "TOCTOU: changeHash mismatch PO=$($u.po)"}

    $record=[ordered]@{}
    foreach($f in @($u.changedFields)){$record[$f]=@{value=[string]$afterDict[$f]}}
    $putItems.Add([ordered]@{id=[int64]$u.recordId;revision=[int64]$u.revision;record=$record})
}

# U1は1件。将来複数時はpolicy batch sizeで分割。
$allItems=$putItems.ToArray()
for($offset=0;$offset -lt $allItems.Count;$offset += [int]$policy.PutBatchSize){
    $end=[Math]::Min($offset+[int]$policy.PutBatchSize,$allItems.Count)
    $batch=@($allItems[$offset..($end-1)])
    [void](Invoke-MultiPut -Token $ApiToken292 -Items $batch)

    # バッチ後再GET検証
    $post=@(Invoke-KintoneGetAll -Token $ApiToken292)
    $postByPo=@{}
    foreach($r in $post){$p=Get-KValue $r '数値';if($p){$postByPo[$p]=$r}}
    $batchIds=@($batch | ForEach-Object {[string]$_.id})
    foreach($u in $updates | Where-Object {$batchIds -contains [string]$_.recordId}){
        $r=$postByPo[$u.po]
        $afterDict=[ordered]@{}
        foreach($p in $u.after.PSObject.Properties){$afterDict[$p.Name]=[string]$p.Value}
        foreach($f in @($u.changedFields)){
            $actual=Get-KValue $r $f
            if($actual -ne [string]$afterDict[$f]){
                throw "Post verification failed PO=$($u.po) Field=$f expected='$($afterDict[$f])' actual='$actual'"
            }
        }
    }
}

Write-Host ""
Write-Host "WRITE            : MULTI PUT EXECUTED"
Write-Host "RESULT           : EXECUTE PASS / post reGET verified"
