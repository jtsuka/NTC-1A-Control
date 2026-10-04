#requires -Version 5.1
<#
.SYNOPSIS
  App272 AcceptanceSync evidence exporter (READ ONLY).

.DESCRIPTION
  v0.1.3 DIAGNOSTIC CANDIDATE based on:
    App272_受入更新_成果残課題エビデンス仕様書_v0.3.1_20261004.md

  - GET only against App293/App272.
  - SourceApiToken is used only for SourceAppId GET; TargetApiToken only for TargetAppId GET.
  - Reproduces the existing AcceptanceSync 7-class analysis.
  - Writes:
      1) App272_AcceptanceAnalysis_ALL_*.csv
      2) App272_未受入期限超過_*.csv
      3) App272_AcceptanceEvidence_*_summary.txt
  - Optional formal RunId JSONL verification.
  - -ReadOnlyTest permits analysis without a RunLog; output is explicitly NON-FORMAL.

  IMPORTANT:
  This script NEVER sends POST/PUT/DELETE to kintone.
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)]
    [string]$Subdomain,

    [int]$SourceAppId = 293,
    [int]$TargetAppId = 272,

    [Parameter(Mandatory=$true)]
    [Security.SecureString]$SourceApiToken,

    [Parameter(Mandatory=$true)]
    [Security.SecureString]$TargetApiToken,

    [string]$OutputRoot = 'C:\HPDB\App272_Export\AcceptanceEvidence',

    [string]$RunLogPath = '',

    [switch]$ReadOnlyTest,

    [datetime]$AsOfDate = (Get-Date).Date,

    [string]$SecondaryDeadlineFieldCode = 'K完成検査日',
    [string]$SecondaryDeadlineLabel = 'K完成検査日',

    [string]$PoFieldCode = '数値',
    [string]$OrderNoFieldCode = 'ODERNO',
    [string]$OrderSlipNoFieldCode = '数値_44',
    [string]$AcceptanceDateFieldCode = '受入れ日',
    [string]$ReceiptDateFieldCode = '',
    [string]$DueDateFieldCode = '日付_1',
    [string]$VendorFieldCode = 'VENDOR_DD',
    [string]$VendorFallbackFieldCode = 'VENDOR_TEXT',
    [string]$StaffFieldCode = 'STAFF_DD',
    [string]$StaffFallbackFieldCode = 'STAFF_TEXT',
    [string]$ProductFieldCode = '商品名'
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

$ScriptVersion = '0.1.3.6-FAILCLOSED-HARDENING-FINAL-CANDIDATE'
$SchemaVersion = '1'
$WriteCount = 0

function ConvertFrom-SecureStringPlain {
    param([Security.SecureString]$Value)
    $ptr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($Value)
    try { [Runtime.InteropServices.Marshal]::PtrToStringBSTR($ptr) }
    finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($ptr) }
}

function Get-JstNow {
    $tz = [TimeZoneInfo]::FindSystemTimeZoneById('Tokyo Standard Time')
    [TimeZoneInfo]::ConvertTimeFromUtc([DateTime]::UtcNow, $tz)
}

function Get-ScalarValue {
    param($Record, [string]$Code)
    if ($null -eq $Record) { return '' }
    $p = $Record.PSObject.Properties[$Code]
    if ($null -eq $p -or $null -eq $p.Value) { return '' }
    $vprop = $p.Value.PSObject.Properties['value']
    if ($null -eq $vprop -or $null -eq $vprop.Value) { return '' }
    $v = $vprop.Value
    if ($v -is [System.Array]) {
        return (($v | ForEach-Object {
            if ($_ -is [string]) { $_ }
            elseif ($_.PSObject.Properties['name']) { [string]$_.name }
            elseif ($_.PSObject.Properties['code']) { [string]$_.code }
            else { [string]$_ }
        }) -join ';').Trim()
    }
    return ([string]$v).Trim()
}

function Test-Ymd {
    param([string]$Value)
    if ([string]::IsNullOrWhiteSpace($Value)) { return $false }
    $dt = [datetime]::MinValue
    return [datetime]::TryParseExact(
        $Value, 'yyyy-MM-dd',
        [Globalization.CultureInfo]::InvariantCulture,
        [Globalization.DateTimeStyles]::None,
        [ref]$dt
    )
}

function Parse-YmdOrNull {
    param([string]$Value, [string]$Context)
    if ([string]::IsNullOrWhiteSpace($Value)) { return $null }
    $dt = [datetime]::MinValue
    $ok = [datetime]::TryParseExact(
        $Value, 'yyyy-MM-dd',
        [Globalization.CultureInfo]::InvariantCulture,
        [Globalization.DateTimeStyles]::None,
        [ref]$dt
    )
    if (-not $ok) { throw "Invalid DATE [$Context]: '$Value'" }
    return $dt.Date
}

function Convert-NumberOrNull {
    param([string]$Value)
    if ([string]::IsNullOrWhiteSpace($Value)) { return $null }
    $n = 0.0
    if ([double]::TryParse($Value, [Globalization.NumberStyles]::Any,
        [Globalization.CultureInfo]::InvariantCulture, [ref]$n)) { return $n }
    return $null
}

function Get-UniqueSorted {
    param([object[]]$Values)
    @($Values | Where-Object { -not [string]::IsNullOrWhiteSpace([string]$_) } |
        ForEach-Object { [string]$_ } | Sort-Object -Unique)
}

function Get-MaxYmd {
    param([object[]]$Values)
    $a = @($Values | Where-Object { Test-Ymd ([string]$_) } | Sort-Object)
    if ($a.Count -eq 0) { return '' }
    return [string]$a[-1]
}

function Get-MinYmd {
    param([object[]]$Values)
    $a = @($Values | Where-Object { Test-Ymd ([string]$_) } | Sort-Object)
    if ($a.Count -eq 0) { return '' }
    return [string]$a[0]
}

function Invoke-KintoneGet {
    param(
        [string]$Path,
        [hashtable]$Query,
        [Parameter(Mandatory=$true)]
        [string]$PlainToken,
        [string]$DiagnosticLabel = ''
    )

    $pairs = @()
    foreach ($k in @($Query.Keys | Sort-Object)) {
        $key = [string]$k
        $value = [string]$Query[$k]

        # kintone parameter names such as fields[0] must remain literal.
        # Encode only the value. Keys are generated internally by this script.
        $pairs += ('{0}={1}' -f $key, [uri]::EscapeDataString($value))
    }

    $uri = "https://${Subdomain}.cybozu.com${Path}"
    if ($pairs.Count -gt 0) {
        $uri += '?' + ($pairs -join '&')
    }

    if ($DiagnosticLabel) {
        Write-Host ("[GET ] {0}" -f $DiagnosticLabel)
        Write-Host ("[URI ] {0}" -f $uri)
    }

    try {
        $result = Invoke-RestMethod `
            -Method Get `
            -Uri $uri `
            -Headers @{ 'X-Cybozu-API-Token' = $PlainToken }

        if ($DiagnosticLabel) {
            Write-Host ("[PASS] {0}" -f $DiagnosticLabel)
        }
        return $result
    }
    catch {
        if ($DiagnosticLabel) {
            Write-Host ("[FAIL] {0}" -f $DiagnosticLabel)
        }
        throw
    }
}

function Get-FormProperties {
    param(
        [int]$AppId,
        [Parameter(Mandatory=$true)]
        [string]$PlainToken,
        [string]$DiagnosticLabel
    )
    (Invoke-KintoneGet '/k/v1/app/form/fields.json' @{ app = $AppId } $PlainToken $DiagnosticLabel).properties
}

function Get-AllRecords {
    param(
        [int]$AppId,
        [string[]]$Fields,
        [Parameter(Mandatory=$true)]
        [string]$PlainToken,
        [string]$DiagnosticLabel
    )
    $all = New-Object System.Collections.ArrayList
    $lastId = 0
    while ($true) {
        $q = '$id > {0} order by $id asc limit 500' -f $lastId
        $query = @{ app = $AppId; query = $q }
        for ($i=0; $i -lt $Fields.Count; $i++) {
            $query["fields[$i]"] = $Fields[$i]
        }
        $pageNo = [int][Math]::Floor($lastId / 500) + 1
        $label = if ($DiagnosticLabel) {
            "{0} PageFromId={1}" -f $DiagnosticLabel, $lastId
        } else { '' }
        $r = Invoke-KintoneGet '/k/v1/records.json' $query $PlainToken $label
        $batch = @($r.records)
        foreach ($x in $batch) { [void]$all.Add($x) }
        if ($batch.Count -lt 500) { break }
        $last = Get-ScalarValue $batch[-1] '$id'
        $n = 0
        if (-not [int]::TryParse($last, [ref]$n) -or $n -le $lastId) {
            throw "App ${AppId}: invalid paging id '$last'"
        }
        $lastId = $n
    }
    return @($all)
}

function Assert-Field {
    param($Props, [string]$Code, [string]$ExpectedType = '')
    $p = $Props.PSObject.Properties[$Code]
    if ($null -eq $p) { throw "Required field not found: $Code" }
    if ($ExpectedType -and [string]$p.Value.type -ne $ExpectedType) {
        throw "Field type mismatch: $Code expected=$ExpectedType actual=$($p.Value.type)"
    }
}

function Read-RunLog {
    param([string]$Path)
    $events = New-Object System.Collections.ArrayList
    $corrupt = 0
    if ([string]::IsNullOrWhiteSpace($Path)) {
        return [pscustomobject]@{ Events=@(); CorruptCount=0; RunId='' }
    }
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "RunLog not found: $Path"
    }
    $lines = @(Get-Content -LiteralPath $Path -Encoding UTF8)
    for ($i=0; $i -lt $lines.Count; $i++) {
        $line = [string]$lines[$i]
        if ([string]::IsNullOrWhiteSpace($line)) { continue }
        try {
            [void]$events.Add(($line | ConvertFrom-Json -ErrorAction Stop))
        } catch {
            $corrupt++
            if ($i -ne ($lines.Count - 1)) {
                throw "JSONL corrupt line is not final line: line=$($i+1)"
            }
        }
    }
    $runIds = @($events | ForEach-Object { [string]$_.RunId } |
        Where-Object { $_ } | Sort-Object -Unique)
    if (@($runIds).Count -gt 1) { throw "RunLog contains multiple RunIds: $($runIds -join ',')" }
    [pscustomobject]@{
        Events=@($events)
        CorruptCount=$corrupt
        RunId= if (@($runIds).Count -eq 1) { $runIds[0] } else { '' }
    }
}

function Get-RunEvidenceByPO {
    param([object[]]$Events)
    $map = @{}
    $groups = @($Events | Where-Object { $_.発注番号 } | Group-Object -Property 発注番号)
    foreach ($g in $groups) {
        $ev = @($g.Group)
        $pre = @($ev | Where-Object { $_.EventType -eq 'PRE_EXECUTE' }) | Select-Object -Last 1
        $put = @($ev | Where-Object { $_.EventType -eq 'PUT_SUCCEEDED' }) | Select-Object -Last 1
        $pv  = @($ev | Where-Object { $_.EventType -eq 'POST_VERIFY_PASS' }) | Select-Object -Last 1
        $map[[string]$g.Name] = [pscustomobject]@{
            HasPre = ($null -ne $pre)
            HasPut = ($null -ne $put)
            HasPostVerify = ($null -ne $pv)
            Before = if ($pre) { [string]$pre.BeforeAcceptanceDate } else { '' }
            Candidate = if ($pre) { [string]$pre.CandidateAcceptanceDate }
                        elseif ($put) { [string]$put.CandidateAcceptanceDate } else { '' }
            After = if ($pv) { [string]$pv.AfterAcceptanceDate }
                    elseif ($put) { [string]$put.AfterAcceptanceDate } else { '' }
            PostVerifyStatus = if ($pv) { 'PASS' } else { '' }
        }
    }
    return $map
}

function Build-AcceptanceAnalysis {
    param([object[]]$SourceRecords, [object[]]$TargetRecords)

    $sourceReceiptNo='受入番号'; $sourcePO='発注番号'; $sourceDate='受入日'
    $sourceQty='受入数'; $sourceOrder='手配書番号'; $sourceSlip='注文伝票番号'

    $safetyStops = New-Object System.Collections.ArrayList
    $blankReceiptNos = New-Object System.Collections.ArrayList
    $receiptNoMap = @{}
    foreach ($r in $SourceRecords) {
        $rn = Get-ScalarValue $r $sourceReceiptNo
        if (-not $rn) { [void]$blankReceiptNos.Add((Get-ScalarValue $r '$id')); continue }
        if (-not $receiptNoMap.ContainsKey($rn)) { $receiptNoMap[$rn] = @() }
        $receiptNoMap[$rn] += ,$r
    }
    $dupReceiptNos = @($receiptNoMap.Keys | Where-Object { @($receiptNoMap[$_]).Count -gt 1 })
    if (@($blankReceiptNos).Count -gt 0) { [void]$safetyStops.Add("App293 blank receipt no: $(@($blankReceiptNos).Count)") }
    if (@($dupReceiptNos).Count -gt 0) { [void]$safetyStops.Add("App293 duplicate receipt no: $(@($dupReceiptNos).Count)") }

    Write-Host ("[ANALYSIS] Source receipt precheck: Records={0} BlankReceiptNo={1} DuplicateReceiptNo={2}" -f @($SourceRecords).Count, @($blankReceiptNos).Count, @($dupReceiptNos).Count)
    $targetByPO=@{}; $targetMissingPO=0
    foreach ($r in $TargetRecords) {
        $po=Get-ScalarValue $r $PoFieldCode
        if (-not $po) { $targetMissingPO++; continue }
        if (-not $targetByPO.ContainsKey($po)) { $targetByPO[$po]=@() }
        $targetByPO[$po] += ,$r
    }
    $dupTargetPOs=@($targetByPO.Keys | Where-Object { @($targetByPO[$_]).Count -gt 1 } | Sort-Object)
    if (@($dupTargetPOs).Count -gt 0) { [void]$safetyStops.Add("App272 duplicate PO: $(@($dupTargetPOs).Count)") }

    Write-Host ("[ANALYSIS] Target PO index: UniquePO={0} DuplicatePO={1} MissingPO={2}" -f @($targetByPO.Keys).Count, @($dupTargetPOs).Count, $targetMissingPO)
    $sourceByPO=@{}; $sourceMissingPO=0; $sourceInvalidDate=0
    foreach ($r in $SourceRecords) {
        $po=Get-ScalarValue $r $sourcePO
        $rd=Get-ScalarValue $r $sourceDate
        if (-not $po) { $sourceMissingPO++; continue }
        if (-not $sourceByPO.ContainsKey($po)) { $sourceByPO[$po]=@() }
        $sourceByPO[$po] += ,$r
        if (-not (Test-Ymd $rd)) { $sourceInvalidDate++ }
    }

    Write-Host ("[ANALYSIS] Source PO index: UniquePO={0} MissingPO={1} InvalidDate={2}" -f @($sourceByPO.Keys).Count, $sourceMissingPO, $sourceInvalidDate)
    $rows=New-Object System.Collections.ArrayList
    foreach ($po in @($sourceByPO.Keys | Sort-Object)) {
        if ($dupTargetPOs -contains $po) { continue }
        $group=@($sourceByPO[$po])
        $dates=@($group | ForEach-Object { Get-ScalarValue $_ $sourceDate })
        $validDates=@($dates | Where-Object { Test-Ymd $_ })
        $invalidDateCount=@($dates).Count-@($validDates).Count
        $negativeCount=0
        foreach ($r in $group) {
            $q=Convert-NumberOrNull (Get-ScalarValue $r $sourceQty)
            if ($null -ne $q -and $q -lt 0) { $negativeCount++ }
        }
        $candidate=Get-MaxYmd $validDates
        $first=Get-MinYmd $validDates
        $last=Get-MaxYmd $validDates
        $distinctDates = @((Get-UniqueSorted $validDates)).Count
        $receiptNumbers=@(Get-UniqueSorted @($group | ForEach-Object { Get-ScalarValue $_ $sourceReceiptNo }))
        $targets=if($targetByPO.ContainsKey($po)){@($targetByPO[$po])}else{@()}

        if (@($targets).Count -eq 0) {
            [void]$rows.Add([pscustomobject]@{
                po=$po; targetId=''; targetRevision=''; currentAcceptanceDate='';
                sourceRowCount=@($group).Count; distinctReceiptDateCount=$distinctDates;
                firstReceiptDate=$first; lastReceiptDate=$last; candidateReceiptDate=$candidate;
                negativeReceiptCount=$negativeCount; receiptNumbers=$receiptNumbers;
                className='SOURCE_ONLY_NO_APP272'; futureAutoEligible='NO';
                reason='App293に受入実績あり / 対象Appに同一発注番号なし'
            })
            continue
        }

        $target=$targets[0]
        $current=Get-ScalarValue $target $AcceptanceDateFieldCode
        $className=''; $eligible='NO'; $reason=''
        if ($negativeCount -gt 0 -or $invalidDateCount -gt 0 -or -not $candidate) {
            $className='SOURCE_REVIEW_REQUIRED'
            $parts=@()
            if($negativeCount -gt 0){$parts+="負数受入 $negativeCount 件"}
            if($invalidDateCount -gt 0){$parts+="受入日不正/空欄 $invalidDateCount 件"}
            if(-not $candidate){$parts+='有効な候補受入日なし'}
            $reason=$parts -join ' / '
        } elseif ($current) {
            if ($current -eq $candidate) { $className='ALREADY_SAME'; $reason='既存受入れ日と候補日が一致' }
            else { $className='EXISTING_DIFFERENT'; $reason='既存受入れ日を保護（候補日と不一致）' }
        } elseif (@($group).Count -eq 1) {
            $className='CANDIDATE_EMPTY_SINGLE'; $eligible='YES'
            $reason='受入れ日空欄 / 単一受入 / レビュー要因なし'
        } else {
            $className='CANDIDATE_EMPTY_MULTI'
            $reason='受入れ日空欄 / 複数受入。MAX(受入日)は完納日と定義しない'
        }
        [void]$rows.Add([pscustomobject]@{
            po=$po; targetId=(Get-ScalarValue $target '$id');
            targetRevision=(Get-ScalarValue $target '$revision');
            currentAcceptanceDate=$current; sourceRowCount=@($group).Count;
            distinctReceiptDateCount=$distinctDates; firstReceiptDate=$first;
            lastReceiptDate=$last; candidateReceiptDate=$candidate;
            negativeReceiptCount=$negativeCount; receiptNumbers=$receiptNumbers;
            className=$className; futureAutoEligible=$eligible; reason=$reason
        })
    }

    Write-Host ("[ANALYSIS] Source-side classification complete: Rows={0}" -f @($rows).Count)
    foreach ($po in @($targetByPO.Keys | Sort-Object)) {
        if ($dupTargetPOs -contains $po -or $sourceByPO.ContainsKey($po)) { continue }
        $t=@($targetByPO[$po])[0]
        [void]$rows.Add([pscustomobject]@{
            po=$po; targetId=(Get-ScalarValue $t '$id');
            targetRevision=(Get-ScalarValue $t '$revision');
            currentAcceptanceDate=(Get-ScalarValue $t $AcceptanceDateFieldCode);
            sourceRowCount=0; distinctReceiptDateCount=0; firstReceiptDate='';
            lastReceiptDate=''; candidateReceiptDate=''; negativeReceiptCount=0;
            receiptNumbers=@(); className='NO_SOURCE_DATA_UNKNOWN';
            futureAutoEligible='NO';
            reason='Prones受入実績として確認できない（未受入とは判定しない）'
        })
    }

    Write-Host ("[ANALYSIS] Target-only classification complete: Rows={0}" -f @($rows).Count)
    [pscustomobject]@{
        Rows=@($rows | Sort-Object className,po)
        SafetyStops=@($safetyStops)
        SourceMissingPOCount=$sourceMissingPO
        SourceInvalidReceiptDateCount=$sourceInvalidDate
        SourceBlankReceiptNoCount=@($blankReceiptNos).Count
        SourceDuplicateReceiptNoCount=@($dupReceiptNos).Count
        TargetMissingPOCount=$targetMissingPO
        DuplicateTargetPOs=@($dupTargetPOs)
    }
}

function Write-Rfc4180Csv {
    param([object[]]$Rows, [string[]]$Columns, [string]$Path)
    if(Test-Path -LiteralPath $Path){throw "Output already exists; overwrite prohibited: $Path"}
    function Q([object]$v) {
        $s = if ($null -eq $v) { '' } else { [string]$v }
        '"' + $s.Replace('"','""') + '"'
    }
    $lines=New-Object System.Collections.Generic.List[string]
    $lines.Add((($Columns | ForEach-Object { Q $_ }) -join ','))
    foreach($r in $Rows){
        $vals=foreach($c in $Columns){
            $p=$r.PSObject.Properties[$c]
            if($p){Q $p.Value}else{Q ''}
        }
        $lines.Add(($vals -join ','))
    }
    $utf8Bom = New-Object Text.UTF8Encoding($true)
    [IO.File]::WriteAllText($Path, (($lines -join "`r`n")+"`r`n"), $utf8Bom)
}

function Get-Sha256([string]$Path) { (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash }

$exitCode=90
$summaryPath=''
$script:SourcePlainToken = ConvertFrom-SecureStringPlain $SourceApiToken
$script:TargetPlainToken = ConvertFrom-SecureStringPlain $TargetApiToken
try {
    Write-Host "================================================================="
    Write-Host "App272 Acceptance Evidence v0.1.3.6 FAILCLOSED HARDENING FINAL CANDIDATE"
    Write-Host "GET ONLY / WRITE=0"
    Write-Host "================================================================="

    if (-not $ReadOnlyTest -and [string]::IsNullOrWhiteSpace($RunLogPath)) {
        throw 'Formal evidence mode requires -RunLogPath. Use -ReadOnlyTest only for GET/analysis verification.'
    }

    $jstNow=Get-JstNow
    $stamp=$jstNow.ToString('yyyyMMdd_HHmmss')
    $month=$jstNow.ToString('yyyyMM')
    $outDir=Join-Path $OutputRoot $month
    New-Item -ItemType Directory -Path $outDir -Force | Out-Null

    $analysisPath=Join-Path $outDir "App272_AcceptanceAnalysis_ALL_$stamp.csv"
    $overduePath=Join-Path $outDir "App272_未受入期限超過_$stamp.csv"
    $summaryPath=Join-Path $outDir "App272_AcceptanceEvidence_${stamp}_summary.txt"
    foreach($p in @($analysisPath,$overduePath,$summaryPath)){
        if(Test-Path -LiteralPath $p){throw "Output collision: $p"}
    }

    $srcProps=Get-FormProperties $SourceAppId $script:SourcePlainToken 'App293 Form Preflight'
    $tgtProps=Get-FormProperties $TargetAppId $script:TargetPlainToken 'App272 Form Preflight'

    foreach($c in @('受入番号','発注番号','受入日','受入数','手配書番号','注文伝票番号')){ Assert-Field $srcProps $c }
    Assert-Field $srcProps '受入日' 'DATE'
    Assert-Field $srcProps '受入数' 'NUMBER'
    $receiptNoProp=$srcProps.PSObject.Properties['受入番号'].Value
    if(-not [bool]$receiptNoProp.unique){throw 'App293.受入番号 unique=false'}

    foreach($c in @($PoFieldCode,$AcceptanceDateFieldCode,$OrderNoFieldCode,$OrderSlipNoFieldCode,
                    $DueDateFieldCode,$SecondaryDeadlineFieldCode,$ProductFieldCode)){
        Assert-Field $tgtProps $c
    }
    foreach($c in @($AcceptanceDateFieldCode,$DueDateFieldCode,$SecondaryDeadlineFieldCode)){
        Assert-Field $tgtProps $c 'DATE'
    }

    $receiptDateEnabled = -not [string]::IsNullOrWhiteSpace($ReceiptDateFieldCode)
    if($receiptDateEnabled){
        Assert-Field $tgtProps $ReceiptDateFieldCode 'DATE'
        Write-Host ("[INFO] Optional receipt date field enabled: {0}" -f $ReceiptDateFieldCode)
    } else {
        Write-Host "[INFO] Optional receipt date field disabled. No substitute field is assumed."
    }

    $sourceFields=@('$id','$revision','受入番号','発注番号','受入日','受入数','手配書番号','注文伝票番号')
    $targetFields=@('$id','$revision',$PoFieldCode,$AcceptanceDateFieldCode,$OrderNoFieldCode,$OrderSlipNoFieldCode,
                    $DueDateFieldCode,$SecondaryDeadlineFieldCode,
                    $VendorFieldCode,$VendorFallbackFieldCode,$StaffFieldCode,$StaffFallbackFieldCode,$ProductFieldCode)
    if($receiptDateEnabled){ $targetFields += $ReceiptDateFieldCode }
    $targetFields=@($targetFields | Sort-Object -Unique)
    # Optional display fields are requested only if present.
    $targetFields=@($targetFields | Where-Object {
        $_ -in @('$id','$revision') -or $tgtProps.PSObject.Properties[$_]
    })

    $sourceRecords=Get-AllRecords $SourceAppId $sourceFields $script:SourcePlainToken 'App293 Records'
    $targetRecords=Get-AllRecords $TargetAppId $targetFields $script:TargetPlainToken 'App272 Records'
    Write-Host ("[INFO] GET complete: App293={0} App272={1}" -f @($sourceRecords).Count, @($targetRecords).Count)
    Write-Host "[STEP] Build Acceptance Analysis"
    $analysis=Build-AcceptanceAnalysis $sourceRecords $targetRecords
    Write-Host ("[PASS] Build Acceptance Analysis: Rows={0} SafetyStops={1}" -f @($analysis.Rows).Count, @($analysis.SafetyStops).Count)

    if(@($analysis.SafetyStops).Count -gt 0){
        throw "AcceptanceSync SafetyStop: $($analysis.SafetyStops -join ' / ')"
    }

    $run=Read-RunLog $RunLogPath
    if($run.CorruptCount -gt 0){throw "JSONL corrupt final line detected: $($run.CorruptCount)"}
    $runByPO=Get-RunEvidenceByPO $run.Events

    $targetMap=@{}
    foreach($r in $targetRecords){
        $po=Get-ScalarValue $r $PoFieldCode
        if($po -and -not $targetMap.ContainsKey($po)){$targetMap[$po]=$r}
    }

    $allRows=New-Object System.Collections.ArrayList
    foreach($r in $analysis.Rows){
        $po=[string]$r.po
        $ev=if($runByPO.ContainsKey($po)){$runByPO[$po]}else{$null}
        $updated=$false
        if($ev){
            $updated=$ev.HasPre -and $ev.HasPut -and $ev.HasPostVerify -and
                [string]::IsNullOrWhiteSpace($ev.Before) -and
                $ev.After -eq $ev.Candidate
        }
        $status = switch([string]$r.className){
            'SOURCE_ONLY_NO_APP272' {'SOURCE_ONLY'}
            'NO_SOURCE_DATA_UNKNOWN' {if($r.currentAcceptanceDate){'ALREADY_ACCEPTED'}else{'SOURCE_UNKNOWN'}}
            'SOURCE_REVIEW_REQUIRED' {'REVIEW_REQUIRED'}
            'CANDIDATE_EMPTY_MULTI' {'REVIEW_REQUIRED'}
            'CANDIDATE_EMPTY_SINGLE' {if($updated){'UPDATED_THIS_RUN'}else{'CANDIDATE_REMAINING'}}
            'ALREADY_SAME' {if($updated){'UPDATED_THIS_RUN'}else{'ALREADY_ACCEPTED'}}
            'EXISTING_DIFFERENT' {'ALREADY_ACCEPTED'}
            default {'REVIEW_REQUIRED'}
        }
        $t=if($targetMap.ContainsKey($po)){$targetMap[$po]}else{$null}
        [void]$allRows.Add([pscustomobject][ordered]@{
            ExportTimestamp=$jstNow.ToString('yyyy-MM-ddTHH:mm:ssK')
            AsOfDate=$AsOfDate.ToString('yyyy-MM-dd')
            RunId=$run.RunId
            Class=$r.className
            EvidenceStatus=$status
            UpdatedThisRun=if($updated){'YES'}else{'NO'}
            PostVerifyStatus=if($ev){$ev.PostVerifyStatus}else{''}
            発注番号=$po
            手配書番号=if($t){Get-ScalarValue $t $OrderNoFieldCode}else{''}
            注文伝票番号=if($t){Get-ScalarValue $t $OrderSlipNoFieldCode}else{''}
            App272RecordId=$r.targetId
            App272Revision=$r.targetRevision
            App293RecordCount=$r.sourceRowCount
            App293ReceiptNumbers=(@($r.receiptNumbers) -join ';')
            CandidateAcceptanceDate=$r.candidateReceiptDate
            CurrentAcceptanceDate=$r.currentAcceptanceDate
            BeforeAcceptanceDate=if($ev){$ev.Before}else{''}
            AfterAcceptanceDate=if($ev){$ev.After}else{''}
            外注入荷実績日=if($t -and $receiptDateEnabled){Get-ScalarValue $t $ReceiptDateFieldCode}else{''}
            納期=if($t){Get-ScalarValue $t $DueDateFieldCode}else{''}
            補助期限名=$SecondaryDeadlineLabel
            補助期限日=if($t){Get-ScalarValue $t $SecondaryDeadlineFieldCode}else{''}
            外注先=if($t){$v=Get-ScalarValue $t $VendorFieldCode;if($v){$v}else{Get-ScalarValue $t $VendorFallbackFieldCode}}else{''}
            担当者=if($t){$v=Get-ScalarValue $t $StaffFieldCode;if($v){$v}else{Get-ScalarValue $t $StaffFallbackFieldCode}}else{''}
            商品名=if($t){Get-ScalarValue $t $ProductFieldCode}else{''}
            Reason=$r.reason
        })
    }

    $analysisRowByPO=@{}
    foreach($ar in $analysis.Rows){
        if($ar.po){$analysisRowByPO[[string]$ar.po]=$ar}
    }

    $overdue=New-Object System.Collections.ArrayList
    foreach($t in $targetRecords){
        $po=Get-ScalarValue $t $PoFieldCode
        $accept=Get-ScalarValue $t $AcceptanceDateFieldCode
        if($accept){continue}
        $dueS=Get-ScalarValue $t $DueDateFieldCode
        $secS=Get-ScalarValue $t $SecondaryDeadlineFieldCode
        $due=Parse-YmdOrNull $dueS "$po/$DueDateFieldCode"
        $sec=Parse-YmdOrNull $secS "$po/$SecondaryDeadlineFieldCode"
        $dueOver=($null -ne $due -and $due -lt $AsOfDate.Date)
        $secOver=($null -ne $sec -and $sec -lt $AsOfDate.Date)
        if(-not($dueOver -or $secOver)){continue}
        $class=if($dueOver -and $secOver){'OVERDUE_BOTH'}elseif($dueOver){'OVERDUE_DUE'}else{'OVERDUE_INSPECTION'}
        $receipt=if($receiptDateEnabled){Get-ScalarValue $t $ReceiptDateFieldCode}else{''}
        $receiptState=if(-not $receiptDateEnabled){
            'RECEIPT_STATE_NOT_EVALUATED'
        } elseif($receipt){
            'RECEIPT_EXISTS_ACCEPTANCE_MISSING'
        } else {
            'NO_RECEIPT_NO_ACCEPTANCE'
        }

        # PowerShell 5.1 parser compatibility:
        # resolve display fallback values before constructing the ordered hashtable.
        $overdueVendor = Get-ScalarValue $t $VendorFieldCode
        if (-not $overdueVendor) {
            $overdueVendor = Get-ScalarValue $t $VendorFallbackFieldCode
        }
        $overdueStaff = Get-ScalarValue $t $StaffFieldCode
        if (-not $overdueStaff) {
            $overdueStaff = Get-ScalarValue $t $StaffFallbackFieldCode
        }

        [void]$overdue.Add([pscustomobject][ordered]@{
            ExportTimestamp=$jstNow.ToString('yyyy-MM-ddTHH:mm:ssK')
            AsOfDate=$AsOfDate.ToString('yyyy-MM-dd')
            発注番号=$po
            App272RecordId=Get-ScalarValue $t '$id'
            App272Revision=Get-ScalarValue $t '$revision'
            手配書番号=Get-ScalarValue $t $OrderNoFieldCode
            注文伝票番号=Get-ScalarValue $t $OrderSlipNoFieldCode
            Class=if($analysisRowByPO.ContainsKey([string]$po)){$analysisRowByPO[[string]$po].className}else{'UNKNOWN'}
            App293RecordCount=if($analysisRowByPO.ContainsKey([string]$po)){$analysisRowByPO[[string]$po].sourceRowCount}else{0}
            CandidateAcceptanceDate=if($analysisRowByPO.ContainsKey([string]$po)){$analysisRowByPO[[string]$po].candidateReceiptDate}else{''}
            受入れ日=''
            外注入荷実績日=$receipt
            ReceiptAcceptanceState=$receiptState
            納期=$dueS
            納期超過日数=if($dueOver){[int]($AsOfDate.Date-$due).TotalDays}else{''}
            補助期限名=$SecondaryDeadlineLabel
            補助期限日=$secS
            補助期限超過日数=if($secOver){[int]($AsOfDate.Date-$sec).TotalDays}else{''}
            OverdueClass=$class
            外注先=$overdueVendor
            担当者=$overdueStaff
            商品名=Get-ScalarValue $t $ProductFieldCode
        })
    }

    $analysisCols=@('ExportTimestamp','AsOfDate','RunId','Class','EvidenceStatus','UpdatedThisRun','PostVerifyStatus',
        '発注番号','手配書番号','注文伝票番号','App272RecordId','App272Revision','App293RecordCount',
        'App293ReceiptNumbers','CandidateAcceptanceDate','CurrentAcceptanceDate','BeforeAcceptanceDate','AfterAcceptanceDate',
        '外注入荷実績日','納期','補助期限名','補助期限日','外注先','担当者','商品名','Reason')
    $overdueCols=@('ExportTimestamp','AsOfDate','発注番号','App272RecordId','App272Revision','手配書番号','注文伝票番号',
        'Class','App293RecordCount','CandidateAcceptanceDate','受入れ日','外注入荷実績日','ReceiptAcceptanceState','納期','納期超過日数','補助期限名','補助期限日',
        '補助期限超過日数','OverdueClass','外注先','担当者','商品名')

    $representedSourceRows=0
    foreach($ar in $analysis.Rows){$representedSourceRows += [int]$ar.sourceRowCount}
    if(($representedSourceRows + [int]$analysis.SourceMissingPOCount) -ne @($sourceRecords).Count){
        throw "Source accounting mismatch: Fetched=$(@($sourceRecords).Count) Represented=$representedSourceRows MissingPO=$($analysis.SourceMissingPOCount)"
    }

    Write-Rfc4180Csv @($allRows) $analysisCols $analysisPath
    Write-Rfc4180Csv @($overdue) $overdueCols $overduePath

    # ExportVerify by re-import
    $a2=@(Import-Csv -LiteralPath $analysisPath -Encoding UTF8)
    $o2=@(Import-Csv -LiteralPath $overduePath -Encoding UTF8)
    if(@($a2).Count -ne @($allRows).Count){throw "ExportVerify Analysis count mismatch"}
    if(@($o2).Count -ne @($overdue).Count){throw "ExportVerify Overdue count mismatch"}
    foreach($r in $a2){
        if($r.EvidenceStatus -eq 'UPDATED_THIS_RUN'){
            if($r.BeforeAcceptanceDate){throw "ExportVerify UPDATED Before not blank PO=$($r.発注番号)"}
            if($r.PostVerifyStatus -ne 'PASS'){throw "ExportVerify UPDATED PostVerify not PASS PO=$($r.発注番号)"}
            if($r.AfterAcceptanceDate -ne $r.CandidateAcceptanceDate){throw "ExportVerify UPDATED After!=Candidate PO=$($r.発注番号)"}
        }
        if($r.EvidenceStatus -eq 'SOURCE_ONLY'){
            if($r.App272RecordId -or $r.App272Revision -or $r.CurrentAcceptanceDate -or $r.BeforeAcceptanceDate -or $r.AfterAcceptanceDate){
                throw "ExportVerify SOURCE_ONLY App272 columns not blank PO=$($r.発注番号)"
            }
        }
        $nums=@()
        if($r.App293ReceiptNumbers){$nums=@($r.App293ReceiptNumbers -split ';' | Where-Object {$_})}
        if([int]$r.App293RecordCount -gt 0 -and @($nums).Count -gt [int]$r.App293RecordCount){
            throw "ExportVerify receipt numbers > source rows PO=$($r.発注番号)"
        }
        if(($nums -join ';') -ne (($nums | Sort-Object -Unique) -join ';')){
            throw "ExportVerify receipt numbers not sorted/unique PO=$($r.発注番号)"
        }
    }
    foreach($r in $o2){
        if($r.受入れ日){throw "ExportVerify overdue row accepted PO=$($r.発注番号)"}
        if(-not $r.Class){throw "ExportVerify overdue Class blank PO=$($r.発注番号)"}
        if($r.Class -eq 'UNKNOWN'){throw "ExportVerify overdue Class UNKNOWN PO=$($r.発注番号)"}
        if(-not $receiptDateEnabled -and $r.ReceiptAcceptanceState -ne 'RECEIPT_STATE_NOT_EVALUATED'){
            throw "ExportVerify receipt state must be NOT_EVALUATED when receipt field disabled PO=$($r.発注番号)"
        }
        $d=Parse-YmdOrNull $r.納期 "verify/$($r.発注番号)/due"
        $s=Parse-YmdOrNull $r.補助期限日 "verify/$($r.発注番号)/secondary"
        if(-not(($null -ne $d -and $d -lt $AsOfDate.Date) -or ($null -ne $s -and $s -lt $AsOfDate.Date))){
            throw "ExportVerify overdue condition false PO=$($r.発注番号)"
        }
    }

    $analysisHash=Get-Sha256 $analysisPath
    $overdueHash=Get-Sha256 $overduePath
    $jsonHash=if($RunLogPath){Get-Sha256 $RunLogPath}else{''}

    $statusCounts=@{}
    foreach($r in $allRows){$k=[string]$r.EvidenceStatus;if(-not$statusCounts.ContainsKey($k)){$statusCounts[$k]=0};$statusCounts[$k]++}
    $classCounts=@{}
    foreach($r in $allRows){$k=[string]$r.Class;if(-not$classCounts.ContainsKey($k)){$classCounts[$k]=0};$classCounts[$k]++}
    $overCounts=@{}
    foreach($r in $overdue){$k=[string]$r.OverdueClass;if(-not$overCounts.ContainsKey($k)){$overCounts[$k]=0};$overCounts[$k]++}

    $summary=@()
    $summary+="ScriptVersion=$ScriptVersion"
    $summary+="Mode=$(if($ReadOnlyTest){'READ_ONLY_TEST_NON_FORMAL'}else{'FORMAL_EVIDENCE'})"
    $summary+="Environment=PROD"
    $summary+="SourceAppId=$SourceAppId"
    $summary+="TargetAppId=$TargetAppId"
    $summary+="SourceApiTokenRoute=App$SourceAppId GET only"
    $summary+="TargetApiTokenRoute=App$TargetAppId GET only"
    $summary+="RunId=$($run.RunId)"
    $summary+="ExportTimestampJST=$($jstNow.ToString('yyyy-MM-ddTHH:mm:ss'))"
    $summary+="AsOfDate=$($AsOfDate.ToString('yyyy-MM-dd'))"
    $summary+="App293Fetched=$(@($sourceRecords).Count)"
    $summary+="App293MissingPO=$($analysis.SourceMissingPOCount)"
    $summary+="App293InvalidReceiptDate=$($analysis.SourceInvalidReceiptDateCount)"
    $summary+="App293BlankReceiptNo=$($analysis.SourceBlankReceiptNoCount)"
    $summary+="App293DuplicateReceiptNo=$($analysis.SourceDuplicateReceiptNoCount)"
    $summary+="App272Fetched=$(@($targetRecords).Count)"
    $summary+="AnalysisRows=$(@($allRows).Count)"
    foreach($k in @($statusCounts.Keys|Sort-Object)){$summary+="EvidenceStatus.$k=$($statusCounts[$k])"}
    foreach($k in @($classCounts.Keys|Sort-Object)){$summary+="Class.$k=$($classCounts[$k])"}
    $summary+="OverdueRows=$(@($overdue).Count)"
    foreach($k in @($overCounts.Keys|Sort-Object)){$summary+="OverdueClass.$k=$($overCounts[$k])"}
    $summary+="ReceiptDateFieldEnabled=$(if($receiptDateEnabled){'YES'}else{'NO'})"
    $summary+="ReceiptDateFieldCode=$(if($receiptDateEnabled){$ReceiptDateFieldCode}else{''})"
    if($receiptDateEnabled){
        $summary+="ReceiptExistsAcceptanceMissing=$(@($overdue|Where-Object{$_.ReceiptAcceptanceState -eq 'RECEIPT_EXISTS_ACCEPTANCE_MISSING'}).Count)"
    } else {
        $summary+="ReceiptExistsAcceptanceMissing=NOT_EVALUATED"
    }
    $summary+="ReceiptStateNote=$(if($receiptDateEnabled){'Receipt state evaluated using configured DATE field.'}else{'Receipt date field disabled; receipt state was not evaluated and no substitute field was assumed.'})"
    $summary+="OverdueNote=App272 acceptance date blank does not prove that App293/PRONES has no receipt evidence; inspect Class/App293RecordCount/CandidateAcceptanceDate."
    $summary+="BothDeadlinesBlank=$(@($targetRecords|Where-Object{ -not(Get-ScalarValue $_ $DueDateFieldCode) -and -not(Get-ScalarValue $_ $SecondaryDeadlineFieldCode)}).Count)"
    $summary+="TargetMissingPO=$($analysis.TargetMissingPOCount)"
    $summary+="DuplicateTargetPO=$(@($analysis.DuplicateTargetPOs).Count)"
    $summary+="CorruptJsonlLineCount=$($run.CorruptCount)"
    $summary+="JsonlPath=$RunLogPath"
    $summary+="JsonlSHA256=$jsonHash"
    $summary+="AnalysisCsvPath=$analysisPath"
    $summary+="AnalysisCsvSHA256=$analysisHash"
    $summary+="OverdueCsvPath=$overduePath"
    $summary+="OverdueCsvSHA256=$overdueHash"
    $summary+="WRITE=0"
    $summary+="ExportVerify=PASS"
    $summary+="Result=PASS"
    $summary+="ExitCode=0"
    if(Test-Path -LiteralPath $summaryPath){throw "Output already exists; overwrite prohibited: $summaryPath"}
    [IO.File]::WriteAllText($summaryPath,(($summary -join "`r`n")+"`r`n"),(New-Object Text.UTF8Encoding($true)))

    Write-Host "PASS WRITE=0 Analysis=$(@($allRows).Count) Overdue=$(@($overdue).Count)"
    Write-Host "Analysis: $analysisPath"
    Write-Host "Overdue : $overduePath"
    Write-Host "Summary : $summaryPath"
    $exitCode=0
}
catch {
    Write-Error $_
    if($summaryPath){
        try {
            $fail=@(
                "ScriptVersion=$ScriptVersion",
                "WRITE=0",
                "Result=FAIL",
                "ExitCode=90",
                "Error=$($_.Exception.Message)"
            ) -join "`r`n"
            $failPath=$summaryPath
            if(Test-Path -LiteralPath $failPath){
                $failDir=Split-Path -Parent $failPath
                $failBase=[IO.Path]::GetFileNameWithoutExtension($failPath)
                $failExt=[IO.Path]::GetExtension($failPath)
                do {
                    $suffix=[guid]::NewGuid().ToString('N').Substring(0,8)
                    $failPath=Join-Path $failDir ($failBase+"_FAIL_"+$suffix+$failExt)
                } while(Test-Path -LiteralPath $failPath)
            }
            [IO.File]::WriteAllText($failPath,($fail+"`r`n"),(New-Object Text.UTF8Encoding($true)))
        } catch {}
    }
    $exitCode=90
}
finally {
    $script:SourcePlainToken=$null
    $script:TargetPlainToken=$null
}
exit $exitCode
