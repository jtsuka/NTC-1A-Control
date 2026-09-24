#requires -Version 5.1
<#
.SYNOPSIS
  kintone App272 API自動取込 Phase 1C MasterAuto版 v0.7

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
  - 日程マスタの正本はExcel「加工先別設定」シートとし、CSVマスタは参照しない。
  - Execute対象は新規App272のみ。既存App272のPUTはPhase 1Cでは禁止する。
  - App270追加はv0.1では禁止する。
  - 新規App272は 数値(発注番号) / ODERNO / VENDOR_TEXT / VENDOR_DD /
  STAFF_TEXT / STAFF_DD / 日付 / 日付_1 / ルックアップ の9項目を明示POSTする。
  - 新規App272は、正常計算できた場合はK日程（Batch）もPOSTする。
  - 新規で ShortLeadTime または ScheduleMasterNotFound の場合はフォールバックとして
    K加工着手日=手配日、K完成検査日=納期、K日程種別=Batch、K暫定設定=ON とする。
    K計算根拠は空欄（Company/Provisionalを偽装しない）。
  - 要確認・使用しない・不正マスタ等は従来どおり安全側でK日程を書かない。
  - 新規候補の発注番号重複、および -ExpectedNewPo と候補集合の不一致は安全停止する。

.NOTES
  既定EnvironmentはTESTで、Target Appは292固定。
  TEST書込みには -Execute と -ConfirmExecute 'APP292-NEW-EXECUTE' が必要。
  PRODは -Environment PROD -App272 272 を明示し、
  -ConfirmExecute 'APP272-NEW-EXECUTE' が必要。
  Execute時は -ExpectedNewPo で新規対象の発注番号を全件明示する。
  TEST Executeは新規候補1件だけ許可する。
  v0.7は複数新規POST（最大100件）に対応する。
  VENDOR_DD / STAFF_DD の未知値は、Preview Form Fieldsへ不足optionのみ追加し、
  Deploy完了・Live反映確認後にPOSTする。
  既存App272レコードのPUT/DELETEは実行しない。
  Deploy状態確認GETは必ず /k/v1/preview/app/deploy.json を使用する。
  Status確認用の別パスは作らない。
  複数POSTは /k/v1/records.json を使用し、1リクエスト内の登録は全件成功/全件取消とする。
  Lookup参照解決のため、POST時はApp272 + App270のAPIトークンを同一ヘッダーに送信する。
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)]
    [string]$CsvPath,

    [string]$ScheduleMasterPath = 'C:\HPDB\加工先別_日程暫定シート_20260910.xlsx',
    [string]$CalendarPath       = 'C:\HPDB\カレンダー(2023).xlsx',
    [string]$JapanHolidayPath   = 'C:\HPDB\JapanHolidays_2026_2027.csv',
    [string]$LogDirectory       = 'C:\HPDB\logs',

    [string]$BaseUrl = 'https://bcurbkixz609.cybozu.com',

    [ValidateSet('TEST','PROD')]
    [string]$Environment = 'TEST',

    [int]$App270 = 270,
    [int]$App272 = 292,

    [string]$ApiToken270 = '',
    [string]$ApiToken272 = '',

    [switch]$SkipKintone,
    [switch]$DryRun,
    [switch]$Execute,
    [string]$ConfirmExecute = '',
    [string[]]$ExpectedNewPo = @(),
    [ValidateRange(1,100)]
    [int]$MaxCreates = 1,

    [ValidateRange(0,100)]
    [int]$MaxNewVendorOptions = 20,

    [ValidateRange(0,100)]
    [int]$MaxNewStaffOptions = 10
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

    # StrictMode対応:
    # 例外型によっては Exception に Response プロパティ自体が存在しない。
    # 直接 $ex.Response を参照すると、元のAPIエラーをこの診断処理が上書きしてしまう。
    $response = $null
    try {
        $responseProp = $ex.PSObject.Properties['Response']
        if ($null -ne $responseProp) {
            $response = $responseProp.Value
        }
    } catch {}

    if ($null -ne $response) {
        try { $statusCode = [int]$response.StatusCode } catch {}
        try { $statusDescription = [string]$response.StatusDescription } catch {}
        try {
            $headersProp = $response.PSObject.Properties['Headers']
            if ($null -ne $headersProp -and $null -ne $headersProp.Value) {
                $pairs = New-Object System.Collections.Generic.List[string]
                foreach ($key in $headersProp.Value.AllKeys) {
                    $pairs.Add(('{0}={1}' -f $key, $headersProp.Value[$key]))
                }
                $responseHeadersText = $pairs -join '; '
            }
        } catch {}
        try {
            $method = $response.PSObject.Methods['GetResponseStream']
            if ($null -ne $method) {
                $stream = $response.GetResponseStream()
                if ($null -ne $stream) {
                    $reader = New-Object System.IO.StreamReader($stream, [System.Text.Encoding]::UTF8)
                    try { $responseBody = $reader.ReadToEnd() }
                    finally { $reader.Dispose() }
                }
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
    $lines.Add('kintone API request failed.')
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




function Get-AppLiveFormFields {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)][string]$BaseUrl,
        [Parameter(Mandatory=$true)][int]$AppId,
        [Parameter(Mandatory=$true)][string]$Token
    )

    $uri = $BaseUrl.TrimEnd('/') + '/k/v1/app/form/fields.json'
    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    try {
        return Invoke-RestMethod -Method Get -Uri $uri -Headers $headers -Body @{ app = $AppId; lang = 'ja' }
    }
    catch {
        $detail = Get-KintoneHttpErrorDetail -ErrorRecord $_ -RequestUri $uri -QueryText 'live form fields GET' -AppId $AppId
        throw $detail
    }
}

function Get-AppPreviewFormFields {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)][string]$BaseUrl,
        [Parameter(Mandatory=$true)][int]$AppId,
        [Parameter(Mandatory=$true)][string]$Token
    )

    $uri = $BaseUrl.TrimEnd('/') + '/k/v1/preview/app/form/fields.json'
    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    try {
        return Invoke-RestMethod -Method Get -Uri $uri -Headers $headers -Body @{ app = $AppId; lang = 'ja' }
    }
    catch {
        $detail = Get-KintoneHttpErrorDetail -ErrorRecord $_ -RequestUri $uri -QueryText 'preview form fields GET' -AppId $AppId
        throw $detail
    }
}

function ConvertTo-CanonicalObject {
    param([AllowNull()]$InputObject)

    if ($null -eq $InputObject) { return $null }

    if ($InputObject -is [string] -or
        $InputObject -is [char] -or
        $InputObject -is [bool] -or
        $InputObject -is [byte] -or
        $InputObject -is [sbyte] -or
        $InputObject -is [int16] -or
        $InputObject -is [uint16] -or
        $InputObject -is [int32] -or
        $InputObject -is [uint32] -or
        $InputObject -is [int64] -or
        $InputObject -is [uint64] -or
        $InputObject -is [single] -or
        $InputObject -is [double] -or
        $InputObject -is [decimal] -or
        $InputObject -is [datetime]) {
        return $InputObject
    }

    if ($InputObject -is [System.Collections.IDictionary]) {
        $ordered = [ordered]@{}
        foreach ($key in @($InputObject.Keys | ForEach-Object { [string]$_ } | Sort-Object)) {
            $ordered[$key] = ConvertTo-CanonicalObject $InputObject[$key]
        }
        return [pscustomobject]$ordered
    }

    if ($InputObject -is [System.Collections.IEnumerable] -and -not ($InputObject -is [string])) {
        $items = New-Object System.Collections.Generic.List[object]
        foreach ($item in $InputObject) {
            $items.Add((ConvertTo-CanonicalObject $item))
        }
        return $items.ToArray()
    }

    $props = @($InputObject.PSObject.Properties | Where-Object {
        $_.MemberType -eq 'NoteProperty' -or
        $_.MemberType -eq 'Property' -or
        $_.MemberType -eq 'AliasProperty'
    } | Sort-Object Name)

    if ($props.Count -gt 0) {
        $ordered = [ordered]@{}
        foreach ($p in $props) {
            $ordered[$p.Name] = ConvertTo-CanonicalObject $p.Value
        }
        return [pscustomobject]$ordered
    }

    return $InputObject
}

function ConvertTo-CanonicalJson {
    param([AllowNull()]$InputObject)
    return (ConvertTo-CanonicalObject $InputObject | ConvertTo-Json -Depth 100 -Compress)
}

function Get-Sha256Text {
    param([Parameter(Mandatory=$true)][string]$Text)
    $sha = [System.Security.Cryptography.SHA256]::Create()
    try {
        $bytes = [Text.Encoding]::UTF8.GetBytes($Text)
        $hash = $sha.ComputeHash($bytes)
        return ([BitConverter]::ToString($hash)).Replace('-','').ToLowerInvariant()
    }
    finally {
        $sha.Dispose()
    }
}

function Copy-JsonObject {
    param([Parameter(Mandatory=$true)]$InputObject)
    return ($InputObject | ConvertTo-Json -Depth 100 | ConvertFrom-Json)
}

function Assert-MasterDropdownFields {
    param(
        [Parameter(Mandatory=$true)]$Form,
        [Parameter(Mandatory=$true)][string[]]$FieldCodes,
        [string]$Context = 'Form'
    )

    foreach ($code in $FieldCodes) {
        $p = $Form.properties.PSObject.Properties[$code]
        if ($null -eq $p) {
            throw "DropdownFieldMissing: $Context に '$code' が存在しません。"
        }
        if ([string]$p.Value.type -ne 'DROP_DOWN') {
            throw "DropdownFieldTypeMismatch: $Context '$code' type=$($p.Value.type)"
        }
        if ($null -eq $p.Value.options) {
            throw "DropdownFieldMissingOptions: $Context '$code' にoptionsがありません。"
        }
    }
}

function Get-DropdownOptionKeys {
    param(
        [Parameter(Mandatory=$true)]$Form,
        [Parameter(Mandatory=$true)][string]$FieldCode
    )
    $field = $Form.properties.PSObject.Properties[$FieldCode].Value
    return @($field.options.PSObject.Properties | ForEach-Object { [string]$_.Name })
}

function New-DropdownOptionsWithAdditions {
    param(
        [Parameter(Mandatory=$true)]$OptionsObject,
        [Parameter(Mandatory=$true)][string[]]$AddValues
    )

    $options = [ordered]@{}
    $usedIndexes = @{}
    $maxIndex = -1

    foreach ($p in $OptionsObject.PSObject.Properties) {
        $key = [string]$p.Name
        $label = [string]$p.Value.label
        $indexText = [string]$p.Value.index
        $idx = 0
        if (-not [int]::TryParse($indexText, [ref]$idx)) {
            throw "DropdownOptionIndexInvalid: option='$key' index='$indexText'"
        }
        if ($usedIndexes.ContainsKey($idx)) {
            throw "DropdownOptionIndexDuplicate: index=$idx option='$key'"
        }
        $usedIndexes[$idx] = $true
        if ($idx -gt $maxIndex) { $maxIndex = $idx }
        if ($label -ne $key) {
            throw "DropdownOptionLabelKeyMismatch: key='$key' label='$label'"
        }

        $options[$key] = [ordered]@{
            label = $label
            index = [string]$idx
        }
    }

    $added = New-Object System.Collections.Generic.List[string]
    foreach ($value in @($AddValues | Sort-Object -Unique)) {
        $v = ([string]$value).Trim()
        if ([string]::IsNullOrWhiteSpace($v)) { continue }
        if (-not $options.Contains($v)) {
            $maxIndex++
            $options[$v] = [ordered]@{
                label = $v
                index = [string]$maxIndex
            }
            $added.Add($v)
        }
    }

    return [pscustomobject]@{
        Options = $options
        Added = $added.ToArray()
    }
}

function Test-FormSnapshotEquivalent {
    param(
        [Parameter(Mandatory=$true)]$A,
        [Parameter(Mandatory=$true)]$B
    )
    return ((ConvertTo-CanonicalJson $A.properties) -eq (ConvertTo-CanonicalJson $B.properties))
}

function Assert-NoUndeployedFormChanges {
    param(
        [Parameter(Mandatory=$true)]$Live,
        [Parameter(Mandatory=$true)]$Preview,
        [string]$Context = 'Preflight'
    )

    if ([string]$Live.revision -ne [string]$Preview.revision) {
        throw "PreviewHasUndeployedChanges: $Context revision mismatch Live=$($Live.revision) Preview=$($Preview.revision)"
    }
    if (-not (Test-FormSnapshotEquivalent -A $Live -B $Preview)) {
        throw "PreviewHasUndeployedChanges: $Context Live/Preview form properties differ."
    }
}

function New-ExpectedPreviewProperties {
    param(
        [Parameter(Mandatory=$true)]$BaselineProperties,
        [Parameter(Mandatory=$true)][hashtable]$ExpectedFields
    )

    $expected = Copy-JsonObject $BaselineProperties
    foreach ($fieldCode in $ExpectedFields.Keys) {
        $fieldProp = $expected.PSObject.Properties[$fieldCode]
        if ($null -eq $fieldProp) {
            throw "ExpectedPreviewBuildFailed: field '$fieldCode' not found."
        }
        $fieldProp.Value.options = $ExpectedFields[$fieldCode]
    }
    return $expected
}

function Update-MasterDropdownOptionsAndDeploy {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)][string]$BaseUrl,
        [Parameter(Mandatory=$true)][int]$AppId,
        [Parameter(Mandatory=$true)][string]$Token,
        [Parameter(Mandatory=$true)]$BaselineLive,
        [Parameter(Mandatory=$true)]$BaselinePreview,
        [Parameter(Mandatory=$true)][string[]]$MissingVendorOptions,
        [Parameter(Mandatory=$true)][string[]]$MissingStaffOptions
    )

    $missingVendor = @($MissingVendorOptions | ForEach-Object { ([string]$_).Trim() } |
        Where-Object { -not [string]::IsNullOrWhiteSpace($_) } | Sort-Object -Unique)
    $missingStaff = @($MissingStaffOptions | ForEach-Object { ([string]$_).Trim() } |
        Where-Object { -not [string]::IsNullOrWhiteSpace($_) } | Sort-Object -Unique)

    if ($missingVendor.Count -eq 0 -and $missingStaff.Count -eq 0) {
        return [pscustomobject]@{
            Changed = $false
            AddedVendor = @()
            AddedStaff = @()
            RevisionAfterPut = [string]$BaselinePreview.revision
            DeployStatus = 'NONE'
        }
    }

    Write-Host '      [M1] PUT直前 Live / Preview 再GET...'
    # TOCTOU guard 1: PUT直前にLive/Previewを再取得し、
    # 最初の安全確認時点から何も変わっていないことを要求する。
    $liveNow = Get-AppLiveFormFields -BaseUrl $BaseUrl -AppId $AppId -Token $Token
    $previewNow = Get-AppPreviewFormFields -BaseUrl $BaseUrl -AppId $AppId -Token $Token
    Assert-MasterDropdownFields -Form $liveNow -FieldCodes @('VENDOR_DD','STAFF_DD') -Context 'Live immediately before PUT'
    Assert-MasterDropdownFields -Form $previewNow -FieldCodes @('VENDOR_DD','STAFF_DD') -Context 'Preview immediately before PUT'
    Assert-NoUndeployedFormChanges -Live $liveNow -Preview $previewNow -Context 'immediately before PUT'

    $baselineJson = ConvertTo-CanonicalJson $BaselineLive.properties
    $liveNowJson = ConvertTo-CanonicalJson $liveNow.properties
    $previewNowJson = ConvertTo-CanonicalJson $previewNow.properties

    if ($liveNowJson -ne $baselineJson -or $previewNowJson -ne $baselineJson) {
        throw 'PreviewRevisionConflict: form definition changed after initial preflight. Automatic retry is prohibited.'
    }
    if ([string]$liveNow.revision -ne [string]$BaselineLive.revision -or
        [string]$previewNow.revision -ne [string]$BaselinePreview.revision) {
        throw "PreviewRevisionConflict: revision changed after initial preflight. Live $($BaselineLive.revision)->$($liveNow.revision), Preview $($BaselinePreview.revision)->$($previewNow.revision)"
    }

    Write-Host '      [M2] 不足option追加Payload構築...'
    $propertiesToPut = [ordered]@{}
    $expectedFieldOptions = @{}
    $addedVendor = @()
    $addedStaff = @()

    if ($missingVendor.Count -gt 0) {
        $vendorField = $previewNow.properties.PSObject.Properties['VENDOR_DD'].Value
        $vendorBuild = New-DropdownOptionsWithAdditions -OptionsObject $vendorField.options -AddValues $missingVendor
        $propertiesToPut['VENDOR_DD'] = [ordered]@{
            type = 'DROP_DOWN'
            options = $vendorBuild.Options
        }
        $expectedFieldOptions['VENDOR_DD'] = $vendorBuild.Options
        $addedVendor = @($vendorBuild.Added)
    }

    if ($missingStaff.Count -gt 0) {
        $staffField = $previewNow.properties.PSObject.Properties['STAFF_DD'].Value
        $staffBuild = New-DropdownOptionsWithAdditions -OptionsObject $staffField.options -AddValues $missingStaff
        $propertiesToPut['STAFF_DD'] = [ordered]@{
            type = 'DROP_DOWN'
            options = $staffBuild.Options
        }
        $expectedFieldOptions['STAFF_DD'] = $staffBuild.Options
        $addedStaff = @($staffBuild.Added)
    }

    $expectedPreviewProperties = New-ExpectedPreviewProperties `
        -BaselineProperties $liveNow.properties `
        -ExpectedFields $expectedFieldOptions

    $uriFields = $BaseUrl.TrimEnd('/') + '/k/v1/preview/app/form/fields.json'
    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $bodyObj = [ordered]@{
        app = $AppId
        revision = [string]$previewNow.revision
        properties = $propertiesToPut
    }
    $body = $bodyObj | ConvertTo-Json -Depth 100

    Write-Host ("      [M3] Preview Form Fields PUT revision={0}..." -f $previewNow.revision)
    try {
        $putResp = Invoke-RestMethod -Method Put -Uri $uriFields -Headers $headers `
            -ContentType 'application/json; charset=utf-8' `
            -Body ([Text.Encoding]::UTF8.GetBytes($body))
    }
    catch {
        $detail = Get-KintoneHttpErrorDetail -ErrorRecord $_ -RequestUri $uriFields -QueryText 'preview form fields PUT' -AppId $AppId
        throw "FormUpdateFailed: $detail"
    }

    Write-Host ("      [M3] PUT PASS revision={0}" -f $putResp.revision) -ForegroundColor Green
    Write-Host '      [M4] PUT後 / Deploy前 Live / Preview 再GET・意図差分照合...'
    # TOCTOU guard 2: PUT後・Deploy前に、Liveはベースラインのまま、
    # Previewの差分は今回意図したoptions追加だけであることを厳密確認する。
    $liveBeforeDeploy = Get-AppLiveFormFields -BaseUrl $BaseUrl -AppId $AppId -Token $Token
    $previewBeforeDeploy = Get-AppPreviewFormFields -BaseUrl $BaseUrl -AppId $AppId -Token $Token

    if ((ConvertTo-CanonicalJson $liveBeforeDeploy.properties) -ne $baselineJson) {
        throw 'PreviewChangedUnexpectedlyAfterPut: Live form changed after our PUT and before Deploy.'
    }
    if ([string]$liveBeforeDeploy.revision -ne [string]$BaselineLive.revision) {
        throw "PreviewChangedUnexpectedlyAfterPut: Live revision changed before Deploy: $($BaselineLive.revision) -> $($liveBeforeDeploy.revision)"
    }
    if ([string]$previewBeforeDeploy.revision -ne [string]$putResp.revision) {
        throw "PreviewRevisionConflict: Preview revision after PUT is unexpected. expected=$($putResp.revision) actual=$($previewBeforeDeploy.revision)"
    }

    $expectedPreviewJson = ConvertTo-CanonicalJson $expectedPreviewProperties
    $actualPreviewJson = ConvertTo-CanonicalJson $previewBeforeDeploy.properties
    if ($actualPreviewJson -ne $expectedPreviewJson) {
        throw 'PreviewChangedUnexpectedlyAfterPut: actual Preview diff is not exactly the intended master option additions. Deploy blocked.'
    }

    $uriDeploy = $BaseUrl.TrimEnd('/') + '/k/v1/preview/app/deploy.json'
    $deployBody = @{
        apps = @(@{
            app = $AppId
            revision = [string]$putResp.revision
        })
        revert = $false
    } | ConvertTo-Json -Depth 20

    Write-Host ("      [M5] Deploy POST revision={0}..." -f $putResp.revision)
    try {
        [void](Invoke-RestMethod -Method Post -Uri $uriDeploy -Headers $headers `
            -ContentType 'application/json; charset=utf-8' `
            -Body ([Text.Encoding]::UTF8.GetBytes($deployBody)))
    }
    catch {
        $detail = Get-KintoneHttpErrorDetail -ErrorRecord $_ -RequestUri $uriDeploy -QueryText 'preview deploy POST' -AppId $AppId
        throw "DeployFailed: $detail"
    }

    # IMPORTANT:
    # Deploy開始POSTとStatus確認GETは同じ /preview/app/deploy.json。
    # Status確認用の別エンドポイントは使用しない。
    Write-Host '      [M5] Deploy POST accepted.'
    # Windows PowerShell 5.1 / Invoke-RestMethod は GET に -Body を付けると
    # ProtocolViolationException（「コンテンツ本体をこの verb-type では送信できません」）
    # になるため、kintone公式サンプルどおり apps[0] をURLクエリへ付与する。
    $uriDeployStatus = $BaseUrl.TrimEnd('/') + '/k/v1/preview/app/deploy.json?apps[0]=' + [uri]::EscapeDataString([string]$AppId)
    $success = $false
    $lastStatus = ''
    for ($i = 1; $i -le 40; $i++) {
        Start-Sleep -Seconds 3
        try {
            $statusResp = Invoke-RestMethod -Method Get -Uri $uriDeployStatus -Headers $headers
        }
        catch {
            $detail = Get-KintoneHttpErrorDetail -ErrorRecord $_ -RequestUri $uriDeployStatus -QueryText 'preview deploy status GET' -AppId $AppId
            throw "DeployStatusGetFailed: $detail"
        }

        $statusEntry = @($statusResp.apps | Where-Object { [string]$_.app -eq [string]$AppId } | Select-Object -First 1)
        if ($statusEntry.Count -eq 0) {
            throw "DeployStatusGetFailed: App$AppId status entry not found."
        }
        $lastStatus = [string]$statusEntry[0].status
        Write-Host ("      Deploy status {0}/40: {1}" -f $i, $lastStatus)

        if ($lastStatus -eq 'SUCCESS') {
            $success = $true
            break
        }
        if ($lastStatus -eq 'FAIL' -or $lastStatus -eq 'CANCEL') {
            throw "DeployFailed: App$AppId status=$lastStatus"
        }
    }

    if (-not $success) {
        throw "DeployTimeout: App$AppId status=$lastStatus"
    }

    Write-Host '      [M6] Deploy SUCCESS後 Live / Preview 最終確認...'
    # Deploy後、Live/Preview双方が期待定義と一致し、追加optionが存在することを確認。
    $liveAfter = Get-AppLiveFormFields -BaseUrl $BaseUrl -AppId $AppId -Token $Token
    $previewAfter = Get-AppPreviewFormFields -BaseUrl $BaseUrl -AppId $AppId -Token $Token

    if ([string]$liveAfter.revision -ne [string]$previewAfter.revision) {
        throw "MasterOptionDeployVerificationFailed: revision mismatch Live=$($liveAfter.revision) Preview=$($previewAfter.revision)"
    }
    if ((ConvertTo-CanonicalJson $liveAfter.properties) -ne $expectedPreviewJson) {
        throw 'MasterOptionDeployVerificationFailed: Live form does not exactly match expected form after Deploy.'
    }
    if ((ConvertTo-CanonicalJson $previewAfter.properties) -ne $expectedPreviewJson) {
        throw 'MasterOptionDeployVerificationFailed: Preview form does not exactly match expected form after Deploy.'
    }

    foreach ($v in $missingVendor) {
        if ($null -eq $liveAfter.properties.PSObject.Properties['VENDOR_DD'].Value.options.PSObject.Properties[$v]) {
            throw "MasterOptionDeployVerificationFailed: VENDOR_DD missing '$v' after Deploy."
        }
    }
    foreach ($v in $missingStaff) {
        if ($null -eq $liveAfter.properties.PSObject.Properties['STAFF_DD'].Value.options.PSObject.Properties[$v]) {
            throw "MasterOptionDeployVerificationFailed: STAFF_DD missing '$v' after Deploy."
        }
    }

    return [pscustomobject]@{
        Changed = $true
        AddedVendor = $addedVendor
        AddedStaff = $addedStaff
        RevisionAfterPut = [string]$putResp.revision
        DeployStatus = $lastStatus
    }
}

function Test-LiveMasterOptionExists {
    param(
        [Parameter(Mandatory=$true)]$LiveForm,
        [Parameter(Mandatory=$true)][string]$FieldCode,
        [Parameter(Mandatory=$true)][string]$Value
    )
    $field = $LiveForm.properties.PSObject.Properties[$FieldCode]
    if ($null -eq $field) { return $false }
    return ($null -ne $field.Value.options.PSObject.Properties[$Value])
}


function Invoke-KintoneCreateRecord {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)][string]$BaseUrl,
        [Parameter(Mandatory=$true)][int]$AppId,
        [Parameter(Mandatory=$true)][string]$Token,
        [string]$LookupSourceToken = '',
        [Parameter(Mandatory=$true)][hashtable]$RecordFields
    )

    $uri = $BaseUrl.TrimEnd('/') + '/k/v1/record.json'
    $tokenHeader = $Token
    if (-not [string]::IsNullOrWhiteSpace($LookupSourceToken)) {
        $tokenHeader = $Token + ',' + $LookupSourceToken
    }
    $headers = @{ 'X-Cybozu-API-Token' = $tokenHeader }
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


function Invoke-KintoneCreateRecords {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)][string]$BaseUrl,
        [Parameter(Mandatory=$true)][int]$AppId,
        [Parameter(Mandatory=$true)][string]$Token,
        [string]$LookupSourceToken = '',
        [Parameter(Mandatory=$true)][object[]]$Records
    )

    if ($Records.Count -lt 1) {
        throw 'POST対象レコードが0件です。'
    }
    if ($Records.Count -gt 100) {
        throw "kintone複数レコード登録APIの上限100件を超えています: $($Records.Count)"
    }

    $uri = $BaseUrl.TrimEnd('/') + '/k/v1/records.json'
    $tokenHeader = $Token
    if (-not [string]::IsNullOrWhiteSpace($LookupSourceToken)) {
        $tokenHeader = $Token + ',' + $LookupSourceToken
    }
    $headers = @{ 'X-Cybozu-API-Token' = $tokenHeader }
    $bodyObj = [ordered]@{
        app = $AppId
        records = $Records
    }
    $body = $bodyObj | ConvertTo-Json -Depth 10

    try {
        return Invoke-RestMethod -Method Post -Uri $uri -Headers $headers `
            -ContentType 'application/json; charset=utf-8' -Body $body
    }
    catch {
        $detail = Get-KintoneHttpErrorDetail $_
        throw "App$AppId BULK POST failed: count=$($Records.Count) : $detail"
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

# -------------------- main --------------------
$started = Get-Date

$expectedTargetApp = if ($Environment -eq 'TEST') { 292 } else { 272 }
$requiredConfirm = if ($Environment -eq 'TEST') { 'APP292-NEW-EXECUTE' } else { 'APP272-NEW-EXECUTE' }

if ($App272 -ne $expectedTargetApp) {
    Write-Error "安全停止: Environment=$Environment のTarget Appは $expectedTargetApp 固定です。指定App272=$App272"
    exit 35
}

Write-Host ("=== App272 API自動取込 Phase 1C New POST v0.7.4 MasterAuto [{0}] ===" -f $Environment) -ForegroundColor Cyan
Write-Host ("Target App       : {0}" -f $App272)
Write-Host ("Source App270    : {0} (READ ONLY)" -f $App270)

if ($DryRun -and $Execute) {
    Write-Error '-DryRun と -Execute は同時指定できません。'
    exit 30
}
if (-not $DryRun -and -not $Execute) {
    Write-Error '必ず -DryRun または -Execute のどちらかを指定してください。'
    exit 30
}
if ($Execute -and $ConfirmExecute -ne $requiredConfirm) {
    Write-Error "新規作成には -ConfirmExecute '$requiredConfirm' が必要です。"
    exit 31
}
if ($Execute) {
    $expectedPoInput = @($ExpectedNewPo | ForEach-Object { ([string]$_).Trim() } | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
    if ($expectedPoInput.Count -eq 0) {
        Write-Error '-Execute時は -ExpectedNewPo で対象の発注番号を全件明示してください。'
        exit 34
    }
}
if ($Execute -and $SkipKintone) {
    Write-Error '-Execute と -SkipKintone は同時指定できません。'
    exit 32
}

try {
    if (-not (Test-Path -LiteralPath $CsvPath)) { throw "入力CSVが見つかりません: $CsvPath" }
    if (-not (Test-Path -LiteralPath $ScheduleMasterPath)) { throw "日程マスタが見つかりません: $ScheduleMasterPath" }

    if (-not (Test-Path -LiteralPath $LogDirectory)) {
        New-Item -ItemType Directory -Path $LogDirectory -Force | Out-Null
    }

    Write-Host "CSV              : $CsvPath"
    Write-Host "Schedule Master  : $ScheduleMasterPath (Excel / 加工先別設定)"
    Write-Host "Company Calendar : $CalendarPath"
    Write-Host "Japan Holidays   : $JapanHolidayPath"
    if ($DryRun) { Write-Host 'WRITE             : NONE (DryRun / GET only)' }
    else { Write-Host "WRITE             : ENABLED (POST only / MaxCreates=$MaxCreates)" -ForegroundColor Yellow }

    # 実運用CSVはShift-JIS/CP932。Windows PowerShell 5.1では Default がCP932相当。
    $csvRows = @(Import-CsvAllowDuplicateHeaders -Path $CsvPath)
    if ($csvRows.Count -eq 0) { throw '入力CSVが0件です。' }

    $required = @('発注番号','手配書番号','手配日','納期','指示先名','手配担当者名')
    $props = @($csvRows[0].PSObject.Properties.Name)
    foreach ($name in $required) {
        if ($props -notcontains $name) { throw "必須列がありません: $name" }
    }

    # TESTではExpectedNewPoをDryRun時からCSV絞り込みにも使用する。
    # これによりApp292統合試験を1件単位で安全に実施できる。
    if ($Environment -eq 'TEST' -and $ExpectedNewPo.Count -gt 0) {
        $testPoSet = @{}
        foreach ($poRaw in $ExpectedNewPo) {
            $po = ([string]$poRaw).Trim()
            if ($po) { $testPoSet[$po] = $true }
        }
        $beforeFilterCount = $csvRows.Count
        $csvRows = @($csvRows | Where-Object {
            $po = ([string]$_.発注番号).Trim()
            $testPoSet.ContainsKey($po)
        })
        Write-Host ("TEST CSV filter   : {0} -> {1} rows / ExpectedNewPo={2}" -f `
            $beforeFilterCount, $csvRows.Count, (@($testPoSet.Keys | Sort-Object) -join ','))
        if ($csvRows.Count -eq 0) {
            throw 'TEST安全停止: -ExpectedNewPo に一致するCSV行がありません。'
        }
    }

    $masterRows = @(Import-ScheduleMasterExcel -Path $ScheduleMasterPath)
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
        if ([string]::IsNullOrWhiteSpace($ApiToken270)) { $ApiToken270 = Get-ApiTokenInteractive "Source App$App270" }
        if ([string]::IsNullOrWhiteSpace($ApiToken272)) { $ApiToken272 = Get-ApiTokenInteractive "Target App$App272" }

        Write-Host '[3/5] kintone App270 GET...'
        $app270Records = @(Invoke-KintoneGetAll -BaseUrl $BaseUrl -AppId $App270 -Token $ApiToken270)
        foreach ($r in $app270Records) {
            $key = Get-KValue $r '文字列__1行_'
            if ($key -and -not $app270ByOrder.ContainsKey($key)) { $app270ByOrder[$key] = $r }
        }

        Write-Host ("[4/5] kintone Target App{0} GET..." -f $App272)
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
        $staff = ([string]$row.手配担当者名).Trim()
        $errors = New-Object System.Collections.Generic.List[string]

        $orderDate = $null
        $dueDate = $null
        try { $orderDate = Normalize-DateStrict ([string]$row.手配日) } catch { $errors.Add("手配日: $($_.Exception.Message)") }
        try { $dueDate = Normalize-DateStrict ([string]$row.納期) } catch { $errors.Add("納期: $($_.Exception.Message)") }

        if (-not $po) { $errors.Add('発注番号が空欄') }
        if (-not $vendor) { $errors.Add('指示先名が空欄') }
        if (-not $staff) { $errors.Add('手配担当者名が空欄') }
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
                if ($scheduleTargetKind -eq 'New' -and $orderDate -and $dueDate) {
                    # v0.7.4: 新規で日程マスタ未登録の場合は、元日付を安全なフォールバックとして使用する。
                    $start = $orderDate
                    $inspection = $dueDate
                    $proposedKType = 'Batch'
                    $proposedKOnOff = 'ON'
                    $calcBasis = ''
                    $scheduleStatus = 'Fallback-New-OrderDateToDueDate(MasterNotFound)'
                    $scheduleNote = 'Fallback: K加工着手日=手配日 / K完成検査日=納期（ScheduleMasterNotFound）'
                } else {
                    $scheduleStatus = 'ScheduleMasterNotFound'
                }
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
                            if ($scheduleTargetKind -eq 'New' -and $calc.Reason -eq 'ShortLeadTime' -and $orderDate -and $dueDate) {
                                # v0.7.4: 新規で前倒し計算が手配日より前になる場合は、元日付へフォールバックする。
                                $start = $orderDate
                                $inspection = $dueDate
                                $proposedKType = 'Batch'
                                $proposedKOnOff = 'ON'
                                $calcBasis = ''
                                $scheduleStatus = 'Fallback-New-OrderDateToDueDate(ShortLeadTime)'
                                $scheduleNote = 'Fallback: K加工着手日=手配日 / K完成検査日=納期（ShortLeadTime）'
                            } else {
                                $scheduleStatus = $calc.Reason
                            }
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
            手配担当者名 = $staff
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


    # -------------------- Phase 1C v0.7 master lifecycle preflight / execute --------------------
    $createCandidates = @($details | Where-Object { $_.WRITE -eq 'POST:NEW' })
    $blockedApp270Add = @($details | Where-Object WRITE -eq 'BLOCKED_APP270_ADD')
    $blockedExistingWrite = @($details | Where-Object WRITE -eq 'BLOCKED_EXISTING_WRITE')
    $createCountPlanned = $createCandidates.Count
    $recordErrorCountPreflight = @($details | Where-Object { -not [string]::IsNullOrWhiteSpace([string]$_.Errors) }).Count

    $masterDefinitions = @(
        [pscustomobject]@{
            MasterType = 'Vendor'
            SourceColumn = '指示先名'
            TextFieldCode = 'VENDOR_TEXT'
            DropdownFieldCode = 'VENDOR_DD'
            DisplayName = '外注先'
        },
        [pscustomobject]@{
            MasterType = 'Staff'
            SourceColumn = '手配担当者名'
            TextFieldCode = 'STAFF_TEXT'
            DropdownFieldCode = 'STAFF_DD'
            DisplayName = '担当者'
        }
    )

    $requiredVendor = @($createCandidates | ForEach-Object { ([string]$_.指示先名).Trim() } |
        Where-Object { -not [string]::IsNullOrWhiteSpace($_) } | Sort-Object -Unique)
    $requiredStaff = @($createCandidates | ForEach-Object { ([string]$_.手配担当者名).Trim() } |
        Where-Object { -not [string]::IsNullOrWhiteSpace($_) } | Sort-Object -Unique)

    $formPreflightDone = $false
    $previewLiveDiffCount = 0
    $missingVendorOptions = @()
    $missingStaffOptions = @()
    $baselineLive = $null
    $baselinePreview = $null
    $baselineFormHash = ''
    $formPutExecuted = $false
    $deployExecuted = $false
    $masterResult = $null
    $verificationErrors = New-Object System.Collections.Generic.List[string]
    $postVerificationRows = New-Object System.Collections.Generic.List[object]
    $masterOptionRows = New-Object System.Collections.Generic.List[object]

    if (-not $SkipKintone) {
        Write-Host ''
        Write-Host '=== Master Lifecycle Preflight (READ ONLY) ===' -ForegroundColor Cyan
        $baselineLive = Get-AppLiveFormFields -BaseUrl $BaseUrl -AppId $App272 -Token $ApiToken272
        $baselinePreview = Get-AppPreviewFormFields -BaseUrl $BaseUrl -AppId $App272 -Token $ApiToken272
        Assert-MasterDropdownFields -Form $baselineLive -FieldCodes @('VENDOR_DD','STAFF_DD') -Context 'Live'
        Assert-MasterDropdownFields -Form $baselinePreview -FieldCodes @('VENDOR_DD','STAFF_DD') -Context 'Preview'

        if (-not (Test-FormSnapshotEquivalent -A $baselineLive -B $baselinePreview) -or
            [string]$baselineLive.revision -ne [string]$baselinePreview.revision) {
            $previewLiveDiffCount = 1
            throw "PreviewHasUndeployedChanges: Live revision=$($baselineLive.revision), Preview revision=$($baselinePreview.revision)"
        }

        $baselineFormJson = ConvertTo-CanonicalJson $baselineLive.properties
        $baselineFormHash = Get-Sha256Text $baselineFormJson
        $formPreflightDone = $true

        $vendorOptionKeys = @(Get-DropdownOptionKeys -Form $baselineLive -FieldCode 'VENDOR_DD')
        $staffOptionKeys = @(Get-DropdownOptionKeys -Form $baselineLive -FieldCode 'STAFF_DD')

        $missingVendorOptions = @($requiredVendor | Where-Object { $vendorOptionKeys -notcontains $_ } | Sort-Object -Unique)
        $missingStaffOptions = @($requiredStaff | Where-Object { $staffOptionKeys -notcontains $_ } | Sort-Object -Unique)

        Write-Host ("Live / Preview revision  : {0} / {1}" -f $baselineLive.revision, $baselinePreview.revision)
        Write-Host ("Preview / Live diff      : NONE")
        Write-Host ("Form SHA256              : {0}" -f $baselineFormHash)
        Write-Host ("Unique Vendor / Missing  : {0} / {1}" -f $requiredVendor.Count, $missingVendorOptions.Count)
        Write-Host ("Unique Staff / Missing   : {0} / {1}" -f $requiredStaff.Count, $missingStaffOptions.Count)

        if ($missingVendorOptions.Count -gt 0) {
            Write-Host ("Missing VENDOR_DD        : {0}" -f ($missingVendorOptions -join ' | ')) -ForegroundColor Yellow
        }
        if ($missingStaffOptions.Count -gt 0) {
            Write-Host ("Missing STAFF_DD         : {0}" -f ($missingStaffOptions -join ' | ')) -ForegroundColor Yellow
        }

        if ($missingVendorOptions.Count -gt $MaxNewVendorOptions) {
            throw "TooManyNewMasterOptions: Vendor missing=$($missingVendorOptions.Count) MaxNewVendorOptions=$MaxNewVendorOptions"
        }
        if ($missingStaffOptions.Count -gt $MaxNewStaffOptions) {
            throw "TooManyNewMasterOptions: Staff missing=$($missingStaffOptions.Count) MaxNewStaffOptions=$MaxNewStaffOptions"
        }

        foreach ($v in $requiredVendor) {
            $exists = ($vendorOptionKeys -contains $v)
            $masterOptionRows.Add([pscustomobject][ordered]@{
                MasterType='Vendor'; SourceValue=$v; OptionExistsBefore=$exists
                PlannedAdd=(-not $exists); Added=$false; ExistsAfterDeploy=$exists
                Status=$(if ($exists) {'Existing'} else {'PlannedAdd'})
            })
        }
        foreach ($v in $requiredStaff) {
            $exists = ($staffOptionKeys -contains $v)
            $masterOptionRows.Add([pscustomobject][ordered]@{
                MasterType='Staff'; SourceValue=$v; OptionExistsBefore=$exists
                PlannedAdd=(-not $exists); Added=$false; ExistsAfterDeploy=$exists
                Status=$(if ($exists) {'Existing'} else {'PlannedAdd'})
            })
        }
    }
    elseif ($Execute) {
        throw '安全停止: v0.7 Executeでは -SkipKintone を使用できません。'
    }

    # Execute前の業務安全条件
    if ($Execute) {
        if ($recordErrorCountPreflight -gt 0) {
            throw "安全停止: RecordErrorが $recordErrorCountPreflight 件あります。Form PUT/Deploy/POSTは行いません。"
        }
        if ($blockedApp270Add.Count -gt 0) {
            throw "安全停止: App270追加候補が $($blockedApp270Add.Count) 件あります。Phase1C v0.7.4ではApp270追加を禁止しています。"
        }
        # 既存App272の書込み候補はログのみ。Phase1Cは既存PUTを実行しない。
        if ($createCountPlanned -lt 1) {
            throw '安全停止: 新規App272候補が0件です。'
        }
        if ($createCountPlanned -gt $MaxCreates) {
            throw "CreateCountLimitExceeded: POST予定件数 $createCountPlanned が MaxCreates=$MaxCreates を超えています。"
        }
        if ($Environment -eq 'TEST' -and $createCountPlanned -ne 1) {
            throw "TEST安全停止: App292 Executeは新規候補1件だけ許可します。planned=$createCountPlanned"
        }

        $candidatePoList = @($createCandidates | ForEach-Object { ([string]$_.発注番号).Trim() })
        $duplicateCandidatePo = @($candidatePoList | Group-Object | Where-Object Count -gt 1)
        if ($duplicateCandidatePo.Count -gt 0) {
            $dupText = ($duplicateCandidatePo | ForEach-Object { "$($_.Name)x$($_.Count)" }) -join ','
            throw "DuplicatePoInCsv: 新規候補内に発注番号重複があります: $dupText"
        }

        $expectedPoList = @($ExpectedNewPo | ForEach-Object { ([string]$_).Trim() } |
            Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
        $duplicateExpectedPo = @($expectedPoList | Group-Object | Where-Object Count -gt 1)
        if ($duplicateExpectedPo.Count -gt 0) {
            $dupText = ($duplicateExpectedPo | ForEach-Object { "$($_.Name)x$($_.Count)" }) -join ','
            throw "ExpectedNewPoMismatch: -ExpectedNewPo に重複があります: $dupText"
        }
        if ($expectedPoList.Count -ne $createCountPlanned) {
            throw "ExpectedNewPoMismatch: ExpectedNewPo件数 $($expectedPoList.Count) と新規候補件数 $createCountPlanned が一致しません。"
        }

        $poDiff = @(Compare-Object `
            -ReferenceObject @($candidatePoList | Sort-Object) `
            -DifferenceObject @($expectedPoList | Sort-Object))
        if ($poDiff.Count -gt 0) {
            $diffText = ($poDiff | ForEach-Object { "$($_.InputObject)[$($_.SideIndicator)]" }) -join ','
            throw "ExpectedNewPoMismatch: ExpectedNewPo と新規候補の発注番号集合が一致しません: $diffText"
        }

        if (-not $formPreflightDone) {
            throw '安全停止: Master Lifecycle Preflight未実施です。'
        }

        # 不足optionsがある場合のみPreview PUT + Deploy。
        if ($missingVendorOptions.Count -gt 0 -or $missingStaffOptions.Count -gt 0) {
            Write-Host ''
            Write-Host '=== MASTER OPTION UPDATE START ===' -ForegroundColor Yellow
            $masterResult = Update-MasterDropdownOptionsAndDeploy `
                -BaseUrl $BaseUrl `
                -AppId $App272 `
                -Token $ApiToken272 `
                -BaselineLive $baselineLive `
                -BaselinePreview $baselinePreview `
                -MissingVendorOptions $missingVendorOptions `
                -MissingStaffOptions $missingStaffOptions

            $formPutExecuted = $masterResult.Changed
            $deployExecuted = $masterResult.Changed

            Write-Host ("Added Vendor: {0}" -f (@($masterResult.AddedVendor) -join ' | '))
            Write-Host ("Added Staff : {0}" -f (@($masterResult.AddedStaff) -join ' | '))
            Write-Host ("Deploy      : {0}" -f $masterResult.DeployStatus)

            foreach ($row in $masterOptionRows) {
                if ($row.MasterType -eq 'Vendor' -and @($masterResult.AddedVendor) -contains $row.SourceValue) {
                    $row.Added = $true
                    $row.ExistsAfterDeploy = $true
                    $row.Status = 'AddedAndDeployed'
                }
                elseif ($row.MasterType -eq 'Staff' -and @($masterResult.AddedStaff) -contains $row.SourceValue) {
                    $row.Added = $true
                    $row.ExistsAfterDeploy = $true
                    $row.Status = 'AddedAndDeployed'
                }
            }
        }
        else {
            Write-Host ''
            Write-Host 'Master options: all required values already exist. Form PUT/Deploy = NONE'
        }

        # POST直前: Live Formを再GETし、全Vendor/Staff optionの存在を保証。
        $liveBeforePost = Get-AppLiveFormFields -BaseUrl $BaseUrl -AppId $App272 -Token $ApiToken272
        Assert-MasterDropdownFields -Form $liveBeforePost -FieldCodes @('VENDOR_DD','STAFF_DD') -Context 'Live before POST'

        foreach ($v in $requiredVendor) {
            if (-not (Test-LiveMasterOptionExists -LiveForm $liveBeforePost -FieldCode 'VENDOR_DD' -Value $v)) {
                throw "MasterOptionDeployVerificationFailed: POST直前Live VENDOR_DDに '$v' がありません。"
            }
        }
        foreach ($v in $requiredStaff) {
            if (-not (Test-LiveMasterOptionExists -LiveForm $liveBeforePost -FieldCode 'STAFF_DD' -Value $v)) {
                throw "MasterOptionDeployVerificationFailed: POST直前Live STAFF_DDに '$v' がありません。"
            }
        }

        # POST直前: App272を再GETし、競合新規作成が無いことを確認。
        Write-Host 'POST直前 App272再GET / 重複PO確認...'
        $app272BeforePost = @(Invoke-KintoneGetAll -BaseUrl $BaseUrl -AppId $App272 -Token $ApiToken272)
        $beforePostByPo = @{}
        foreach ($r in $app272BeforePost) {
            $key = Get-KValue $r '数値'
            if ($key -and -not $beforePostByPo.ContainsKey($key)) { $beforePostByPo[$key] = $r }
        }
        foreach ($d in $createCandidates) {
            if ($beforePostByPo.ContainsKey([string]$d.発注番号)) {
                throw "DuplicatePoDetectedBeforePost: 発注番号 $($d.発注番号) はPOST直前にApp272へ存在しました。"
            }
        }

        $recordsForPost = New-Object System.Collections.Generic.List[object]
        foreach ($d in $createCandidates) {
            if ($d.App270 -ne 'Exists(NoUpdate)') {
                throw "App270LookupSourceMissing: PO=$($d.発注番号) App270=$($d.App270)"
            }
            if ($d.Lookup -ne 'SyncAfterCreate') {
                throw "安全停止: Lookup判定が想定外です。PO=$($d.発注番号) Lookup=$($d.Lookup)"
            }
            if (-not [string]::IsNullOrWhiteSpace([string]$d.Errors)) {
                throw "CsvRequiredValueBlank: PO=$($d.発注番号) Errors=$($d.Errors)"
            }

            $recordFields = [ordered]@{
                '数値'         = @{ value = [string]$d.発注番号 }
                'ODERNO'       = @{ value = [string]$d.ODERNO }
                'VENDOR_TEXT'  = @{ value = [string]$d.指示先名 }
                'VENDOR_DD'    = @{ value = [string]$d.指示先名 }
                'STAFF_TEXT'   = @{ value = [string]$d.手配担当者名 }
                'STAFF_DD'     = @{ value = [string]$d.手配担当者名 }
                '日付'         = @{ value = [string]$d.手配日 }
                '日付_1'       = @{ value = [string]$d.納期 }
                'ルックアップ' = @{ value = [string]$d.ODERNO }
            }

            # v0.7.4: 新規日程は、正常計算または明示フォールバックの場合だけPOSTする。
            $writeNewSchedule = (
                [string]$d.ScheduleDecision -eq 'Calculated-NewBatchCandidate' -or
                [string]$d.ScheduleDecision -eq 'Fallback-New-OrderDateToDueDate(ShortLeadTime)' -or
                [string]$d.ScheduleDecision -eq 'Fallback-New-OrderDateToDueDate(MasterNotFound)'
            )
            if ($writeNewSchedule) {
                if ([string]::IsNullOrWhiteSpace([string]$d.K加工着手日_候補) -or
                    [string]::IsNullOrWhiteSpace([string]$d.K完成検査日_候補)) {
                    throw "NewScheduleCandidateBlank: PO=$($d.発注番号) Decision=$($d.ScheduleDecision)"
                }
                $recordFields['K加工着手日'] = @{ value = [string]$d.K加工着手日_候補 }
                $recordFields['K完成検査日'] = @{ value = [string]$d.K完成検査日_候補 }
                $recordFields['K日程種別']   = @{ value = [string]$d.K日程種別_候補 }
                $recordFields['K暫定設定']   = @{ value = [string]$d.K暫定設定_候補 }
                if (-not [string]::IsNullOrWhiteSpace([string]$d.K計算根拠_候補)) {
                    $recordFields['K計算根拠'] = @{ value = [string]$d.K計算根拠_候補 }
                }
            }

            $recordsForPost.Add($recordFields)
        }

        Write-Host ''
        Write-Host ("=== EXECUTE START: App272 BULK POST {0}件 ===" -f $createCountPlanned) -ForegroundColor Yellow
        $postIndex = 0
        foreach ($d in $createCandidates) {
            $postIndex++
            Write-Host ("POST {0}/{1} Row={2} PO={3} ODERNO={4} Vendor={5} Staff={6}" -f `
                $postIndex, $createCountPlanned, $d.Row, $d.発注番号, $d.ODERNO, $d.指示先名, $d.手配担当者名)
        }
        Write-Host 'Fields=基幹9項目 + 新規日程対象は K加工着手日,K完成検査日,K日程種別,K暫定設定[,K計算根拠]'

        $resp = Invoke-KintoneCreateRecords `
            -BaseUrl $BaseUrl `
            -AppId $App272 `
            -Token $ApiToken272 `
            -LookupSourceToken $ApiToken270 `
            -Records $recordsForPost.ToArray()

        $ids = @($resp.ids)
        $revisions = @($resp.revisions)
        if ($ids.Count -ne $createCountPlanned -or $revisions.Count -ne $createCountPlanned) {
            throw "BulkPostFailed: response count mismatch. planned=$createCountPlanned ids=$($ids.Count) revisions=$($revisions.Count)"
        }

        for ($i = 0; $i -lt $createCountPlanned; $i++) {
            $d = $createCandidates[$i]
            $d.WRITE = "DONE:POST:NEW:id=$($ids[$i]):revision=$($revisions[$i])"
        }

        # POST後再GETし、CSV正本とApp272の9項目を完全照合する。
        Write-Host 'POST後 App272再GET / 9項目検証...'
        $app272AfterPost = @(Invoke-KintoneGetAll -BaseUrl $BaseUrl -AppId $App272 -Token $ApiToken272)
        $afterPostByPo = @{}
        foreach ($r in $app272AfterPost) {
            $key = Get-KValue $r '数値'
            if ($key) {
                if ($afterPostByPo.ContainsKey($key)) {
                    $verificationErrors.Add("Duplicate PO after POST: $key")
                }
                else {
                    $afterPostByPo[$key] = $r
                }
            }
        }

        foreach ($d in $createCandidates) {
            $po = [string]$d.発注番号
            $result = 'PASS'
            $errorText = ''
            $actual = $null

            if (-not $afterPostByPo.ContainsKey($po)) {
                $result = 'FAIL'
                $errorText = 'Created record not found after POST'
                $verificationErrors.Add("PO=$po created record not found")
            }
            else {
                $actual = $afterPostByPo[$po]
                $checks = [ordered]@{
                    '数値'         = [string]$d.発注番号
                    'ODERNO'       = [string]$d.ODERNO
                    'VENDOR_TEXT'  = [string]$d.指示先名
                    'VENDOR_DD'    = [string]$d.指示先名
                    'STAFF_TEXT'   = [string]$d.手配担当者名
                    'STAFF_DD'     = [string]$d.手配担当者名
                    '日付'         = [string]$d.手配日
                    '日付_1'       = [string]$d.納期
                    'ルックアップ' = [string]$d.ODERNO
                }

                $verifyNewSchedule = (
                    [string]$d.ScheduleDecision -eq 'Calculated-NewBatchCandidate' -or
                    [string]$d.ScheduleDecision -eq 'Fallback-New-OrderDateToDueDate(ShortLeadTime)' -or
                    [string]$d.ScheduleDecision -eq 'Fallback-New-OrderDateToDueDate(MasterNotFound)'
                )
                if ($verifyNewSchedule) {
                    $checks['K加工着手日'] = [string]$d.K加工着手日_候補
                    $checks['K完成検査日'] = [string]$d.K完成検査日_候補
                    $checks['K日程種別'] = [string]$d.K日程種別_候補
                    $checks['K暫定設定'] = [string]$d.K暫定設定_候補
                    if (-not [string]::IsNullOrWhiteSpace([string]$d.K計算根拠_候補)) {
                        $checks['K計算根拠'] = [string]$d.K計算根拠_候補
                    }
                }

                $mismatches = New-Object System.Collections.Generic.List[string]
                foreach ($fieldCode in $checks.Keys) {
                    $expected = [string]$checks[$fieldCode]
                    $actualValue = Get-KValue $actual $fieldCode
                    if ($actualValue -ne $expected) {
                        $mismatches.Add("$fieldCode expected='$expected' actual='$actualValue'")
                    }
                }

                if ($mismatches.Count -gt 0) {
                    $result = 'FAIL'
                    $errorText = ($mismatches -join ' | ')
                    $verificationErrors.Add("PO=$po $errorText")
                }
            }

            $postVerificationRows.Add([pscustomobject][ordered]@{
                Row = $d.Row
                発注番号 = $po
                ODERNO = [string]$d.ODERNO
                指示先名 = [string]$d.指示先名
                手配担当者名 = [string]$d.手配担当者名
                VENDOR_TEXT_after = $(if ($null -ne $actual) { Get-KValue $actual 'VENDOR_TEXT' } else { '' })
                VENDOR_DD_after = $(if ($null -ne $actual) { Get-KValue $actual 'VENDOR_DD' } else { '' })
                STAFF_TEXT_after = $(if ($null -ne $actual) { Get-KValue $actual 'STAFF_TEXT' } else { '' })
                STAFF_DD_after = $(if ($null -ne $actual) { Get-KValue $actual 'STAFF_DD' } else { '' })
                K加工着手日_after = $(if ($null -ne $actual) { Get-KValue $actual 'K加工着手日' } else { '' })
                K完成検査日_after = $(if ($null -ne $actual) { Get-KValue $actual 'K完成検査日' } else { '' })
                K日程種別_after = $(if ($null -ne $actual) { Get-KValue $actual 'K日程種別' } else { '' })
                K暫定設定_after = $(if ($null -ne $actual) { Get-KValue $actual 'K暫定設定' } else { '' })
                K計算根拠_after = $(if ($null -ne $actual) { Get-KValue $actual 'K計算根拠' } else { '' })
                Result = $result
                Error = $errorText
            })
        }

        if ($verificationErrors.Count -gt 0) {
            throw "PostVerificationMismatch: $($verificationErrors.Count) records/conditions failed. $($verificationErrors -join ' || ')"
        }

        Write-Host ("=== EXECUTE COMPLETE: App272新規 {0}件 / 基幹9項目＋対象K日程検証PASS ===" -f $createCountPlanned) -ForegroundColor Green
    }
    else {
        Write-Host ''
        Write-Host '=== DRYRUN MASTER PLAN ==='
        Write-Host ("Unique Vendor              : {0}" -f $requiredVendor.Count)
        Write-Host ("Unique Staff               : {0}" -f $requiredStaff.Count)
        Write-Host ("Missing VENDOR_DD options  : {0}" -f $missingVendorOptions.Count)
        Write-Host ("Missing STAFF_DD options   : {0}" -f $missingStaffOptions.Count)
        Write-Host ("Planned POST               : {0}" -f $createCountPlanned)
        Write-Host 'FORM PUT                   : NONE'
        Write-Host 'DEPLOY                     : NONE'
        Write-Host 'RECORD WRITE               : NONE'
    }

    $stamp = (Get-Date).ToString('yyyyMMdd_HHmmss')
    $modeName = $(if ($Execute) { "Execute" } else { "DryRun" })
    $detailPath = Join-Path $LogDirectory "Import-App272_Phase1C_${modeName}_Detail_$stamp.csv"
    $summaryPath = Join-Path $LogDirectory "Import-App272_Phase1C_${modeName}_Summary_$stamp.txt"
    $masterOptionPath = Join-Path $LogDirectory "Import-App272_Phase1C_${modeName}_MasterOptionDetail_$stamp.csv"
    $postVerifyPath = Join-Path $LogDirectory "Import-App272_Phase1C_${modeName}_PostVerification_$stamp.csv"

    $details | Export-Csv -LiteralPath $detailPath -NoTypeInformation -Encoding UTF8
    if ($masterOptionRows.Count -gt 0) {
        $masterOptionRows | Export-Csv -LiteralPath $masterOptionPath -NoTypeInformation -Encoding UTF8
    }
    if ($postVerificationRows.Count -gt 0) {
        $postVerificationRows | Export-Csv -LiteralPath $postVerifyPath -NoTypeInformation -Encoding UTF8
    }

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
    $countNewFallbackShort = @($details | Where-Object ScheduleDecision -eq 'Fallback-New-OrderDateToDueDate(ShortLeadTime)').Count
    $countNewFallbackMaster = @($details | Where-Object ScheduleDecision -eq 'Fallback-New-OrderDateToDueDate(MasterNotFound)').Count
    $countNewFallback = $countNewFallbackShort + $countNewFallbackMaster
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
New fallback schedules        : $countNewFallback
  ShortLeadTime               : $countNewFallbackShort
  MasterNotFound              : $countNewFallbackMaster
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
Base diff scope               : existing-record audit only; Phase1C performs no existing record PUT
Vendor sync rule              : CSV 指示先名 -> VENDOR_TEXT + VENDOR_DD
Staff sync rule                 : CSV 手配担当者名 -> STAFF_TEXT + STAFF_DD
Unique Vendor                   : $($requiredVendor.Count)
Unique Staff                    : $($requiredStaff.Count)
Missing VENDOR_DD options       : $($missingVendorOptions.Count)
Missing STAFF_DD options        : $($missingStaffOptions.Count)
Preview/Live diff count         : $previewLiveDiffCount
Baseline Form SHA256            : $baselineFormHash
Form PUT executed               : $formPutExecuted
Deploy executed                 : $deployExecuted
Deploy status endpoint          : GET /k/v1/preview/app/deploy.json?apps[0]=<AppId>
Post verification errors        : $($verificationErrors.Count)
Schedule type rule            : Manual > Batch > Auto (active in DryRun decision)
Existing Manual/Batch          : Protected
Existing Auto/blank            : Batch recalculation candidate
New schedule source            : Batch
Schedule algorithm            : 暦日差引 → 検査=前営業日 / 着手=翌営業日
New fallback rule             : ShortLeadTime/MasterNotFound -> K着手=手配日 / K検査=納期 / Batch / ON / K計算根拠空欄
Planned POST creates          : $createCountPlanned
Blocked existing writes       : $($blockedExistingWrite.Count)
MaxCreates                    : $MaxCreates
MaxNewVendorOptions           : $MaxNewVendorOptions
MaxNewStaffOptions            : $MaxNewStaffOptions
WRITE                         : $(if ($Execute) { "BULK POST executed for $createCountPlanned verified new App272 records only" } else { 'NONE' })
FORM UPDATE                   : $(if ($formPutExecuted) { 'EXECUTED' } else { 'NONE' })
DEPLOY                        : $(if ($deployExecuted) { 'EXECUTED' } else { 'NONE' })
DetailLog                     : $detailPath
MasterOptionLog               : $(if ($masterOptionRows.Count -gt 0) { $masterOptionPath } else { '<none>' })
PostVerificationLog           : $(if ($postVerificationRows.Count -gt 0) { $postVerifyPath } else { '<none>' })
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
