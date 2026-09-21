#requires -Version 5.1
<#
.SYNOPSIS
  App292 NoJOIN TEST ドロップダウンoption準備 v0.2
.DESCRIPTION
  Windows PowerShell 5.1用。
  C:\HPDB に置いたNoJOIN想定CSVから、VENDOR_DD / STAFF_DD の不足optionを確認する。
  既定DryRun。Execute時だけApp292(TEST)のPreview更新→Deploy→Live再確認を行う。
#>

[CmdletBinding()]
param(
    [string]$CsvPath = 'C:\HPDB\外注課納期管理_NoJOIN想定_全937件_App292_TEST_20260921.csv',
    [string]$BaseUrl = 'https://bcurbkixz609.cybozu.com',
    [int]$AppId = 292,
    [string]$TokenPath = 'C:\HPDB\Secrets\App292_Test.token',
    [switch]$DryRun,
    [switch]$Execute,
    [string]$ConfirmExecute = '',
    [ValidateRange(0,200)][int]$MaxNewVendorOptions = 100,
    [ValidateRange(0,50)][int]$MaxNewStaffOptions = 20
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

if ($AppId -ne 292) { throw 'SafetyStop: AppId must be 292.' }
if ($DryRun -and $Execute) { throw 'SafetyStop: DryRun/Execute conflict.' }
if (-not $DryRun -and -not $Execute) { $DryRun = $true }
if ($Execute -and $ConfirmExecute -ne 'APP292-NOJOIN-MASTER-EXECUTE') {
    throw "SafetyStop: Execute requires -ConfirmExecute 'APP292-NOJOIN-MASTER-EXECUTE'"
}
if (-not (Test-Path -LiteralPath $CsvPath -PathType Leaf)) { throw "CSV not found: $CsvPath" }

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

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "Token not found: $Path"
    }

    $enc = (Get-Content -LiteralPath $Path -Raw).Trim()
    $sec = $enc | ConvertTo-SecureString
    $ptr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($sec)

    try { return [Runtime.InteropServices.Marshal]::PtrToStringBSTR($ptr) }
    finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($ptr) }
}

function Get-Form {
    param([string]$Token, [switch]$Preview)

    $headers = @{ 'X-Cybozu-API-Token' = $Token }
    $uri = if ($Preview) {
        $BaseUrl.TrimEnd('/') + '/k/v1/preview/app/form/fields.json'
    } else {
        $BaseUrl.TrimEnd('/') + '/k/v1/app/form/fields.json'
    }

    return Invoke-RestMethod -Method Get -Uri $uri -Headers $headers -Body @{ app = $AppId; lang = 'ja' }
}

function Get-OptionKeys {
    param($Form, [string]$Code)

    $p = $Form.properties.PSObject.Properties[$Code]
    if ($null -eq $p) { throw "Field not found: $Code" }
    if ([string]$p.Value.type -ne 'DROP_DOWN') {
        throw "Field type mismatch: $Code type=$($p.Value.type)"
    }

    return @($p.Value.options.PSObject.Properties | ForEach-Object { $_.Name })
}

function New-OptionsWithAdditions {
    param($Options, [string[]]$AddValues)

    $out = [ordered]@{}
    $maxIndex = -1

    foreach ($p in $Options.PSObject.Properties) {
        $idx = [int]$p.Value.index
        if ($idx -gt $maxIndex) { $maxIndex = $idx }

        $out[$p.Name] = [ordered]@{
            label = [string]$p.Value.label
            index = [string]$idx
        }
    }

    foreach ($v in @($AddValues | Sort-Object -Unique)) {
        if (-not $out.Contains($v)) {
            $maxIndex++
            $out[$v] = [ordered]@{ label = $v; index = [string]$maxIndex }
        }
    }

    return $out
}

$rows = @(Import-CsvAllowDuplicateHeaders -Path $CsvPath)

if ($rows.Count -ne 937) {
    throw "SafetyStop: Expected 937 CSV rows. Actual=$($rows.Count)"
}

$vendors = @(
    $rows |
    ForEach-Object { ([string]$_.指示先名).Trim() } |
    Where-Object { $_ } |
    Sort-Object -Unique
)

$staff = @(
    $rows |
    ForEach-Object { ([string]$_.手配担当者名).Trim() } |
    Where-Object { $_ } |
    Sort-Object -Unique
)

$token = Get-PlainToken -Path $TokenPath
$live = Get-Form -Token $token
$preview = Get-Form -Token $token -Preview

if ([string]$live.revision -ne [string]$preview.revision) {
    throw "SafetyStop: undeployed form changes exist. Live=$($live.revision) Preview=$($preview.revision)"
}

$vendorExisting = @(Get-OptionKeys -Form $live -Code 'VENDOR_DD')
$staffExisting = @(Get-OptionKeys -Form $live -Code 'STAFF_DD')

$missingVendors = @($vendors | Where-Object { $vendorExisting -notcontains $_ })
$missingStaff = @($staff | Where-Object { $staffExisting -notcontains $_ })

Write-Host '================================================================='
Write-Host 'Prepare App292 NoJOIN Master Options v0.2'
Write-Host '================================================================='
Write-Host "CSV rows               : $($rows.Count)"
Write-Host "Unique vendors         : $($vendors.Count)"
Write-Host "Missing VENDOR_DD      : $($missingVendors.Count)"
Write-Host "Unique staff           : $($staff.Count)"
Write-Host "Missing STAFF_DD       : $($missingStaff.Count)"
Write-Host 'App270/App272          : NOT USED'

foreach ($x in $missingVendors) { Write-Host "  [VENDOR+] $x" }
foreach ($x in $missingStaff) { Write-Host "  [STAFF+]  $x" }

if ($missingVendors.Count -gt $MaxNewVendorOptions) {
    throw 'SafetyStop: missing vendors exceed MaxNewVendorOptions.'
}
if ($missingStaff.Count -gt $MaxNewStaffOptions) {
    throw 'SafetyStop: missing staff exceed MaxNewStaffOptions.'
}

if (-not $Execute) {
    Write-Host 'WRITE                  : NONE'
    Write-Host 'RESULT                 : DRYRUN PASS'
    exit 0
}

if ($missingVendors.Count -eq 0 -and $missingStaff.Count -eq 0) {
    Write-Host 'No option additions required.'
    Write-Host 'RESULT                 : PASS'
    exit 0
}

$props = [ordered]@{}

if ($missingVendors.Count -gt 0) {
    $props['VENDOR_DD'] = [ordered]@{
        type = 'DROP_DOWN'
        options = New-OptionsWithAdditions `
            -Options $preview.properties.VENDOR_DD.options `
            -AddValues $missingVendors
    }
}

if ($missingStaff.Count -gt 0) {
    $props['STAFF_DD'] = [ordered]@{
        type = 'DROP_DOWN'
        options = New-OptionsWithAdditions `
            -Options $preview.properties.STAFF_DD.options `
            -AddValues $missingStaff
    }
}

$headers = @{ 'X-Cybozu-API-Token' = $token }
$fieldsUri = $BaseUrl.TrimEnd('/') + '/k/v1/preview/app/form/fields.json'
$body = [ordered]@{
    app = $AppId
    revision = [string]$preview.revision
    properties = $props
} | ConvertTo-Json -Depth 100

$put = Invoke-RestMethod -Method Put -Uri $fieldsUri -Headers $headers `
    -ContentType 'application/json; charset=utf-8' `
    -Body ([Text.Encoding]::UTF8.GetBytes($body))

$deployUri = $BaseUrl.TrimEnd('/') + '/k/v1/preview/app/deploy.json'
$deployBody = [ordered]@{
    apps = @(@{ app = $AppId; revision = [string]$put.revision })
    revert = $false
} | ConvertTo-Json -Depth 10

Invoke-RestMethod -Method Post -Uri $deployUri -Headers $headers `
    -ContentType 'application/json; charset=utf-8' `
    -Body ([Text.Encoding]::UTF8.GetBytes($deployBody)) | Out-Null

$statusUri = $deployUri + '?apps[0]=' + [uri]::EscapeDataString([string]$AppId)

$success = $false
for ($i = 1; $i -le 40; $i++) {
    Start-Sleep -Seconds 3

    $status = Invoke-RestMethod -Method Get -Uri $statusUri -Headers $headers
    $entry = @(
        $status.apps |
        Where-Object { [string]$_.app -eq [string]$AppId } |
        Select-Object -First 1
    )

    if ($entry.Count -eq 0) { throw 'Deploy status entry missing.' }

    Write-Host "Deploy $i/40 : $($entry[0].status)"

    if ($entry[0].status -eq 'SUCCESS') {
        $success = $true
        break
    }

    if ($entry[0].status -in @('FAIL','CANCEL')) {
        throw "Deploy failed: $($entry[0].status)"
    }
}

if (-not $success) { throw 'Deploy timeout.' }

$after = Get-Form -Token $token
$vendorAfter = @(Get-OptionKeys -Form $after -Code 'VENDOR_DD')
$staffAfter = @(Get-OptionKeys -Form $after -Code 'STAFF_DD')

foreach ($x in $missingVendors) {
    if ($vendorAfter -notcontains $x) { throw "PostVerifyFailed VENDOR_DD: $x" }
}

foreach ($x in $missingStaff) {
    if ($staffAfter -notcontains $x) { throw "PostVerifyFailed STAFF_DD: $x" }
}

Write-Host 'RESULT                 : PASS'
