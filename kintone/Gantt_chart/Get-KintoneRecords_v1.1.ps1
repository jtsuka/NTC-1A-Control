<#
.SYNOPSIS
  kintone のレコードを REST API で取得し、JSON / CSV / LOG を保存します。

.DESCRIPTION
  - PowerShell 5.1 対応
  - APIトークンは実行後にCUIで入力（マスク表示）
  - Cursor APIを使って全件取得
  - JSON / CSV / LOG を保存
  - App ID は引数指定または実行時入力
#>

[CmdletBinding()]
param(
    [string]$BaseUrl,
    [Nullable[int]]$AppId = $null,
    [string]$Query = "",
    [string]$OutputDir = ".\kintone_export"
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"

function Write-Log {
    param(
        [string]$Message,
        [ValidateSet("INFO","WARN","ERROR")]
        [string]$Level = "INFO"
    )

    $line = "{0} [{1}] {2}" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss"), $Level, $Message
    Write-Host $line
    if ($script:LogFile) {
        $line | Out-File -FilePath $script:LogFile -Append -Encoding utf8
    }
}

function Convert-SecureStringToPlainText {
    param([Security.SecureString]$SecureString)

    $ptr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($SecureString)
    try {
        [Runtime.InteropServices.Marshal]::PtrToStringBSTR($ptr)
    }
    finally {
        [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($ptr)
    }
}

function Convert-KintoneRecordToFlatObject {
    param($Record)

    $result = [ordered]@{}

    foreach ($prop in $Record.PSObject.Properties) {
        $fieldCode = $prop.Name
        $field = $prop.Value

        if ($null -eq $field) {
            $result[$fieldCode] = $null
            continue
        }

        if ($field.PSObject.Properties.Name -contains "value") {
            $value = $field.value

            if (($value -is [System.Array]) -or ($value -is [PSCustomObject])) {
                $result[$fieldCode] = ($value | ConvertTo-Json -Depth 20 -Compress)
            }
            else {
                $result[$fieldCode] = $value
            }
        }
        else {
            $result[$fieldCode] = ($field | ConvertTo-Json -Depth 20 -Compress)
        }
    }

    [PSCustomObject]$result
}

try {
    if ([string]::IsNullOrWhiteSpace($BaseUrl)) {
        $BaseUrl = Read-Host "kintone Base URL を入力してください 例: https://example.cybozu.com"
    }

    $BaseUrl = $BaseUrl.TrimEnd("/")

    if ($null -eq $AppId -or $AppId -le 0) {
        while ($true) {
            $appIdInput = Read-Host "kintone App ID を入力してください 例: 270"
            $parsedAppId = 0
            if ([int]::TryParse($appIdInput, [ref]$parsedAppId) -and $parsedAppId -gt 0) {
                $AppId = $parsedAppId
                break
            }

            Write-Host "App ID は正の整数で入力してください。" -ForegroundColor Yellow
        }
    }

    if (-not (Test-Path -LiteralPath $OutputDir)) {
        New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
    }

    $timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $script:LogFile = Join-Path $OutputDir ("kintone_get_{0}_app{1}.log" -f $timestamp, $AppId)
    $jsonFile = Join-Path $OutputDir ("kintone_records_{0}_app{1}.json" -f $timestamp, $AppId)
    $csvFile  = Join-Path $OutputDir ("kintone_records_{0}_app{1}.csv"  -f $timestamp, $AppId)

    Write-Log "=== kintone レコード取得開始 ==="
    Write-Log ("BaseUrl : {0}" -f $BaseUrl)
    Write-Log ("AppId   : {0}" -f $AppId)
    Write-Log ("Query   : {0}" -f ($(if ($Query) { $Query } else { "(なし)" })))
    Write-Log ("Output  : {0}" -f (Resolve-Path $OutputDir).Path)

    $secureToken = Read-Host "APIトークンを入力してください" -AsSecureString
    $apiToken = Convert-SecureStringToPlainText $secureToken

    if ([string]::IsNullOrWhiteSpace($apiToken)) {
        throw "APIトークンが入力されていません。"
    }

    $headers = @{
        "X-Cybozu-API-Token" = $apiToken
    }

    $createCursorUri = "$BaseUrl/k/v1/records/cursor.json"

    $createBodyObj = @{
        app  = $AppId
        size = 500
    }

    if (-not [string]::IsNullOrWhiteSpace($Query)) {
        $createBodyObj["query"] = $Query
    }

    $createBody = $createBodyObj | ConvertTo-Json -Depth 10

    Write-Log "Cursor を作成します。"
    $cursorResponse = Invoke-RestMethod `
        -Method Post `
        -Uri $createCursorUri `
        -Headers $headers `
        -Body $createBody `
        -ContentType "application/json; charset=utf-8"

    $cursorId = $cursorResponse.id

    if ([string]::IsNullOrWhiteSpace($cursorId)) {
        throw "Cursor ID を取得できませんでした。"
    }

    Write-Log ("Cursor ID: {0}" -f $cursorId)

    $allRecords = New-Object System.Collections.ArrayList
    $hasNext = $true
    $page = 0

    while ($hasNext) {
        $page++

        $getUri = "$BaseUrl/k/v1/records/cursor.json?id=$([uri]::EscapeDataString($cursorId))"

        $resp = Invoke-RestMethod `
            -Method Get `
            -Uri $getUri `
            -Headers $headers

        if ($resp.records) {
            foreach ($record in $resp.records) {
                [void]$allRecords.Add($record)
            }
        }

        $hasNext = [bool]$resp.next

        $countThisPage = 0
        if ($resp.records) {
            $countThisPage = $resp.records.Count
        }

        Write-Log ("取得ページ={0} / 今回={1}件 / 累計={2}件 / next={3}" -f `
            $page, $countThisPage, $allRecords.Count, $hasNext)
    }

    $allRecords | ConvertTo-Json -Depth 50 | Out-File -FilePath $jsonFile -Encoding utf8
    Write-Log ("JSON保存: {0}" -f $jsonFile)

    if ($allRecords.Count -gt 0) {
        $flatRecords = foreach ($record in $allRecords) {
            Convert-KintoneRecordToFlatObject -Record $record
        }

        $flatRecords | Export-Csv -Path $csvFile -NoTypeInformation -Encoding UTF8
        Write-Log ("CSV保存 : {0}" -f $csvFile)
    }
    else {
        Write-Log "取得件数が0件のためCSVは作成しません。" "WARN"
    }

    Write-Log ("取得完了: {0}件" -f $allRecords.Count)
    Write-Log "=== 正常終了 ==="

    Write-Host ""
    Write-Host "取得件数 : $($allRecords.Count)"
    Write-Host "JSON     : $jsonFile"
    if (Test-Path -LiteralPath $csvFile) {
        Write-Host "CSV      : $csvFile"
    }
    Write-Host "LOG      : $script:LogFile"
}
catch {
    $msg = $_.Exception.Message
    try {
        Write-Log $msg "ERROR"
    }
    catch {
        Write-Host ("ERROR: {0}" -f $msg)
    }

    Write-Host ""
    Write-Host "異常終了しました。" -ForegroundColor Red
    exit 1
}
finally {
    $apiToken = $null
    $secureToken = $null
}
