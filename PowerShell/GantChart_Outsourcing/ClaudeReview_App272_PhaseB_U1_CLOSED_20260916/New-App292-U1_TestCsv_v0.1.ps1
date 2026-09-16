#requires -Version 5.1
<#
.SYNOPSIS
  App292 U1用 1行テストCSV作成 v0.1

.DESCRIPTION
  本番用整形済みCSVから指定POを1行だけ抽出し、
  U1用に「納期だけ」を変更したテストCSVを作成する。
  kintoneへの通信・書込みは一切行わない。

  元CSVの重複ヘッダーに対応するため TextFieldParser を使用する。
#>

[CmdletBinding()]
param(
    [string]$SourceCsv = 'C:\HPDB\外注課納期管理_20260916_整形済み.csv',
    [string]$OutputCsv = 'C:\HPDB\App292_U1_2609249755_DueChanged_20260916.csv',
    [string]$Po = '2609249755',
    [string]$ExpectedOldDueDate = '2026-11-02',
    [string]$NewDueDate = '2026-11-06'
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName Microsoft.VisualBasic

function Import-CsvAllowDuplicateHeaders {
    param([Parameter(Mandatory=$true)][string]$LiteralPath)

    $enc = [Text.Encoding]::GetEncoding(932)
    $parser = New-Object Microsoft.VisualBasic.FileIO.TextFieldParser($LiteralPath, $enc, $true)
    try {
        $parser.TextFieldType = [Microsoft.VisualBasic.FileIO.FieldType]::Delimited
        $parser.SetDelimiters(',')
        $parser.HasFieldsEnclosedInQuotes = $true
        $rawHeaders = $parser.ReadFields()
        if ($null -eq $rawHeaders) { throw "CSV header not found." }

        $seen = @{}
        $headers = New-Object System.Collections.Generic.List[string]
        foreach ($h0 in $rawHeaders) {
            $h = [string]$h0
            if (-not $seen.ContainsKey($h)) {
                $seen[$h] = 1
                $headers.Add($h)
            } else {
                $seen[$h]++
                $headers.Add(("{0}__DUP{1}" -f $h,$seen[$h]))
            }
        }

        $rows = New-Object System.Collections.Generic.List[object]
        while (-not $parser.EndOfData) {
            $fields = $parser.ReadFields()
            if ($null -eq $fields) { continue }
            if ($fields.Count -ne $headers.Count) {
                throw "CSV field count mismatch: fields=$($fields.Count), headers=$($headers.Count)"
            }

            $o = [ordered]@{}
            for ($i=0; $i -lt $headers.Count; $i++) {
                $o[$headers[$i]] = [string]$fields[$i]
            }
            $rows.Add([pscustomobject]$o)
        }
        return $rows.ToArray()
    }
    finally {
        $parser.Close()
        $parser.Dispose()
    }
}

if (-not (Test-Path -LiteralPath $SourceCsv)) {
    throw "SourceCsv not found: $SourceCsv"
}

$rows = @(Import-CsvAllowDuplicateHeaders -LiteralPath $SourceCsv)
$match = @($rows | Where-Object { [string]$_.発注番号 -eq $Po })

if ($match.Count -ne 1) {
    throw "SafetyStop: PO=$Po matched=$($match.Count), expected exactly 1."
}

$row = $match[0]
$oldDue = [string]$row.納期
if ($oldDue -ne $ExpectedOldDueDate) {
    throw "SafetyStop: old due unexpected. actual=$oldDue expected=$ExpectedOldDueDate"
}

# 変更前スナップショット
$before = [ordered]@{}
foreach ($p in $row.PSObject.Properties) { $before[$p.Name] = [string]$p.Value }

# U1では納期だけ変更
$row.納期 = $NewDueDate

# 変更フィールド検査
$changed = New-Object System.Collections.Generic.List[string]
foreach ($p in $row.PSObject.Properties) {
    $name = $p.Name
    if ([string]$p.Value -ne [string]$before[$name]) { $changed.Add($name) }
}

if ($changed.Count -ne 1 -or $changed[0] -ne '納期') {
    throw "SafetyStop: U1 CSV must change only 納期. Changed=$($changed -join ',')"
}

$row | Export-Csv -LiteralPath $OutputCsv -NoTypeInformation -Encoding Default

# 再読込して1行であることを確認
$verify = @(Import-Csv -LiteralPath $OutputCsv -Encoding Default)
if ($verify.Count -ne 1) { throw "Output verification failed: rows=$($verify.Count)" }
if ([string]$verify[0].発注番号 -ne $Po) { throw "Output PO mismatch." }
if ([string]$verify[0].納期 -ne $NewDueDate) { throw "Output due mismatch." }

Write-Host "=== App292 U1 Test CSV Builder v0.1 ==="
Write-Host "Source CSV       : $SourceCsv"
Write-Host "Output CSV       : $OutputCsv"
Write-Host "Rows             : 1"
Write-Host "PO               : $Po"
Write-Host "手配日           : $($row.手配日)"
Write-Host "旧納期           : $oldDue"
Write-Host "新納期           : $NewDueDate"
Write-Host "指示先名         : $($row.指示先名)"
Write-Host "手配担当者名     : $($row.手配担当者名)"
Write-Host "Changed fields   : $($changed -join ',')"
Write-Host "Kintone WRITE    : NONE"
Write-Host "RESULT           : PASS"
