#requires -version 5.1
<#
.SYNOPSIS
  RK-10 が開いている在庫帳の「数字シート」を、G列を明示キーとして並べ替える。

.DESCRIPTION
  従来の RK-10 は Excel の UI 操作（Alt -> H -> S -> U -> Enter）に依存していたため、
  Excel が「前回の並べ替え列=G」を記憶していない場合、並べ替えキーが空欄となり、
  G列による並べ替えが実行されないことがあった。

  本スクリプトは、新しい Excel.Application を起動せず、
  RK-10 が既に開いている Excel インスタンスに接続し、
  対象ブック・対象シートを明示的に取得して G列降順でソートする。

  最終行は A列では判定しない。
  管理番号A列は管理対象外行で空欄になるため、商品コードB列の最終非空行を使用する。

  安全方針:
    - 対象ブックが開いていなければ何もしないで異常終了
    - 対象シートがなければ何もしないで異常終了
    - 新しい Excel は起動しない
    - 既定では保存しない（RK-10 が後続処理・保存を担当）
    - Audit モードでは一切変更しない
    - Update モードでも指定した1シートのみ処理する

  想定フロー:
    RK-10でExcelを開く
      -> シートを選択
      -> 本スクリプトを Update で呼ぶ
      -> RK-10でA列の採番
      -> 次シートへ

.VERSION
  ScriptVersion = 2026-09-03-r1-OpenExcel-GSort

.TEST TARGET
  仙台在庫帳 / Sheet=1
  ProbeCode=BN0365SH
  期待: B列最終行までを対象に G列降順ソートし、G=1 の BN0365SH が管理対象側へ移動する。
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$WorkbookName,

    [Parameter(Mandatory = $true)]
    [string]$SheetName,

    [ValidateSet("Audit", "Update")]
    [string]$Mode = "Audit",

    [string]$LogPath = "",

    # 並べ替え範囲
    [int]$HeaderRow = 2,
    [int]$DataStartRow = 3,
    [string]$FirstColumn = "A",
    [string]$LastColumn = "P",

    # 最終行判定列。A列は空欄があるためB列を既定とする。
    [string]$LastRowColumn = "B",

    # 並べ替えキー列
    [string]$SortColumn = "G",

    # テスト確認用。指定した商品コードの行位置/G値を前後でログする。
    [string]$ProbeCode = "",

    # 通常は指定しない。RK-10が後続で保存する。
    [switch]$Save
)

$ErrorActionPreference = "Stop"
$ScriptVersion = "2026-09-03-r1-OpenExcel-GSort"

# Excel constants
$xlSortOnValues = 0
$xlDescending = 2
$xlYes = 1
$xlTopToBottom = 1
$xlPinYin = 1
$xlValues = -4163
$xlWhole = 1
$xlByRows = 1
$xlPrevious = 2

function Write-Log {
    param(
        [ValidateSet("INFO","WARN","ERROR","AUDIT","UPDATE")]
        [string]$Level,
        [string]$Message
    )

    $line = "{0} [{1}] {2}" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss"), $Level, $Message
    Write-Host $line

    if (-not [string]::IsNullOrWhiteSpace($LogPath)) {
        $dir = Split-Path -Parent $LogPath
        if (-not [string]::IsNullOrWhiteSpace($dir) -and -not (Test-Path -LiteralPath $dir)) {
            New-Item -ItemType Directory -Path $dir -Force | Out-Null
        }
        Add-Content -LiteralPath $LogPath -Value $line -Encoding UTF8
    }
}

function Release-ComObject {
    param($Object)
    if ($null -ne $Object -and [Runtime.InteropServices.Marshal]::IsComObject($Object)) {
        [void][Runtime.InteropServices.Marshal]::ReleaseComObject($Object)
    }
}

function Get-OpenExcelApplication {
    try {
        return [Runtime.InteropServices.Marshal]::GetActiveObject("Excel.Application")
    }
    catch {
        throw "開いている Excel.Application を取得できませんでした。RK-10/Excel が対象ブックを開いた状態で実行してください。"
    }
}

function Get-TargetWorkbook {
    param(
        $ExcelApp,
        [string]$Name
    )

    $found = $null

    foreach ($wb in @($ExcelApp.Workbooks)) {
        try {
            if ([string]::Equals([string]$wb.Name, $Name, [StringComparison]::OrdinalIgnoreCase)) {
                $found = $wb
                break
            }
        }
        finally {
            if ($null -ne $wb -and $null -eq $found) {
                Release-ComObject $wb
            }
        }
    }

    if ($null -eq $found) {
        $openNames = @()
        foreach ($wb2 in @($ExcelApp.Workbooks)) {
            try { $openNames += [string]$wb2.Name }
            finally { Release-ComObject $wb2 }
        }
        throw "対象ブック '$Name' が開かれていません。OpenWorkbooks=[$($openNames -join ', ')]"
    }

    return $found
}

function Get-LastDataRow {
    param(
        $Worksheet,
        [string]$ColumnLetter,
        [int]$MinimumRow
    )

    $col = $null
    $found = $null
    try {
        $col = $Worksheet.Columns.Item($ColumnLetter)

        # B列全体から最後の非空セルを Find で取得。
        # End(xlUp) よりもフィルター/途中空白の影響を受けにくくする。
        $found = $col.Find(
            "*",
            $null,
            $xlValues,
            $xlWhole,
            $xlByRows,
            $xlPrevious,
            $false,
            $false,
            $false
        )

        if ($null -eq $found) {
            return 0
        }

        $row = [int]$found.Row
        if ($row -lt $MinimumRow) {
            return 0
        }
        return $row
    }
    finally {
        Release-ComObject $found
        Release-ComObject $col
    }
}

function Get-GSummary {
    param(
        $Worksheet,
        [int]$StartRow,
        [int]$EndRow,
        [string]$ColumnLetter
    )

    $count1 = 0
    $count0 = 0
    $countBlank = 0
    $countOther = 0

    for ($r = $StartRow; $r -le $EndRow; $r++) {
        $cell = $null
        try {
            $cell = $Worksheet.Range("$ColumnLetter$r")
            $v = $cell.Value2

            if ($null -eq $v -or [string]::IsNullOrWhiteSpace([string]$v)) {
                $countBlank++
            }
            else {
                try {
                    $num = [double]$v
                    if ($num -eq 1) { $count1++ }
                    elseif ($num -eq 0) { $count0++ }
                    else { $countOther++ }
                }
                catch {
                    $countOther++
                }
            }
        }
        finally {
            Release-ComObject $cell
        }
    }

    return [pscustomobject]@{
        One   = $count1
        Zero  = $count0
        Blank = $countBlank
        Other = $countOther
    }
}

function Find-ProbeCode {
    param(
        $Worksheet,
        [string]$Code,
        [int]$StartRow,
        [int]$EndRow
    )

    if ([string]::IsNullOrWhiteSpace($Code)) {
        return $null
    }

    for ($r = $StartRow; $r -le $EndRow; $r++) {
        $b = $null
        $g = $null
        $a = $null
        try {
            $b = $Worksheet.Range("B$r")
            if ([string]::Equals(([string]$b.Value2).Trim(), $Code.Trim(), [StringComparison]::OrdinalIgnoreCase)) {
                $a = $Worksheet.Range("A$r")
                $g = $Worksheet.Range("G$r")
                return [pscustomobject]@{
                    Row = $r
                    A   = $a.Value2
                    B   = $b.Value2
                    G   = $g.Value2
                }
            }
        }
        finally {
            Release-ComObject $a
            Release-ComObject $g
            Release-ComObject $b
        }
    }

    return $null
}

$excel = $null
$workbook = $null
$worksheet = $null
$sort = $null
$sortFields = $null
$sortRange = $null
$keyRange = $null

try {
    Write-Log INFO "Start"
    Write-Log INFO "ScriptVersion=$ScriptVersion"
    Write-Log INFO "Mode=$Mode WorkbookName=$WorkbookName SheetName=$SheetName"
    Write-Log INFO "LastRowColumn=$LastRowColumn SortColumn=$SortColumn Range=$FirstColumn$HeaderRow`:$LastColumn(lastRow)"
    Write-Log INFO "Save=$($Save.IsPresent)"

    $excel = Get-OpenExcelApplication
    $workbook = Get-TargetWorkbook -ExcelApp $excel -Name $WorkbookName

    try {
        $worksheet = $workbook.Worksheets.Item($SheetName)
    }
    catch {
        throw "対象シート '$SheetName' がブック '$WorkbookName' にありません。"
    }

    # G列等の数式結果を最新化してから判定・ソートする。
    $worksheet.Calculate()

    $lastRow = Get-LastDataRow -Worksheet $worksheet -ColumnLetter $LastRowColumn -MinimumRow $DataStartRow
    if ($lastRow -lt $DataStartRow) {
        throw "データ最終行を取得できませんでした。Sheet=$SheetName Column=$LastRowColumn"
    }

    Write-Log INFO "DetectedLastRow=$lastRow by Column=$LastRowColumn"

    $beforeSummary = Get-GSummary -Worksheet $worksheet -StartRow $DataStartRow -EndRow $lastRow -ColumnLetter $SortColumn
    Write-Log INFO "BeforeSort $SortColumn Summary: 1=$($beforeSummary.One) 0=$($beforeSummary.Zero) Blank=$($beforeSummary.Blank) Other=$($beforeSummary.Other)"

    $probeBefore = Find-ProbeCode -Worksheet $worksheet -Code $ProbeCode -StartRow $DataStartRow -EndRow $lastRow
    if ($null -ne $probeBefore) {
        Write-Log INFO "ProbeBefore Code=$ProbeCode Row=$($probeBefore.Row) A=$($probeBefore.A) G=$($probeBefore.G)"
    }
    elseif (-not [string]::IsNullOrWhiteSpace($ProbeCode)) {
        Write-Log WARN "ProbeBefore Code=$ProbeCode not found."
    }

    if ($Mode -eq "Audit") {
        Write-Log AUDIT "Audit completed. Worksheet was not changed."
        exit 0
    }

    $sortRange = $worksheet.Range("$FirstColumn$HeaderRow`:$LastColumn$lastRow")
    $keyRange  = $worksheet.Range("$SortColumn$DataStartRow`:$SortColumn$lastRow")

    $sort = $worksheet.Sort
    $sortFields = $sort.SortFields
    $sortFields.Clear()

    # G列そのものを明示的にソートキーとして指定。
    # Excel の「前回の並べ替え条件」やダイアログ状態には依存しない。
    [void]$sortFields.Add(
        $keyRange,
        $xlSortOnValues,
        $xlDescending,
        $null,
        0
    )

    $sort.SetRange($sortRange)
    $sort.Header = $xlYes
    $sort.MatchCase = $false
    $sort.Orientation = $xlTopToBottom
    $sort.SortMethod = $xlPinYin

    Write-Log UPDATE "SortApply Sheet=$SheetName Key=$SortColumn Order=Descending Range=$FirstColumn$HeaderRow`:$LastColumn$lastRow"
    $sort.Apply()

    # ソート後の数式再計算
    $worksheet.Calculate()

    $afterSummary = Get-GSummary -Worksheet $worksheet -StartRow $DataStartRow -EndRow $lastRow -ColumnLetter $SortColumn
    Write-Log INFO "AfterSort $SortColumn Summary: 1=$($afterSummary.One) 0=$($afterSummary.Zero) Blank=$($afterSummary.Blank) Other=$($afterSummary.Other)"

    $probeAfter = Find-ProbeCode -Worksheet $worksheet -Code $ProbeCode -StartRow $DataStartRow -EndRow $lastRow
    if ($null -ne $probeAfter) {
        Write-Log INFO "ProbeAfter Code=$ProbeCode Row=$($probeAfter.Row) A=$($probeAfter.A) G=$($probeAfter.G)"
    }
    elseif (-not [string]::IsNullOrWhiteSpace($ProbeCode)) {
        Write-Log WARN "ProbeAfter Code=$ProbeCode not found."
    }

    if ($beforeSummary.One -ne $afterSummary.One -or
        $beforeSummary.Zero -ne $afterSummary.Zero -or
        $beforeSummary.Blank -ne $afterSummary.Blank -or
        $beforeSummary.Other -ne $afterSummary.Other) {
        throw "ソート前後でG列の件数構成が変化しました。データ保全のため異常終了します。"
    }

    if ($Save.IsPresent) {
        $workbook.Save()
        Write-Log INFO "Workbook saved by script."
    }
    else {
        Write-Log INFO "Workbook NOT saved by script. RK-10/Excel側で後続処理・保存してください。"
    }

    Write-Log INFO "Completed."
    exit 0
}
catch {
    Write-Log ERROR $_.Exception.Message
    exit 10
}
finally {
    Release-ComObject $keyRange
    Release-ComObject $sortRange
    Release-ComObject $sortFields
    Release-ComObject $sort
    Release-ComObject $worksheet
    Release-ComObject $workbook
    Release-ComObject $excel

    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()
}
