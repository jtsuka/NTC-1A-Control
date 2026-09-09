<#
.SYNOPSIS
    Phase 3 Rows.Insert / 条件付き書式 / A-C-D FormulaR1C1 継承実験。

.DESCRIPTION
    ProcessSheetRows Phase 3 実装前の安全確認用スクリプト。
    原本Workbookは絶対に変更せず、実験用コピーを作成してExcel COMで検証する。

    主な確認:
      1. FullCalculation後、末尾 #N/A クラスタを除外した最終アクティブ商品行を特定
      2. その直後へ Rows.Insert()
      3. F/G/H/I 条件付き書式が新規行へ継承されるか
      4. A/C/D列について既存 Find-NearestFormulaRow と同じ探索方式でテンプレートを選択
      5. FormulaR1C1を新規行へ複製できるか
      6. 元Workbookは非更新

    重要:
      - C列のアクティブ判定は .HasFormula ではなく .Text を使用する。
        #N/A行でもC列VLOOKUP数式自体は残っているため。
      - 実験対象は自動生成したコピーのみ。
      - -SaveTestCopy を指定しない限り、実験コピーも保存せず削除する。
      - 本スクリプトはPhase 3本番実装ではない。

.EXAMPLE
    powershell.exe -NoProfile -ExecutionPolicy Bypass `
      -File "C:\HPDB\Test_Phase3_RowsInsert_Inheritance_20260909.ps1" `
      -WorkbookPath "\\192.168.0.35\disk\企画室\倉庫\本社商品在庫日報.xlsx" `
      -SheetName "12"

.EXAMPLE
    # 結果をExcelで目視したい場合。保存先は OutputDirectory 内のテストコピーだけ。
    powershell.exe -NoProfile -ExecutionPolicy Bypass `
      -File "C:\HPDB\Test_Phase3_RowsInsert_Inheritance_20260909.ps1" `
      -WorkbookPath "\\192.168.0.35\disk\企画室\倉庫\本社商品在庫日報.xlsx" `
      -SheetName "12" `
      -SaveTestCopy
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$WorkbookPath,

    [string]$SheetName = "12",

    [string]$OutputDirectory = "C:\HPDB\Phase3_Test",

    [int]$MaxSearchRows = 200,

    [switch]$SaveTestCopy
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"

$excel = $null
$workbook = $null
$worksheet = $null
$testPath = $null
$logPath = $null
$csvPath = $null

function Write-TestLog {
    param(
        [string]$Level,
        [string]$Message
    )
    $line = "[{0}][{1}][P3-TEST] {2}" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss"), $Level, $Message
    Write-Host $line
    if (-not [string]::IsNullOrWhiteSpace($script:logPath)) {
        Add-Content -LiteralPath $script:logPath -Value $line -Encoding UTF8
    }
}

function Release-Com {
    param([object]$Object)
    if ($null -ne $Object) {
        try {
            [void][System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($Object)
        }
        catch {
        }
    }
}

function Get-LastRow {
    param([object]$Sheet)
    $usedRange = $null
    try {
        $usedRange = $Sheet.UsedRange
        return ([int]$usedRange.Row + [int]$usedRange.Rows.Count - 1)
    }
    finally {
        Release-Com $usedRange
    }
}

function Invoke-ExcelFullCalculation {
    param(
        [Parameter(Mandatory = $true)]$Excel,
        [int]$TimeoutSeconds = 60
    )
    Write-TestLog "INFO" "FullCalculation started."
    $Excel.Calculation = -4105  # xlCalculationAutomatic
    $Excel.CalculateFull()

    $started = Get-Date
    while ($Excel.CalculationState -ne 0) {
        if (((Get-Date) - $started).TotalSeconds -ge $TimeoutSeconds) {
            throw "Excel formula recalculation timed out after $TimeoutSeconds seconds."
        }
        Start-Sleep -Milliseconds 100
    }
    Write-TestLog "INFO" "FullCalculation completed."
}

function Get-CellTextSafe {
    param(
        [object]$Sheet,
        [int]$Row,
        [int]$Column
    )
    $cell = $null
    try {
        $cell = $Sheet.Cells.Item($Row, $Column)
        try {
            return [string]$cell.Text
        }
        catch {
            return [string]$cell.Value2
        }
    }
    finally {
        Release-Com $cell
    }
}

function Get-CellValue2Safe {
    param(
        [object]$Sheet,
        [int]$Row,
        [int]$Column
    )
    $cell = $null
    try {
        $cell = $Sheet.Cells.Item($Row, $Column)
        return $cell.Value2
    }
    finally {
        Release-Com $cell
    }
}

function Find-LastActiveProductRow {
    param(
        [Parameter(Mandatory = $true)]$Sheet,
        [Parameter(Mandatory = $true)][int]$PhysicalLastRow,
        [int]$MinRow = 3
    )

    for ($row = $PhysicalLastRow; $row -ge $MinRow; $row--) {
        $codeText = (Get-CellTextSafe -Sheet $Sheet -Row $row -Column 2).Trim()
        $nameText = (Get-CellTextSafe -Sheet $Sheet -Row $row -Column 3).Trim()

        # 重要: .HasFormula ではなくC列の再計算済み表示値で #N/A を判定。
        $isNA = ($nameText -match '^\s*#N/A\s*$')

        if (-not [string]::IsNullOrWhiteSpace($codeText) -and -not $isNA) {
            return $row
        }
    }

    return $null
}

# r14の探索プリミティブと同じ基本仕様。
function Find-NearestFormulaRow {
    param(
        [Parameter(Mandatory = $true)]$Worksheet,
        [Parameter(Mandatory = $true)][int]$Column,
        [Parameter(Mandatory = $true)][int]$StartRow,
        [int]$MinRow = 2,
        [int]$MaxSearchRows = 200
    )

    $searchRow = $StartRow - 1
    $checked = 0

    while ($searchRow -ge $MinRow -and $checked -lt $MaxSearchRows) {
        $cell = $null
        try {
            $cell = $Worksheet.Cells.Item($searchRow, $Column)
            if ([bool]$cell.HasFormula) {
                return $searchRow
            }
        }
        finally {
            Release-Com $cell
        }

        $searchRow--
        $checked++
    }

    return $null
}

function Get-FormulaSnapshot {
    param(
        [Parameter(Mandatory = $true)]$Sheet,
        [Parameter(Mandatory = $true)][int]$Row,
        [Parameter(Mandatory = $true)][int]$Column,
        [Parameter(Mandatory = $true)][string]$ColumnName
    )

    $cell = $null
    try {
        $cell = $Sheet.Cells.Item($Row, $Column)
        return [PSCustomObject]@{
            Column      = $ColumnName
            ColumnIndex = $Column
            Row         = $Row
            HasFormula  = [bool]$cell.HasFormula
            Formula     = [string]$cell.Formula
            FormulaR1C1 = [string]$cell.FormulaR1C1
            Text        = [string]$cell.Text
        }
    }
    finally {
        Release-Com $cell
    }
}

function Get-FormatConditionCount {
    param(
        [Parameter(Mandatory = $true)]$Sheet,
        [Parameter(Mandatory = $true)][int]$Row,
        [Parameter(Mandatory = $true)][int]$Column
    )

    $cell = $null
    $conditions = $null
    try {
        $cell = $Sheet.Cells.Item($Row, $Column)
        $conditions = $cell.FormatConditions
        return [int]$conditions.Count
    }
    finally {
        Release-Com $conditions
        Release-Com $cell
    }
}

function Copy-FormulaR1C1FromNearestTemplate {
    param(
        [Parameter(Mandatory = $true)]$Sheet,
        [Parameter(Mandatory = $true)][int]$DestinationRow,
        [Parameter(Mandatory = $true)][int]$Column,
        [Parameter(Mandatory = $true)][string]$ColumnName,
        [int]$MaxSearchRows = 200
    )

    $templateRow = Find-NearestFormulaRow `
        -Worksheet $Sheet `
        -Column $Column `
        -StartRow $DestinationRow `
        -MaxSearchRows $MaxSearchRows

    if ($null -eq $templateRow) {
        return [PSCustomObject]@{
            Column = $ColumnName
            Status = "NO_TEMPLATE"
            TemplateRow = $null
            Match = $false
            SourceFormulaR1C1 = ""
            DestinationFormulaR1C1 = ""
        }
    }

    $src = $null
    $dst = $null
    try {
        $src = $Sheet.Cells.Item($templateRow, $Column)
        $dst = $Sheet.Cells.Item($DestinationRow, $Column)

        $sourceFormula = [string]$src.FormulaR1C1
        $dst.FormulaR1C1 = $sourceFormula
        $destinationFormula = [string]$dst.FormulaR1C1

        return [PSCustomObject]@{
            Column = $ColumnName
            Status = "COPIED"
            TemplateRow = $templateRow
            Match = ($destinationFormula -eq $sourceFormula)
            SourceFormulaR1C1 = $sourceFormula
            DestinationFormulaR1C1 = $destinationFormula
        }
    }
    finally {
        Release-Com $dst
        Release-Com $src
    }
}

try {
    if (-not (Test-Path -LiteralPath $WorkbookPath -PathType Leaf)) {
        throw "Workbook not found: $WorkbookPath"
    }

    if ($SheetName -notmatch '^\d+$') {
        throw "SheetName must be numeric only: $SheetName"
    }

    if (-not (Test-Path -LiteralPath $OutputDirectory -PathType Container)) {
        New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
    }

    $timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $base = [System.IO.Path]::GetFileNameWithoutExtension($WorkbookPath)
    $ext = [System.IO.Path]::GetExtension($WorkbookPath)

    $script:logPath = Join-Path $OutputDirectory ("Phase3_RowsInsertTest_{0}_{1}.log" -f $SheetName, $timestamp)
    $csvPath = Join-Path $OutputDirectory ("Phase3_RowsInsertTest_{0}_{1}.csv" -f $SheetName, $timestamp)
    $testPath = Join-Path $OutputDirectory ("{0}_P3TEST_{1}{2}" -f $base, $timestamp, $ext)

    Write-TestLog "INFO" ("Original={0}" -f $WorkbookPath)
    Write-TestLog "INFO" ("Sheet={0}" -f $SheetName)
    Write-TestLog "INFO" ("TestCopy={0}" -f $testPath)
    Write-TestLog "INFO" ("SaveTestCopy={0}" -f [bool]$SaveTestCopy)
    Write-TestLog "INFO" ("MaxSearchRows={0}" -f $MaxSearchRows)

    # 原本は絶対に開いて更新しない。先にファイルコピーを作る。
    Copy-Item -LiteralPath $WorkbookPath -Destination $testPath -Force
    Write-TestLog "PASS" "Disposable test copy created. Original workbook will not be opened by Excel."

    $excel = New-Object -ComObject Excel.Application
    $excel.Visible = $false
    $excel.DisplayAlerts = $false
    $excel.ScreenUpdating = $false
    $excel.EnableEvents = $false

    $workbook = $excel.Workbooks.Open($testPath, 0, $false)

    try {
        $worksheet = $workbook.Worksheets.Item($SheetName)
    }
    catch {
        throw "Worksheet not found: $SheetName"
    }

    Invoke-ExcelFullCalculation -Excel $excel

    $physicalLastRow = Get-LastRow -Sheet $worksheet
    $lastActiveRow = Find-LastActiveProductRow -Sheet $worksheet -PhysicalLastRow $physicalLastRow

    if ($null -eq $lastActiveRow) {
        throw "No active product row found. Fail-closed."
    }

    $naClusterLength = $physicalLastRow - $lastActiveRow
    $insertRow = $lastActiveRow + 1

    Write-TestLog "INFO" ("PhysicalLastRow={0}" -f $physicalLastRow)
    Write-TestLog "INFO" ("LastActiveRow={0}" -f $lastActiveRow)
    Write-TestLog "INFO" ("TrailingNAClusterLength={0}" -f $naClusterLength)
    Write-TestLog "INFO" ("InsertRow={0}" -f $insertRow)

    if ($naClusterLength -ge $MaxSearchRows) {
        Write-TestLog "WARN" ("TrailingNAClusterLength={0} >= MaxSearchRows={1}. Existing restore search limit should be reviewed." -f $naClusterLength, $MaxSearchRows)
    }
    else {
        Write-TestLog "PASS" ("TrailingNAClusterLength={0} < MaxSearchRows={1}" -f $naClusterLength, $MaxSearchRows)
    }

    $results = New-Object System.Collections.ArrayList

    # 直前のアクティブ行情報
    foreach ($def in @(
        @{Name="A"; Col=1},
        @{Name="C"; Col=3},
        @{Name="D"; Col=4},
        @{Name="G"; Col=7},
        @{Name="I"; Col=9},
        @{Name="J"; Col=10},
        @{Name="K"; Col=11},
        @{Name="O"; Col=15}
    )) {
        $snap = Get-FormulaSnapshot -Sheet $worksheet -Row $lastActiveRow -Column $def.Col -ColumnName $def.Name
        Write-TestLog "INFO" ("TemplateBeforeInsert Col={0} Row={1} HasFormula={2} FormulaR1C1={3}" -f `
            $snap.Column, $snap.Row, $snap.HasFormula, $snap.FormulaR1C1)
    }

    # 挿入前のF/G/H/I条件付き書式数
    foreach ($def in @(
        @{Name="F"; Col=6},
        @{Name="G"; Col=7},
        @{Name="H"; Col=8},
        @{Name="I"; Col=9}
    )) {
        $count = Get-FormatConditionCount -Sheet $worksheet -Row $lastActiveRow -Column $def.Col
        Write-TestLog "INFO" ("CFBeforeInsert Col={0} Row={1} Count={2}" -f $def.Name, $lastActiveRow, $count)
    }

    # Excelデータ領域の「内側」、末尾NAクラスタ直前に1行挿入。
    $rowRange = $null
    try {
        $rowRange = $worksheet.Rows.Item($insertRow)
        [void]$rowRange.Insert(-4121)  # xlShiftDown
    }
    finally {
        Release-Com $rowRange
    }

    Write-TestLog "PASS" ("Rows.Insert completed at Row={0}" -f $insertRow)

    # Insert直後、Excel既定継承だけを確認。
    foreach ($def in @(
        @{Name="F"; Col=6},
        @{Name="G"; Col=7},
        @{Name="H"; Col=8},
        @{Name="I"; Col=9}
    )) {
        $count = Get-FormatConditionCount -Sheet $worksheet -Row $insertRow -Column $def.Col
        [void]$results.Add([PSCustomObject]@{
            Test = "ConditionalFormatting"
            Column = $def.Name
            InsertRow = $insertRow
            TemplateRow = $lastActiveRow
            Status = $(if ($count -gt 0) { "PASS" } else { "FAIL" })
            Detail = "FormatConditions.Count=$count"
        })
        Write-TestLog $(if ($count -gt 0) {"PASS"} else {"ERROR"}) `
            ("CFAfterInsert Col={0} Row={1} Count={2}" -f $def.Name, $insertRow, $count)
    }

    # A/C/DだけPhase3用の軽量Restore実験。
    # 探索プリミティブはr14と同じFind-NearestFormulaRow方式。
    foreach ($def in @(
        @{Name="A"; Col=1},
        @{Name="C"; Col=3},
        @{Name="D"; Col=4}
    )) {
        $copyResult = Copy-FormulaR1C1FromNearestTemplate `
            -Sheet $worksheet `
            -DestinationRow $insertRow `
            -Column $def.Col `
            -ColumnName $def.Name `
            -MaxSearchRows $MaxSearchRows

        $status = "FAIL"
        if ($copyResult.Status -eq "COPIED" -and $copyResult.Match) {
            $status = "PASS"
        }

        [void]$results.Add([PSCustomObject]@{
            Test = "FormulaR1C1Copy"
            Column = $def.Name
            InsertRow = $insertRow
            TemplateRow = $copyResult.TemplateRow
            Status = $status
            Detail = ("Source={0} Destination={1}" -f $copyResult.SourceFormulaR1C1, $copyResult.DestinationFormulaR1C1)
        })

        Write-TestLog $(if ($status -eq "PASS") {"PASS"} else {"ERROR"}) `
            ("FormulaCopy Col={0} TemplateRow={1} Status={2} Match={3}" -f `
            $def.Name, $copyResult.TemplateRow, $copyResult.Status, $copyResult.Match)
    }

    Invoke-ExcelFullCalculation -Excel $excel

    # 挿入後のC列表示を記録。B未設定なので値の正しさではなく「数式が存在し再計算できること」を見る。
    foreach ($def in @(
        @{Name="A"; Col=1},
        @{Name="C"; Col=3},
        @{Name="D"; Col=4}
    )) {
        $snap = Get-FormulaSnapshot -Sheet $worksheet -Row $insertRow -Column $def.Col -ColumnName $def.Name
        Write-TestLog "INFO" ("AfterFormulaCopy Col={0} Row={1} HasFormula={2} Text={3} FormulaR1C1={4}" -f `
            $snap.Column, $snap.Row, $snap.HasFormula, $snap.Text, $snap.FormulaR1C1)
    }

    # 新規行の上下を記録して、挿入位置がNAクラスタ直前であることを確認。
    $aboveName = Get-CellTextSafe -Sheet $worksheet -Row ($insertRow - 1) -Column 3
    $belowName = Get-CellTextSafe -Sheet $worksheet -Row ($insertRow + 1) -Column 3

    Write-TestLog "INFO" ("RowAbove C.Text={0}" -f $aboveName)
    Write-TestLog "INFO" ("RowBelow C.Text={0}" -f $belowName)

    $belowIsNA = ($belowName.Trim() -match '^\s*#N/A\s*$')
    [void]$results.Add([PSCustomObject]@{
        Test = "InsertPosition"
        Column = "-"
        InsertRow = $insertRow
        TemplateRow = $lastActiveRow
        Status = $(if ($naClusterLength -eq 0 -or $belowIsNA) { "PASS" } else { "WARN" })
        Detail = ("TrailingNAClusterLength={0}; RowBelowIsNA={1}" -f $naClusterLength, $belowIsNA)
    })

    $results | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding UTF8
    Write-TestLog "INFO" ("ResultCSV={0}" -f $csvPath)

    $failCount = @($results | Where-Object { $_.Status -eq "FAIL" }).Count
    $warnCount = @($results | Where-Object { $_.Status -eq "WARN" }).Count

    Write-TestLog "INFO" ("Summary Tests={0} Fail={1} Warn={2}" -f $results.Count, $failCount, $warnCount)

    if ($SaveTestCopy) {
        # 保存するのは自動生成したテストコピーだけ。原本には一切触れない。
        $workbook.Save()
        Write-TestLog "PASS" ("Modified TEST COPY saved for visual inspection: {0}" -f $testPath)
    }
    else {
        Write-TestLog "PASS" "Experiment completed. Test workbook will be closed WITHOUT SAVE."
    }

    if ($failCount -gt 0) {
        Write-TestLog "ERROR" "RESULT=FAIL. Do not proceed to Phase 3 implementation until failures are reviewed."
        $exitCode = 1
    }
    else {
        Write-TestLog "PASS" "RESULT=PASS for this sheet. Continue with visual review and other-sheet/site checks."
        $exitCode = 0
    }
}
catch {
    Write-TestLog "ERROR" $_.Exception.Message
    $exitCode = 2
}
finally {
    Release-Com $worksheet

    if ($null -ne $workbook) {
        try {
            # SaveTestCopy指定時は上でSave済み。Close自体では追加保存しない。
            $workbook.Close($false)
        }
        catch {
        }
    }

    if ($null -ne $excel) {
        try { $excel.Quit() } catch {}
    }

    Release-Com $workbook
    Release-Com $excel

    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()

    if (-not [bool]$SaveTestCopy -and -not [string]::IsNullOrWhiteSpace($testPath)) {
        try {
            if (Test-Path -LiteralPath $testPath -PathType Leaf) {
                Remove-Item -LiteralPath $testPath -Force
                Write-TestLog "INFO" "Disposable test copy deleted."
            }
        }
        catch {
            Write-TestLog "WARN" ("Failed to delete disposable test copy: {0}" -f $_.Exception.Message)
        }
    }
}

exit $exitCode
