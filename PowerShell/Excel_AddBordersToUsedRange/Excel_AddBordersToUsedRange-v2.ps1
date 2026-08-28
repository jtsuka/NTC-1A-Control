<#
  Excel_AddBordersToUsedRange-v2.ps1
  Apply thin borders to UsedRange of an Excel file and save.
  STDOUT : OK / NG / ERROR only (no newline, no extra output)
  EXITCODE: 0=OK  1=NG  9=ERROR

  v2 changes from v1:
    - Parameter renamed: -TargetFilePath -> -FilePath
    - STDOUT changed to OK/NG/ERROR only
    - Exit code ERROR changed: 2 -> 9
    - HeaderYellow default OFF; use -HeaderYellow to enable
    - [void] added to all COM calls to suppress extra output

  USAGE:
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "C:\HPDB\Excel_AddBordersToUsedRange-v2.ps1" -FilePath "C:\Convert\testfile.xlsx"
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "C:\HPDB\Excel_AddBordersToUsedRange-v2.ps1" -FilePath "C:\Convert\testfile.xlsx" -HeaderYellow
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "C:\HPDB\Excel_AddBordersToUsedRange-v2.ps1" -FilePath "C:\Convert\testfile.xlsx" -AutoFit
#>

param(
    [Parameter(Mandatory = $true)]
    [string]$FilePath,
    [string]$SheetName = "",
    [switch]$Visible,
    [switch]$AutoFit,
    [switch]$HeaderYellow
)

$ErrorActionPreference = "Stop"

function Release-ComObject {
    param([object]$ComObject)
    if ($null -ne $ComObject) {
        try { [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($ComObject) }
        catch {}
    }
}

$excel        = $null
$workbook     = $null
$targetSheets = @()

$xlContinuous       = 1
$xlThin             = 2
$xlColorIndexYellow = 6

try {
    if (-not (Test-Path -LiteralPath $FilePath)) {
        [Console]::Write("NG")
        exit 1
    }

    $fullPath = (Resolve-Path -LiteralPath $FilePath).Path

    $excel               = New-Object -ComObject Excel.Application
    $excel.Visible       = [bool]$Visible
    $excel.DisplayAlerts = $false
    $excel.EnableEvents  = $false

    $workbook = $excel.Workbooks.Open($fullPath)

    if ([string]::IsNullOrWhiteSpace($SheetName)) {
        foreach ($ws in $workbook.Worksheets) {
            $targetSheets += $ws
        }
    }
    else {
        $sheetFound = $false
        foreach ($ws in $workbook.Worksheets) {
            if ($ws.Name -eq $SheetName) {
                $targetSheets += $ws
                $sheetFound   = $true
                break
            }
        }
        if (-not $sheetFound) {
            [Console]::Write("NG")
            exit 1
        }
    }

    foreach ($sheet in $targetSheets) {
        $range = $sheet.UsedRange

        if ($null -eq $range) {
            Release-ComObject $range
            continue
        }

        $rows = $range.Rows.Count
        $cols = $range.Columns.Count

        if ($rows -le 1 -and $cols -le 1 -and [string]::IsNullOrWhiteSpace([string]$range.Value2)) {
            Release-ComObject $range
            continue
        }

        $range.Borders.LineStyle = $xlContinuous
        $range.Borders.Weight    = $xlThin

        if ($HeaderYellow) {
            $headerRange = $range.Rows.Item(1)
            $headerRange.Interior.ColorIndex = $xlColorIndexYellow
            Release-ComObject $headerRange
        }

        if ($AutoFit) {
            try { [void]$range.Columns.AutoFit() }
            catch {}
        }

        Release-ComObject $range
    }

    [void]$workbook.Save()
    [Console]::Write("OK")
    exit 0
}
catch {
    [Console]::Write("ERROR")
    exit 9
}
finally {
    if ($null -ne $workbook) {
        try { [void]$workbook.Close($true) } catch {}
    }
    if ($null -ne $excel) {
        try { [void]$excel.Quit() } catch {}
    }
    foreach ($sheet in $targetSheets) {
        Release-ComObject $sheet
    }
    Release-ComObject $workbook
    Release-ComObject $excel

    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()
    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()
}
