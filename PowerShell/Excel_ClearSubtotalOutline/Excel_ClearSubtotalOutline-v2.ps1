<#
  Excel_ClearSubtotalOutline-v2.ps1

  Remove Subtotal and Outline (grouping) from an Excel file and save.
  STDOUT : OK / NG / ERROR only (no newline)
  EXITCODE: 0=OK  1=NG  9=ERROR

  v2 changes from v1:
    - Parameter renamed: -TargetFilePath -> -FilePath  (align with Check scripts)
    - STDOUT changed to OK/NG/ERROR only               (align with Check scripts)
    - Exit code ERROR changed: 2 -> 9                  (align with Check scripts)
    - Process order: RemoveSubtotal -> ClearOutline -> ShowLevels(補助)

  USAGE:
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "C:\HPDB\Excel_ClearSubtotalOutline-v2.ps1" -FilePath "C:\Convert	estfile.xlsx"
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "C:\HPDB\Excel_ClearSubtotalOutline-v2.ps1" -FilePath "C:\Convert	estfile.xlsx" -SheetName "Sheet1"
#>

param(
    [Parameter(Mandatory = $true)]
    [string]$FilePath,

    [string]$SheetName = "",

    [switch]$Visible
)

$ErrorActionPreference = "Stop"

function Release-ComObject {
    param([object]$ComObject)
    if ($null -ne $ComObject) {
        try {
            [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($ComObject)
        }
        catch {}
    }
}

$excel        = $null
$workbook     = $null
$targetSheets = @()

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
        try { [void]$sheet.Cells.RemoveSubtotal() }  catch {}
        try { [void]$sheet.Cells.ClearOutline() }    catch {}
        try { [void]$sheet.Outline.ShowLevels(8, 8) } catch {}
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
        try { $workbook.Close($true) | Out-Null } catch {}
    }
    if ($null -ne $excel) {
        try { $excel.Quit() | Out-Null } catch {}
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
