<#
.SYNOPSIS
    Excelファイルのデータ範囲に罫線を引くRK-10用PowerShellスクリプト。

.DESCRIPTION
    RK-10やバッチファイルから Excel ファイル名（フルパス）を引数で渡して実行します。
    Excel COM APIを使用し、画面操作・キー操作を行わずに以下を実施します。

        ・指定Excelファイルを開く
        ・対象シートのデータ範囲（UsedRange）を取得
        ・データ範囲に罫線を設定
        ・必要に応じて列幅自動調整
        ・保存して閉じる

.PARAMETER TargetFilePath
    処理対象のExcelファイルのフルパスを指定します。

.PARAMETER SheetName
    対象シート名を指定します。省略時は全ワークシートを対象にします。

.PARAMETER Visible
    指定するとExcel画面を表示して処理します。通常のRK-10自動実行では指定しないことを推奨します。

.PARAMETER AutoFit
    指定すると、罫線設定後にUsedRangeの列幅を自動調整します。

.EXITCODE
    0 : OK    正常終了
    1 : NG    対象ファイル/対象シートが存在しない等
    2 : ERROR Excel COM操作中の例外

.EXAMPLE
    powershell.exe -ExecutionPolicy Bypass -File "C:\RPA\Scripts\Excel_AddBordersToUsedRange.ps1" -TargetFilePath "\\192.168.0.35\disk\RPA-DATA\営業\大営\日計\大営日計(20260522).xlsx"

.EXAMPLE
    powershell.exe -ExecutionPolicy Bypass -File "C:\RPA\Scripts\Excel_AddBordersToUsedRange.ps1" -TargetFilePath "C:\Temp\test.xlsx" -SheetName "Sheet1" -AutoFit

.NOTES
    作成目的:
        RK-10のExcelキー操作を減らし、Excel内部処理をPowerShell + Excel COM APIで安定化するため。

    注意事項:
        ・ExcelがインストールされているPCで実行してください。
        ・対象ファイルを他ユーザーやExcelが開いていると保存できない場合があります。
        ・UsedRangeに過去の書式残りが含まれている場合、想定より広い範囲に罫線が引かれることがあります。
#>

param(
    [Parameter(Mandatory = $true)]
    [string]$TargetFilePath,

    [string]$SheetName = "",

    [switch]$Visible,

    [switch]$AutoFit
)

function Release-ComObject {
    param([object]$ComObject)
    if ($null -ne $ComObject) {
        try { [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($ComObject) } catch {}
    }
}

$excel = $null
$workbook = $null

try {
    Write-Output "INFO: Excel罫線設定 開始"
    Write-Output "INFO: 対象ファイル = $TargetFilePath"

    if (-not (Test-Path -LiteralPath $TargetFilePath)) {
        Write-Output "NG: 対象ファイルが存在しません。"
        exit 1
    }

    $excel = New-Object -ComObject Excel.Application
    $excel.Visible = [bool]$Visible
    $excel.DisplayAlerts = $false
    $excel.EnableEvents = $false

    $workbook = $excel.Workbooks.Open($TargetFilePath)

    $targetSheets = @()
    if ([string]::IsNullOrWhiteSpace($SheetName)) {
        foreach ($ws in $workbook.Worksheets) { $targetSheets += $ws }
        Write-Output "INFO: 対象シート = 全シート"
    } else {
        $sheetFound = $false
        foreach ($ws in $workbook.Worksheets) {
            if ($ws.Name -eq $SheetName) {
                $targetSheets += $ws
                $sheetFound = $true
                break
            }
        }
        if (-not $sheetFound) {
            Write-Output "NG: 指定シートが存在しません。SheetName=$SheetName"
            $workbook.Close($false)
            $excel.Quit()
            exit 1
        }
        Write-Output "INFO: 対象シート = $SheetName"
    }

    $xlContinuous = 1
    $xlThin = 2

    foreach ($sheet in $targetSheets) {
        Write-Output "INFO: 処理中シート = $($sheet.Name)"
        $range = $sheet.UsedRange

        if ($null -eq $range) {
            Write-Output "INFO: UsedRange が取得できませんでした。スキップします。"
            continue
        }

        $rows = $range.Rows.Count
        $cols = $range.Columns.Count
        Write-Output "INFO: UsedRange 行数=$rows 列数=$cols"

        if ($rows -le 1 -and $cols -le 1 -and [string]::IsNullOrWhiteSpace([string]$range.Value2)) {
            Write-Output "INFO: 空シートと判断しスキップします。"
            continue
        }

        $range.Borders.LineStyle = $xlContinuous
        $range.Borders.Weight = $xlThin
        Write-Output "INFO: 罫線設定 完了"

        if ($AutoFit) {
            try {
                $range.Columns.AutoFit() | Out-Null
                Write-Output "INFO: 列幅自動調整 完了"
            } catch {
                Write-Output "INFO: 列幅自動調整は実行できませんでした。$($_.Exception.Message)"
            }
        }
    }

    $workbook.Save()
    Write-Output "OK: Excel罫線設定 完了"
    exit 0
}
catch {
    Write-Output "ERROR: Excel罫線設定中にエラーが発生しました。"
    Write-Output "ERROR: $($_.Exception.Message)"
    exit 2
}
finally {
    if ($null -ne $workbook) { try { $workbook.Close($true) | Out-Null } catch {} }
    if ($null -ne $excel) { try { $excel.Quit() | Out-Null } catch {} }
    Release-ComObject $workbook
    Release-ComObject $excel
    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()
}
