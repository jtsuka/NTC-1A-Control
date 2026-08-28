<#
.SYNOPSIS
    Excelファイルの小計（集計）解除、アウトライン/グループ化解除を行うRK-10用PowerShellスクリプト。

.DESCRIPTION
    RK-10やバッチファイルから Excel ファイル名（フルパス）を引数で渡して実行します。
    Excel COM APIを使用し、画面操作・キー操作を行わずに以下を実施します。

        ・指定Excelファイルを開く
        ・対象シートの小計（Subtotal）解除を試行
        ・アウトライン表示レベルを通常表示へ戻す
        ・行/列のグループ化解除を可能な範囲で試行
        ・保存して閉じる

.PARAMETER TargetFilePath
    処理対象のExcelファイルのフルパスを指定します。

.PARAMETER SheetName
    対象シート名を指定します。省略時は全ワークシートを対象にします。

.PARAMETER Visible
    指定するとExcel画面を表示して処理します。通常のRK-10自動実行では指定しないことを推奨します。

.EXITCODE
    0 : OK    正常終了
    1 : NG    対象ファイル/対象シートが存在しない等
    2 : ERROR Excel COM操作中の例外

.EXAMPLE
    powershell.exe -ExecutionPolicy Bypass -File "C:\RPA\Scripts\Excel_ClearSubtotalOutline.ps1" -TargetFilePath "\\192.168.0.35\disk\RPA-DATA\営業\大営\日計\大営日計(20260522).xlsx"

.EXAMPLE
    powershell.exe -ExecutionPolicy Bypass -File "C:\RPA\Scripts\Excel_ClearSubtotalOutline.ps1" -TargetFilePath "C:\Temp\test.xlsx" -SheetName "Sheet1"

.NOTES
    作成目的:
        RK-10のExcelキー操作を減らし、Excel内部処理をPowerShell + Excel COM APIで安定化するため。

    注意事項:
        ・ExcelがインストールされているPCで実行してください。
        ・対象ファイルを他ユーザーやExcelが開いていると保存できない場合があります。
        ・処理前に必要に応じてバックアップを取得してください。
#>

param(
    [Parameter(Mandatory = $true)]
    [string]$TargetFilePath,

    [string]$SheetName = "",

    [switch]$Visible
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
    Write-Output "INFO: Excel集計/アウトライン解除 開始"
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

    foreach ($sheet in $targetSheets) {
        Write-Output "INFO: 処理中シート = $($sheet.Name)"

        try {
            $sheet.Cells.RemoveSubtotal()
            Write-Output "INFO: RemoveSubtotal 実行"
        } catch {
            Write-Output "INFO: RemoveSubtotal は実行できませんでした。$($_.Exception.Message)"
        }

        try {
            $sheet.Outline.ShowLevels(1, 1)
            Write-Output "INFO: Outline.ShowLevels(1,1) 実行"
        } catch {
            Write-Output "INFO: Outline.ShowLevels は実行できませんでした。$($_.Exception.Message)"
        }

        try {
            $sheet.Cells.ClearOutline()
            Write-Output "INFO: ClearOutline 実行"
        } catch {
            Write-Output "INFO: ClearOutline は実行できませんでした。$($_.Exception.Message)"
        }
    }

    $workbook.Save()
    Write-Output "OK: Excel集計/アウトライン解除 完了"
    exit 0
}
catch {
    Write-Output "ERROR: Excel集計/アウトライン解除中にエラーが発生しました。"
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
