<#
===============================================================================
スクリプト名:
  Check-ExcelBorders-Simple.ps1

概要:
  Excelファイルを非表示で開き、データが入っている各セルに
  四辺すべての罫線が設定されているかを確認するスクリプトです。

用途:
  RK-10 のコマンドライン実行で使いやすいように、
  標準出力は OK / NG / ERROR のみ返す簡易版です。
  ※ 標準出力は改行なしで返します。

判定内容:
  ・使用範囲のうち、値または数式が入っているセルを対象にチェック
  ・各セルについて、左・上・下・右 の四辺すべてに罫線があるかを確認
  ・四辺のどれか1つでも欠けていれば NG

標準出力:
  OK
  NG
  ERROR

終了コード:
  0 = OK
  1 = NG
  9 = ERROR

補足:
  ・詳細を残したい場合は -ResultFile を指定すると JSON を保存します
  ・STDOUT には改行なしで OK / NG / ERROR のみ出力します
===============================================================================
#>

param(
    [Parameter(Mandatory = $true)]
    [string]$FilePath,

    [string]$SheetName = "",

    [string]$ResultFile = ""
)

$ErrorActionPreference = "Stop"

function Save-DetailResult {
    param(
        [string]$Status,
        [string]$Message,
        [array]$Details = @()
    )

    if ([string]::IsNullOrWhiteSpace($ResultFile)) {
        return
    }

    $result = [PSCustomObject]@{
        status  = $Status
        message = $Message
        file    = $FilePath
        sheet   = $SheetName
        details = $Details
        time    = (Get-Date).ToString("yyyy-MM-dd HH:mm:ss")
    }

    $result | ConvertTo-Json -Depth 5 | Set-Content -Path $ResultFile -Encoding UTF8
}

$excel = $null
$workbook = $null

# Excel罫線定数
$xlEdgeLeft     = 7
$xlEdgeTop      = 8
$xlEdgeBottom   = 9
$xlEdgeRight    = 10
$xlLineStyleNone = -4142

try {
    if (-not (Test-Path -LiteralPath $FilePath)) {
        Save-DetailResult -Status "ERROR" -Message "対象ファイルが存在しません。" -Details @()
        [Console]::Write("ERROR")
        exit 9
    }

    $excel = New-Object -ComObject Excel.Application
    $excel.Visible = $false
    $excel.DisplayAlerts = $false
    $excel.AskToUpdateLinks = $false

    $workbook = $excel.Workbooks.Open($FilePath, 0, $true)

    if ([string]::IsNullOrWhiteSpace($SheetName)) {
        $worksheet = $workbook.Worksheets.Item(1)
        $SheetName = $worksheet.Name
    }
    else {
        $worksheet = $workbook.Worksheets.Item($SheetName)
    }

    $usedRange = $worksheet.UsedRange
    $rowCount  = [int]$usedRange.Rows.Count
    $colCount  = [int]$usedRange.Columns.Count
    $startRow  = [int]$usedRange.Row
    $startCol  = [int]$usedRange.Column
    $endRow    = $startRow + $rowCount - 1
    $endCol    = $startCol + $colCount - 1

    $details = New-Object System.Collections.Generic.List[string]
    $dataCellCount = 0
    $ngCellCount = 0

    for ($r = $startRow; $r -le $endRow; $r++) {
        for ($c = $startCol; $c -le $endCol; $c++) {
            $cell = $worksheet.Cells.Item($r, $c)

            $valueText = ""
            try { $valueText = [string]$cell.Text } catch {}

            $hasFormula = $false
            try { $hasFormula = [bool]$cell.HasFormula } catch {}

            # 値または数式があるセルだけ判定対象
            if (-not [string]::IsNullOrWhiteSpace($valueText) -or $hasFormula) {
                $dataCellCount++

                $leftStyle   = $cell.Borders.Item($xlEdgeLeft).LineStyle
                $topStyle    = $cell.Borders.Item($xlEdgeTop).LineStyle
                $bottomStyle = $cell.Borders.Item($xlEdgeBottom).LineStyle
                $rightStyle  = $cell.Borders.Item($xlEdgeRight).LineStyle

                $hasLeft   = ($leftStyle   -ne $xlLineStyleNone)
                $hasTop    = ($topStyle    -ne $xlLineStyleNone)
                $hasBottom = ($bottomStyle -ne $xlLineStyleNone)
                $hasRight  = ($rightStyle  -ne $xlLineStyleNone)

                $hasAllBorders = $hasLeft -and $hasTop -and $hasBottom -and $hasRight

                if (-not $hasAllBorders) {
                    $ngCellCount++

                    if ($details.Count -lt 30) {
                        $addr = $cell.Address($false, $false)
                        $missing = @()
                        if (-not $hasLeft)   { $missing += "左" }
                        if (-not $hasTop)    { $missing += "上" }
                        if (-not $hasBottom) { $missing += "下" }
                        if (-not $hasRight)  { $missing += "右" }

                        $details.Add("セル $addr は四辺罫線不足。欠け: $($missing -join ',') / 値='$valueText'")
                    }
                }
            }
        }
    }

    if ($dataCellCount -eq 0) {
        Save-DetailResult -Status "ERROR" -Message "データセルが見つかりませんでした。" -Details @()
        [Console]::Write("ERROR")
        exit 9
    }

    if ($ngCellCount -gt 0) {
        Save-DetailResult -Status "NG" -Message "四辺罫線が不足しているデータセルがあります。対象セル数=$dataCellCount / NG=$ngCellCount" -Details $details
        [Console]::Write("NG")
        exit 1
    }
    else {
        Save-DetailResult -Status "OK" -Message "すべてのデータセルに四辺の罫線があります。対象セル数=$dataCellCount" -Details @()
        [Console]::Write("OK")
        exit 0
    }
}
catch {
    Save-DetailResult -Status "ERROR" -Message $_.Exception.Message -Details @()
    [Console]::Write("ERROR")
    exit 9
}
finally {
    if ($workbook -ne $null) {
        try { $workbook.Close($false) } catch {}
        [System.Runtime.InteropServices.Marshal]::ReleaseComObject($workbook) | Out-Null
    }

    if ($excel -ne $null) {
        try { $excel.Quit() } catch {}
        [System.Runtime.InteropServices.Marshal]::ReleaseComObject($excel) | Out-Null
    }

    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()
    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()
}