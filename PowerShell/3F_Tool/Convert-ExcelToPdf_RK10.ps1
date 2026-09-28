<#
.SYNOPSIS
    ExcelファイルをPDFへ変換します（RK-10 GUIキー操作の代替テスト用）。

.DESCRIPTION
    Microsoft Excel の COM オートメーションを使用し、
    キーボード操作や「PDFまたはXPSで発行」ダイアログを使わずに
    ExcelファイルをPDFへ変換します。

    PDFは一度ローカルのTEMPフォルダへ生成し、
    完成後に指定された保存先へコピーします。
    ネットワークフォルダへ直接ExcelからPDF出力しないため、
    PDF変換エラーとネットワーク保存エラーを切り分けやすくしています。

    PowerShell 5.1 / Microsoft Excel インストール済みPCを想定しています。

    【終了コード】
      0  : 正常終了
      10 : 入力Excelが存在しない
      11 : PDF保存先フォルダが存在しない
      12 : 出力PDFが既に存在する（-Force未指定）
      20 : Excel/PDF変換処理エラー
      21 : PDF生成確認エラー
      22 : 保存先へのコピー/確認エラー

    【基本的な使い方】

    1) アクティブシートだけPDF化
       powershell.exe -NoProfile -ExecutionPolicy Bypass -File "C:\HPDB\Convert-ExcelToPdf_RK10_Test.ps1" `
         -ExcelPath "\\192.168.0.34\kintai\20260928残業申請.xlsx" `
         -PdfPath "\\192.168.0.34\kintai\PDF\20260928残業申請.pdf"

    2) ブック全体をPDF化
       powershell.exe -NoProfile -ExecutionPolicy Bypass -File "C:\HPDB\Convert-ExcelToPdf_RK10_Test.ps1" `
         -ExcelPath "\\192.168.0.34\kintai\20260928残業申請.xlsx" `
         -PdfPath "\\192.168.0.34\kintai\PDF\20260928残業申請.pdf" `
         -Scope Workbook

    3) シート名を指定してPDF化
       powershell.exe -NoProfile -ExecutionPolicy Bypass -File "C:\HPDB\Convert-ExcelToPdf_RK10_Test.ps1" `
         -ExcelPath "\\192.168.0.34\kintai\20260928残業申請.xlsx" `
         -PdfPath "\\192.168.0.34\kintai\PDF\20260928残業申請.pdf" `
         -SheetName "前営業日"

    4) 同じPDF名が既に存在しても上書きする
       powershell.exe -NoProfile -ExecutionPolicy Bypass -File "C:\HPDB\Convert-ExcelToPdf_RK10_Test.ps1" `
         -ExcelPath "\\192.168.0.34\kintai\20260928残業申請.xlsx" `
         -PdfPath "\\192.168.0.34\kintai\PDF\20260928残業申請.pdf" `
         -Force

    【最初の単独テスト推奨】
      ・まずRK-10からではなく、PowerShell 5.1のコンソールから実行してください。
      ・元Excelと、現在のRK-10で作成したPDFを残して比較してください。
      ・ページ数、印刷範囲、縮尺、改ページ、向き、余白を確認してください。
      ・成功時は最後に「RESULT=SUCCESS」「EXITCODE=0」と表示されます。

    【RK-10へ組み込む場合】
      ・PowerShell実行後の終了コードを確認してください。
      ・終了コードが0以外ならシナリオを終了させる構成を推奨します。
      ・PDF化を本スクリプトに任せる場合、RK-10側のExcelを開く処理、
        Altキー等によるPDF発行操作、PDF保存ダイアログ操作、
        Excelを閉じる処理は原則不要です。

.PARAMETER ExcelPath
    変換元Excelファイルのフルパス。

.PARAMETER PdfPath
    出力PDFファイルのフルパス。

.PARAMETER Scope
    PDF化する範囲。
      ActiveSheet : ブックを開いた時点のアクティブシート（既定）
      Workbook    : ブック全体

.PARAMETER SheetName
    指定したシートだけをPDF化します。
    指定時は -Scope よりこちらを優先します。

.PARAMETER Force
    出力PDFが既に存在する場合に上書きします。

.NOTES
    File    : Convert-ExcelToPdf_RK10_Test.ps1
    Purpose : RK-10「残業申請のExcelをPDFにしてメールする」
              GUIキー操作によるPDF化の代替検証
    Target  : Windows PowerShell 5.1
    Excel   : Microsoft Excel Desktop版が必要
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$ExcelPath,

    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$PdfPath,

    [ValidateSet("ActiveSheet", "Workbook")]
    [string]$Scope = "ActiveSheet",

    [string]$SheetName,

    [switch]$Force
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"

# 終了コード
$EXIT_OK                    = 0
$EXIT_INPUT_NOT_FOUND       = 10
$EXIT_OUTPUT_DIR_NOT_FOUND  = 11
$EXIT_OUTPUT_EXISTS         = 12
$EXIT_CONVERT_ERROR         = 20
$EXIT_PDF_VERIFY_ERROR      = 21
$EXIT_COPY_VERIFY_ERROR     = 22

$exitCode = $EXIT_OK

$excel = $null
$book  = $null
$sheet = $null

$tempPdf = Join-Path $env:TEMP (
    "RK10_ExcelToPdf_{0}.pdf" -f ([Guid]::NewGuid().ToString("N"))
)

function Write-Step {
    param([string]$Message)
    Write-Host ("[{0}] {1}" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss"), $Message)
}

try {
    Write-Host "============================================================"
    Write-Host " RK-10 Excel -> PDF PowerShell Test"
    Write-Host "============================================================"
    Write-Host ("ExcelPath : {0}" -f $ExcelPath)
    Write-Host ("PdfPath   : {0}" -f $PdfPath)
    Write-Host ("Scope     : {0}" -f $Scope)
    Write-Host ("SheetName : {0}" -f $SheetName)
    Write-Host ("Force     : {0}" -f $Force.IsPresent)
    Write-Host ""

    # ------------------------------------------------------------
    # 1. 入力確認
    # ------------------------------------------------------------
    Write-Step "入力Excelを確認します。"

    if (-not (Test-Path -LiteralPath $ExcelPath -PathType Leaf)) {
        $exitCode = $EXIT_INPUT_NOT_FOUND
        throw "入力Excelファイルが存在しません: $ExcelPath"
    }

    $pdfFolder = Split-Path -Parent $PdfPath

    if ([string]::IsNullOrWhiteSpace($pdfFolder)) {
        $exitCode = $EXIT_OUTPUT_DIR_NOT_FOUND
        throw "PdfPathから保存先フォルダを取得できません: $PdfPath"
    }

    if (-not (Test-Path -LiteralPath $pdfFolder -PathType Container)) {
        $exitCode = $EXIT_OUTPUT_DIR_NOT_FOUND
        throw "PDF保存先フォルダが存在しません: $pdfFolder"
    }

    if ((Test-Path -LiteralPath $PdfPath -PathType Leaf) -and (-not $Force.IsPresent)) {
        $exitCode = $EXIT_OUTPUT_EXISTS
        throw "出力PDFが既に存在します。上書きする場合は -Force を指定してください: $PdfPath"
    }

    # ------------------------------------------------------------
    # 2. Excel起動
    # ------------------------------------------------------------
    Write-Step "Excel COMを起動します。"

    try {
        $excel = New-Object -ComObject Excel.Application
        $excel.Visible = $false
        $excel.DisplayAlerts = $false
        $excel.AskToUpdateLinks = $false

        # Workbooks.Open(FileName, UpdateLinks, ReadOnly)
        $book = $excel.Workbooks.Open($ExcelPath, 0, $true)
    }
    catch {
        if ($exitCode -eq $EXIT_OK) {
            $exitCode = $EXIT_CONVERT_ERROR
        }
        throw
    }

    # ------------------------------------------------------------
    # 3. PDF変換
    # ------------------------------------------------------------
    Write-Step "PDF変換を開始します。"

    try {
        if (-not [string]::IsNullOrWhiteSpace($SheetName)) {
            Write-Step ("指定シートをPDF化します: {0}" -f $SheetName)

            try {
                $sheet = $book.Worksheets.Item($SheetName)
            }
            catch {
                throw "指定されたシートが見つかりません: $SheetName"
            }

            # 0 = xlTypePDF
            # 0 = xlQualityStandard
            # IncludeDocProperties = true
            # IgnorePrintAreas     = false
            $sheet.ExportAsFixedFormat(
                0,
                $tempPdf,
                0,
                $true,
                $false
            )
        }
        elseif ($Scope -eq "Workbook") {
            Write-Step "ブック全体をPDF化します。"

            $book.ExportAsFixedFormat(
                0,
                $tempPdf,
                0,
                $true,
                $false
            )
        }
        else {
            Write-Step "アクティブシートをPDF化します。"

            $sheet = $book.ActiveSheet

            $sheet.ExportAsFixedFormat(
                0,
                $tempPdf,
                0,
                $true,
                $false
            )
        }
    }
    catch {
        if ($exitCode -eq $EXIT_OK) {
            $exitCode = $EXIT_CONVERT_ERROR
        }
        throw
    }

    # ------------------------------------------------------------
    # 4. TEMP側PDF確認
    # ------------------------------------------------------------
    Write-Step "TEMP側のPDF生成結果を確認します。"

    if (-not (Test-Path -LiteralPath $tempPdf -PathType Leaf)) {
        $exitCode = $EXIT_PDF_VERIFY_ERROR
        throw "TEMP側にPDFが生成されませんでした: $tempPdf"
    }

    $tempInfo = Get-Item -LiteralPath $tempPdf

    if ($tempInfo.Length -le 0) {
        $exitCode = $EXIT_PDF_VERIFY_ERROR
        throw "TEMP側に生成されたPDFが0バイトです: $tempPdf"
    }

    Write-Step ("TEMP PDF生成成功: {0:N0} bytes" -f $tempInfo.Length)

    # ------------------------------------------------------------
    # 5. 保存先へコピー
    # ------------------------------------------------------------
    Write-Step "PDFを最終保存先へコピーします。"

    try {
        Copy-Item `
            -LiteralPath $tempPdf `
            -Destination $PdfPath `
            -Force:$Force.IsPresent

        if (-not (Test-Path -LiteralPath $PdfPath -PathType Leaf)) {
            $exitCode = $EXIT_COPY_VERIFY_ERROR
            throw "コピー後のPDFを保存先で確認できません: $PdfPath"
        }

        $pdfInfo = Get-Item -LiteralPath $PdfPath

        if ($pdfInfo.Length -le 0) {
            $exitCode = $EXIT_COPY_VERIFY_ERROR
            throw "保存先PDFが0バイトです: $PdfPath"
        }

        # TEMPと保存先のサイズが一致するか確認
        if ($pdfInfo.Length -ne $tempInfo.Length) {
            $exitCode = $EXIT_COPY_VERIFY_ERROR
            throw ("PDFサイズが一致しません。TEMP={0} bytes / 保存先={1} bytes" -f `
                $tempInfo.Length, $pdfInfo.Length)
        }
    }
    catch {
        if ($exitCode -eq $EXIT_OK) {
            $exitCode = $EXIT_COPY_VERIFY_ERROR
        }
        throw
    }

    Write-Step ("保存成功: {0}" -f $PdfPath)
    Write-Step ("PDF Size : {0:N0} bytes" -f $pdfInfo.Length)

    $exitCode = $EXIT_OK
}
catch {
    if ($exitCode -eq $EXIT_OK) {
        $exitCode = $EXIT_CONVERT_ERROR
    }

    Write-Host ""
    Write-Error ("PDF変換失敗: {0}" -f $_.Exception.Message)
}
finally {
    # ------------------------------------------------------------
    # 6. Excel COM後始末
    # ------------------------------------------------------------
    Write-Step "Excel COMを終了します。"

    if ($book -ne $null) {
        try {
            $book.Close($false)
        }
        catch {
            Write-Warning ("Workbook.Closeでエラー: {0}" -f $_.Exception.Message)
        }
    }

    if ($excel -ne $null) {
        try {
            $excel.Quit()
        }
        catch {
            Write-Warning ("Excel.Quitでエラー: {0}" -f $_.Exception.Message)
        }
    }

    # COMオブジェクト解放
    if ($sheet -ne $null) {
        try {
            [void][System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($sheet)
        }
        catch {}
        $sheet = $null
    }

    if ($book -ne $null) {
        try {
            [void][System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($book)
        }
        catch {}
        $book = $null
    }

    if ($excel -ne $null) {
        try {
            [void][System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($excel)
        }
        catch {}
        $excel = $null
    }

    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()
    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()

    # TEMP PDF削除
    if (Test-Path -LiteralPath $tempPdf -PathType Leaf) {
        try {
            Remove-Item -LiteralPath $tempPdf -Force
        }
        catch {
            Write-Warning ("TEMP PDFを削除できませんでした: {0}" -f $tempPdf)
        }
    }
}

Write-Host ""
Write-Host "============================================================"

if ($exitCode -eq $EXIT_OK) {
    Write-Host "RESULT=SUCCESS"
}
else {
    Write-Host "RESULT=ERROR"
}

Write-Host ("EXITCODE={0}" -f $exitCode)
Write-Host "============================================================"

exit $exitCode
