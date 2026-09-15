<#
.SYNOPSIS
    船舶値付用紙 - Apeos API検証用 単発テスト印刷スクリプト

.DESCRIPTION
    Print-ValueTag.ps1 本体やExcelを使わず、.NET PrintDocumentで1ページだけ
    テスト印刷を行う。目的は次の2点のみ。

      1. PrintDocument.DocumentName に設定した一意な識別子が、
         Apeos側の NetInFilename にどう記録されるかを確認する材料を作る
      2. Print() 呼び出し直前・直後の正確なローカル時刻を記録し、
         Check-ApeosPrintJob.ps1 側の API Created / Completed との
         比較材料（項目②）とする

    このスクリプトは船舶値付用紙の本番帳票とは無関係の、時刻とDocumentName確認専用の
    最小限のテスト印刷であることに注意。

.PARAMETER PrinterName
    印刷先プリンター名。既定値は船舶機械部の登録名
    「【船舶】FUJIFILM Apeos C3570」。環境によって実際の登録名と
    完全一致している必要があるため、事前に Get-Printer で確認すること。

.PARAMETER DocumentNamePrefix
    DocumentNameの先頭に付ける業務識別子。既定値 "SHIP_VALUETAG_TEST"。

.EXAMPLE
    .\Test-PrintApeosDocumentName.ps1

.EXAMPLE
    .\Test-PrintApeosDocumentName.ps1 -PrinterName "【船舶】FUJIFILM Apeos C3570"

.NOTES
    実行後にコンソールへ表示される DocumentName と「Print()呼び出し直前時刻」を、
    そのまま Check-ApeosPrintJob.ps1 の -ExpectedDocumentName と、
    項目②の比較記録用にメモしておくこと。
#>

[CmdletBinding()]
param(
    [string]$PrinterName = "【船舶】FUJIFILM Apeos C3570",
    [string]$DocumentNamePrefix = "SHIP_VALUETAG_TEST"
)

Add-Type -AssemblyName System.Drawing

# 一意なDocumentNameを生成（仕様書6章の識別子案に準拠）
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$randomSuffix = [guid]::NewGuid().ToString("N").Substring(0, 4).ToUpper()
$documentName = "${DocumentNamePrefix}_${timestamp}_${randomSuffix}"

Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host " Apeos API検証用 テスト印刷" -ForegroundColor Cyan
Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host "プリンター名 : $PrinterName"
Write-Host "DocumentName : $documentName"
Write-Host ""

# --- プリンターの存在確認 ---
$doc = New-Object System.Drawing.Printing.PrintDocument
$doc.PrinterSettings.PrinterName = $PrinterName
$doc.DocumentName = $documentName

if (-not $doc.PrinterSettings.IsValid) {
    Write-Error "指定したプリンター '$PrinterName' が見つからないか無効です。"
    Write-Host "利用可能なプリンター一覧:"
    Get-Printer | Select-Object Name | Format-Table -AutoSize
    exit 1
}

# --- 印刷内容（1ページのみ。DocumentNameと時刻を紙面にも印字し、後から現物と突合できるようにする） ---
$printPageHandler = {
    param($sender, $e)
    $font = New-Object System.Drawing.Font("MS Gothic", 14)
    $brush = [System.Drawing.Brushes]::Black
    $y = 50
    $e.Graphics.DrawString("Apeos API検証用テスト印刷", $font, $brush, 50, $y); $y += 30
    $e.Graphics.DrawString("DocumentName: $documentName", $font, $brush, 50, $y); $y += 30
    $e.Graphics.DrawString("Print()呼び出し時刻(Local): $script:printCallLocalTimeStr", $font, $brush, 50, $y)
}
$doc.add_PrintPage($printPageHandler)

# --- 印刷投入（ここが「実際の投入時刻」。Check-ApeosPrintJob.ps1のスクリプト開始時刻とは別物） ---
$printCallLocalTime = Get-Date
$script:printCallLocalTimeStr = $printCallLocalTime.ToString("yyyy-MM-dd HH:mm:ss.fff")

Write-Host "Print()呼び出し直前時刻（Local・項目②の基準値）: $($printCallLocalTime.ToString('yyyy-MM-dd HH:mm:ss.fff'))" -ForegroundColor Yellow

try {
    $doc.Print()
    $printReturnedLocalTime = Get-Date
    Write-Host "Print()呼び出しが返った時刻（Local）        : $($printReturnedLocalTime.ToString('yyyy-MM-dd HH:mm:ss.fff'))" -ForegroundColor Yellow
} catch {
    Write-Error "印刷に失敗しました: $($_.Exception.Message)"
    exit 1
}

Write-Host ""
Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host " 次のステップ" -ForegroundColor Cyan
Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host "このDocumentNameを Check-ApeosPrintJob.ps1 に渡して、今すぐ確認処理を開始してください:"
Write-Host ""
Write-Host "  .\Check-ApeosPrintJob.ps1 ``"
Write-Host "      -ExpectedPCName `"$env:COMPUTERNAME`" ``"
Write-Host "      -ExpectedDocumentName `"$documentName`" ``"
Write-Host "      -ExpectedCopies 1"
Write-Host ""
Write-Host "項目②の比較記録用に、以下をメモしておいてください:"
Write-Host "  Print()呼び出し直前時刻: $($printCallLocalTime.ToString('yyyy-MM-dd HH:mm:ss.fff'))"
Write-Host "=================================================================" -ForegroundColor Cyan
