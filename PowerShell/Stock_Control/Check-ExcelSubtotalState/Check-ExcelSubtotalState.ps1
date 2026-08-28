<#
===============================================================================
スクリプト名:
  Check-ExcelSubtotalState.ps1

概要:
  .xlsx ファイルを ZIP として開き、内部XMLを解析して
  「集計あり / 集計なし」を判定するスクリプトです。

用途:
  RK-10 のコマンドライン実行で使いやすいように、
  標準出力は OK / NG / ERROR のみ返す簡易版です。
  ※ 標準出力は改行なしで返します。

判定ルール:
  1. シートXML内の行要素に outlineLevel 属性がある
     → グループ化 / 集計の痕跡あり

  2. シートXML内の数式セルに SUBTOTAL(...) がある
     → 集計行あり

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
  ・対象は .xlsx ファイルです
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

function Get-ZipEntryText {
    param(
        [Parameter(Mandatory = $true)] $Zip,
        [Parameter(Mandatory = $true)] [string]$EntryPath
    )

    $entry = $Zip.GetEntry($EntryPath)
    if ($null -eq $entry) {
        throw "ZIP内に '$EntryPath' が見つかりません。"
    }

    $stream = $entry.Open()
    $reader = New-Object System.IO.StreamReader($stream, [System.Text.Encoding]::UTF8)
    try {
        return $reader.ReadToEnd()
    }
    finally {
        $reader.Close()
        $stream.Close()
    }
}

Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem

$zip = $null

try {
    if (-not (Test-Path -LiteralPath $FilePath)) {
        Save-DetailResult -Status "ERROR" -Message "対象ファイルが存在しません。" -Details @()
        [Console]::Write("ERROR")
        exit 9
    }

    if ([System.IO.Path]::GetExtension($FilePath).ToLower() -ne ".xlsx") {
        Save-DetailResult -Status "ERROR" -Message "対象は .xlsx ファイルを指定してください。" -Details @()
        [Console]::Write("ERROR")
        exit 9
    }

    $zip = [System.IO.Compression.ZipFile]::OpenRead($FilePath)

    [xml]$workbookXml = Get-ZipEntryText -Zip $zip -EntryPath "xl/workbook.xml"
    [xml]$relsXml     = Get-ZipEntryText -Zip $zip -EntryPath "xl/_rels/workbook.xml.rels"

    $nsMain = New-Object System.Xml.XmlNamespaceManager($workbookXml.NameTable)
    $nsMain.AddNamespace("x", "http://schemas.openxmlformats.org/spreadsheetml/2006/main")

    $nsRel = New-Object System.Xml.XmlNamespaceManager($relsXml.NameTable)
    $nsRel.AddNamespace("r", "http://schemas.openxmlformats.org/package/2006/relationships")

    $sheetNodes = $workbookXml.SelectNodes("//x:sheets/x:sheet", $nsMain)
    if ($sheetNodes.Count -eq 0) {
        throw "workbook.xml からシート情報を取得できませんでした。"
    }

    $targetSheetNode = $null

    if ([string]::IsNullOrWhiteSpace($SheetName)) {
        $targetSheetNode = $sheetNodes[0]
        $SheetName = $targetSheetNode.GetAttribute("name")
    }
    else {
        foreach ($node in $sheetNodes) {
            if ($node.GetAttribute("name") -eq $SheetName) {
                $targetSheetNode = $node
                break
            }
        }

        if ($null -eq $targetSheetNode) {
            Save-DetailResult -Status "ERROR" -Message "指定シート '$SheetName' が見つかりません。" -Details @()
            [Console]::Write("ERROR")
            exit 9
        }
    }

    $relId = $targetSheetNode.GetAttribute("id", "http://schemas.openxmlformats.org/officeDocument/2006/relationships")
    if ([string]::IsNullOrWhiteSpace($relId)) {
        throw "対象シートの relationship id を取得できませんでした。"
    }

    $relNode = $relsXml.SelectSingleNode("//r:Relationship[@Id='$relId']", $nsRel)
    if ($null -eq $relNode) {
        throw "対象シートの rel 情報を取得できませんでした。 Id=$relId"
    }

    $target = $relNode.GetAttribute("Target")
    if ([string]::IsNullOrWhiteSpace($target)) {
        throw "対象シートの Target が空です。"
    }

    $sheetXmlPath = "xl/" + $target.Replace("\", "/").TrimStart("/")
    [xml]$sheetXml = Get-ZipEntryText -Zip $zip -EntryPath $sheetXmlPath

    $nsSheet = New-Object System.Xml.XmlNamespaceManager($sheetXml.NameTable)
    $nsSheet.AddNamespace("x", "http://schemas.openxmlformats.org/spreadsheetml/2006/main")

    $details = New-Object System.Collections.Generic.List[string]

    # outlineLevel 判定
    $outlineRows = $sheetXml.SelectNodes("//x:sheetData/x:row[@outlineLevel]", $nsSheet)
    $outlineCount = 0
    if ($outlineRows -ne $null) {
        $outlineCount = $outlineRows.Count
        $sample = $outlineRows | Select-Object -First 5
        foreach ($row in $sample) {
            $r  = $row.GetAttribute("r")
            $lv = $row.GetAttribute("outlineLevel")
            $details.Add("行 $r に outlineLevel=$lv を検出")
        }
    }

    # SUBTOTAL 判定
    $formulaNodes = $sheetXml.SelectNodes("//x:c[x:f]", $nsSheet)
    $subtotalHits = New-Object System.Collections.Generic.List[string]

    if ($formulaNodes -ne $null) {
        foreach ($cell in $formulaNodes) {
            $ref = $cell.GetAttribute("r")
            $fNode = $cell.SelectSingleNode("x:f", $nsSheet)
            if ($null -ne $fNode) {
                $formulaText = [string]$fNode.InnerText
                if ($formulaText -match "(?i)SUBTOTAL\s*\(") {
                    $subtotalHits.Add("セル $ref に SUBTOTAL 関数を検出: =$formulaText")
                    if ($subtotalHits.Count -ge 5) {
                        break
                    }
                }
            }
        }
    }

    foreach ($hit in $subtotalHits) {
        $details.Add($hit)
    }

    $hasOutline  = $outlineCount -gt 0
    $hasSubtotal = $subtotalHits.Count -gt 0

    if ($hasOutline -or $hasSubtotal) {
        $msgParts = @()
        if ($hasOutline)  { $msgParts += "outlineLevel あり（$outlineCount 行）" }
        if ($hasSubtotal) { $msgParts += "SUBTOTAL あり" }

        Save-DetailResult -Status "NG" -Message ("集計状態の痕跡があります。 " + ($msgParts -join " / ")) -Details $details
        [Console]::Write("NG")
        exit 1
    }
    else {
        Save-DetailResult -Status "OK" -Message "集計状態の明確な痕跡は見つかりませんでした。" -Details @()
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
    if ($null -ne $zip) {
        $zip.Dispose()
    }
}