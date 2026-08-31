<#
===============================================================================
AdjustInventoryChart_XML.ps1

目的
  本社商品在庫日報.xlsx の「欠品」「在庫額」グラフについて、
  処理基準日当日の列をグラフ履歴範囲から除外し、
  基準日より前の最新日付列までを参照する正式運用スクリプト。

r4 確定内容（2026-08-31）
  - Excel COM / Series.Formula によるグラフ書換えを完全に廃止。
  - xlsx を ZIP/OOXML として扱い、対象 chart XML の参照式だけを直接置換。
  - PowerShell 5.1 で問題になった正規表現文字列のエスケープを安全化。
  - Generic List.Add(PSCustomObject) を使わず、PowerShell 5.1互換の配列処理へ変更。
  - PowerShell 自動変数 $Matches との衝突を避け、$chartMatches を使用。
  - 置換後 chart XML を [xml] として再読込し、well-formed を検証。
  - ZIP 全エントリを SHA-256 比較し、対象 chart XML 以外の変更を禁止。
  - Update 前にバックアップを作成し、全検証通過後のみ File.Replace で反映。
  - 同一条件の2回目 Update は変更なし（冪等性 no-op）とする。
  - 対象外ブックはエラーにせず TargetMatched=False を記録して exit 0 で正常スキップ。

単体テスト実績（2026-08-31）
  ReferenceDate = 2026-08-31
  欠品   : B:J + L:X -> B:J + L:W  （42系列）
  在庫額 : B:G + I:T -> B:G + I:S  （46系列）
  DesiredDate = 2026-08-28
  Audit PASS / Update PASS / 2回目 Update no-op PASS

重要
  - Excel COM の Series.Formula は一切書き換えない。
  - 日付判定は xlsx 内部 worksheet XML の保存済み値を使用する。
  - Excel が対象ブックを開いている状態では Update しない。
  - 本番統合時は、ProcessSheetRows.ps1 が再計算・保存・Excel終了した後に実行する。
  - 対象外の ZIP エントリ内容が変化した場合は fail closed とし、元ファイルを置換しない。
  - 欠品・在庫額のどちらか一方でも検証に失敗した場合は、両方とも反映しない。

対象仕様（2026-08-31 時点）
  欠品   : 固定側 B:J / 履歴側 L:<前営業日列> / 日付行 37
  在庫額 : 固定側 B:G / 履歴側 I:<前営業日列> / 日付行 37

処理基準日より前の日付のうち「最大の日付」を採用する。
空白列・非日付セルは許容し、最初の空白で走査を打ち切らない。
ただし、採用終端列までの履歴範囲内に基準日以降の日付が混在する場合は、
連続範囲では安全に除外できないため fail closed とする。
===============================================================================
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$WorkbookPath,

    [ValidateSet("Audit", "Update")]
    [string]$Mode = "Audit",

    # 単体テスト用。未指定時は当日。
    [string]$ReferenceDate = "",

    [string]$LogPath = ""
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"

$ScriptVersion = "2026-08-31-chart-xml-r5"

# 本番対象のファイル名。単体テストでもファイル名はこのまま使う。
$ExpectedFileName = "本社商品在庫日報.xlsx"

$ChartConfigs = @(
    [PSCustomObject]@{
        SheetName          = "欠品"
        DateRow            = 37
        FixedStartColumn   = "B"
        FixedEndColumn     = "J"
        HistoryStartColumn = "L"
    },
    [PSCustomObject]@{
        SheetName          = "在庫額"
        DateRow            = 37
        FixedStartColumn   = "B"
        FixedEndColumn     = "G"
        HistoryStartColumn = "I"
    }
)

Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem

function Write-Log {
    param([string]$Level, [string]$Message)
    $line = "{0} [{1}] {2}" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss"), $Level, $Message
    Write-Host $line
    if (-not [string]::IsNullOrWhiteSpace($script:LogPath)) {
        Add-Content -LiteralPath $script:LogPath -Value $line -Encoding UTF8
    }
}

function ConvertTo-ColumnNumber {
    param([Parameter(Mandatory = $true)][string]$ColumnName)
    $name = $ColumnName.Trim().ToUpperInvariant()
    if ($name -notmatch '^[A-Z]+$') { throw "Invalid column name: $ColumnName" }
    $n = 0
    foreach ($ch in $name.ToCharArray()) {
        $n = ($n * 26) + ([int][char]$ch - [int][char]'A' + 1)
    }
    return $n
}

function ConvertTo-ColumnName {
    param([Parameter(Mandatory = $true)][int]$ColumnNumber)
    if ($ColumnNumber -lt 1) { throw "Invalid column number: $ColumnNumber" }
    $n = $ColumnNumber
    $result = ""
    while ($n -gt 0) {
        $n--
        $result = [char](65 + ($n % 26)) + $result
        $n = [math]::Floor($n / 26)
    }
    return $result
}

function Get-CellColumnNumber {
    param([Parameter(Mandatory = $true)][string]$CellRef)
    if ($CellRef -notmatch '^([A-Z]+)\d+$') { throw "Invalid cell reference: $CellRef" }
    return (ConvertTo-ColumnNumber -ColumnName $Matches[1])
}

function Open-ZipRead {
    param([Parameter(Mandatory = $true)][string]$Path)
    $fs = [System.IO.File]::Open($Path, [System.IO.FileMode]::Open, [System.IO.FileAccess]::Read, [System.IO.FileShare]::Read)
    try {
        $zip = New-Object System.IO.Compression.ZipArchive($fs, [System.IO.Compression.ZipArchiveMode]::Read, $false)
        return [PSCustomObject]@{ Stream = $fs; Zip = $zip }
    }
    catch {
        $fs.Dispose()
        throw
    }
}

function Read-ZipEntryText {
    param(
        [Parameter(Mandatory = $true)][System.IO.Compression.ZipArchive]$Zip,
        [Parameter(Mandatory = $true)][string]$EntryName
    )
    $entry = $Zip.GetEntry($EntryName)
    if ($null -eq $entry) { throw "ZIP entry not found: $EntryName" }
    $stream = $entry.Open()
    $reader = $null
    try {
        $reader = New-Object System.IO.StreamReader($stream, [System.Text.Encoding]::UTF8, $true)
        return $reader.ReadToEnd()
    }
    finally {
        if ($null -ne $reader) { $reader.Dispose() } else { $stream.Dispose() }
    }
}

function Get-WorksheetEntryName {
    param(
        [Parameter(Mandatory = $true)][System.IO.Compression.ZipArchive]$Zip,
        [Parameter(Mandatory = $true)][string]$SheetName
    )

    [xml]$wb = Read-ZipEntryText -Zip $Zip -EntryName "xl/workbook.xml"
    [xml]$rels = Read-ZipEntryText -Zip $Zip -EntryName "xl/_rels/workbook.xml.rels"

    $wbNs = New-Object System.Xml.XmlNamespaceManager($wb.NameTable)
    $wbNs.AddNamespace("m", "http://schemas.openxmlformats.org/spreadsheetml/2006/main")
    $wbNs.AddNamespace("r", "http://schemas.openxmlformats.org/officeDocument/2006/relationships")

    $sheetNodes = $wb.SelectNodes("/m:workbook/m:sheets/m:sheet", $wbNs)
    $targetNode = $null
    foreach ($node in $sheetNodes) {
        if ([string]$node.GetAttribute("name") -eq $SheetName) {
            $targetNode = $node
            break
        }
    }
    if ($null -eq $targetNode) { throw "Worksheet not found: $SheetName" }

    $rid = $targetNode.GetAttribute("id", "http://schemas.openxmlformats.org/officeDocument/2006/relationships")
    if ([string]::IsNullOrWhiteSpace($rid)) { throw "Worksheet relationship id not found: $SheetName" }

    $relNs = New-Object System.Xml.XmlNamespaceManager($rels.NameTable)
    $relNs.AddNamespace("p", "http://schemas.openxmlformats.org/package/2006/relationships")
    $relNodes = $rels.SelectNodes("/p:Relationships/p:Relationship", $relNs)
    $target = $null
    foreach ($rel in $relNodes) {
        if ([string]$rel.GetAttribute("Id") -eq $rid) {
            $target = [string]$rel.GetAttribute("Target")
            break
        }
    }
    if ([string]::IsNullOrWhiteSpace($target)) { throw "Worksheet target not found: $SheetName" }

    if ($target.StartsWith("/")) {
        $entryName = $target.TrimStart('/')
    }
    else {
        $entryName = "xl/" + $target
    }

    # workbook.xml.rels の Target が worksheets/sheetN.xml の通常形であることを想定。
    $entryName = $entryName.Replace("xl/worksheets/../", "xl/")
    if ($null -eq $Zip.GetEntry($entryName)) { throw "Worksheet ZIP entry not found: $entryName" }
    return $entryName
}

function Get-HistoryEndInfoFromWorksheetXml {
    param(
        [Parameter(Mandatory = $true)][System.IO.Compression.ZipArchive]$Zip,
        [Parameter(Mandatory = $true)][object]$Config,
        [Parameter(Mandatory = $true)][datetime]$CutoffDate
    )

    $entryName = Get-WorksheetEntryName -Zip $Zip -SheetName $Config.SheetName
    [xml]$sheetXml = Read-ZipEntryText -Zip $Zip -EntryName $entryName
    $ns = New-Object System.Xml.XmlNamespaceManager($sheetXml.NameTable)
    $ns.AddNamespace("m", "http://schemas.openxmlformats.org/spreadsheetml/2006/main")

    $row = $sheetXml.SelectSingleNode(("/m:worksheet/m:sheetData/m:row[@r='{0}']" -f $Config.DateRow), $ns)
    if ($null -eq $row) { throw "Date row not found. Sheet=$($Config.SheetName) Row=$($Config.DateRow)" }

    $historyStartNum = ConvertTo-ColumnNumber -ColumnName $Config.HistoryStartColumn
    $dateCells = @()

    foreach ($cell in $row.SelectNodes("m:c", $ns)) {
        $cellRef = [string]$cell.GetAttribute("r")
        if ([string]::IsNullOrWhiteSpace($cellRef)) { continue }
        $colNum = Get-CellColumnNumber -CellRef $cellRef
        if ($colNum -lt $historyStartNum) { continue }

        $v = $cell.SelectSingleNode("m:v", $ns)
        if ($null -eq $v) { continue }

        $number = [double]0
        if (-not [double]::TryParse([string]$v.InnerText,
                [System.Globalization.NumberStyles]::Any,
                [System.Globalization.CultureInfo]::InvariantCulture,
                [ref]$number)) {
            continue
        }
        if ($number -lt 30000 -or $number -gt 80000) { continue }

        try { $d = [datetime]::FromOADate($number).Date }
        catch { continue }

        $dateCells += [PSCustomObject]@{
            ColumnNumber = $colNum
            ColumnName   = (ConvertTo-ColumnName -ColumnNumber $colNum)
            Date         = $d
            CellRef      = $cellRef
        }
    }

    if ($dateCells.Count -eq 0) {
        throw "No date cells found in history area. Sheet=$($Config.SheetName)"
    }

    $prior = @($dateCells | Where-Object { $_.Date -lt $CutoffDate.Date })
    if ($prior.Count -eq 0) {
        throw "No history date earlier than cutoff. Sheet=$($Config.SheetName) Cutoff=$($CutoffDate.ToString('yyyy-MM-dd'))"
    }

    $maxDate = ($prior | Measure-Object -Property Date -Maximum).Maximum
    $sameMax = @($prior | Where-Object { $_.Date -eq $maxDate } | Sort-Object ColumnNumber)
    # 同一日付が複数列にある場合は、その日の全列を含めるため最も右を採用。
    $selected = $sameMax[-1]

    # 履歴側は連続範囲になるため、終端までの途中に基準日以降の日付が存在したら
    # それを巻き込んでしまう。安全のため更新せず停止する。
    $unsafe = @($dateCells | Where-Object {
        $_.ColumnNumber -ge $historyStartNum -and
        $_.ColumnNumber -le $selected.ColumnNumber -and
        $_.Date -ge $CutoffDate.Date
    })
    if ($unsafe.Count -gt 0) {
        $detail = ($unsafe | ForEach-Object { "{0}={1:yyyy-MM-dd}" -f $_.CellRef, $_.Date }) -join ", "
        throw "Unsafe date ordering in history range. Sheet=$($Config.SheetName) $detail"
    }

    return [PSCustomObject]@{
        SheetName      = $Config.SheetName
        WorksheetEntry = $entryName
        EndColumn      = $selected.ColumnName
        EndColumnNumber= $selected.ColumnNumber
        EndDate        = $selected.Date
        DateCellCount  = $dateCells.Count
    }
}

function Get-TargetChartInfo {
    param(
        [Parameter(Mandatory = $true)][System.IO.Compression.ZipArchive]$Zip,
        [Parameter(Mandatory = $true)][object]$Config
    )

    $sheetEsc = [regex]::Escape($Config.SheetName)
    $fixedStart = [regex]::Escape($Config.FixedStartColumn)
    $fixedEnd = [regex]::Escape($Config.FixedEndColumn)
    $histStart = [regex]::Escape($Config.HistoryStartColumn)

    # 行番号はカテゴリ行37だけでなく系列データ行にも適用するためキャプチャする。
    # PowerShellの二重引用符＋バッククォート混在を避け、単一引用符テンプレートで
    # 「$」を正規表現上のリテラルとして安全に構築する。
    $pattern = ('\({0}!\${1}\$(?<row>\d+):\${2}\$\k<row>,{0}!\${3}\$\k<row>:\$(?<end>[A-Z]+)\$\k<row>\)' -f `
        $sheetEsc, $fixedStart, $fixedEnd, $histStart)

    $chartMatches = @()
    foreach ($entry in $Zip.Entries) {
        if ($entry.FullName -notmatch '^xl/charts/chart\d+\.xml$') { continue }
        $text = Read-ZipEntryText -Zip $Zip -EntryName $entry.FullName

        $all = [regex]::Matches($text, $pattern)
        if ($all.Count -gt 0) {
            $endCols = @($all | ForEach-Object { $_.Groups['end'].Value } | Select-Object -Unique)
            $chartMatches += [PSCustomObject]@{
                EntryName = $entry.FullName
                Text      = $text
                Regex     = $pattern
                MatchCount= $all.Count
                EndColumns= $endCols
            }
        }
    }

    if ($chartMatches.Count -ne 1) {
        throw "Expected exactly one chart XML for Sheet=$($Config.SheetName), found $($chartMatches.Count)."
    }

    $info = $chartMatches[0]
    if ($info.EndColumns.Count -ne 1) {
        throw "Chart formulas have inconsistent history end columns. Sheet=$($Config.SheetName) Ends=$($info.EndColumns -join ',')"
    }

    # 同じシートの multi-area 参照が、想定パターン以外にも存在しないことを確認。
    # 予期しない構造を黙って部分更新しないための fail closed。
    $allUnionPattern = ('\({0}!\$[A-Z]+\$\d+:\$[A-Z]+\$\d+,{0}!\$[A-Z]+\$\d+:\$[A-Z]+\$\d+\)' -f $sheetEsc)
    $allUnionForSheet = [regex]::Matches($info.Text, $allUnionPattern)
    if ($allUnionForSheet.Count -ne $info.MatchCount) {
        throw "Unexpected multi-area chart formula structure. Sheet=$($Config.SheetName) ExpectedMatches=$($info.MatchCount) AllUnionMatches=$($allUnionForSheet.Count)"
    }

    return [PSCustomObject]@{
        EntryName   = $info.EntryName
        Text        = $info.Text
        Regex       = $info.Regex
        MatchCount  = $info.MatchCount
        CurrentEnd  = $info.EndColumns[0]
    }
}

function Get-ZipEntryHashes {
    param([Parameter(Mandatory = $true)][string]$Path)
    $result = @{}
    $opened = Open-ZipRead -Path $Path
    $sha = [System.Security.Cryptography.SHA256]::Create()
    try {
        foreach ($entry in $opened.Zip.Entries) {
            $stream = $entry.Open()
            try {
                $hash = $sha.ComputeHash($stream)
                $result[$entry.FullName] = ([BitConverter]::ToString($hash)).Replace("-", "")
            }
            finally { $stream.Dispose() }
        }
    }
    finally {
        $sha.Dispose()
        $opened.Zip.Dispose()
        $opened.Stream.Dispose()
    }
    return $result
}

function Replace-ZipEntryText {
    param(
        [Parameter(Mandatory = $true)][string]$ZipPath,
        [Parameter(Mandatory = $true)][string]$EntryName,
        [Parameter(Mandatory = $true)][string]$NewText
    )

    $fs = [System.IO.File]::Open($ZipPath, [System.IO.FileMode]::Open, [System.IO.FileAccess]::ReadWrite, [System.IO.FileShare]::None)
    $zip = $null
    try {
        $zip = New-Object System.IO.Compression.ZipArchive($fs, [System.IO.Compression.ZipArchiveMode]::Update, $false)
        $old = $zip.GetEntry($EntryName)
        if ($null -eq $old) { throw "ZIP entry not found during update: $EntryName" }
        $old.Delete()
        $newEntry = $zip.CreateEntry($EntryName, [System.IO.Compression.CompressionLevel]::Optimal)
        $stream = $newEntry.Open()
        $writer = $null
        try {
            $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
            $writer = New-Object System.IO.StreamWriter($stream, $utf8NoBom)
            $writer.Write($NewText)
            $writer.Flush()
        }
        finally {
            if ($null -ne $writer) { $writer.Dispose() } else { $stream.Dispose() }
        }
    }
    finally {
        if ($null -ne $zip) { $zip.Dispose() }
        $fs.Dispose()
    }
}

function Test-ZipIntegrity {
    param([Parameter(Mandatory = $true)][string]$Path)
    $opened = Open-ZipRead -Path $Path
    try {
        foreach ($entry in $opened.Zip.Entries) {
            $stream = $entry.Open()
            try {
                $buffer = New-Object byte[] 8192
                while ($stream.Read($buffer, 0, $buffer.Length) -gt 0) { }
            }
            finally { $stream.Dispose() }
        }
    }
    finally {
        $opened.Zip.Dispose()
        $opened.Stream.Dispose()
    }
}

try {
    if (-not (Test-Path -LiteralPath $WorkbookPath -PathType Leaf)) {
        throw "Workbook not found: $WorkbookPath"
    }
    $WorkbookPath = (Resolve-Path -LiteralPath $WorkbookPath).ProviderPath
    $fileName = [System.IO.Path]::GetFileName($WorkbookPath)
    if ($fileName -ne $ExpectedFileName) {
        Write-Log "INFO" ("TargetMatched=False FileName={0} Expected={1} - Skip" -f `
            $fileName, $ExpectedFileName)
        exit 0
    }

    $folder = Split-Path -Parent $WorkbookPath
    if ([string]::IsNullOrWhiteSpace($LogPath)) {
        $LogPath = Join-Path $folder "AdjustInventoryChart_XMLTest.log"
    }
    $script:LogPath = $LogPath

    $cutoffDate = (Get-Date).Date
    if (-not [string]::IsNullOrWhiteSpace($ReferenceDate)) {
        $parsed = [datetime]::MinValue
        if (-not [datetime]::TryParseExact(
            $ReferenceDate, "yyyy-MM-dd",
            [System.Globalization.CultureInfo]::InvariantCulture,
            [System.Globalization.DateTimeStyles]::None,
            [ref]$parsed)) {
            throw "ReferenceDate must be yyyy-MM-dd: $ReferenceDate"
        }
        $cutoffDate = $parsed.Date
    }

    Write-Log "INFO" "Start"
    Write-Log "INFO" ("ScriptVersion={0}" -f $ScriptVersion)
    Write-Log "INFO" ("Mode={0}" -f $Mode)
    Write-Log "INFO" ("Workbook={0}" -f $WorkbookPath)
    Write-Log "INFO" ("CutoffDate={0:yyyy-MM-dd}" -f $cutoffDate)

    $opened = Open-ZipRead -Path $WorkbookPath
    try {
        $plans = @()
        foreach ($cfg in $ChartConfigs) {
            $endInfo = Get-HistoryEndInfoFromWorksheetXml -Zip $opened.Zip -Config $cfg -CutoffDate $cutoffDate
            $chartInfo = Get-TargetChartInfo -Zip $opened.Zip -Config $cfg

            $desiredEnd = $endInfo.EndColumn
            $changed = ($chartInfo.CurrentEnd -ne $desiredEnd)
            $plans += [PSCustomObject]@{
                Config       = $cfg
                ChartEntry   = $chartInfo.EntryName
                CurrentText  = $chartInfo.Text
                Regex        = $chartInfo.Regex
                MatchCount   = $chartInfo.MatchCount
                CurrentEnd   = $chartInfo.CurrentEnd
                DesiredEnd   = $desiredEnd
                DesiredDate  = $endInfo.EndDate
                Changed      = $changed
            }

            Write-Log "INFO" ("Plan Sheet={0} Chart={1} CurrentEnd={2} DesiredEnd={3} DesiredDate={4:yyyy-MM-dd} Matches={5} Changed={6}" -f `
                $cfg.SheetName, $chartInfo.EntryName, $chartInfo.CurrentEnd, $desiredEnd, $endInfo.EndDate, $chartInfo.MatchCount, $changed)
        }
    }
    finally {
        $opened.Zip.Dispose()
        $opened.Stream.Dispose()
    }

    $changePlans = @($plans | Where-Object { $_.Changed })
    if ($Mode -eq "Audit") {
        Write-Log "INFO" ("Audit complete. ChangeTargets={0}. Workbook not modified." -f $changePlans.Count)
        exit 0
    }

    if ($changePlans.Count -eq 0) {
        Write-Log "INFO" "No chart change required. Idempotent no-op."
        exit 0
    }

    # 念のため同一 chart XML を複数設定が同時に変更しようとしていないことを確認。
    $dupEntries = $changePlans | Group-Object ChartEntry | Where-Object { $_.Count -gt 1 }
    if ($dupEntries) {
        throw "Multiple chart configs target the same chart XML: $($dupEntries.Name -join ',')"
    }

    $backupFolder = Join-Path $folder "Backup"
    if (-not (Test-Path -LiteralPath $backupFolder)) {
        New-Item -ItemType Directory -Path $backupFolder -Force | Out-Null
    }
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $base = [System.IO.Path]::GetFileNameWithoutExtension($WorkbookPath)
    $ext = [System.IO.Path]::GetExtension($WorkbookPath)
    $backupPath = Join-Path $backupFolder ("{0}_ChartBefore_{1}{2}" -f $base, $stamp, $ext)
    Copy-Item -LiteralPath $WorkbookPath -Destination $backupPath -Force
    Write-Log "INFO" ("Backup={0}" -f $backupPath)

    $beforeHashes = Get-ZipEntryHashes -Path $WorkbookPath

    $tempPath = Join-Path $folder (".{0}.chartxml.{1}.tmp.xlsx" -f $base, [guid]::NewGuid().ToString("N"))
    Copy-Item -LiteralPath $WorkbookPath -Destination $tempPath -Force

    try {
        foreach ($plan in $changePlans) {
            $cfg = $plan.Config
            $newText = [regex]::Replace($plan.CurrentText, $plan.Regex, {
                param($m)
                $row = $m.Groups['row'].Value
                return ("({0}!`${1}`${2}:`${3}`${2},{0}!`${4}`${2}:`${5}`${2})" -f `
                    $cfg.SheetName, $cfg.FixedStartColumn, $row, $cfg.FixedEndColumn, $cfg.HistoryStartColumn, $plan.DesiredEnd)
            })

            if ($newText -eq $plan.CurrentText) {
                throw "Replacement produced no text change unexpectedly. Sheet=$($cfg.SheetName)"
            }

            # 置換後chart XMLがwell-formedであることを明示検証する。
            # DOMで再保存はせず、検証のために読み込むだけなので元テキストは変化しない。
            $xmlCheck = [xml]$newText
            if ($null -eq $xmlCheck.DocumentElement) {
                throw "Patched chart XML is not well-formed. Sheet=$($cfg.SheetName)"
            }

            $afterMatch = [regex]::Matches($newText, $plan.Regex)
            if ($afterMatch.Count -ne $plan.MatchCount) {
                throw "Formula match count changed unexpectedly. Sheet=$($cfg.SheetName) Before=$($plan.MatchCount) After=$($afterMatch.Count)"
            }
            $endsAfter = @($afterMatch | ForEach-Object { $_.Groups['end'].Value } | Select-Object -Unique)
            if ($endsAfter.Count -ne 1 -or $endsAfter[0] -ne $plan.DesiredEnd) {
                throw "Post-replacement end column validation failed. Sheet=$($cfg.SheetName) Ends=$($endsAfter -join ',')"
            }

            Replace-ZipEntryText -ZipPath $tempPath -EntryName $plan.ChartEntry -NewText $newText
            Write-Log "INFO" ("Patched Sheet={0} Chart={1} {2}->{3}" -f $cfg.SheetName, $plan.ChartEntry, $plan.CurrentEnd, $plan.DesiredEnd)
        }

        Test-ZipIntegrity -Path $tempPath

        # tempを再読込して、最終状態の参照式を再検証する。
        $verifyZip = Open-ZipRead -Path $tempPath
        try {
            foreach ($plan in $plans) {
                $verify = Get-TargetChartInfo -Zip $verifyZip.Zip -Config $plan.Config
                if ($verify.CurrentEnd -ne $plan.DesiredEnd) {
                    throw "Final chart verification failed. Sheet=$($plan.Config.SheetName) Actual=$($verify.CurrentEnd) Expected=$($plan.DesiredEnd)"
                }
            }
        }
        finally {
            $verifyZip.Zip.Dispose()
            $verifyZip.Stream.Dispose()
        }

        $afterHashes = Get-ZipEntryHashes -Path $tempPath
        $allowedChanged = @($changePlans | ForEach-Object { $_.ChartEntry } | Select-Object -Unique)

        $allNames = @($beforeHashes.Keys + $afterHashes.Keys | Select-Object -Unique)
        foreach ($name in $allNames) {
            if (-not $beforeHashes.ContainsKey($name) -or -not $afterHashes.ContainsKey($name)) {
                throw "ZIP entry set changed unexpectedly: $name"
            }
            if ($beforeHashes[$name] -ne $afterHashes[$name] -and $name -notin $allowedChanged) {
                throw "Unexpected ZIP entry content change detected: $name"
            }
        }

        foreach ($name in $allowedChanged) {
            if ($beforeHashes[$name] -eq $afterHashes[$name]) {
                throw "Expected chart XML did not change: $name"
            }
        }

        # 検証済みtempだけを本体へ反映する。失敗時は元ファイルを残す。
        try {
            [System.IO.File]::Replace($tempPath, $WorkbookPath, $null)
        }
        catch {
            # File.Replaceが利用できない環境向けフォールバック。
            Copy-Item -LiteralPath $tempPath -Destination $WorkbookPath -Force
            Remove-Item -LiteralPath $tempPath -Force -ErrorAction SilentlyContinue
        }

        Write-Log "INFO" ("Update complete. ChangedCharts={0}" -f $changePlans.Count)
        Write-Log "INFO" "Only chart XML entries were changed; all other ZIP entry contents matched SHA-256."
    }
    catch {
        Write-Log "ERROR" ("Update failed before final replacement. Original workbook remains restorable from backup. {0}" -f $_.Exception.Message)
        throw
    }
    finally {
        if (Test-Path -LiteralPath $tempPath) {
            Remove-Item -LiteralPath $tempPath -Force -ErrorAction SilentlyContinue
        }
    }

    exit 0
}
catch {
    try { Write-Log "ERROR" $_.Exception.Message } catch { Write-Host $_.Exception.Message }
    exit 1
}
