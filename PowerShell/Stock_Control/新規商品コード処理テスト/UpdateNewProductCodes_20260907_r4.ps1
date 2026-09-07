<#
.SYNOPSIS
    Phase 1 r4: 未管理の商品コードを「追加新規商品コード」シートへ見える化する。

.DESCRIPTION
    - バーコード貼付シートには存在するが、数字シート（1,2,3...）のB列に存在しない商品コードを検出する。
    - 在庫数が0でも対象とする。
    - 拠点別変数表の「新規コード管理_実行」が「有」の拠点だけ処理する。
    - Auditモードでは対象ブックを一切更新しない。
    - Updateモードでは「追加新規商品コード」シートのみを作成・追記する。
    - 数字シートは一切変更しない。
    - 同じ商品コードを「追加新規商品コード」へ重複登録しない。
    - Phase 1では数字シートへの移動処理は行わない。

.NOTES
    初期除外拠点は変数表側で制御する。
    2026-09-04 仕様:
      外注鉄                 新規コード管理_実行 = 無
      成果連絡表(色物)       新規コード管理_実行 = 無
    スクリプト内に拠点名をハードコードしない。
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$WorkbookPath,

    [Parameter(Mandatory = $false)]
    [string]$SiteName = "",

    [Parameter(Mandatory = $false)]
    [ValidateSet("Audit", "Update")]
    [string]$Mode = "Audit",

    [Parameter(Mandatory = $false)]
    [string]$VariableTablePath = "C:\HPDB\在庫チェックRPA統合_拠点別変数表.xlsx",

    [Parameter(Mandatory = $false)]
    [string]$BarcodeSheetName = "バーコード貼付",

    [Parameter(Mandatory = $false)]
    [string]$NewCodeSheetName = "追加新規商品コード",

    [Parameter(Mandatory = $false)]
    [string]$OutputDirectory = ""
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Write-Log {
    param(
        [string]$Level,
        [string]$Message
    )
    $ts = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    Write-Host "[$ts][$Level] $Message"
}

function Release-ComObject {
    param([object]$Object)
    if ($null -ne $Object) {
        try {
            [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($Object)
        } catch {
        }
    }
}

function Normalize-Code {
    param([object]$Value)
    if ($null -eq $Value) { return "" }
    return ([string]$Value).Trim()
}

function Get-HeaderMap {
    param(
        [object]$Worksheet,
        [int]$HeaderRow = 3
    )

    $map = @{}
    $used = $Worksheet.UsedRange
    try {
        $lastCol = $used.Column + $used.Columns.Count - 1
        for ($col = 1; $col -le $lastCol; $col++) {
            $v = [string]$Worksheet.Cells.Item($HeaderRow, $col).Text
            if (-not [string]::IsNullOrWhiteSpace($v)) {
                $name = $v.Trim()
                if (-not $map.ContainsKey($name)) {
                    $map[$name] = $col
                }
            }
        }
    }
    finally {
        Release-ComObject $used
    }
    return $map
}

function Get-SiteSetting {
    param(
        [string]$SettingsPath,
        [string]$RequestedSiteName,
        [string]$WorkbookFileName
    )

    if (-not (Test-Path -LiteralPath $SettingsPath)) {
        throw "変数表が見つかりません: $SettingsPath"
    }

    $excel = $null
    $book = $null
    $sheet = $null

    try {
        $excel = New-Object -ComObject Excel.Application
        $excel.Visible = $false
        $excel.DisplayAlerts = $false

        $book = $excel.Workbooks.Open($SettingsPath, 0, $true)
        $sheet = $book.Worksheets.Item("拠点別設定値")
        $headers = Get-HeaderMap -Worksheet $sheet -HeaderRow 3

        foreach ($required in @("拠点名", "在庫帳ブック名", "新規コード管理_実行")) {
            if (-not $headers.ContainsKey($required)) {
                throw "変数表に必要列「$required」がありません。"
            }
        }

        $used = $sheet.UsedRange
        try {
            $lastRow = $used.Row + $used.Rows.Count - 1
        }
        finally {
            Release-ComObject $used
        }

        $matchedRow = 0

        # 1) SiteName指定がある場合は拠点名を優先
        if (-not [string]::IsNullOrWhiteSpace($RequestedSiteName)) {
            for ($row = 4; $row -le $lastRow; $row++) {
                $v = ([string]$sheet.Cells.Item($row, $headers["拠点名"]).Text).Trim()
                if ($v -eq $RequestedSiteName.Trim()) {
                    $matchedRow = $row
                    break
                }
            }
        }

        # 2) 見つからなければ在庫帳ブック名で解決
        if ($matchedRow -eq 0) {
            for ($row = 4; $row -le $lastRow; $row++) {
                $v = ([string]$sheet.Cells.Item($row, $headers["在庫帳ブック名"]).Text).Trim()
                if ($v -eq $WorkbookFileName) {
                    $matchedRow = $row
                    break
                }
            }
        }

        if ($matchedRow -eq 0) {
            throw "変数表で拠点を特定できません。SiteName='$RequestedSiteName' Workbook='$WorkbookFileName'"
        }

        $resolvedSite = ([string]$sheet.Cells.Item($matchedRow, $headers["拠点名"]).Text).Trim()
        $enabled = ([string]$sheet.Cells.Item($matchedRow, $headers["新規コード管理_実行"]).Text).Trim()

        $note = ""
        if ($headers.ContainsKey("新規コード管理_備考")) {
            $note = ([string]$sheet.Cells.Item($matchedRow, $headers["新規コード管理_備考"]).Text).Trim()
        }

        return [pscustomobject]@{
            SiteName = $resolvedSite
            Enabled  = $enabled
            Note     = $note
            Row      = $matchedRow
        }
    }
    finally {
        if ($null -ne $book) {
            try { $book.Close($false) } catch {}
        }
        if ($null -ne $excel) {
            try { $excel.Quit() } catch {}
        }
        Release-ComObject $sheet
        Release-ComObject $book
        Release-ComObject $excel
        [GC]::Collect()
        [GC]::WaitForPendingFinalizers()
    }
}

function Export-CandidateCsv {
    param(
        [System.Collections.IEnumerable]$Candidates,
        [string]$Directory,
        [string]$ResolvedSiteName,
        [string]$CurrentMode
    )

    if ([string]::IsNullOrWhiteSpace($Directory)) {
        return $null
    }

    if (-not (Test-Path -LiteralPath $Directory)) {
        New-Item -ItemType Directory -Path $Directory -Force | Out-Null
    }

    $safeSite = $ResolvedSiteName -replace '[\\/:*?"<>|]', '_'
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $path = Join-Path $Directory ("NewProductCandidates_{0}_{1}_{2}.csv" -f $safeSite, $CurrentMode, $stamp)

    $rows = @($Candidates | Select-Object `
        @{N="拠点";E={$ResolvedSiteName}}, `
        @{N="商品コード";E={$_.Code}}, `
        @{N="商品名";E={$_.Name}}, `
        @{N="在庫数";E={$_.Stock}}, `
        @{N="バーコード行";E={$_.SourceRow}}, `
        @{N="重複行数";E={$_.DuplicateCount}}, `
        @{N="状態";E={$_.State}}
    )

    if ($rows.Count -gt 0) {
        $rows | Export-Csv -LiteralPath $path -NoTypeInformation -Encoding UTF8
    }
    else {
        "拠点,商品コード,商品名,在庫数,バーコード行,重複行数,状態" | Set-Content -LiteralPath $path -Encoding UTF8
    }

    return $path
}

# -------------------------
# Main
# -------------------------
if (-not (Test-Path -LiteralPath $WorkbookPath)) {
    Write-Log "ERROR" "対象ブックが見つかりません: $WorkbookPath"
    exit 2
}

$workbookFullPath = (Get-Item -LiteralPath $WorkbookPath).FullName
$workbookFileName = [System.IO.Path]::GetFileName($workbookFullPath)

if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $OutputDirectory = Join-Path ([System.IO.Path]::GetDirectoryName($workbookFullPath)) "作業用CSV"
}

try {
    $setting = Get-SiteSetting `
        -SettingsPath $VariableTablePath `
        -RequestedSiteName $SiteName `
        -WorkbookFileName $workbookFileName

    Write-Log "INFO" "拠点=$($setting.SiteName) Mode=$Mode 新規コード管理_実行=$($setting.Enabled)"

    if ($setting.Enabled -ne "有") {
        $msg = "新規コード管理_実行が「有」ではないためスキップします。"
        if (-not [string]::IsNullOrWhiteSpace($setting.Note)) {
            $msg += " 備考=$($setting.Note)"
        }
        Write-Log "SKIP" $msg
        exit 0
    }
}
catch {
    Write-Log "ERROR" $_.Exception.Message
    exit 3
}

$excel = $null
$book = $null
$barcodeSheet = $null
$newSheet = $null
$saveNeeded = $false

try {
    $excel = New-Object -ComObject Excel.Application
    $excel.Visible = $false
    $excel.DisplayAlerts = $false
    $excel.ScreenUpdating = $false

    $readOnly = ($Mode -eq "Audit")
    $book = $excel.Workbooks.Open($workbookFullPath, 0, $readOnly)

    # バーコード貼付シート確認
    try {
        $barcodeSheet = $book.Worksheets.Item($BarcodeSheetName)
    }
    catch {
        throw "「$BarcodeSheetName」シートが見つかりません。"
    }

    # 数字シートB列の商品コード集合
    $managedCodes = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)
    $numericSheetCount = 0

    foreach ($ws in @($book.Worksheets)) {
        try {
            $sheetName = [string]$ws.Name
            if ($sheetName -match '^\d+$') {
                $numericSheetCount++
                $used = $ws.UsedRange
                try {
                    $lastRow = $used.Row + $used.Rows.Count - 1
                }
                finally {
                    Release-ComObject $used
                }

                for ($row = 3; $row -le $lastRow; $row++) {
                    $code = Normalize-Code $ws.Cells.Item($row, 2).Value2
                    if (-not [string]::IsNullOrWhiteSpace($code)) {
                        [void]$managedCodes.Add($code)
                    }
                }
            }
        }
        finally {
            Release-ComObject $ws
        }
    }

    if ($numericSheetCount -eq 0) {
        throw "数字シート（シート名が1,2,3...）が1枚も見つかりません。"
    }

    Write-Log "INFO" "数字シート数=$numericSheetCount 管理済商品コード数=$($managedCodes.Count)"

    # 既存の「追加新規商品コード」シートのコード集合
    $existingNewCodes = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)
    $newSheetExists = $false

    try {
        $newSheet = $book.Worksheets.Item($NewCodeSheetName)
        $newSheetExists = $true

        $used = $newSheet.UsedRange
        try {
            $lastRow = $used.Row + $used.Rows.Count - 1
        }
        finally {
            Release-ComObject $used
        }

        for ($row = 3; $row -le $lastRow; $row++) {
            $code = Normalize-Code $newSheet.Cells.Item($row, 1).Value2
            if (-not [string]::IsNullOrWhiteSpace($code)) {
                [void]$existingNewCodes.Add($code)
            }
        }

        Write-Log "INFO" "既存「$NewCodeSheetName」コード数=$($existingNewCodes.Count)"
    }
    catch {
        $newSheetExists = $false
        Release-ComObject $newSheet
        $newSheet = $null
        Write-Log "INFO" "「$NewCodeSheetName」シートは未作成です。"
    }

    # バーコード貼付 A=商品コード / B=商品名 / C=在庫数
    # 同一コードが複数行に存在した場合は1件にまとめ、重複状態を明示する。
    $barcodeByCode = @{}

    $used = $barcodeSheet.UsedRange
    try {
        $barcodeLastRow = $used.Row + $used.Rows.Count - 1
    }
    finally {
        Release-ComObject $used
    }

    for ($row = 3; $row -le $barcodeLastRow; $row++) {
        $code = Normalize-Code $barcodeSheet.Cells.Item($row, 1).Value2
        if ([string]::IsNullOrWhiteSpace($code)) {
            continue
        }

        $name = ([string]$barcodeSheet.Cells.Item($row, 2).Text).Trim()
        $stockValue = $barcodeSheet.Cells.Item($row, 3).Value2

        if (-not $barcodeByCode.ContainsKey($code)) {
            $barcodeByCode[$code] = [pscustomobject]@{
                Code           = $code
                Name           = $name
                Stock          = $stockValue
                SourceRow      = $row
                DuplicateCount = 1
            }
        }
        else {
            $barcodeByCode[$code].DuplicateCount++
        }
    }

    # 未管理かつ待避シート未登録のものを候補化
    $candidates = New-Object System.Collections.Generic.List[object]

    foreach ($entry in $barcodeByCode.Values) {
        if ($managedCodes.Contains($entry.Code)) {
            continue
        }
        if ($existingNewCodes.Contains($entry.Code)) {
            continue
        }

        $state = "未振分"
        if ($entry.DuplicateCount -gt 1) {
            $state = "未振分（バーコード重複確認要）"
        }

        $candidates.Add([pscustomobject]@{
            Code           = $entry.Code
            Name           = $entry.Name
            Stock          = $entry.Stock
            SourceRow      = $entry.SourceRow
            DuplicateCount = $entry.DuplicateCount
            State          = $state
        })
    }

    # コード順で安定化
    $candidates = @($candidates | Sort-Object Code)

    $dupCandidateCount = @($candidates | Where-Object { $_.DuplicateCount -gt 1 }).Count
    Write-Log "INFO" "未管理新規候補=$($candidates.Count) うちバーコード重複候補=$dupCandidateCount"

    $csvPath = Export-CandidateCsv `
        -Candidates $candidates `
        -Directory $OutputDirectory `
        -ResolvedSiteName $setting.SiteName `
        -CurrentMode $Mode

    if ($null -ne $csvPath) {
        Write-Log "INFO" "候補CSV=$csvPath"
    }

    if ($Mode -eq "Audit") {
        Write-Log "PASS" "Audit完了。対象ブックは更新していません。"
        exit 0
    }

    # Updateで候補0件なら何も更新しない
    if ($candidates.Count -eq 0) {
        Write-Log "PASS" "Update対象は0件です。ブックは変更していません。"
        exit 0
    }

    # 待避シートがなければ作成
    if (-not $newSheetExists) {
        $newSheet = $book.Worksheets.Add()
        $newSheet.Name = $NewCodeSheetName

        # A1:F1 タイトル、A2:F2 見出し。データはA3から。
        $newSheet.Range("A1:F1").Merge()
        $newSheet.Range("A1").Value2 = "追加新規商品コード（自動検出／人確認用）"

        # COM Range への2次元配列一括代入は環境差を避け、見出しは個別代入する。
        $headers = @("商品コード", "商品名", "在庫数", "移動先シート", "初回検知日", "状態")
        for ($i = 0; $i -lt $headers.Count; $i++) {
            $newSheet.Cells.Item(2, $i + 1).Value2 = $headers[$i]
        }

        $newSheet.Range("A1:F1").Font.Bold = $true
        $newSheet.Range("A2:F2").Font.Bold = $true
        $newSheet.Range("A2:F2").Interior.ColorIndex = 15

        $newSheet.Columns.Item("A").ColumnWidth = 18
        $newSheet.Columns.Item("B").ColumnWidth = 34
        $newSheet.Columns.Item("C").ColumnWidth = 12
        $newSheet.Columns.Item("D").ColumnWidth = 16
        $newSheet.Columns.Item("E").ColumnWidth = 14
        $newSheet.Columns.Item("F").ColumnWidth = 30

        $newSheet.Columns.Item("A").NumberFormat = "@"
        $newSheet.Columns.Item("D").NumberFormat = "@"
        $newSheet.Columns.Item("E").NumberFormat = "yyyy/mm/dd"

        # 新規作成直後はデータ行が存在しないため、末尾探索をせずA3から開始する。
        # r1では作成直後に Cells.Item(Rows.Count,1).End(xlUp) を評価する経路で
        # 一部Excel COM環境に NullReferenceException が発生したため、この経路を廃止。
        $nextRow = 3

        $saveNeeded = $true
        Write-Log "INFO" "「$NewCodeSheetName」シートを新規作成しました。次回追加行=$nextRow"
    }
    else {
        # 既存シートの場合だけA列の最終データ行を安全に探索する。
        $used = $newSheet.UsedRange
        try {
            $lastUsedRow = $used.Row + $used.Rows.Count - 1
        }
        finally {
            Release-ComObject $used
        }

        # UsedRangeには見出し等も含まれるため、A列を下から上へ確認する。
        $nextRow = 3
        for ($row = $lastUsedRow; $row -ge 3; $row--) {
            $codeAtRow = Normalize-Code $newSheet.Cells.Item($row, 1).Value2
            if (-not [string]::IsNullOrWhiteSpace($codeAtRow)) {
                $nextRow = $row + 1
                break
            }
        }
        Write-Log "INFO" "既存「$NewCodeSheetName」への次回追加行=$nextRow"
    }

    $today = (Get-Date).Date
    $added = 0

    foreach ($item in $candidates) {
        $newSheet.Cells.Item($nextRow, 1).Value2 = [string]$item.Code
        $newSheet.Cells.Item($nextRow, 2).Value2 = [string]$item.Name

        # Excel COM / PowerShell の Value2 setter は環境によって Double を String へ
        # 暗黙変換しようとして失敗する場合があるため、Phase1待避シートでは表示値として文字列化する。
        # このシートは人による確認用であり、C/E列を計算用途には使用しない。
        if ($null -eq $item.Stock -or [string]::IsNullOrWhiteSpace([string]$item.Stock)) {
            $newSheet.Cells.Item($nextRow, 3).Value2 = ""
        }
        else {
            $newSheet.Cells.Item($nextRow, 3).Value2 = [string]$item.Stock
        }

        # D: 人が入力するため空欄
        $newSheet.Cells.Item($nextRow, 4).Value2 = ""

        # E: 初回検知日。COM型変換差を避けるため yyyy/MM/dd の表示文字列で保持。
        $newSheet.Cells.Item($nextRow, 5).Value2 = $today.ToString("yyyy/MM/dd")

        # F: 状態
        $newSheet.Cells.Item($nextRow, 6).Value2 = [string]$item.State

        $added++
        $nextRow++
    }

    if ($added -gt 0) {
        $saveNeeded = $true
    }

    if ($saveNeeded) {
        $book.Save()
        Write-Log "PASS" "Update完了。追加件数=$added 保存しました。"
    }
    else {
        Write-Log "PASS" "変更なし。保存していません。"
    }

    exit 0
}
catch {
    Write-Log "ERROR" $_.Exception.Message
    if ($null -ne $_.InvocationInfo) {
        Write-Log "ERROR" ("Line=" + $_.InvocationInfo.ScriptLineNumber + " Position=" + $_.InvocationInfo.PositionMessage)
    }
    exit 10
}
finally {
    if ($null -ne $book) {
        try { $book.Close($false) } catch {}
    }
    if ($null -ne $excel) {
        try { $excel.Quit() } catch {}
    }

    Release-ComObject $newSheet
    Release-ComObject $barcodeSheet
    Release-ComObject $book
    Release-ComObject $excel

    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()
}
