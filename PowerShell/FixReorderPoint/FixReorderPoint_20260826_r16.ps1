<#
===============================================================================
FixReorderPoint.ps1

概要
  バーコード貼付シートの在庫を参照し、
  数字シートの「発注点=0 かつ 在庫>0」の行のみ
  発注点(E列)を在庫数へ補正します。

モード
  Audit  : 更新候補のみ表示（Excelは変更しません）
  Update : Backupフォルダへ退避後、Excelを更新します。

実行例
  Audit
    powershell.exe -NoProfile -ExecutionPolicy Bypass `
      -File "C:\HPDB\FixReorderPoint.ps1" `
      -WorkbookPath "C:\HPDB\福岡在庫帳.xlsx" `
      -SiteName "福岡" `
      -Mode Audit `
      -LogPath "C:\HPDB\FixReorderPoint.log"

  Update
    powershell.exe -NoProfile -ExecutionPolicy Bypass `
      -File "C:\HPDB\FixReorderPoint.ps1" `
      -WorkbookPath "C:\HPDB\福岡在庫帳.xlsx" `
      -SiteName "福岡" `
      -Mode Update `
      -LogPath "C:\HPDB\FixReorderPoint.log" `
      -CorrectionEnabled "有" `
      -CsvFolderPath "F:\Users\塚田淳一\在庫帳" `
      -CsvFileName "倉庫在庫データ.csvNoSpace.csv"

  CsvFolderPath / CsvFileName
    RK-10がローカルへ保存した処理用CSVと、在庫帳の「バーコード貼付」シートを
    発注点補正の前に照合するための引数。CsvFileNameは実際のファイル名をそのまま渡す。
    現行例：倉庫在庫データ.csvNoSpace.csv
    ※スクリプト側では "NoSpace.csv" 等の接尾辞を付加しない。将来RK-10側で
      ファイル名を変更しても、CsvFileNameに新しい実ファイル名を渡せばそのまま動作する。

  CorrectionEnabled
    拠点別変数表の「発注点補正_実行」列の値（RK-10から渡す想定）。
    "有"以外（例："無"）を指定すると、バーコード貼付検証までは実施した後、
    発注点補正をスキップして正常終了（exit 0）する。
    省略時は"有"扱いとなり、従来通り全件が対象となる。

  MaxCandidates
    Update時、候補件数がこの値を超えると異常終了（exit 1）する安全装置の閾値。
    既定値は1000。誤検知等で候補が異常に多い場合の歯止めとして残しているが、
    拠点の事情で1000件を超える正当な候補が出る場合は、この値を引数で調整すること。

  HistoryPath
    「発注点書き換え履歴」を記録するCSVファイルのパス。
    省略時はワークブックと同じフォルダの「ReorderPointHistory.csv」に自動決定される。
    このCSVは実際にセルを書き換えた行のみ追記される（Auditモードでは書き込まれない）。
    ファイルが存在しない場合は新規作成（ヘッダー付き）、存在する場合は追記のみ行う。
    文字コードはExcelでの可読性を優先しUTF-8(BOM付き)で出力する。

仕様
  ・対象シート : シート名が数字のみ
  ・品番       : B列
  ・発注点     : E列
  ・在庫参照   : バーコード貼付(A列=品番、C列=在庫)
  ・終了コード : 0=正常 / 1=その他異常 / 20=CSVとバーコード貼付不一致
                 21=CSVファイルなし / 22=バーコード貼付シートなし

バージョン管理について
  このスクリプトは各拠点PCのローカル（例：C:\HPDB）に個別配置して運用するため、
  改修後に配布漏れが起きると、PCごとに版がバラバラになっても気づきにくい。
  そのため冒頭の $ScriptVersion を改修のたびに更新し、実行時にログ
  （FixReorderPoint.log）へ出力することで、各PCが最新版かどうかを後から
  追跡できるようにしている。

===============================================================================
#>

# スクリプトのバージョン（更新日）。改修のたびにここを更新すること。
# 各PCのローカルに配置する運用のため、ログにこの値を出力することで
# 「そのPCが最新版を持っているか」をFixReorderPoint.logから後追いできるようにする。
# ※param()ブロックより前に実行文を置くとPowerShellのパースエラーになるため、
#   この代入は必ずparam()の後（Set-StrictModeの近く）に置くこと。

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$WorkbookPath,

    [Parameter(Mandatory = $true)]
    [string]$SiteName,

    [ValidateSet("Audit", "Update")]
    [string]$Mode = "Audit",

    [string]$LogPath = "",

    [string]$CorrectionEnabled = "有",

    [int]$MaxCandidates = 1000,

    # RK-10がローカルへ保存した処理用CSVの格納フォルダ
    # 旧RK-10との互換性維持のため任意。CsvFolderPath/CsvFileNameの両方が指定された場合のみCSV照合を行う。
    [string]$CsvFolderPath = "",

    # 実際の処理用CSVファイル名をそのまま渡す（例：倉庫在庫データ.csvNoSpace.csv）
    # スクリプト側では NoSpace.csv 等の文字列を付加しない。
    # 未指定時はCSV照合をスキップし、従来の発注点補正処理を継続する。
    [string]$CsvFileName = "",

    # CSV1 の「バーコード貼付」シート上の開始行。従来互換のため既定値は3。
    [int]$CsvStartRow = 3,

    # CSV1 の貼付開始セルをA1形式のまま受け取る任意引数（例: A3 / A4034 / $A$4034）。
    # 指定時は CsvStartRow より優先する。RK-10の「主CSV貼付開始セル」をそのまま渡せる。
    [string]$CsvStartCell = "",

    # 2データセット構成用の第2CSV。未指定なら従来どおり1CSVのみを照合する。
    [string]$CsvFileName2 = "",

    # CSV2 の「バーコード貼付」シート上の開始行。CsvFileName2指定時は1以上を指定する。
    [int]$CsvStartRow2 = 0,

    # CSV2 の貼付開始セルをA1形式のまま受け取る任意引数。指定時は CsvStartRow2 より優先する。
    # 副CSVがA3固定の場合は未指定のまま CsvStartRow2=3 を利用できる。
    [string]$CsvStartCell2 = "",

    # 発注点書き換え履歴CSVの出力先（省略時はワークブックと同じフォルダに自動決定）
    [string]$HistoryPath = ""
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"

# バージョン管理：改修のたびにこの値を更新する（詳細はヘッダーコメント参照）
# r10 (2026-08-21): TIMS処理用CSVと「バーコード貼付」A3:Dの事前照合を追加。
#                    ExitCode 20=不一致 / 21=CSVなし / 22=シートなし。
#                    CSVファイル名は引数で完全名を受け取り、NoSpace等を内部付加しない。
# r11 (2026-08-21): Fix multidimensional Value2 index expression by parenthesizing row index.
# r13 (2026-08-26): 品番比較で科学表記（例: 2.50E+03）と通常表記（2500）を同一品番として扱う。
# r15 (2026-08-26): CSV照合を最大2データセット対応へ拡張。CsvStartRow/CsvFileName2/CsvStartRow2を追加。
#                    引数なし時のCsvValidation=Skipped（旧RK-10互換）は維持。2CSV時は開始行で各ブロック境界を自動決定する。
# r16 (2026-08-26): CsvStartCell/CsvStartCell2を追加。A1形式（A4034、$A$4034、シート名付き）を行番号へ変換し、
#                    指定時は整数のCsvStartRow/CsvStartRow2より優先。RK-10側の文字列加工を不要化。
$ScriptVersion = "2026-08-26-r16"

$excel = $null
$workbook = $null
$barcodeSheet = $null
$startTime = Get-Date

function Write-Log {
    param(
        [string]$Level,
        [string]$Message
    )

    $line = "{0} [{1}] {2}" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss"), $Level, $Message
    Write-Host $line
    Add-Content -LiteralPath $script:LogPath -Value $line -Encoding UTF8
}

# ---------------------------------------------------------------------------
# Backupフォルダの古いバックアップを削除する。
# ・当日を含めて直近$RetentionDays日分のみ残し、それより古いものを削除する
#   （既定2日＝当日・前日を保持し、3日以上前のものを削除）。
# ・判定はファイル名ではなく実際の最終更新日時(LastWriteTime)で行う。
# ・対象はこのワークブックのバックアップ（{ファイル名}_yyyyMMdd{拡張子}）のみに
#   限定し、同じBackupフォルダを共用する他スクリプトのバックアップには触れない。
# ・削除に失敗しても本処理（発注点補正）自体は継続する（ログにWARNを残すのみ）。
# ---------------------------------------------------------------------------
function Remove-OldBackups {
    param(
        [string]$BackupFolder,
        [string]$BaseName,
        [string]$Extension,
        [int]$RetentionDays = 2
    )

    if (-not (Test-Path -LiteralPath $BackupFolder)) {
        return
    }

    $cutoff = (Get-Date).Date.AddDays(-($RetentionDays - 1))
    $pattern = "{0}_*{1}" -f $BaseName, $Extension

    Get-ChildItem -LiteralPath $BackupFolder -Filter $pattern -File | Where-Object {
        $_.LastWriteTime.Date -lt $cutoff
    } | ForEach-Object {
        try {
            Remove-Item -LiteralPath $_.FullName -Force
            Write-Log "INFO" ("OldBackupRemoved={0}" -f $_.FullName)
        } catch {
            Write-Log "WARN" ("Failed to remove old backup: Path={0} Error={1}" -f $_.FullName, $_.Exception.Message)
        }
    }
}

# ---------------------------------------------------------------------------
# 発注点書き換え履歴（ReorderPointHistory.csv）へ1バッチ分をまとめて追記する。
# ・実際に書き換えが確定した行（Updateモードで保存まで成功した候補）のみを渡すこと。
# ・ファイルが無ければヘッダー付きで新規作成、あれば追記する（Export-Csv -Append）。
# ・列は「日時／拠点／ワークブック／シート／セル／品番／旧発注点／新発注点」の8列固定。
# ---------------------------------------------------------------------------
function Write-ReorderPointHistory {
    param(
        [string]$Path,
        [System.Collections.IEnumerable]$Records
    )

    if ($null -eq $Records) {
        return
    }

    $list = @($Records)
    if ($list.Count -eq 0) {
        return
    }

    $historyFolder = Split-Path -Parent $Path
    if (-not [string]::IsNullOrWhiteSpace($historyFolder)) {
        if (-not (Test-Path -LiteralPath $historyFolder)) {
            New-Item -ItemType Directory -Path $historyFolder -Force | Out-Null
        }
    }

    # -Append と -Encoding UTF8 を指定。ファイル未作成なら自動的にヘッダー付きで新規作成される。
    # Windows PowerShell 5.1の -Encoding UTF8 はBOM付きで出力されるため、Excelでもそのまま開ける。
    $list | Export-Csv -LiteralPath $Path -Append -NoTypeInformation -Encoding UTF8

    Write-Log "INFO" ("HistoryAppended={0} File={1}" -f $list.Count, $Path)
}

function Release-Com {
    param([object]$Object)

    if ($null -ne $Object) {
        try {
            [void][System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($Object)
        }
        catch {
        }
    }
}

function Normalize-Code {
    param([object]$Value)

    if ($null -eq $Value) {
        return ""
    }

    return ([string]$Value).Trim().Trim([char]0x3000)
}

function Try-Decimal {
    param(
        [object]$Value,
        [ref]$Result
    )

    $Result.Value = [decimal]0

    if ($null -eq $Value) {
        return $false
    }

    $text = ([string]$Value).Trim()
    if ([string]::IsNullOrWhiteSpace($text)) {
        return $false
    }

    $number = [decimal]0
    if ([decimal]::TryParse($text, [ref]$number)) {
        $Result.Value = $number
        return $true
    }

    return $false
}

function Get-LastRow {
    param([object]$Sheet)

    # UsedRangeの開始行＋行数から最終行番号を求める（COMオブジェクトは必ず解放する）
    $usedRange = $null
    try {
        $usedRange = $Sheet.UsedRange
        return ([int]$usedRange.Row + [int]$usedRange.Rows.Count - 1)
    }
    finally {
        Release-Com $usedRange
    }
}

# 指定した終了コードを保持した例外を投げる。catch側でData["ExitCode"]を参照する。
function Throw-WithExitCode {
    param(
        [int]$ExitCode,
        [string]$Message
    )

    $ex = New-Object System.Exception($Message)
    $ex.Data["ExitCode"] = $ExitCode
    throw $ex
}

function Normalize-CompareText {
    param([object]$Value)

    if ($null -eq $Value) {
        return ""
    }

    return [string]$Value
}

function Format-CompareValue {
    param([object]$Value)

    if ($null -eq $Value) {
        return "<blank>"
    }

    $text = ([string]$Value).Replace("`r", "\r").Replace("`n", "\n")
    if ($text.Length -gt 120) {
        return ($text.Substring(0, 120) + "...")
    }
    if ($text.Length -eq 0) {
        return "<blank>"
    }
    return $text
}

function Test-CodeEquivalent {
    param(
        [object]$Left,
        [object]$Right
    )

    $leftText = Normalize-Code $Left
    $rightText = Normalize-Code $Right

    # まず従来どおり、品番を文字列として完全一致比較する。
    if ($leftText -ceq $rightText) {
        return $true
    }

    # 通常の数字文字列同士（例: "00123" と "123"）は同一扱いにしない。
    # CSV側などが科学表記になった場合だけ、数値として再比較する。
    $scientificPattern = '^[+-]?(?:\d+(?:\.\d*)?|\.\d+)[eE][+-]?\d+$'
    $leftScientific = ($leftText -match $scientificPattern)
    $rightScientific = ($rightText -match $scientificPattern)

    if (-not ($leftScientific -or $rightScientific)) {
        return $false
    }

    $styles = [System.Globalization.NumberStyles]::Float
    $culture = [System.Globalization.CultureInfo]::InvariantCulture
    $leftNumber = [decimal]0
    $rightNumber = [decimal]0

    $leftOk = [decimal]::TryParse($leftText, $styles, $culture, [ref]$leftNumber)
    $rightOk = [decimal]::TryParse($rightText, $styles, $culture, [ref]$rightNumber)

    if ($leftOk -and $rightOk) {
        return ($leftNumber -eq $rightNumber)
    }

    return $false
}

function Test-NumericEquivalent {
    param(
        [object]$Left,
        [object]$Right
    )

    $leftNumber = [decimal]0
    $rightNumber = [decimal]0
    $leftOk = Try-Decimal $Left ([ref]$leftNumber)
    $rightOk = Try-Decimal $Right ([ref]$rightNumber)

    if ($leftOk -and $rightOk) {
        return ($leftNumber -eq $rightNumber)
    }

    return ((Normalize-CompareText $Left) -ceq (Normalize-CompareText $Right))
}

# ---------------------------------------------------------------------------
# A1形式のセル参照から行番号のみを取り出す。
# 例: A4034 / $A$4034 / バーコード貼付!A4034 / 'バーコード貼付'!$A$4034
# 列部分は照合開始行の決定には使用しない。
# ---------------------------------------------------------------------------
function ConvertTo-RowNumberFromCellRef {
    param(
        [string]$CellRef,
        [string]$ParamLabel
    )

    if ([string]::IsNullOrWhiteSpace($CellRef)) {
        throw ("Cell reference is empty for {0}." -f $ParamLabel)
    }

    $text = $CellRef.Trim()
    $bangIndex = $text.LastIndexOf('!')
    if ($bangIndex -ge 0) {
        $text = $text.Substring($bangIndex + 1)
    }
    $text = $text.Replace('$', '')

    if ($text -match '^[A-Za-z]+([1-9][0-9]*)$') {
        return [int]$Matches[1]
    }

    throw ("Invalid cell reference for {0}: '{1}'" -f $ParamLabel, $CellRef)
}

# ---------------------------------------------------------------------------
# TIMS処理用CSVの先頭4列と、在庫帳「バーコード貼付」A3:D最終行を照合する。
# CSV列: 1=品番 / 2=品名 / 3=在庫 / 4=発注点
# Excel : A=品番 / B=品名 / C=在庫 / D=発注点
# 不一致時はExcelを書き換えず、ExitCode=20として上位へ返す。
# ---------------------------------------------------------------------------
function Test-BarcodePasteAgainstCsv {
    param(
        [string]$CsvPath,
        [object]$Sheet,
        [int]$ExcelStartRow = 3,
        [int]$ExcelEndRow = 0,
        [string]$ValidationName = "CSV1",
        [int]$MaxMismatchLogs = 10
    )

    if (-not (Test-Path -LiteralPath $CsvPath -PathType Leaf)) {
        Throw-WithExitCode 21 ("CSV file not found: {0}" -f $CsvPath)
    }

    if ($ExcelStartRow -lt 1) {
        throw ("Invalid ExcelStartRow: {0}" -f $ExcelStartRow)
    }
    if (($ExcelEndRow -gt 0) -and ($ExcelEndRow -lt $ExcelStartRow)) {
        throw ("Invalid Excel row range: Start={0} End={1}" -f $ExcelStartRow, $ExcelEndRow)
    }

    Write-Log "INFO" ("BarcodeValidationStart Label={0} CSV={1} ExcelStartRow={2} ExcelEndRow={3}" -f
        $ValidationName, $CsvPath, $ExcelStartRow, $ExcelEndRow)

    # ヘッダーなしCSVの先頭行もデータとして読む。照合対象は先頭4列のみ。
    $headers = 1..32 | ForEach-Object { "C{0}" -f $_ }
    $csvRows = @(Import-Csv -LiteralPath $CsvPath -Header $headers -Encoding Default)

    # シートA列の最終データ行を取得する。
    $lastCell = $null
    try {
        $lastCell = $Sheet.Cells.Item($Sheet.Rows.Count, 1).End(-4162)  # xlUp
        $sheetLastRow = [int]$lastCell.Row
    }
    finally {
        Release-Com $lastCell
    }

    # 2CSV時は、もう一方の開始行直前をこのブロックの終端にする。
    # ブロック内のA列最終非空白行から実データ件数を求め、前回データの残骸も件数不一致として検出する。
    $scanEndRow = $sheetLastRow
    if ($ExcelEndRow -gt 0) {
        $scanEndRow = [Math]::Min($ExcelEndRow, $sheetLastRow)
    }

    $blockLastRow = $ExcelStartRow - 1
    if ($scanEndRow -ge $ExcelStartRow) {
        $codeStartCell = $null
        $codeEndCell = $null
        $codeRange = $null
        $codeValues = $null
        try {
            $codeStartCell = $Sheet.Cells.Item($ExcelStartRow, 1)
            $codeEndCell = $Sheet.Cells.Item($scanEndRow, 1)
            $codeRange = $Sheet.Range($codeStartCell, $codeEndCell)
            $codeValues = $codeRange.Value2

            for ($row = $scanEndRow; $row -ge $ExcelStartRow; $row--) {
                $offset = $row - $ExcelStartRow + 1
                if ($scanEndRow -eq $ExcelStartRow) {
                    $codeValue = $codeValues
                }
                else {
                    $codeValue = $codeValues[$offset, 1]
                }

                if (-not [string]::IsNullOrWhiteSpace((Normalize-Code $codeValue))) {
                    $blockLastRow = $row
                    break
                }
            }
        }
        finally {
            Release-Com $codeRange
            Release-Com $codeEndCell
            Release-Com $codeStartCell
        }
    }

    $excelRows = 0
    if ($blockLastRow -ge $ExcelStartRow) {
        $excelRows = $blockLastRow - $ExcelStartRow + 1
    }

    Write-Log "INFO" ("BarcodeValidationRows Label={0} CSV={1} Excel={2} ExcelStartRow={3} ExcelLastDataRow={4}" -f
        $ValidationName, $csvRows.Count, $excelRows, $ExcelStartRow, $blockLastRow)

    $mismatchCount = 0
    $loggedMismatchCount = 0
    $compareRows = [Math]::Min($csvRows.Count, $excelRows)
    $columnNames = @("Code", "Name", "Stock", "ReorderPoint")

    # Excel COMを1セルずつ読まず、対象A:Dを一括取得する。
    $excelValues = $null
    if ($compareRows -gt 0) {
        $startCell = $null
        $endCell = $null
        $dataRange = $null
        try {
            $startCell = $Sheet.Cells.Item($ExcelStartRow, 1)
            $endCell = $Sheet.Cells.Item($ExcelStartRow + $compareRows - 1, 4)
            $dataRange = $Sheet.Range($startCell, $endCell)
            $excelValues = $dataRange.Value2
        }
        finally {
            Release-Com $dataRange
            Release-Com $endCell
            Release-Com $startCell
        }
    }

    for ($i = 0; $i -lt $compareRows; $i++) {
        $excelRow = $ExcelStartRow + $i
        $csvValues = @(
            $csvRows[$i].C1,
            $csvRows[$i].C2,
            $csvRows[$i].C3,
            $csvRows[$i].C4
        )

        for ($col = 1; $col -le 4; $col++) {
            $excelValue = $excelValues[($i + 1), $col]
            $csvValue = $csvValues[$col - 1]

            if ($col -eq 1) {
                $same = Test-CodeEquivalent $csvValue $excelValue
            }
            elseif (($col -eq 3) -or ($col -eq 4)) {
                $same = Test-NumericEquivalent $csvValue $excelValue
            }
            else {
                $same = ((Normalize-CompareText $csvValue) -ceq (Normalize-CompareText $excelValue))
            }

            if (-not $same) {
                $mismatchCount++
                if ($loggedMismatchCount -lt $MaxMismatchLogs) {
                    Write-Log "ERROR" (
                        "BarcodeMismatch Label={0} CsvRow={1} ExcelRow={2} Column={3} CSV=[{4}] Excel=[{5}]" -f
                        $ValidationName,
                        ($i + 1),
                        $excelRow,
                        $columnNames[$col - 1],
                        (Format-CompareValue $csvValue),
                        (Format-CompareValue $excelValue)
                    )
                    $loggedMismatchCount++
                }
            }
        }
    }

    if ($csvRows.Count -ne $excelRows) {
        Write-Log "ERROR" ("BarcodeRowCountMismatch Label={0} CSV={1} Excel={2}" -f $ValidationName, $csvRows.Count, $excelRows)
        $mismatchCount++
    }

    if ($mismatchCount -gt 0) {
        Write-Log "ERROR" ("BarcodeValidation=NG Label={0} Mismatches={1}" -f $ValidationName, $mismatchCount)
        Throw-WithExitCode 20 (
            "CSV and barcode sheet do not match. Label={0} CSVRows={1} ExcelRows={2} Mismatches={3}" -f
            $ValidationName, $csvRows.Count, $excelRows, $mismatchCount
        )
    }

    Write-Log "INFO" ("BarcodeValidation=OK Label={0} ComparedRows={1}" -f $ValidationName, $csvRows.Count)
}

try {
    if (-not (Test-Path -LiteralPath $WorkbookPath -PathType Leaf)) {
        throw "Workbook not found: $WorkbookPath"
    }

    $WorkbookPath = (Resolve-Path -LiteralPath $WorkbookPath).ProviderPath
    $folder = Split-Path -Parent $WorkbookPath

    if ([string]::IsNullOrWhiteSpace($LogPath)) {
        $LogPath = Join-Path $folder "FixReorderPoint.log"
    }

    $script:LogPath = $LogPath

    $logFolder = Split-Path -Parent $LogPath
    if (-not [string]::IsNullOrWhiteSpace($logFolder)) {
        if (-not (Test-Path -LiteralPath $logFolder)) {
            New-Item -ItemType Directory -Path $logFolder -Force | Out-Null
        }
    }

    # 履歴CSVの出力先を確定（未指定ならワークブックと同じフォルダの下の
    # 「作業用CSV」サブフォルダを既定とする。成果物のExcelと並んで見えると
    # 見づらいための対応。NAS環境ではWindowsの隠しファイル属性が保存されない
    # 場合があるため、隠し属性ではなくサブフォルダへの退避で対応する）
    if ([string]::IsNullOrWhiteSpace($HistoryPath)) {
        $HistoryPath = Join-Path (Join-Path $folder "作業用CSV") "ReorderPointHistory.csv"
    }

    # A1形式の開始セルが指定されていれば、整数開始行より優先して解決する。
    # CSV引数自体が無い旧RK-10互換モードでは、比較を完全スキップするためここでは変換しない。
    $CsvPrimaryAnySpecifiedForCellResolve = (
        -not [string]::IsNullOrWhiteSpace($CsvFolderPath) -or
        -not [string]::IsNullOrWhiteSpace($CsvFileName)
    )
    if ($CsvPrimaryAnySpecifiedForCellResolve) {
        if (-not [string]::IsNullOrWhiteSpace($CsvStartCell)) {
            $CsvStartRow = ConvertTo-RowNumberFromCellRef -CellRef $CsvStartCell -ParamLabel "CsvStartCell"
        }
        if (-not [string]::IsNullOrWhiteSpace($CsvStartCell2)) {
            $CsvStartRow2 = ConvertTo-RowNumberFromCellRef -CellRef $CsvStartCell2 -ParamLabel "CsvStartCell2"
        }
    }

    Add-Content -LiteralPath $LogPath -Value ("=" * 80) -Encoding UTF8
    Write-Log "INFO" "Start"
    Write-Log "INFO" ("ScriptVersion={0}" -f $ScriptVersion)
    Write-Log "INFO" ("Site={0}" -f $SiteName)
    Write-Log "INFO" ("Mode={0}" -f $Mode)
    Write-Log "INFO" ("Workbook={0}" -f $WorkbookPath)
    Write-Log "INFO" ("CorrectionEnabled={0}" -f $CorrectionEnabled)
    Write-Log "INFO" ("HistoryPath={0}" -f $HistoryPath)
    if ($CsvPrimaryAnySpecifiedForCellResolve -and -not [string]::IsNullOrWhiteSpace($CsvStartCell)) {
        Write-Log "INFO" ("CsvStartCell={0} => CsvStartRow={1}" -f $CsvStartCell, $CsvStartRow)
    }
    if ($CsvPrimaryAnySpecifiedForCellResolve -and -not [string]::IsNullOrWhiteSpace($CsvStartCell2)) {
        Write-Log "INFO" ("CsvStartCell2={0} => CsvStartRow2={1}" -f $CsvStartCell2, $CsvStartRow2)
    }

    # CSV照合は新RK-10向けの追加安全機能。
    # CsvFolderPath/CsvFileNameが両方未指定なら、旧RK-10互換として照合を完全にスキップする。
    # CsvFileName2は任意。指定時だけ第2CSVを照合する。
    $CsvPrimaryAnySpecified = (
        -not [string]::IsNullOrWhiteSpace($CsvFolderPath) -or
        -not [string]::IsNullOrWhiteSpace($CsvFileName)
    )
    $CsvValidationEnabled = (
        -not [string]::IsNullOrWhiteSpace($CsvFolderPath) -and
        -not [string]::IsNullOrWhiteSpace($CsvFileName)
    )
    $CsvValidation2Enabled = (-not [string]::IsNullOrWhiteSpace($CsvFileName2))

    if (-not $CsvPrimaryAnySpecified -and -not $CsvValidation2Enabled) {
        $CsvPath = $null
        $CsvPath2 = $null
        Write-Log "WARN" (
            "CsvValidation=Skipped: CsvFolderPath/CsvFileName未指定。旧RK-10互換モードで処理を継続します。"
        )
    }
    else {
        if (-not $CsvValidationEnabled) {
            throw "Csv validation parameter error: CsvFolderPath and CsvFileName must both be specified."
        }
        if ($CsvStartRow -lt 1) {
            throw ("Csv validation parameter error: CsvStartRow must be 1 or greater. Value={0}" -f $CsvStartRow)
        }
        if ($CsvValidation2Enabled -and ($CsvStartRow2 -lt 1)) {
            throw ("Csv validation parameter error: CsvStartRow2 must be 1 or greater when CsvFileName2 is specified. Value={0}" -f $CsvStartRow2)
        }
        if ($CsvValidation2Enabled -and ($CsvStartRow2 -eq $CsvStartRow)) {
            throw ("Csv validation parameter error: CsvStartRow and CsvStartRow2 must be different. Row={0}" -f $CsvStartRow)
        }

        $CsvPath = Join-Path $CsvFolderPath $CsvFileName
        $CsvPath2 = $null
        $dataSetCount = 1
        if ($CsvValidation2Enabled) { $dataSetCount = 2 }

        Write-Log "INFO" ("CsvValidation=Enabled DataSets={0}" -f $dataSetCount)
        Write-Log "INFO" ("CsvFolderPath={0}" -f $CsvFolderPath)
        Write-Log "INFO" ("CsvFileName={0}" -f $CsvFileName)
        Write-Log "INFO" ("CsvStartRow={0}" -f $CsvStartRow)
        Write-Log "INFO" ("CsvPath={0}" -f $CsvPath)

        if (-not (Test-Path -LiteralPath $CsvPath -PathType Leaf)) {
            Throw-WithExitCode 21 ("CSV file not found: {0}" -f $CsvPath)
        }

        if ($CsvValidation2Enabled) {
            $CsvPath2 = Join-Path $CsvFolderPath $CsvFileName2
            Write-Log "INFO" ("CsvFileName2={0}" -f $CsvFileName2)
            Write-Log "INFO" ("CsvStartRow2={0}" -f $CsvStartRow2)
            Write-Log "INFO" ("CsvPath2={0}" -f $CsvPath2)

            if (-not (Test-Path -LiteralPath $CsvPath2 -PathType Leaf)) {
                Throw-WithExitCode 21 ("CSV file not found: {0}" -f $CsvPath2)
            }
        }
    }

    $excel = New-Object -ComObject Excel.Application
    $excel.Visible = $false
    $excel.DisplayAlerts = $false
    $excel.ScreenUpdating = $false
    $excel.EnableEvents = $false

    $workbook = $excel.Workbooks.Open($WorkbookPath, 0, $false)

    if (($Mode -eq "Update") -and [bool]$workbook.ReadOnly) {
        throw "Workbook opened read-only. Close Excel and retry."
    }

    try {
        $barcodeSheet = $workbook.Worksheets.Item("バーコード貼付")
    }
    catch {
        Throw-WithExitCode 22 "Sheet not found: バーコード貼付"
    }

    # 発注点補正より先に、TIMS処理用CSVとバーコード貼付シートの一致を検証する。
    # CsvFolderPath/CsvFileNameの両方が指定された場合のみ実施する。
    # この段階ではExcelを書き換えない。NGならExitCode=20でRK-10へ返す。
    if ($CsvValidationEnabled) {
        $CsvEndRow = 0
        $CsvEndRow2 = 0

        if ($CsvValidation2Enabled) {
            # 開始行の早い方のブロックは、遅い方の開始行直前までを検査範囲とする。
            # 主/副CSVの順序には依存しないため、「主=A4034、副=A3」の設定にも対応する。
            if ($CsvStartRow -lt $CsvStartRow2) {
                $CsvEndRow = $CsvStartRow2 - 1
            }
            else {
                $CsvEndRow2 = $CsvStartRow - 1
            }
        }

        Test-BarcodePasteAgainstCsv `
            -CsvPath $CsvPath `
            -Sheet $barcodeSheet `
            -ExcelStartRow $CsvStartRow `
            -ExcelEndRow $CsvEndRow `
            -ValidationName "CSV1" `
            -MaxMismatchLogs 10

        if ($CsvValidation2Enabled) {
            Test-BarcodePasteAgainstCsv `
                -CsvPath $CsvPath2 `
                -Sheet $barcodeSheet `
                -ExcelStartRow $CsvStartRow2 `
                -ExcelEndRow $CsvEndRow2 `
                -ValidationName "CSV2" `
                -MaxMismatchLogs 10
        }
    }

    if ($CorrectionEnabled -ne "有") {
        if ($CsvValidationEnabled) {
            Write-Log "INFO" (
                "CorrectionSkipped: Site={0} CorrectionEnabled={1}（バーコード貼付検証はOK、発注点補正のみスキップ）" -f
                $SiteName, $CorrectionEnabled
            )
        }
        else {
            Write-Log "INFO" (
                "CorrectionSkipped: Site={0} CorrectionEnabled={1}（CSV照合は未指定のためスキップ、発注点補正もスキップ）" -f
                $SiteName, $CorrectionEnabled
            )
        }

        $workbook.Close($false)
        $excel.Quit()

        Release-Com $barcodeSheet
        Release-Com $workbook
        Release-Com $excel

        [GC]::Collect()
        [GC]::WaitForPendingFinalizers()

        Add-Content -LiteralPath $LogPath -Value "" -Encoding UTF8
        exit 0
    }

    # 品番→在庫数の対応表。品番が重複している場合は在庫参照として使えないため
    # 後段でduplicatesにまとめて除外する（誤補正防止）。
    $inventory = New-Object "System.Collections.Generic.Dictionary[string,decimal]" ([System.StringComparer]::OrdinalIgnoreCase)
    $duplicates = New-Object "System.Collections.Generic.HashSet[string]" ([System.StringComparer]::OrdinalIgnoreCase)

    $barcodeLastRow = Get-LastRow $barcodeSheet

    # --- バーコード貼付シートを1行ずつ読み、品番(A列)と在庫数(C列)を対応表へ格納 ---
    for ($row = 2; $row -le $barcodeLastRow; $row++) {
        $codeCell = $null
        $stockCell = $null

        try {
            $codeCell = $barcodeSheet.Cells.Item($row, 1)
            $stockCell = $barcodeSheet.Cells.Item($row, 3)

            $code = Normalize-Code $codeCell.Value2
            if ([string]::IsNullOrWhiteSpace($code)) {
                continue
            }

            $stock = [decimal]0
            if (-not (Try-Decimal $stockCell.Value2 ([ref]$stock))) {
                continue
            }

            if ($inventory.ContainsKey($code)) {
                [void]$duplicates.Add($code)
            }
            else {
                $inventory.Add($code, $stock)
            }
        }
        finally {
            Release-Com $stockCell
            Release-Com $codeCell
        }
    }

    foreach ($code in $duplicates) {
        [void]$inventory.Remove($code)
        Write-Log "WARN" ("Duplicate product code skipped: {0}" -f $code)
    }

    $candidates = New-Object System.Collections.ArrayList
    $sheetNames = New-Object System.Collections.ArrayList

    # --- シート名が数字のみの拠点別シートを走査し、発注点=0 かつ 在庫>0 の候補を抽出 ---
    foreach ($sheet in $workbook.Worksheets) {
        try {
            $sheetName = [string]$sheet.Name
            if ($sheetName -notmatch "^\d+$") {
                continue
            }

            [void]$sheetNames.Add($sheetName)
            $lastRow = Get-LastRow $sheet

            for ($row = 2; $row -le $lastRow; $row++) {
                $codeCell = $null
                $pointCell = $null

                try {
                    $codeCell = $sheet.Cells.Item($row, 2)
                    $pointCell = $sheet.Cells.Item($row, 5)

                    $code = Normalize-Code $codeCell.Value2
                    if ([string]::IsNullOrWhiteSpace($code)) {
                        continue
                    }

                    $point = [decimal]0
                    if (-not (Try-Decimal $pointCell.Value2 ([ref]$point))) {
                        # 発注点が数値として解釈できない行（空欄・"#N/A"等）は候補から除外。
                        # ※この分岐に入った件数はログに出力されないため見逃しに注意（既知の課題）。
                        continue
                    }

                    if ($point -ne 0) {
                        continue
                    }

                    if (-not $inventory.ContainsKey($code)) {
                        continue
                    }

                    $stock = [decimal]$inventory[$code]
                    if ($stock -le 0) {
                        continue
                    }

                    $candidate = [PSCustomObject]@{
                        SheetName = $sheetName
                        Row       = $row
                        Cell      = ("E{0}" -f $row)
                        Code      = $code
                        OldValue  = $point
                        NewValue  = $stock
                    }

                    [void]$candidates.Add($candidate)
                }
                finally {
                    Release-Com $pointCell
                    Release-Com $codeCell
                }
            }
        }
        finally {
            Release-Com $sheet
        }
    }

    Write-Log "INFO" ("NumericSheets={0}" -f (($sheetNames | Sort-Object {[int]$_}) -join ","))
    Write-Log "INFO" ("Candidates={0}" -f $candidates.Count)

    foreach ($item in $candidates) {
        Write-Log "INFO" (
            "Candidate Sheet={0} Cell={1} Code={2} {3}->{4}" -f
            $item.SheetName,
            $item.Cell,
            $item.Code,
            $item.OldValue,
            $item.NewValue
        )
    }

    if ($Mode -eq "Update") {
        if ($candidates.Count -gt $MaxCandidates) {
            throw "Too many candidates: $($candidates.Count) (MaxCandidates=$MaxCandidates)"
        }

        if ($candidates.Count -gt 0) {
            $backupFolder = Join-Path $folder "Backup"
            if (-not (Test-Path -LiteralPath $backupFolder)) {
                New-Item -ItemType Directory -Path $backupFolder -Force | Out-Null
            }

            $baseName = [System.IO.Path]::GetFileNameWithoutExtension($WorkbookPath)
            $extension = [System.IO.Path]::GetExtension($WorkbookPath)
            # ファイル名は日付のみとし、同日中の複数回実行では上書きする
            # （時刻まで含めた退避は不要との判断。世代管理は日次で十分なため）
            $backupName = "{0}_{1}{2}" -f $baseName, (Get-Date -Format "yyyyMMdd"), $extension
            $backupPath = Join-Path $backupFolder $backupName

            Copy-Item -LiteralPath $WorkbookPath -Destination $backupPath -Force
            Write-Log "INFO" ("Backup={0}" -f $backupPath)

            # 直近2日分（当日・前日）のみ残し、3日以上前のバックアップを削除する
            Remove-OldBackups -BackupFolder $backupFolder -BaseName $baseName -Extension $extension -RetentionDays 2

            # 実際に書き換えた候補だけを履歴CSV用に貯めておく（Save成功後にまとめて追記する）
            $historyRecords = New-Object System.Collections.ArrayList
            $updateTimestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"

            foreach ($item in $candidates) {
                $targetSheet = $null
                $targetCell = $null

                try {
                    $targetSheet = $workbook.Worksheets.Item($item.SheetName)
                    $targetCell = $targetSheet.Cells.Item($item.Row, 5)
                    $targetCell.Value2 = [double]$item.NewValue

                    Write-Log "UPDATE" (
                        "Updated Sheet={0} Cell={1} Code={2} {3}->{4}" -f
                        $item.SheetName,
                        $item.Cell,
                        $item.Code,
                        $item.OldValue,
                        $item.NewValue
                    )

                    [void]$historyRecords.Add([PSCustomObject]@{
                        "日時"     = $updateTimestamp
                        "拠点"     = $SiteName
                        "ワークブック" = $WorkbookPath
                        "シート"   = $item.SheetName
                        "セル"     = $item.Cell
                        "品番"     = $item.Code
                        "旧発注点" = $item.OldValue
                        "新発注点" = $item.NewValue
                    })
                }
                finally {
                    Release-Com $targetCell
                    Release-Com $targetSheet
                }
            }

            $workbook.Save()
            Write-Log "INFO" ("Saved. Updated={0}" -f $candidates.Count)

            # ワークブックの保存に成功した後で初めて履歴CSVへ追記する
            # （保存前に書き込むと、Save失敗時に「実際には反映されていない書き換え」が
            #   履歴に残ってしまうため）
            Write-ReorderPointHistory -Path $HistoryPath -Records $historyRecords
        }
        else {
            Write-Log "INFO" "No update required."
        }
    }
    else {
        Write-Log "INFO" "Audit completed. Workbook was not changed."
    }

    $elapsed = (Get-Date) - $startTime
    Write-Log "INFO" ("ElapsedSeconds={0:N2}" -f $elapsed.TotalSeconds)

    $workbook.Close($false)
    $excel.Quit()

    Release-Com $barcodeSheet
    Release-Com $workbook
    Release-Com $excel

    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()

    Add-Content -LiteralPath $LogPath -Value "" -Encoding UTF8
    exit 0
}
catch {
    $errorText = $_.Exception.Message
    $exitCode = 1

    try {
        if ($null -ne $_.Exception.Data["ExitCode"]) {
            $exitCode = [int]$_.Exception.Data["ExitCode"]
        }
    }
    catch {
        $exitCode = 1
    }

    try {
        if (-not [string]::IsNullOrWhiteSpace($script:LogPath)) {
            Write-Log "ERROR" $errorText
        }
        else {
            Write-Host ("ERROR: {0}" -f $errorText)
        }
    }
    catch {
        Write-Host ("ERROR: {0}" -f $errorText)
    }

    # RK-10が「実行結果からエラー出力を取得」で内容を取得できるよう、
    # ログファイルへの記録とは別に標準エラー出力へも1行出す。
    try {
        [Console]::Error.WriteLine($errorText)
    }
    catch {
        # 標準エラー出力に失敗しても、本来の終了コード返却を妨げない。
    }

    if ($null -ne $workbook) {
        try {
            $workbook.Close($false)
        }
        catch {
        }
    }

    if ($null -ne $excel) {
        try {
            $excel.Quit()
        }
        catch {
        }
    }

    Release-Com $barcodeSheet
    Release-Com $workbook
    Release-Com $excel

    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()

    exit $exitCode
}
