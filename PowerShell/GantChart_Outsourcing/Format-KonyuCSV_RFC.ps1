# ============================================================
# Format-KonyuCSV.ps1
# 購買部門CSVを外注課納期管理CSVの形式に整形する
#
# 処理内容:
#   1. 日付列: "yyyy/M/d H:mm" → "yy-MM-dd" に変換
#   2. 文字列セル: 前後スペース・全角スペースをトリム
#   3. 非RFC入力で未クォートのカンマにより列ずれした行だけ補正
#   4. 全フィールドをダブルクォートしてRFC 4180形式で出力
#
# 使用方法:
#   .\Format-KonyuCSV.ps1 -InputPath "外注課納期管理_購買部門.csv" -OutputPath "外注課納期管理_整形済み.csv"
# ============================================================

param(
    [Parameter(Mandatory=$true)]
    [string]$InputPath,

    [Parameter(Mandatory=$false)]
    [string]$OutputPath = ""
)

# 出力パスが未指定の場合は入力ファイル名に "_整形済み" を付加
if ($OutputPath -eq "") {
    $dir  = [System.IO.Path]::GetDirectoryName($InputPath)
    $base = [System.IO.Path]::GetFileNameWithoutExtension($InputPath)
    $ext  = [System.IO.Path]::GetExtension($InputPath)
    $OutputPath = if ($dir) { Join-Path $dir "${base}_整形済み${ext}" } else { "${base}_整形済み${ext}" }
}

# 日付列の定義（元スクリプトと同じ）
$DateColumns = @(
    "手配日",
    "開始日",
    "納期",
    "計画開始日",
    "計画納期",
    "登録日",
    "修正日",
    "登録日時",
    "更新日時",
    "手配納期"
)

# 相対パスを絶対パスに解決（元スクリプトと同じ）
$InputPath  = [System.IO.Path]::GetFullPath((Join-Path (Get-Location).Path $InputPath))
$OutputPath = [System.IO.Path]::GetFullPath((Join-Path (Get-Location).Path $OutputPath))

Write-Host "入力ファイル : $InputPath"
Write-Host "出力ファイル : $OutputPath"
Write-Host ""

# ---- 読み込み (Shift-JIS / CP932) ----
$encoding = [System.Text.Encoding]::GetEncoding("shift_jis")
$rawLines = [System.IO.File]::ReadAllLines($InputPath, $encoding)

if ($rawLines.Count -lt 2) {
    Write-Error "CSVにデータ行がありません。"
    exit 1
}

# カンマ区切り + RFC形式のダブルクォートを解釈する1行パーサー
function Parse-CsvLine([string]$line) {
    $fields = New-Object 'System.Collections.Generic.List[string]'
    $inQuote = $false
    $current = New-Object System.Text.StringBuilder

    for ($i = 0; $i -lt $line.Length; $i++) {
        $ch = $line[$i]

        if ($ch -eq '"') {
            if ($inQuote -and $i + 1 -lt $line.Length -and $line[$i + 1] -eq '"') {
                [void]$current.Append('"')
                $i++
            }
            else {
                $inQuote = -not $inQuote
            }
        }
        elseif ($ch -eq ',' -and -not $inQuote) {
            $fields.Add($current.ToString())
            [void]$current.Clear()
        }
        else {
            [void]$current.Append($ch)
        }
    }

    if ($inQuote) {
        throw "閉じていないダブルクォートがあります。"
    }

    $fields.Add($current.ToString())
    return $fields.ToArray()
}

# RFC 4180用フィールド出力
# 全項目をダブルクォートし、値中の " は "" にエスケープする。
function ConvertTo-RfcCsvField([AllowEmptyString()][string]$value) {
    if ($null -eq $value) { $value = "" }
    return '"' + $value.Replace('"', '""') + '"'
}

# 非RFC入力で未クォートのカンマが混入して列数が増えた場合のみ補正する。
# 現行CSVで自由記述にカンマが入り得る「商品名」と「商品情報」に限定する。
function Repair-ColumnShift {
    param(
        [string[]]$Fields,
        [string[]]$Headers,
        [int]$LineNumber
    )

    $expectedCount = $Headers.Count
    if ($Fields.Count -eq $expectedCount) {
        return $Fields
    }

    if ($Fields.Count -lt $expectedCount) {
        throw "行 $LineNumber : 列数不足です。期待=$expectedCount、実際=$($Fields.Count)"
    }

    $productNameIndex = [Array]::IndexOf($Headers, "商品名")
    $instructionIndex = [Array]::IndexOf($Headers, "指示先番号")
    $productInfoIndex = [Array]::IndexOf($Headers, "商品情報")
    $dueDateIndex     = [Array]::IndexOf($Headers, "手配納期")

    if ($productNameIndex -lt 0 -or $instructionIndex -lt 0 -or
        $productInfoIndex -lt 0 -or $dueDateIndex -lt 0) {
        throw "行 $LineNumber : 補正に必要なヘッダーを特定できません。"
    }

    # まず商品名の直後に来る「指示先番号」を探す。
    # 指示先番号は現行データでは数値、続く在庫場所・手配月度を使って位置を確定する。
    $actualInstructionIndex = -1
    for ($i = $instructionIndex; $i -lt [Math]::Min($Fields.Count - 2, $instructionIndex + 10); $i++) {
        $v0 = $Fields[$i].Trim()
        $v1 = $Fields[$i + 1].Trim()
        $v2 = $Fields[$i + 2].Trim()

        if ($v0 -match '^\d+$' -and
            $v1 -match '^[A-Za-z0-9]+$' -and
            $v2 -match '^\d{6}$') {
            $actualInstructionIndex = $i
            break
        }
    }

    if ($actualInstructionIndex -lt 0) {
        throw "行 $LineNumber : 商品名後の指示先番号を特定できません。"
    }

    $fixed = New-Object 'System.Collections.Generic.List[string]'

    # 商品名より前はそのまま
    for ($i = 0; $i -lt $productNameIndex; $i++) {
        $fixed.Add($Fields[$i])
    }

    # 商品名に紛れ込んだ未クォートカンマ分を再結合
    $fixed.Add(($Fields[$productNameIndex..($actualInstructionIndex - 1)] -join ','))

    # 指示先番号以降をそのまま追加
    for ($i = $actualInstructionIndex; $i -lt $Fields.Count; $i++) {
        $fixed.Add($Fields[$i])
    }

    # まだ列数が多い場合は、残りの余剰を商品情報内の未クォートカンマとみなして再結合
    if ($fixed.Count -gt $expectedCount) {
        $extraCount = $fixed.Count - $expectedCount
        $mergeEnd = $productInfoIndex + $extraCount

        if ($mergeEnd -ge $fixed.Count - 1) {
            throw "行 $LineNumber : 商品情報の補正範囲が不正です。"
        }

        $repaired = New-Object 'System.Collections.Generic.List[string]'

        for ($i = 0; $i -lt $productInfoIndex; $i++) {
            $repaired.Add($fixed[$i])
        }

        $repaired.Add(($fixed[$productInfoIndex..$mergeEnd] -join ','))

        for ($i = $mergeEnd + 1; $i -lt $fixed.Count; $i++) {
            $repaired.Add($fixed[$i])
        }

        $fixed = $repaired
    }

    if ($fixed.Count -ne $expectedCount) {
        throw "行 $LineNumber : 補正後も列数が一致しません。期待=$expectedCount、実際=$($fixed.Count)"
    }

    return $fixed.ToArray()
}

$headers = Parse-CsvLine $rawLines[0]
$expectedColumnCount = $headers.Count

# 日付列のインデックスをマッピング（元スクリプトと同じ）
$dateIndexes = @{}
for ($i = 0; $i -lt $headers.Count; $i++) {
    $colName = $headers[$i].Trim()
    if ($DateColumns -contains $colName) {
        $dateIndexes[$i] = $colName
    }
}

Write-Host "ヘッダー列数 : $expectedColumnCount 列"
Write-Host "検出した日付列 ($($dateIndexes.Count) 列):"
$dateIndexes.Values | Sort-Object | ForEach-Object { Write-Host "  - $_" }
Write-Host ""

# ---- 変換処理 ----
$outputLines = New-Object 'System.Collections.Generic.List[string]'

# ヘッダーもRFC形式で出力
$outputLines.Add((($headers | ForEach-Object { ConvertTo-RfcCsvField $_ }) -join ','))

$processed = 0
$repairedRows = 0
$dateConverted = 0
$trimmed = 0

for ($r = 1; $r -lt $rawLines.Count; $r++) {
    $line = $rawLines[$r]
    if ([string]::IsNullOrWhiteSpace($line)) { continue }

    try {
        $fields = Parse-CsvLine $line

        if ($fields.Count -ne $expectedColumnCount) {
            $beforeCount = $fields.Count
            $fields = Repair-ColumnShift -Fields $fields -Headers $headers -LineNumber ($r + 1)
            $repairedRows++
            Write-Host "列ずれ補正     : 行 $($r + 1)  $beforeCount 列 → $($fields.Count) 列"
        }

        for ($i = 0; $i -lt $fields.Count; $i++) {
            $val = $fields[$i]

            # (1) 日付変換（元スクリプトと同じ）
            if ($dateIndexes.ContainsKey($i) -and $val -ne "") {
                $parsed = [datetime]::MinValue
                $formats = @("yyyy/M/d H:mm", "yyyy/M/d H:m", "yyyy/MM/dd H:mm", "yyyy/M/d")
                $ok = $false

                foreach ($fmt in $formats) {
                    if ([datetime]::TryParseExact(
                            $val.Trim(),
                            $fmt,
                            [System.Globalization.CultureInfo]::InvariantCulture,
                            [System.Globalization.DateTimeStyles]::None,
                            [ref]$parsed)) {
                        $ok = $true
                        break
                    }
                }

                if ($ok) {
                    $newVal = $parsed.ToString("yy-MM-dd")
                    if ($newVal -ne $val) {
                        $fields[$i] = $newVal
                        $dateConverted++
                    }
                }
                continue  # 日付列はトリムをスキップ（元スクリプトと同じ）
            }

            # (2) 文字列トリム（元スクリプトと同じ）
            $trimmedVal = $val.Trim().Trim([char]0x3000)
            if ($trimmedVal -ne $val) {
                $fields[$i] = $trimmedVal
                $trimmed++
            }
        }

        if ($fields.Count -ne $expectedColumnCount) {
            throw "行 $($r + 1) : 出力直前の列数不一致。期待=$expectedColumnCount、実際=$($fields.Count)"
        }

        # (3) RFC 4180準拠: 全項目クォート + 内部ダブルクォートを二重化
        $rebuilt = ($fields | ForEach-Object { ConvertTo-RfcCsvField $_ }) -join ','
        $outputLines.Add($rebuilt)
        $processed++
    }
    catch {
        Write-Error $_.Exception.Message
        exit 2
    }
}

# ---- 書き出し (Shift-JIS / CP932、BOMなし) ----
# Windows PowerShell 5.1上では WriteAllLines によりCRLFで出力される。
[System.IO.File]::WriteAllLines($OutputPath, $outputLines, $encoding)

Write-Host "完了"
Write-Host "  ヘッダー列数 : $expectedColumnCount 列"
Write-Host "  処理行数     : $processed 行"
Write-Host "  列ずれ補正   : $repairedRows 行"
Write-Host "  日付変換     : $dateConverted セル"
Write-Host "  スペーストリム: $trimmed セル"
Write-Host "  出力先       : $OutputPath"
