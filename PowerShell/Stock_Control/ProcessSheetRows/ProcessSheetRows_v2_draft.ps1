<#
===============================================================================
ProcessSheetRows.ps1  (v2 draft - 長期滞留品自動処理を追加)

概要
  従来の4パターン（欠品新規検知/解消、発注点割れ新規検知/解消）に加え、
  荘田室長が手動で行っていた「長期滞留品の自動処理」5項目のうち、
  自動化未実装だった3項目（ルール①③⑤）を追加する。
  （項番④＝発注点0&入庫ありでE=Dは、既存のFixReorderPoint.ps1で対応済みのため
  本スクリプトでは扱わない）

  ルール①（発注点割れ長期化）
    条件：G列=1 かつ F列（発注点割れ検知日）が拠点別リセット日数
          （既定90日、船舶(機械部)の一部シートのみ365日）以上前
    処理：E列を「今のE － 今のD」に書き換える
    ※ただしルール③の条件も同時に満たす場合はルール③を優先する

  ルール③（発注点割れ・欠品の両方が長期化＝管理除外）
    条件：ルール①の条件に加え、I列=1 かつ H列（欠品継続日時）も
          同じ拠点別リセット日数以上前
    処理：E列を 0 にする（ルール①より優先）
    理由：欠品が長期間放置されている＝担当者が管理対象外とする意思、とみなす
          （荘田室長confirmed）

  ルール⑤（廃止コードの復活）
    条件：C列（商品名）が前回まで #N/A エラー（＝TIMS側で商品コードが
          見つからない状態）だった品番が、今回は正常に商品名を
          表示するようになった
    処理：E列を、その時点のD列（在庫数）の値に書き換える
          （荘田室長confirmed：「復活時の在庫値でスタートすればよい。
          担当者の判断値くらいで十分、数ヶ月運用すれば収まる」）
    実装：C列がNA状態の品番を拠点別の追跡CSV（既定：ワークブックと同じ
          フォルダのDiscontinuedCodeTracking.csv）に記録し続け、次回以降
          このCSVに載っている品番の中でNAでなくなったものを「復活」と
          判定する。ワークブック自体には「過去NAだった」履歴が残らない
          ため、この追跡CSVが唯一の記憶装置になる。

  ルール①③の基準日数は、拠点別変数表（在庫チェックRPA統合_拠点別変数表.xlsx）
  の「発注点リセット_部署別設定」シートから、拠点名・シート名をキーに
  都度読み取る。変数表が指定されない、またはそのシート・拠点の行が
  見つからない場合は、既定値90日にフォールバックし、WARNログを出す。

  ※未確定事項として残っている点（要検討）
    - 外注鉄のバーコード貼付!VLOOKUP参照範囲に $D$6500 / $D$8000 の
      表記ゆれがある件は、本スクリプトのロジックとは無関係
      （在庫帳シート側のE/D列の値をそのまま使うため、バーコード貼付
      シートの参照範囲そのものは触らない）。範囲の統一自体は別途、
      在庫帳ファイルのメンテナンス作業として対応する。
    - ルール①のE=E-Dが負値になる理論上のケースへの防御として、
      0未満にはならないようクリップしている（本文コメント参照）。

===============================================================================
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$WorkbookPath,

    [Parameter(Mandatory = $true)]
    [string]$SiteName,

    [ValidateSet("Audit", "Update")]
    [string]$Mode = "Audit",

    [string]$LogPath = "",

    [int]$MaxCandidates = 1000,

    # 拠点別変数表のパス（発注点リセット_部署別設定シートを参照する）。
    # 未指定の場合は全シート既定90日として扱う。
    [string]$VariableTablePath = "",

    # 廃止コード追跡用CSVのパス。未指定の場合はワークブックと同じフォルダに
    # DiscontinuedCodeTracking.csv を自動生成する。
    [string]$DiscontinuedTrackingPath = "",

    # ルール①③の既定リセット日数（変数表から該当行が見つからない場合に使用）
    [int]$DefaultResetDays = 90
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"

# バージョン管理：改修のたびにこの値を更新する
$ScriptVersion = "2026-07-28-draft1"

$excel = $null
$workbook = $null
$varExcel = $null
$varWorkbook = $null
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

function Test-CellBlank {
    param([object]$Value)

    if ($null -eq $Value) {
        return $true
    }

    if ($Value -is [string] -and [string]::IsNullOrWhiteSpace($Value)) {
        return $true
    }

    return $false
}

function ConvertTo-DateSafe {
    param([object]$Value)

    # Excelの日付セルは、遅延バインディングCOM経由で.Value2を読むと
    # DateTime型ではなくOLE Automation日付のシリアル値（数値、例：46077）
    # として返ってくることがある。まずDateTimeならそのまま返し、
    # 数値ならFromOADateで変換する。どちらでもなければ$nullを返す。
    if ($Value -is [datetime]) {
        return $Value
    }

    $num = [double]0
    if ([double]::TryParse([string]$Value, [ref]$num)) {
        try {
            return [datetime]::FromOADate($num)
        }
        catch {
            return $null
        }
    }

    return $null
}

function Get-LastRow {
    param([object]$Sheet)

    $usedRange = $null
    try {
        $usedRange = $Sheet.UsedRange
        return ([int]$usedRange.Row + [int]$usedRange.Rows.Count - 1)
    }
    finally {
        Release-Com $usedRange
    }
}

# --- 拠点別変数表から「拠点名|シート名」→リセット日数のハッシュテーブルを作る ---
function Load-ResetDaysTable {
    param(
        [string]$Path,
        [int]$DefaultDays
    )

    $table = @{}

    if ([string]::IsNullOrWhiteSpace($Path)) {
        Write-Log "WARN" "VariableTablePath not specified. All sheets will use DefaultResetDays=$DefaultDays."
        return $table
    }

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        Write-Log "WARN" "VariableTablePath not found: $Path . Falling back to DefaultResetDays=$DefaultDays."
        return $table
    }

    $localExcel = $null
    $localWorkbook = $null
    $sheet = $null

    try {
        $localExcel = New-Object -ComObject Excel.Application
        $localExcel.Visible = $false
        $localExcel.DisplayAlerts = $false

        $localWorkbook = $localExcel.Workbooks.Open($Path, 0, $true)  # ReadOnly=true
        $sheet = $localWorkbook.Worksheets.Item("発注点リセット_部署別設定")
        $lastRow = Get-LastRow $sheet

        for ($r = 2; $r -le $lastRow; $r++) {
            $siteCell = $null
            $sheetCell = $null
            $daysCell = $null
            try {
                $siteCell = $sheet.Cells.Item($r, 2)   # B列：拠点名
                $sheetCell = $sheet.Cells.Item($r, 3)  # C列：シート名(部署)
                $daysCell = $sheet.Cells.Item($r, 4)   # D列：リセット日数

                $siteName = Normalize-Code $siteCell.Value2
                $sheetName = Normalize-Code $sheetCell.Value2
                $daysVal = [decimal]0

                if ([string]::IsNullOrWhiteSpace($siteName) -or [string]::IsNullOrWhiteSpace($sheetName)) {
                    continue
                }
                if (-not (Try-Decimal $daysCell.Value2 ([ref]$daysVal))) {
                    continue
                }

                $key = "$siteName|$sheetName"
                $table[$key] = [int]$daysVal
            }
            finally {
                Release-Com $daysCell
                Release-Com $sheetCell
                Release-Com $siteCell
            }
        }
    }
    finally {
        Release-Com $sheet
        if ($null -ne $localWorkbook) {
            try { $localWorkbook.Close($false) } catch {}
        }
        if ($null -ne $localExcel) {
            try { $localExcel.Quit() } catch {}
        }
        Release-Com $localWorkbook
        Release-Com $localExcel
    }

    return $table
}

function Get-ResetDays {
    param(
        [hashtable]$Table,
        [string]$SiteName,
        [string]$SheetName,
        [int]$DefaultDays
    )

    $key = "$SiteName|$SheetName"
    if ($Table.ContainsKey($key)) {
        return [int]$Table[$key]
    }

    Write-Log "WARN" "ResetDays not found for Site=$SiteName Sheet=$SheetName . Using DefaultResetDays=$DefaultDays."
    return $DefaultDays
}

# --- 廃止コード追跡CSVの読み書き ---
# CSV列: 検知日,拠点,シート,行番号,商品コード,状態,復活日,復活時発注点
function Load-DiscontinuedTracking {
    param([string]$Path)

    $dict = @{}

    if (Test-Path -LiteralPath $Path -PathType Leaf) {
        $rows = Import-Csv -LiteralPath $Path -Encoding UTF8
        foreach ($r in $rows) {
            $key = "$($r.シート)|$($r.商品コード)"
            $dict[$key] = $r
        }
    }

    return $dict
}

function Save-DiscontinuedTracking {
    param(
        [string]$Path,
        [hashtable]$Dict
    )

    $rows = @()
    foreach ($key in $Dict.Keys) {
        $rows += $Dict[$key]
    }

    if ($rows.Count -gt 0) {
        $rows | Export-Csv -LiteralPath $Path -NoTypeInformation -Encoding UTF8
    }
}

# クリア操作を「未変更（$null）」と区別するための番兵値
$Script:ClearMarker = "##CLEAR##"

try {
    if (-not (Test-Path -LiteralPath $WorkbookPath -PathType Leaf)) {
        throw "Workbook not found: $WorkbookPath"
    }

    $WorkbookPath = (Resolve-Path -LiteralPath $WorkbookPath).ProviderPath
    $folder = Split-Path -Parent $WorkbookPath

    if ([string]::IsNullOrWhiteSpace($LogPath)) {
        $LogPath = Join-Path $folder "ProcessSheetRows.log"
    }
    $script:LogPath = $LogPath

    $logFolder = Split-Path -Parent $LogPath
    if (-not [string]::IsNullOrWhiteSpace($logFolder)) {
        if (-not (Test-Path -LiteralPath $logFolder)) {
            New-Item -ItemType Directory -Path $logFolder -Force | Out-Null
        }
    }

    if ([string]::IsNullOrWhiteSpace($DiscontinuedTrackingPath)) {
        # 拠点名をファイル名に含める。同じNAS共有フォルダに複数拠点の
        # ワークブックが並んで置かれているケース（汎用シナリオ4拠点等）で、
        # かつRK-10のライセンス上、複数拠点が同時並行実行され得るため、
        # 共有の単一CSVにすると書き込み競合（片方の更新が消える等）が
        # 起きる。拠点ごとにファイルを分けることで、この競合を構造的に
        # 回避する。
        $safeSiteName = ($SiteName -replace '[\\/:*?"<>|]', '_')
        $DiscontinuedTrackingPath = Join-Path $folder ("DiscontinuedCodeTracking_{0}.csv" -f $safeSiteName)
    }

    Add-Content -LiteralPath $LogPath -Value ("=" * 80) -Encoding UTF8
    Write-Log "INFO" "Start"
    Write-Log "INFO" ("ScriptVersion={0}" -f $ScriptVersion)
    Write-Log "INFO" ("Site={0}" -f $SiteName)
    Write-Log "INFO" ("Mode={0}" -f $Mode)
    Write-Log "INFO" ("Workbook={0}" -f $WorkbookPath)
    Write-Log "INFO" ("DiscontinuedTrackingPath={0}" -f $DiscontinuedTrackingPath)

    # 拠点別リセット日数テーブルを読み込む（変数表を開いて即閉じる。メインの
    # ワークブックとは別のExcelインスタンスを使い、影響を分離する）
    $resetDaysTable = Load-ResetDaysTable -Path $VariableTablePath -DefaultDays $DefaultResetDays

    # 廃止コード追跡CSVを読み込む
    $trackingDict = Load-DiscontinuedTracking -Path $DiscontinuedTrackingPath
    Write-Log "INFO" ("DiscontinuedTracking loaded. Entries={0}" -f $trackingDict.Count)

    $excel = New-Object -ComObject Excel.Application
    $excel.Visible = $false
    $excel.DisplayAlerts = $false
    $excel.ScreenUpdating = $false
    $excel.EnableEvents = $false

    $workbook = $excel.Workbooks.Open($WorkbookPath, 0, $false)

    if (($Mode -eq "Update") -and [bool]$workbook.ReadOnly) {
        throw "Workbook opened read-only. Close Excel and retry."
    }

    $today = (Get-Date).Date

    $candidates = New-Object System.Collections.ArrayList
    $sheetNames = New-Object System.Collections.ArrayList

    foreach ($sheet in $workbook.Worksheets) {
        try {
            $sheetName = [string]$sheet.Name
            if ($sheetName -notmatch "^\d+$") {
                continue
            }

            [void]$sheetNames.Add($sheetName)
            $resetDays = Get-ResetDays -Table $resetDaysTable -SiteName $SiteName -SheetName $sheetName -DefaultDays $DefaultResetDays
            $lastRow = Get-LastRow $sheet

            for ($row = 2; $row -le $lastRow; $row++) {
                $codeCell = $null
                $nameCell = $null
                $dCell = $null
                $eCell = $null
                $fCell = $null
                $gCell = $null
                $hCell = $null
                $iCell = $null

                try {
                    $codeCell = $sheet.Cells.Item($row, 2)
                    $code = Normalize-Code $codeCell.Value2
                    if ([string]::IsNullOrWhiteSpace($code)) {
                        continue
                    }

                    $nameCell = $sheet.Cells.Item($row, 3)
                    $dCell = $sheet.Cells.Item($row, 4)
                    $eCell = $sheet.Cells.Item($row, 5)
                    $fCell = $sheet.Cells.Item($row, 6)
                    $gCell = $sheet.Cells.Item($row, 7)
                    $hCell = $sheet.Cells.Item($row, 8)
                    $iCell = $sheet.Cells.Item($row, 9)

                    # 商品名(C列)が #N/A エラーかどうか（廃止コード判定用）。
                    # .Text を使うのは、COMの遅延バインディングではエラー値の
                    # 型判定が煩雑なため、表示文字列で判定する方が確実だから。
                    $isNameError = $false
                    try {
                        $isNameError = ($nameCell.Text -match "#N/A")
                    }
                    catch {
                        $isNameError = $false
                    }

                    # 発注点(E)・在庫数(D)・フラグ(G/I)が数値変換できない行
                    # （ヘッダー行、"#N/A"行）は従来の4パターンとしては対象外。
                    # ただし廃止コード判定（ルール⑤）は、E/D等が数値化できない
                    # 行（＝現在NA状態の行）でこそ意味を持つため、先に廃止コード
                    # チェックだけ行ってから、既存の数値変換ガードに入る。
                    $trackKey = "$sheetName|$code"

                    if ($isNameError) {
                        if (-not $trackingDict.ContainsKey($trackKey)) {
                            $trackingDict[$trackKey] = [PSCustomObject]@{
                                検知日     = $today.ToString("yyyy-MM-dd")
                                拠点       = $SiteName
                                シート     = $sheetName
                                行番号     = $row
                                商品コード = $code
                                状態       = "追跡中"
                                復活日     = ""
                                復活時発注点 = ""
                            }
                            Write-Log "INFO" ("DiscontinuedTracking: new NA code tracked Sheet=$sheetName Code=$code")
                        }
                        # NAが続いている間は従来の4パターン判定に進んでも
                        # E/D等が数値化できず自然にスキップされるので、
                        # このままcontinueせず下の数値変換ガードに委ねる。
                    }
                    elseif ($trackingDict.ContainsKey($trackKey) -and $trackingDict[$trackKey].状態 -eq "追跡中") {
                        # --- ルール⑤：廃止コードの復活 ---
                        $dValForRevival = [decimal]0
                        if (Try-Decimal $dCell.Value2 ([ref]$dValForRevival)) {
                            $revivalCandidate = [PSCustomObject]@{
                                SheetName = $sheetName
                                Row       = $row
                                Code      = $code
                                EOld      = $null
                                ENew      = $dValForRevival
                                FOld      = $null
                                FNew      = $null
                                HOld      = $null
                                HNew      = $null
                                Reason    = "廃止コード復活(E=在庫数)"
                            }
                            [void]$candidates.Add($revivalCandidate)

                            $entry = $trackingDict[$trackKey]
                            $entry.状態 = "復活済み"
                            $entry.復活日 = $today.ToString("yyyy-MM-dd")
                            $entry.復活時発注点 = [string]$dValForRevival
                            Write-Log "INFO" ("DiscontinuedTracking: revival detected Sheet=$sheetName Code=$code NewE=$dValForRevival")
                        }
                        # このセルは復活処理を候補に積んだので、以降の
                        # 通常4パターン+長期滞留ルールの判定はスキップする
                        continue
                    }

                    # 発注点(E)・在庫数(D)・フラグ(G/I)が数値変換できない行
                    # （ヘッダー行、現在進行形の"#N/A"行）は対象外として
                    # 無言スキップする。
                    $eVal = [decimal]0
                    if (-not (Try-Decimal $eCell.Value2 ([ref]$eVal))) {
                        continue
                    }

                    $dVal = [decimal]0
                    if (-not (Try-Decimal $dCell.Value2 ([ref]$dVal))) {
                        continue
                    }

                    $gVal = [decimal]0
                    if (-not (Try-Decimal $gCell.Value2 ([ref]$gVal))) {
                        continue
                    }

                    $iVal = [decimal]0
                    if (-not (Try-Decimal $iCell.Value2 ([ref]$iVal))) {
                        continue
                    }

                    $fRaw = $fCell.Value2
                    $hRaw = $hCell.Value2
                    $fBlank = Test-CellBlank $fRaw
                    $hBlank = Test-CellBlank $hRaw

                    $newE = $null
                    $newF = $null
                    $newH = $null
                    $reasons = New-Object System.Collections.ArrayList

                    # --- 欠品系（I列/H列）を先に判定・適用（既存パターン1/2） ---
                    if ($iVal -eq 1 -and $hBlank) {
                        $newE = $eVal + 1
                        $newH = $today
                        [void]$reasons.Add("欠品新規検知(E+1,H更新)")
                    }
                    elseif ($iVal -eq 0 -and -not $hBlank) {
                        $newH = $Script:ClearMarker
                        $newF = $today
                        $fBlank = $false
                        [void]$reasons.Add("欠品解消(Hクリア,F更新)")
                    }

                    # --- 発注点割れ系（G列/F列）を、欠品系の反映後の状態で判定（既存パターン3/4） ---
                    if ($gVal -eq 1 -and $fBlank) {
                        $newF = $today
                        [void]$reasons.Add("発注点割れ新規検知(F更新)")
                    }
                    elseif ($gVal -eq 0 -and -not $fBlank) {
                        $newF = $Script:ClearMarker
                        [void]$reasons.Add("発注点割れ解消(Fクリア)")
                    }

                    # --- ルール①③：長期滞留品の自動処理（今回追加分） ---
                    # 判定は「今読んだ元のfRaw/hRaw」の日付age基準。パターン1-4の
                    # 新規検知（今日日付を書き込む動作）とは別軸なので、
                    # 新規検知直後の行（F/Hが今日付になったばかり）はage=0日と
                    # なり、このルールには該当しない（自然に除外される）。
                    if ($gVal -eq 1 -and -not $fBlank) {
                        $fDateSafe = ConvertTo-DateSafe $fRaw

                        if ($null -ne $fDateSafe) {
                        $fAgeDays = [int]($today - $fDateSafe.Date).TotalDays

                        if ($fAgeDays -ge $resetDays) {
                            $rule3Applied = $false

                            if ($iVal -eq 1 -and -not $hBlank) {
                                $hDateSafe = ConvertTo-DateSafe $hRaw
                                if ($null -ne $hDateSafe) {
                                $hAgeDays = [int]($today - $hDateSafe.Date).TotalDays
                                if ($hAgeDays -ge $resetDays) {
                                    # ルール③：発注点割れ・欠品どちらも長期化 → 管理除外
                                    $newE = [decimal]0
                                    # F/Hも合わせてクリアし、翌日以降の再発火（際限ない
                                    # 繰り返し適用）を防ぐ。もし本当にまだ状態が続いて
                                    # いれば、パターン3/1が改めて今日付でF/Hを立て直す。
                                    $newF = $Script:ClearMarker
                                    $newH = $Script:ClearMarker
                                    [void]$reasons.Add("長期滞留_管理除外(E=0,基準${resetDays}日)")
                                    $rule3Applied = $true
                                }
                                }
                            }

                            if (-not $rule3Applied) {
                                # ルール①：発注点割れのみ長期化 → 差分値に置換
                                $diff = $eVal - $dVal
                                if ($diff -lt 0) { $diff = [decimal]0 }  # 念のための防御的クリップ
                                $newE = $diff
                                # F列もクリアし、再発火を防ぐ（上記③と同じ理由）。
                                $newF = $Script:ClearMarker
                                [void]$reasons.Add("長期滞留_発注点差分置換(E=E-D,基準${resetDays}日)")
                            }
                        }
                        }
                    }

                    if ($null -eq $newE -and $null -eq $newF -and $null -eq $newH) {
                        continue
                    }

                    $candidate = [PSCustomObject]@{
                        SheetName = $sheetName
                        Row       = $row
                        Code      = $code
                        EOld      = $eVal
                        ENew      = $newE
                        FOld      = $fRaw
                        FNew      = $newF
                        HOld      = $hRaw
                        HNew      = $newH
                        Reason    = ($reasons -join "; ")
                    }

                    [void]$candidates.Add($candidate)
                }
                finally {
                    Release-Com $iCell
                    Release-Com $hCell
                    Release-Com $gCell
                    Release-Com $fCell
                    Release-Com $eCell
                    Release-Com $dCell
                    Release-Com $nameCell
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
        $eDesc = if ($null -ne $item.ENew) { "E:{0}->{1}" -f $item.EOld, $item.ENew } else { "" }
        $fDesc = if ($null -ne $item.FNew) {
            if ($item.FNew -eq $Script:ClearMarker) { "F:クリア" } else { "F:->{0:yyyy-MM-dd}" -f $item.FNew }
        } else { "" }
        $hDesc = if ($null -ne $item.HNew) {
            if ($item.HNew -eq $Script:ClearMarker) { "H:クリア" } else { "H:->{0:yyyy-MM-dd}" -f $item.HNew }
        } else { "" }

        Write-Log "INFO" (
            "Candidate Sheet={0} Row={1} Code={2} [{3}] {4} {5} {6}" -f
            $item.SheetName, $item.Row, $item.Code, $item.Reason, $eDesc, $fDesc, $hDesc
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
            $backupName = "{0}_{1}{2}" -f $baseName, (Get-Date -Format "yyyyMMdd_HHmmss"), $extension
            $backupPath = Join-Path $backupFolder $backupName

            Copy-Item -LiteralPath $WorkbookPath -Destination $backupPath -Force
            Write-Log "INFO" ("Backup={0}" -f $backupPath)

            foreach ($item in $candidates) {
                $targetSheet = $null
                $eCell = $null
                $fCell = $null
                $hCell = $null

                try {
                    $targetSheet = $workbook.Worksheets.Item($item.SheetName)

                    if ($null -ne $item.ENew) {
                        $eCell = $targetSheet.Cells.Item($item.Row, 5)
                        $eCell.Value2 = [double]$item.ENew
                    }

                    if ($null -ne $item.FNew) {
                        $fCell = $targetSheet.Cells.Item($item.Row, 6)
                        if ($item.FNew -eq $Script:ClearMarker) {
                            $fCell.Value2 = ""
                        }
                        else {
                            $fCell.Value2 = $item.FNew
                        }
                    }

                    if ($null -ne $item.HNew) {
                        $hCell = $targetSheet.Cells.Item($item.Row, 8)
                        if ($item.HNew -eq $Script:ClearMarker) {
                            $hCell.Value2 = ""
                        }
                        else {
                            $hCell.Value2 = $item.HNew
                        }
                    }

                    Write-Log "UPDATE" (
                        "Updated Sheet={0} Row={1} Code={2} [{3}]" -f
                        $item.SheetName, $item.Row, $item.Code, $item.Reason
                    )
                }
                finally {
                    Release-Com $hCell
                    Release-Com $fCell
                    Release-Com $eCell
                    Release-Com $targetSheet
                }
            }

            $workbook.Save()
            Write-Log "INFO" ("Saved. Updated={0}" -f $candidates.Count)
        }
        else {
            Write-Log "INFO" "No update required."
        }

        # 廃止コード追跡CSVはUpdateモードのときのみ更新する
        # （Auditモードでは何も書き換えないという既存の原則に合わせる）
        Save-DiscontinuedTracking -Path $DiscontinuedTrackingPath -Dict $trackingDict
        Write-Log "INFO" ("DiscontinuedTracking saved. Entries={0}" -f $trackingDict.Count)
    }
    else {
        Write-Log "INFO" "Audit completed. Workbook was not changed."
    }

    $elapsed = (Get-Date) - $startTime
    Write-Log "INFO" ("ElapsedSeconds={0:N2}" -f $elapsed.TotalSeconds)

    $workbook.Close($false)
    $excel.Quit()

    Release-Com $workbook
    Release-Com $excel

    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()

    Add-Content -LiteralPath $LogPath -Value "" -Encoding UTF8
    exit 0
}
catch {
    $errorText = $_.Exception.Message

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

    if ($null -ne $workbook) {
        try { $workbook.Close($false) } catch {}
    }
    if ($null -ne $excel) {
        try { $excel.Quit() } catch {}
    }

    Release-Com $workbook
    Release-Com $excel

    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()

    exit 1
}
