<#
===============================================================================
ProcessSheetRows.ps1

概要
  数字シート（拠点別在庫チェックワークブック）の行ループ（従来はRK-10の
  「バーコードデータ貼り付け」サブルーチン44-79行目が担っていた処理）を
  PowerShell COM操作で肩代わりする。

  対象は在庫チェックRPA統合プロジェクト・引継ぎメモ 2.4節に記載された
  有効パターンのうち、以下の4パターン（発注点0補正は対象外。
  それはFixReorderPoint.ps1に集約済みのため、本スクリプトでは扱わない）。

  パターン1  I列=1 かつ H列=空       → E列を+1、H列に現在日時
             （欠品を新規検知。欠品継続時の発注点自動加算）
             ※F列は更新しない。各種TIMS在庫データシナリオ-V2.0
               仕様書（PDF）24ページを実機確認した結果、以下が確定している。
               - 前段の条件式は「I列=1 かつ H列=空」（塚田氏が実機で確認済み）
               - F列書込みステップにのみ「F列の日付はそのまま残す為、
                 コメントアウトする 2025.04.03」のメモと共に無効ブロックが
                 かかっており、F列書込みは実行されない
               - H列書込みステップ、E列+1ステップは無効ブロックの外にあり、
                 いずれも有効（PDFのインデント階層で確認済み）
               既に発注点割れ検知済みの品番が欠品に至っても、
               発注点割れを最初に検知した日付を欠品発生日で
               上書きしないための意図的な仕様。
  パターン2  I列=0 かつ H列≠空       → H列クリア、F列に現在日時
             （欠品解消時のリセット）
  パターン3  G列=1 かつ F列=空       → F列に現在日時
             （発注点割れを新規検知）
  パターン4  G列=0 かつ F列≠空       → F列クリア
             （発注点割れ解消時のリセット）

  ※パターン1/2とパターン3/4はどちらもF列を書き換えうるため、本スクリプトでは
    元のRK-10行ループが上から順に実行されることを模して、
    「パターン1→2（欠品系）を判定・適用 → その結果を踏まえてパターン3→4
    （発注点割れ系）を判定・適用」という順序でシミュレートする。
    （1行のうちで両系統が同時に発火するケースは実データ上は稀だが、
    その場合は欠品系の判定を先に反映した状態で発注点割れ系を判定する。）
    ※この実行順の解釈が実機のRK-10ロジックと一致するかは、荘田室長への
      確認事項として残すこと（引継ぎメモには順序の明記がないための仮定）。

列レイアウト（数字シート、ヘッダーはB2='コード'等・データは3行目以降）
  B列 : 品番
  D列 : 在庫数
  E列 : 発注点
  F列 : 発注点割れ検知日
  G列 : 発注点割れフラグ（1/0）
  H列 : 欠品継続日時
  I列 : 欠品フラグ（1/0）

モード
  Audit  : 更新候補のみ表示（Excelは変更しません）
  Update : Backupフォルダへ退避後、Excelを更新します。

実行例
  Audit
    powershell.exe -NoProfile -ExecutionPolicy Bypass `
      -File "C:\HPDB\ProcessSheetRows.ps1" `
      -WorkbookPath "C:\HPDB\仙台在庫帳.xlsx" `
      -SiteName "仙台" `
      -Mode Audit `
      -LogPath "C:\HPDB\ProcessSheetRows.log"

  Update
    powershell.exe -NoProfile -ExecutionPolicy Bypass `
      -File "C:\HPDB\ProcessSheetRows.ps1" `
      -WorkbookPath "C:\HPDB\仙台在庫帳.xlsx" `
      -SiteName "仙台" `
      -Mode Update `
      -LogPath "C:\HPDB\ProcessSheetRows.log"

  MaxCandidates
    Update時、候補件数がこの値を超えると異常終了（exit 1）する安全装置の閾値。
    既定値は1000（FixReorderPoint.ps1と同じ考え方）。

仕様
  ・対象シート : シート名が数字のみ（"19"のように全行#N/Aのシートは
                 D/E/G/I列がいずれも数値変換できないため自然にスキップされる）
  ・データ開始行 : 2行目（ヘッダー行はE/G/I列が数値変換できないため自動スキップ）
  ・終端行判定：本スクリプトはUsedRangeベースで各シートの最終行まで
    全行走査する。RK-10側（各種TIMS在庫データシナリオ-V2.0仕様書22ページ、
    ステップ38-42）は「F3セルから下方向に最初の空行を探索し、その行+4を
    LoopCountとする」という方式で走査範囲を絞っていたが、これは意図的に
    採用しない。
    実データ（仙台在庫帳.xlsx）で検証したところ、この+4方式は「発注点割れ
    中の品番がシート先頭にまとまっている」というデータ特性を前提に、
    UI操作ベースの行ループを現実的な時間で終わらせるための荘田室長による
    速度対策と判明した（例：シート13で最終行2470行に対しF3+4方式は44行目
    で打ち切り）。裏を返せば、先頭ブロックの外側で新規に発注点割れ・欠品が
    発生した場合、RK-10の行ループはそれを検知できないリスクを内包していた。
    PowerShell COM版は全行走査でも仙台全体で1分38秒（実測）と、この速度
    制約自体が解消されているため、+4による範囲制限は不要と判断し、
    UsedRangeベースの全行走査を正式仕様として採用する。これは互換性の
    不備ではなく、RK-10が抱えていた検知漏れリスクを解消する機能改善。
  ・欠品/発注点割れの「現在日時」は日付のみ（時刻00:00:00）で書き込む。
    既存データのF列・H列がいずれも日付のみ（時刻情報なし）で格納されて
    いることを仙台在庫帳.xlsxで確認済みのため、それに合わせている。
  ・クリア操作（H列クリア／F列クリア）は空文字列("")を書き込む。
    引継ぎメモ2.4節「ステップ62のH列書き込みは空文字（セルクリア）」の
    確認済み仕様に倣い、F列クリアも同じ流儀（空文字）で統一した。
  ・終了コード : 0=正常 / 1=異常

バージョン管理について
  このスクリプトも各拠点PCのローカル（例：C:\HPDB）に個別配置して運用する
  想定のため、FixReorderPoint.ps1と同じ考え方で $ScriptVersion を改修のたびに
  更新し、実行時にログへ出力することで、各PCが最新版かどうかを後から
  追跡できるようにしている。

===============================================================================
#>

# スクリプトのバージョン（更新日）。改修のたびにここを更新すること。
# ※param()ブロックより前に実行文を置くとPowerShellのパースエラーになるため、
#   この代入は必ずparam()の後（Set-StrictModeの近く）に置くこと。
#   （FixReorderPoint.ps1で実際に踏んだ罠と同じ。詳細はそちらのヘッダー参照）

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$WorkbookPath,

    [Parameter(Mandatory = $true)]
    [string]$SiteName,

    [ValidateSet("Audit", "Update")]
    [string]$Mode = "Audit",

    [string]$LogPath = "",

    [int]$MaxCandidates = 1000
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"

# バージョン管理：改修のたびにこの値を更新する（詳細はヘッダーコメント参照）
$ScriptVersion = "2026-07-24"

$excel = $null
$workbook = $null
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

    Add-Content -LiteralPath $LogPath -Value ("=" * 80) -Encoding UTF8
    Write-Log "INFO" "Start"
    Write-Log "INFO" ("ScriptVersion={0}" -f $ScriptVersion)
    Write-Log "INFO" ("Site={0}" -f $SiteName)
    Write-Log "INFO" ("Mode={0}" -f $Mode)
    Write-Log "INFO" ("Workbook={0}" -f $WorkbookPath)

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

    # --- シート名が数字のみの拠点別シートを走査し、4パターンの更新候補を抽出 ---
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

                    $dCell = $sheet.Cells.Item($row, 4)
                    $eCell = $sheet.Cells.Item($row, 5)
                    $fCell = $sheet.Cells.Item($row, 6)
                    $gCell = $sheet.Cells.Item($row, 7)
                    $hCell = $sheet.Cells.Item($row, 8)
                    $iCell = $sheet.Cells.Item($row, 9)

                    # 発注点(E)・在庫数(D)・フラグ(G/I)が数値変換できない行
                    # （ヘッダー行、"#N/A"行）は対象外として無言スキップする。
                    # ※既知の課題：この無言スキップはログに出力されない
                    #   （引継ぎメモ3.4節「数値変換失敗行の無言スキップ」と同種の制約）。
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

                    # --- 欠品系（I列/H列）を先に判定・適用 ---
                    if ($iVal -eq 1 -and $hBlank) {
                        $newE = $eVal + 1
                        $newH = $today
                        # F列は更新しない（RK-10側で「F列の日付はそのまま残す為、
                        # コメントアウトする 2025.04.03」というコメントと共に
                        # 無効化されていることを確認済み）
                        [void]$reasons.Add("欠品新規検知(E+1,H更新)")
                    }
                    elseif ($iVal -eq 0 -and -not $hBlank) {
                        $newH = $Script:ClearMarker
                        $newF = $today
                        $fBlank = $false
                        [void]$reasons.Add("欠品解消(Hクリア,F更新)")
                    }

                    # --- 発注点割れ系（G列/F列）を、欠品系の反映後の状態で判定 ---
                    if ($gVal -eq 1 -and $fBlank) {
                        $newF = $today
                        [void]$reasons.Add("発注点割れ新規検知(F更新)")
                    }
                    elseif ($gVal -eq 0 -and -not $fBlank) {
                        $newF = $Script:ClearMarker
                        [void]$reasons.Add("発注点割れ解消(Fクリア)")
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

    Release-Com $workbook
    Release-Com $excel

    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()

    exit 1
}
