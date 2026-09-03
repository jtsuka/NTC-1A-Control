<#
===============================================================================
ProcessSheetRows.ps1  (長期滞留品自動処理・廃止コード復活処理・単価シート参照設定
                      による G/I/J/K/O列復元処理を追加した版)

【2026-09-02 追加】ルール⑤拡張：復活時のG/I/J/K/O列復元
  背景：
    ルール⑥（NA行のE:O列クリア）はA列およびE～O列を無条件でClearContentsする。
    このためG列(発注点割れ判定)・I列(欠品判定)・J列(単価)・K列(在庫額)・O列
    (発注点差)の数式もまとめて消える。ルール⑤（廃止コード復活）は従来E列のみ
    復元しており、G/I/J/K/O列は復活後も空欄のまま放置され、発注点割れ・欠品
    判定が機能しなくなる不具合が実運用で発生した（仙台在庫帳シート1・商品
    コードBN0365SHで確認）。

  解決方針：
    「単価シート参照設定」マスタシート（拠点別変数表.xlsx内）に、拠点(拠点名)
    ごとの「単価シート名」および「G/I/J/K/O列復元要否(有/無)」を保持する。
    復活判定が成立した行に対し、フラグが「有」の列だけを復元対象とする。

  復元方式（設計レビューを踏まえた採用方式）：
    ×案A（上行(row-1)のFormula文字列をそのままコピー）
      → 上行がたまたま復活行・数式欠落行だった場合に事故る。
    ×Formula文字列の直接代入（案A・素朴な案Bともに共通する致命的欠陥）
      → .Formula プロパティへ文字列をそのまま代入すると、Excelのセルコピー
        のような相対参照の自動調整が一切行われない。テンプレート行の数式が
        そのまま複製され、本来の行のD/E列ではなくテンプレート行を参照する
        誤った数式になり、実運用で確実に事故る。
    ○採用：案B（直近の正常な数式セルを上方向へ探索）
      ＋ Excelネイティブの Range.Copy() → PasteSpecial(xlPasteFormulas) を
        使用し、相対参照の調整をExcel自身に行わせる（Formula文字列の直接
        代入は行わない）。xlPasteFormulasは数式のみを貼り付け、貼付先の
        書式(罫線・背景色等)は変更しない。
      ・テンプレート行の判定は .HasFormula で行う（.Formula の文字列空白
        判定は「数式なしの通常値セル」でも文字列を返すため不正確であり、
        使用しない）。
      ・上方向探索は既定200行（-MaxFormulaSearchRows で変更可）で打ち切り、
        見つからない場合はSkipNoTemplateとして復元をスキップしログに残す。

  人手入力保護：
    復元対象は「数式が無く、かつ値も空（Test-CellEmptyForRestore）」の
    セルのみに限定する。担当者が暫定的に数値等を手入力している場合は
    その内容を保護し、上書きしない（SkipHasContentとしてログに残す）。

  Audit/Updateモードの扱い：
    復元計画（Get-RestorePlan）はワークブックを一切変更しない読み取り専用
    処理であり、Auditモードでも実行し、ログに計画内容（復元する列・
    テンプレート行、またはスキップ理由）を出力する。実際の書き込み
    （Invoke-RestorePlan、Copy/PasteSpecial）はUpdateモードでのみ行う。

  拠点キーについて：
    「単価シート参照設定」は拠点コードではなく拠点名（B列）をキーに読み込む。
    拠点コードZ3934は「外注鉄」「成果連絡表(色物)」で重複しているため、
    拠点コードをキーにすると設定の取り違えが起こる。ProcessSheetRows.ps1
    は -SiteName パラメータに拠点名を受け取る設計のため、既存の
    Get-ResetDays と同じ考え方で拠点名キーに統一する。

  「単価シート名」列について：
    このロジックは単価シート名を直接参照しない（既存の正常な数式セルを
    そのままコピーするだけのため、その数式が内部で単価シートを参照して
    いればテンプレート行ごとコピーされ、そのまま機能する）。単価シート名
    列はマスタとしての人間向け情報・将来の拡張用に保持している。

  切り戻し・安全装置：
    -SkipFormulaRestore スイッチを指定すると本機能全体を無効化できる
    （旧バージョンと同じ動作に戻す）。
    -MaxFormulaSearchRows で上方探索の最大行数を調整できる（既定200）。

===============================================================================

概要（従来からの記載）
  従来の4パターン（欠品新規検知/解消、発注点割れ新規検知/解消）に加え、
  荘田室長が手動で行っていた「長期滞留品の自動処理」5項目のうち、
  自動化未実装だった3項目（ルール①③⑤）を追加する。
  （項番④＝発注点0&入庫ありでE=Dは、既存のFixReorderPoint.ps1で対応済みのため
  本スクリプトでは扱わない）

  ■ 発注点の考え方（荘田室長の手書き図・2026-07-29 に基づく理論的背景）
    発注点とは「補充リードタイム（＝基準日数。既定90日、船舶(機械部)の
    一部シートのみ365日）の間に消費される数量」である、というのが本来の
    定義。この定義から、ルール①とパターン1（欠品時のE+1）が同じ
    フィードバック制御ループの両方向であることが導かれる。
      ・下方修正（余裕が大きすぎた場合）＝ルール①
        発注点割れ日の在庫はほぼE。そこからリードタイム経過後の在庫がD。
        つまりリードタイム中の実消費量は E-D であり、これがあるべき
        発注点。残ったDは「不要と実証された余裕分」なので丸ごと引く。
        → E_new = E - D
      ・上方修正（足りなかった場合）＝パターン1
        リードタイム中に在庫が0に到達＝消費量がEを上回った。しかし
        どれだけ上回ったかは観測できない（在庫はマイナスにならない）。
        測れないので1個ずつ刻んで探る。
        → E_new = E + 1

  ルール①（発注点割れ長期化）
    条件：G列=1 かつ F列（発注点割れ検知日）が拠点別リセット日数
          （＝補充リードタイム。既定90日、船舶(機械部)の一部シートのみ
          365日）以上前
          ただし D列（在庫数）が 0 以下の場合は対象外（下記の除外条件を参照）
    処理：
      ・E列を「今のE － 今のD」に書き換える（上記の実消費量）
      ・F列は「クリア」ではなく「今日の日付」に更新する
        （荘田室長confirmed・2026-07-29）
      ・回数制限は設けず、条件を満たすたびに毎回適用する

    除外条件（D列=0、2026-07-29 福岡の実データで判明）：
      在庫が0まで落ちている場合、消費量が「E以上」であることしか
      分からず超過分は観測できないため、E-D は E のままとなり補正が
      成立しない（空振り）。D列<=0の行はE列・F列とも一切変更せずスキップ。
      スキップ時はログに LongTermSkip(D=0) を出力する。
    ※ただしルール③の条件も同時に満たす場合はルール③を優先する

  ルール③（発注点割れ・欠品の両方が長期化＝管理除外）
    条件：ルール①の条件に加え、I列=1 かつ H列（欠品継続日時）も
          同じ拠点別リセット日数以上前
    処理：E列を 0 にする（ルール①より優先）
    理由：欠品が長期間放置されている＝担当者が管理対象外とする意思、とみなす

  ルール⑤（廃止コードの復活）
    条件：C列（商品名）が前回まで #N/A エラーだった品番が、今回は正常に
          商品名を表示するようになった
    処理：E列を、バーコード貼付シートD列(発注点マスタ)から取得した本来の
          発注点値に書き換える（2026-08-10 荘田室長指示）
    【2026-09-02追加】あわせて、単価シート参照設定でフラグが「有」の
          G/I/J/K/O列についても、直近の正常な数式セルから復元する
          （Get-RestorePlan / Invoke-RestorePlan、詳細は冒頭参照）。
    実装：C列がNA状態の品番を拠点別の追跡CSVに記録し続け、次回以降
          このCSVに載っている品番の中でNAでなくなったものを「復活」と
          判定する。

  ルール⑥（N/A行のE:O列クリア）※2026/8/7 荘田室長依頼により追加
    条件：C列（商品名）が #N/A エラーである行
    処理：A列およびE列～O列の内容をすべてクリアする
    実装：ルール⑤（廃止コード復活）とは対になる関係。isNameError判定の
          直後、ルール⑤判定より先に評価する。

  ルール①③の基準日数は、拠点別変数表の「発注点リセット_部署別設定」シート
  から、拠点名・シート名をキーに都度読み取る。

  参考：
    - 外注鉄のバーコード貼付!VLOOKUP参照範囲に $D$6500 / $D$8000 の
      表記ゆれがある件は、本スクリプトのロジックとは無関係。
    - ルール①のE=E-Dが負値になる理論上のケースへの防御として、
      0未満にはならないようクリップしている。
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
    [int]$MaxCandidates = 5000,
    # 拠点別変数表のパス（発注点リセット_部署別設定シート・単価シート参照設定
    # シートの両方をここから読み込む）。未指定の場合は全シート既定90日、かつ
    # G/I/J/K/O列の復元処理は全拠点でスキップされる。
    [string]$VariableTablePath = "",
    # 廃止コード追跡用CSVのパス。未指定の場合はワークブックと同じフォルダに
    # DiscontinuedCodeTracking.csv を自動生成する。
    [string]$DiscontinuedTrackingPath = "",
    # ルール①③の既定リセット日数（変数表から該当行が見つからない場合に使用）
    [int]$DefaultResetDays = 90,
    # 【2026-09-02追加】ルール⑤拡張：G/I/J/K/O列復元時、直近の正常な数式
    # セルを上方向へ探索する最大行数。データ量に応じて調整可。
    [int]$MaxFormulaSearchRows = 200,
    # 【2026-09-02追加】指定するとG/I/J/K/O列の復元処理そのものを無効化し、
    # 旧バージョン（E列のみ復元）と同じ動作に戻す（切り戻し用スイッチ）。
    [switch]$SkipFormulaRestore
)
Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"
# バージョン管理：改修のたびにこの値を更新する
$ScriptVersion = "2026-09-02-UnitPriceFormulaRestore-r1"
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
# ---------------------------------------------------------------------------
# Backupフォルダの古いバックアップを削除する。
# ・当日を含めて直近$RetentionDays日分のみ残し、それより古いものを削除する
#   （既定2日＝当日・前日を保持し、3日以上前のものを削除）。
# ・判定はファイル名ではなく実際の最終更新日時(LastWriteTime)で行う。
# ・対象はこのワークブックのバックアップ（{ファイル名}_yyyyMMdd{拡張子}）のみに
#   限定し、同じBackupフォルダを共用する他スクリプトのバックアップには触れない。
# ・削除に失敗しても本処理自体は継続する（ログにWARNを残すのみ）。
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
# ---------------------------------------------------------------------------
# Excel数式を強制的に全再計算し、完了まで待機する。
# ---------------------------------------------------------------------------
function Invoke-ExcelFullCalculation {
    param(
        [Parameter(Mandatory = $true)]
        $Excel,
        [int]$TimeoutSeconds = 60
    )
    Write-Log "INFO" "Workbook formula recalculation started."
    # xlCalculationAutomatic = -4105
    $Excel.Calculation = -4105
    $Excel.CalculateFull()
    $calcWaitStart = Get-Date
    while ($Excel.CalculationState -ne 0) {
        if (((Get-Date) - $calcWaitStart).TotalSeconds -ge $TimeoutSeconds) {
            throw "Excel formula recalculation timed out after $TimeoutSeconds seconds."
        }
        Start-Sleep -Milliseconds 100
    }
    Write-Log "INFO" "Workbook formula recalculation completed."
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
# ---------------------------------------------------------------------------
# 【2026-09-02追加】G/I/J/K/O列復元用：セルが「数式も値も無い、完全な空欄」
# かどうかを判定する。.Formula は数式が無くても値の文字列表現を返すため
# 判定に使えない。.HasFormula と .Value2 の両方をチェックすることで、
# 「ルール⑥でクリアされた直後で未着手のセル」だけを正確に検出し、
# 担当者が既に何らかの値・数式を入力済みのセルを誤って上書きしないようにする。
# ---------------------------------------------------------------------------
function Test-CellEmptyForRestore {
    param([Parameter(Mandatory = $true)]$Cell)
    if ([bool]$Cell.HasFormula) {
        return $false
    }
    return (Test-CellBlank $Cell.Value2)
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
# ---------------------------------------------------------------------------
# 【2026-09-02追加】直近の正常な数式セルを上方向へ探索する（案B）。
# .HasFormula で判定するため、「数式は無いが値だけある」セルや「完全な
# 空欄」セルはテンプレートとして採用しない。見つかった場合はその行番号を
# 返し、MinRow（既定2＝ヘッダー行の次）に達するかMaxSearchRowsに達しても
# 見つからない場合は $null を返す（呼び出し側でSkipNoTemplateとして扱う）。
# 読み取り専用処理であり、ワークブックへの変更は一切行わない。
# ---------------------------------------------------------------------------
function Find-NearestFormulaRow {
    param(
        [Parameter(Mandatory = $true)] $Worksheet,
        [Parameter(Mandatory = $true)][int]$Column,
        [Parameter(Mandatory = $true)][int]$StartRow,
        [int]$MinRow = 2,
        [int]$MaxSearchRows = 200
    )
    $searchRow = $StartRow - 1
    $checked = 0
    while ($searchRow -ge $MinRow -and $checked -lt $MaxSearchRows) {
        $cell = $null
        try {
            $cell = $Worksheet.Cells.Item($searchRow, $Column)
            if ([bool]$cell.HasFormula) {
                return $searchRow
            }
        }
        finally {
            Release-Com $cell
        }
        $searchRow--
        $checked++
    }
    return $null
}
# ---------------------------------------------------------------------------
# 【2026-09-02追加】G/I/J/K/O列の復元計画を算出する（読み取り専用）。
# 単価シート参照設定でフラグが「有」の列だけを対象とし、各列について
#   ・Restore         ：対象セルが完全に空欄かつテンプレート行が見つかった
#   ・SkipHasContent  ：対象セルに既に値または数式が残っている（人手入力保護）
#   ・SkipNoTemplate  ：対象セルは空欄だが、上方向にテンプレートが見つからない
# のいずれかを判定して返す。フラグが「無」の列は計画に含めない。
# Auditモードでも安全に呼び出せる（ワークブックを一切変更しない）。
# ---------------------------------------------------------------------------
function Get-RestorePlan {
    param(
        [Parameter(Mandatory = $true)] $Worksheet,
        [Parameter(Mandatory = $true)][int]$Row,
        [Parameter(Mandatory = $true)] $Config,
        [int]$MaxSearchRows = 200
    )
    $columnDefs = @(
        [PSCustomObject]@{ ColName = "G"; Column = 7;  Enabled = [bool]$Config.RestoreG },
        [PSCustomObject]@{ ColName = "I"; Column = 9;  Enabled = [bool]$Config.RestoreI },
        [PSCustomObject]@{ ColName = "J"; Column = 10; Enabled = [bool]$Config.RestoreJ },
        [PSCustomObject]@{ ColName = "K"; Column = 11; Enabled = [bool]$Config.RestoreK },
        [PSCustomObject]@{ ColName = "O"; Column = 15; Enabled = [bool]$Config.RestoreO }
    )
    $plan = New-Object System.Collections.ArrayList
    foreach ($def in $columnDefs) {
        if (-not $def.Enabled) {
            continue
        }
        $isEmpty = $false
        $targetCell = $null
        try {
            $targetCell = $Worksheet.Cells.Item($Row, $def.Column)
            $isEmpty = Test-CellEmptyForRestore $targetCell
        }
        finally {
            Release-Com $targetCell
        }
        if (-not $isEmpty) {
            [void]$plan.Add([PSCustomObject]@{
                ColName     = $def.ColName
                Column      = $def.Column
                Status      = "SkipHasContent"
                TemplateRow = $null
            })
            continue
        }
        $templateRow = Find-NearestFormulaRow -Worksheet $Worksheet -Column $def.Column -StartRow $Row -MaxSearchRows $MaxSearchRows
        if ($null -eq $templateRow) {
            [void]$plan.Add([PSCustomObject]@{
                ColName     = $def.ColName
                Column      = $def.Column
                Status      = "SkipNoTemplate"
                TemplateRow = $null
            })
        }
        else {
            [void]$plan.Add([PSCustomObject]@{
                ColName     = $def.ColName
                Column      = $def.Column
                Status      = "Restore"
                TemplateRow = $templateRow
            })
        }
    }
    return $plan
}
# ---------------------------------------------------------------------------
# 【2026-09-02追加】Get-RestorePlanで算出した計画を実際に適用する（Updateモード
# でのみ呼び出すこと）。Status="Restore"の列だけを対象に、テンプレート行の
# セルをCopy()し、対象セルへPasteSpecial(xlPasteFormulas = -4123)で貼り付ける。
# Formula文字列の直接代入は行わない。これによりExcel自身が相対参照を自動的に
# 行差分ぶん調整するため、テンプレート行と対象行のずれによる誤参照事故を防ぐ。
# xlPasteFormulasは数式のみを貼り付け、貼付先セルの書式（罫線・背景色等）は
# 変更しない。
# ---------------------------------------------------------------------------
function Invoke-RestorePlan {
    param(
        [Parameter(Mandatory = $true)] $Worksheet,
        [Parameter(Mandatory = $true)][int]$Row,
        [Parameter(Mandatory = $true)] $Plan,
        [Parameter(Mandatory = $true)][string]$SheetName,
        [Parameter(Mandatory = $true)][string]$Code
    )
    $restoredCount = 0
    $appRef = $null
    try {
        $appRef = $Worksheet.Application
        foreach ($entry in $Plan) {
            if ($entry.Status -ne "Restore") {
                Write-Log "INFO" ("RestoreSkip Sheet={0} Row={1} Code={2} Col={3} Reason={4}" -f $SheetName, $Row, $Code, $entry.ColName, $entry.Status)
                continue
            }
            $srcCell = $null
            $dstCell = $null
            try {
                $srcCell = $Worksheet.Cells.Item($entry.TemplateRow, $entry.Column)
                $dstCell = $Worksheet.Cells.Item($Row, $entry.Column)
                $srcCell.Copy() | Out-Null
                $dstCell.PasteSpecial(-4123) | Out-Null  # xlPasteFormulas
                $appRef.CutCopyMode = $false
                $restoredCount++
                Write-Log "UPDATE" ("RestoreFormula Sheet={0} Row={1} Code={2} Col={3} TemplateRow={4}" -f $SheetName, $Row, $Code, $entry.ColName, $entry.TemplateRow)
            }
            finally {
                Release-Com $dstCell
                Release-Com $srcCell
            }
        }
    }
    finally {
        Release-Com $appRef
    }
    return $restoredCount
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
# ---------------------------------------------------------------------------
# 【2026-09-02追加】Get-UnitPriceConfig：単価シート参照設定から拠点名で
# 該当行を引く。見つからない場合は $null を返す（呼び出し側でWARNを出し、
# その拠点ではG/I/J/K/O列の復元処理をスキップする）。
# ---------------------------------------------------------------------------
function Get-UnitPriceConfig {
    param(
        [hashtable]$Table,
        [string]$SiteName
    )
    if ($Table.ContainsKey($SiteName)) {
        return $Table[$SiteName]
    }
    return $null
}
# ---------------------------------------------------------------------------
# 拠点別変数表から以下2シートを1回のExcel起動でまとめて読み込む。
#   ・発注点リセット_部署別設定 → 「拠点名|シート名」→リセット日数
#   ・単価シート参照設定       → 「拠点名」→G/I/J/K/O列復元フラグ等
# 変数表を開いて即閉じる。メインのワークブックとは別のExcelインスタンスを
# 使い、影響を分離する（読み取り専用オープン）。
# 【2026-09-02変更】旧Load-ResetDaysTableを本関数に統合し、Excel起動回数を
# 1回に抑えた（単価シート参照設定を追加読み込みするため）。
# ---------------------------------------------------------------------------
function Load-VariableTableData {
    param(
        [string]$Path,
        [int]$DefaultDays
    )
    $result = [PSCustomObject]@{
        ResetDaysTable       = @{}
        UnitPriceConfigTable = @{}
    }
    if ([string]::IsNullOrWhiteSpace($Path)) {
        Write-Log "WARN" "VariableTablePath not specified. All sheets will use DefaultResetDays=$DefaultDays. 単価シート参照設定が読み込めないため、G/I/J/K/O列の復元処理は全拠点でスキップされます。"
        return $result
    }
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        Write-Log "WARN" "VariableTablePath not found: $Path . Falling back to DefaultResetDays=$DefaultDays. 単価シート参照設定も読み込めません。"
        return $result
    }
    $localExcel = $null
    $localWorkbook = $null
    $resetSheet = $null
    $priceSheet = $null
    try {
        $localExcel = New-Object -ComObject Excel.Application
        $localExcel.Visible = $false
        $localExcel.DisplayAlerts = $false
        $localWorkbook = $localExcel.Workbooks.Open($Path, 0, $true)  # ReadOnly=true

        # --- 発注点リセット_部署別設定シート ---
        $resetSheet = $localWorkbook.Worksheets.Item("発注点リセット_部署別設定")
        $lastRow = Get-LastRow $resetSheet
        for ($r = 2; $r -le $lastRow; $r++) {
            $siteCell = $null
            $sheetCell = $null
            $daysCell = $null
            try {
                $siteCell = $resetSheet.Cells.Item($r, 2)   # B列：拠点名
                $sheetCell = $resetSheet.Cells.Item($r, 3)  # C列：シート名(部署)
                $daysCell = $resetSheet.Cells.Item($r, 4)   # D列：リセット日数
                $siteNameVal = Normalize-Code $siteCell.Value2
                $sheetNameVal = Normalize-Code $sheetCell.Value2
                $daysVal = [decimal]0
                if ([string]::IsNullOrWhiteSpace($siteNameVal) -or [string]::IsNullOrWhiteSpace($sheetNameVal)) {
                    continue
                }
                if (-not (Try-Decimal $daysCell.Value2 ([ref]$daysVal))) {
                    continue
                }
                $key = "$siteNameVal|$sheetNameVal"
                $result.ResetDaysTable[$key] = [int]$daysVal
            }
            finally {
                Release-Com $daysCell
                Release-Com $sheetCell
                Release-Com $siteCell
            }
        }

        # --- 単価シート参照設定シート（G/I/J/K/O列 復元要否マスタ）---
        # 拠点コードではなく拠点名（B列）をキーにする。Z3934(外注鉄／
        # 成果連絡表(色物))のように拠点コードが重複するケースがあるため。
        try {
            $priceSheet = $localWorkbook.Worksheets.Item("単価シート参照設定")
        }
        catch {
            Write-Log "WARN" "単価シート参照設定シートが見つかりません。G/I/J/K/O列の復元処理は全拠点でスキップされます。"
            $priceSheet = $null
        }
        if ($null -ne $priceSheet) {
            $priceLastRow = Get-LastRow $priceSheet
            for ($r = 2; $r -le $priceLastRow; $r++) {
                $pSiteNameCell = $null
                $pSheetCell    = $null
                $gCell = $null
                $iCell = $null
                $jCell = $null
                $kCell = $null
                $oCell = $null
                try {
                    $pSiteNameCell = $priceSheet.Cells.Item($r, 2)  # B列：拠点名
                    $pSheetCell    = $priceSheet.Cells.Item($r, 3)  # C列：単価シート名
                    $gCell = $priceSheet.Cells.Item($r, 4)          # D列：G列復元
                    $iCell = $priceSheet.Cells.Item($r, 5)          # E列：I列復元
                    $jCell = $priceSheet.Cells.Item($r, 6)          # F列：J列復元
                    $kCell = $priceSheet.Cells.Item($r, 7)          # G列：K列復元
                    $oCell = $priceSheet.Cells.Item($r, 8)          # H列：O列復元
                    $pSiteName = Normalize-Code $pSiteNameCell.Value2
                    if ([string]::IsNullOrWhiteSpace($pSiteName)) {
                        continue
                    }
                    if ($result.UnitPriceConfigTable.ContainsKey($pSiteName)) {
                        # 想定外だが防御的に：拠点名の重複は先勝ちとしWARNを出す
                        Write-Log "WARN" ("単価シート参照設定: 拠点名の重複を検出しました。Site={0} 。先に読み込んだ行を優先し、この行は無視します。" -f $pSiteName)
                        continue
                    }
                    $config = [PSCustomObject]@{
                        SiteName       = $pSiteName
                        UnitPriceSheet = Normalize-Code $pSheetCell.Text
                        RestoreG       = (Normalize-Code $gCell.Text) -eq "有"
                        RestoreI       = (Normalize-Code $iCell.Text) -eq "有"
                        RestoreJ       = (Normalize-Code $jCell.Text) -eq "有"
                        RestoreK       = (Normalize-Code $kCell.Text) -eq "有"
                        RestoreO       = (Normalize-Code $oCell.Text) -eq "有"
                    }
                    $result.UnitPriceConfigTable[$pSiteName] = $config
                }
                finally {
                    Release-Com $oCell
                    Release-Com $kCell
                    Release-Com $jCell
                    Release-Com $iCell
                    Release-Com $gCell
                    Release-Com $pSheetCell
                    Release-Com $pSiteNameCell
                }
            }
        }
    }
    finally {
        Release-Com $priceSheet
        Release-Com $resetSheet
        if ($null -ne $localWorkbook) {
            try { $localWorkbook.Close($false) } catch {}
        }
        if ($null -ne $localExcel) {
            try { $localExcel.Quit() } catch {}
        }
        Release-Com $localWorkbook
        Release-Com $localExcel
    }
    return $result
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
function Load-ReorderPointMaster {
    # バーコード貼付シート（A列:商品コード, D列:発注点）から、コード→発注点の
    # 対応表を一度だけ読み込む。行ごとにCOM経由でVLOOKUPを実行すると重いため、
    # UsedRange.Value2で一括取得し、以降はメモリ上の辞書引きで済ませる。
    param($Workbook)
    $dict = @{}
    $bcSheet = $null
    $usedRange = $null
    try {
        try {
            $bcSheet = $Workbook.Worksheets.Item("バーコード貼付")
        }
        catch {
            Write-Log "WARN" "バーコード貼付シートが見つかりません。発注点マスタ参照はスキップします。"
            return $dict
        }
        $usedRange = $bcSheet.UsedRange
        $values = $usedRange.Value2
        if ($null -eq $values) {
            return $dict
        }
        $rowCount = $values.GetLength(0)
        $colCount = $values.GetLength(1)
        if ($colCount -lt 4) {
            Write-Log "WARN" "バーコード貼付シートにD列(発注点)がありません。発注点マスタ参照はスキップします。"
            return $dict
        }
        for ($r = 1; $r -le $rowCount; $r++) {
            $rawCode = $values[$r, 1]
            if ([string]::IsNullOrWhiteSpace([string]$rawCode)) {
                continue
            }
            $normCode = Normalize-Code $rawCode
            if ([string]::IsNullOrWhiteSpace($normCode) -or $normCode -eq "商品コード") {
                continue
            }
            $rawReorder = $values[$r, 4]
            $reorderVal = [decimal]0
            if (Try-Decimal $rawReorder ([ref]$reorderVal)) {
                $dict[$normCode] = $reorderVal
            }
        }
    }
    finally {
        Release-Com $usedRange
        Release-Com $bcSheet
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
        # 出力先サブフォルダが無ければ作成する（「作業用CSV」フォルダへの退避のため）
        $trackingFolder = Split-Path -Parent $Path
        if (-not [string]::IsNullOrWhiteSpace($trackingFolder)) {
            if (-not (Test-Path -LiteralPath $trackingFolder)) {
                New-Item -ItemType Directory -Path $trackingFolder -Force | Out-Null
            }
        }
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
        $DiscontinuedTrackingPath = Join-Path (Join-Path $folder "作業用CSV") ("DiscontinuedCodeTracking_{0}.csv" -f $safeSiteName)
    }
    Add-Content -LiteralPath $LogPath -Value ("=" * 80) -Encoding UTF8
    Write-Log "INFO" "Start"
    Write-Log "INFO" ("ScriptVersion={0}" -f $ScriptVersion)
    Write-Log "INFO" ("Site={0}" -f $SiteName)
    Write-Log "INFO" ("Mode={0}" -f $Mode)
    Write-Log "INFO" ("Workbook={0}" -f $WorkbookPath)
    Write-Log "INFO" ("DiscontinuedTrackingPath={0}" -f $DiscontinuedTrackingPath)
    Write-Log "INFO" ("SkipFormulaRestore={0}" -f [bool]$SkipFormulaRestore)
    Write-Log "INFO" ("MaxFormulaSearchRows={0}" -f $MaxFormulaSearchRows)

    # 拠点別変数表を読み込む（発注点リセット_部署別設定・単価シート参照設定の
    # 両方をこの1回の呼び出しで取得する。変数表を開いて即閉じる。メインの
    # ワークブックとは別のExcelインスタンスを使い、影響を分離する）
    $vtData = Load-VariableTableData -Path $VariableTablePath -DefaultDays $DefaultResetDays
    $resetDaysTable = $vtData.ResetDaysTable
    $unitPriceConfigTable = $vtData.UnitPriceConfigTable

    # 【2026-09-02追加】このサイトのG/I/J/K/O列復元設定を1回だけ引く
    if ($SkipFormulaRestore) {
        Write-Log "INFO" "SkipFormulaRestoreが指定されているため、G/I/J/K/O列の復元処理はスキップします（従来通りE列のみ復元）。"
        $unitPriceConfig = $null
    }
    else {
        $unitPriceConfig = Get-UnitPriceConfig -Table $unitPriceConfigTable -SiteName $SiteName
        if ($null -eq $unitPriceConfig) {
            Write-Log "WARN" ("単価シート参照設定にSite={0}の行が見つかりません。この拠点ではG/I/J/K/O列の復元処理をスキップします（従来通りE列のみ復元）。" -f $SiteName)
        }
        else {
            Write-Log "INFO" ("UnitPriceConfig loaded. Site={0} UnitPriceSheet={1} RestoreG={2} RestoreI={3} RestoreJ={4} RestoreK={5} RestoreO={6}" -f `
                $SiteName, $unitPriceConfig.UnitPriceSheet, $unitPriceConfig.RestoreG, $unitPriceConfig.RestoreI, $unitPriceConfig.RestoreJ, $unitPriceConfig.RestoreK, $unitPriceConfig.RestoreO)
        }
    }

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
    # FixReorderPoint等の直前処理でE列(発注点)が変更された場合に備え、
    # G列(発注点割れ)・I列(欠品)の数式結果を最新状態にしてから判定する。
    Invoke-ExcelFullCalculation -Excel $excel
    # バーコード貼付シートD列(発注点)からマスタ辞書を一括読み込み（ルール⑤用）
    $reorderPointMaster = Load-ReorderPointMaster -Workbook $workbook
    Write-Log "INFO" ("ReorderPointMaster loaded. Entries={0}" -f $reorderPointMaster.Count)
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
                    # ルール⑥のN/A判定は、B列の商品コードが空欄でも必ず行う。
                    $codeCell = $sheet.Cells.Item($row, 2)
                    $nameCell = $sheet.Cells.Item($row, 3)
                    $code = Normalize-Code $codeCell.Value2
                    $isNameError = $false
                    try {
                        $isNameError = ($nameCell.Text -match "#N/A")
                    }
                    catch {
                        $isNameError = $false
                    }
                    if ($isNameError) {
                        if (-not [string]::IsNullOrWhiteSpace($code)) {
                            $trackKey = "$sheetName|$code"
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
                        }
                        # --- ルール⑥：N/A行のE:O列クリア ---
                        $clearCandidate = [PSCustomObject]@{
                            SheetName   = $sheetName
                            Row         = $row
                            Code        = $code
                            EOld        = $null
                            ENew        = $null
                            FOld        = $null
                            FNew        = $null
                            HOld        = $null
                            HNew        = $null
                            ClearBlock  = $true
                            RestorePlan = @()
                            Reason      = "N/A行クリア(管理対象外,A列+E:O列)"
                        }
                        [void]$candidates.Add($clearCandidate)
                        continue
                    }
                    if ([string]::IsNullOrWhiteSpace($code)) {
                        continue
                    }
                    $dCell = $sheet.Cells.Item($row, 4)
                    $eCell = $sheet.Cells.Item($row, 5)
                    $fCell = $sheet.Cells.Item($row, 6)
                    $gCell = $sheet.Cells.Item($row, 7)
                    $hCell = $sheet.Cells.Item($row, 8)
                    $iCell = $sheet.Cells.Item($row, 9)
                    $trackKey = "$sheetName|$code"
                    if ($trackingDict.ContainsKey($trackKey) -and $trackingDict[$trackKey].状態 -eq "追跡中") {
                        # --- ルール⑤：廃止コードの復活 ---
                        if ($reorderPointMaster.ContainsKey($code)) {
                            $masterReorderPoint = $reorderPointMaster[$code]

                            # 【2026-09-02追加】G/I/J/K/O列の復元計画を算出する。
                            # E列書き込みが成立する行に限定して計画を立てる
                            # （Eが更新されない行にG/I/J/K/O等の判定・単価列だけ
                            # 復元すると整合性が壊れるため）。読み取り専用処理
                            # なのでAudit/Updateいずれのモードでも安全に実行できる。
                            $restorePlan = New-Object System.Collections.ArrayList
                            if ($null -ne $unitPriceConfig) {
                                $restorePlan = Get-RestorePlan -Worksheet $sheet -Row $row -Config $unitPriceConfig -MaxSearchRows $MaxFormulaSearchRows
                                foreach ($planItem in $restorePlan) {
                                    Write-Log "INFO" ("RestorePlan Sheet={0} Row={1} Code={2} Col={3} Status={4} TemplateRow={5}" -f $sheetName, $row, $code, $planItem.ColName, $planItem.Status, $planItem.TemplateRow)
                                }
                            }

                            $revivalCandidate = [PSCustomObject]@{
                                SheetName   = $sheetName
                                Row         = $row
                                Code        = $code
                                EOld        = $null
                                ENew        = $masterReorderPoint
                                FOld        = $null
                                FNew        = $null
                                HOld        = $null
                                HNew        = $null
                                ClearBlock  = $false
                                RestorePlan = $restorePlan
                                Reason      = "廃止コード復活(E=発注点マスタ値)"
                            }
                            [void]$candidates.Add($revivalCandidate)
                            $entry = $trackingDict[$trackKey]
                            $entry.状態 = "復活済み"
                            $entry.復活日 = $today.ToString("yyyy-MM-dd")
                            $entry.復活時発注点 = [string]$masterReorderPoint
                            Write-Log "INFO" ("DiscontinuedTracking: revival detected Sheet=$sheetName Code=$code NewE(master)=$masterReorderPoint")
                        }
                        else {
                            Write-Log "WARN" ("ReorderPointMaster: code not found Sheet=$sheetName Code=$code. 発注点書き込みをスキップします。G/I/J/K/O列の復元も同時にスキップします。")
                        }
                        continue
                    }
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
                    # --- 発注点割れ系（G列/F列）を、欠品系の反映後の状態で判定 ---
                    if ($gVal -eq 1 -and $fBlank) {
                        $newF = $today
                        [void]$reasons.Add("発注点割れ新規検知(F更新)")
                    }
                    elseif ($gVal -eq 0 -and -not $fBlank) {
                        $newF = $Script:ClearMarker
                        [void]$reasons.Add("発注点割れ解消(Fクリア)")
                    }
                    # --- ルール①③：長期滞留品の自動処理 ---
                    if ($null -eq $newF -and $gVal -eq 1 -and -not $fBlank) {
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
                                    $newE = [decimal]0
                                    $newF = $Script:ClearMarker
                                    $newH = $Script:ClearMarker
                                    [void]$reasons.Add("長期滞留_管理除外(E=0,基準${resetDays}日)")
                                    $rule3Applied = $true
                                }
                                }
                            }
                            if (-not $rule3Applied) {
                                if ($dVal -le 0) {
                                    Write-Log "INFO" (
                                        "LongTermSkip(D=0) Sheet={0} Row={1} Code={2} E={3} FAgeDays={4}" -f
                                        $sheetName, $row, $code, $eVal, $fAgeDays
                                    )
                                }
                                else {
                                    $diff = $eVal - $dVal
                                    if ($diff -lt 0) { $diff = [decimal]0 }
                                    $newE = $diff
                                    $newF = $today
                                    [void]$reasons.Add("長期滞留_発注点差分置換(E=E-D,基準${resetDays}日,F更新)")
                                }
                            }
                        }
                        }
                    }
                    if ($null -eq $newE -and $null -eq $newF -and $null -eq $newH) {
                        continue
                    }
                    $candidate = [PSCustomObject]@{
                        SheetName   = $sheetName
                        Row         = $row
                        Code        = $code
                        EOld        = $eVal
                        ENew        = $newE
                        FOld        = $fRaw
                        FNew        = $newF
                        HOld        = $hRaw
                        HNew        = $newH
                        ClearBlock  = $false
                        RestorePlan = @()
                        Reason      = ($reasons -join "; ")
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
        # 【2026-09-02追加】復元計画のサマリをログに含める
        $restoreDesc = ""
        if ($item.RestorePlan.Count -gt 0) {
            $restoreParts = @()
            foreach ($p in $item.RestorePlan) {
                if ($p.Status -eq "Restore") {
                    $restoreParts += ("{0}<-Row{1}" -f $p.ColName, $p.TemplateRow)
                }
                else {
                    $restoreParts += ("{0}:{1}" -f $p.ColName, $p.Status)
                }
            }
            $restoreDesc = "Restore[" + ($restoreParts -join ",") + "]"
        }
        Write-Log "INFO" (
            "Candidate Sheet={0} Row={1} Code={2} [{3}] {4} {5} {6} {7}" -f
            $item.SheetName, $item.Row, $item.Code, $item.Reason, $eDesc, $fDesc, $hDesc, $restoreDesc
        )
    }
    if ($Mode -eq "Update") {
        if ($candidates.Count -gt $MaxCandidates) {
            $ex = [System.Exception]::new(
                "候補件数が上限を超過しました。件数=$($candidates.Count), 上限=$MaxCandidates"
            )
            $ex.Data["ExitCode"] = 10
            throw $ex
        }
        if ($candidates.Count -gt 0) {
            $backupFolder = Join-Path $folder "Backup"
            if (-not (Test-Path -LiteralPath $backupFolder)) {
                New-Item -ItemType Directory -Path $backupFolder -Force | Out-Null
            }
            $baseName = [System.IO.Path]::GetFileNameWithoutExtension($WorkbookPath)
            $extension = [System.IO.Path]::GetExtension($WorkbookPath)
            $backupName = "{0}_{1}{2}" -f $baseName, (Get-Date -Format "yyyyMMdd"), $extension
            $backupPath = Join-Path $backupFolder $backupName
            Copy-Item -LiteralPath $WorkbookPath -Destination $backupPath -Force
            Write-Log "INFO" ("Backup={0}" -f $backupPath)
            Remove-OldBackups -BackupFolder $backupFolder -BaseName $baseName -Extension $extension -RetentionDays 2
            foreach ($item in $candidates) {
                $targetSheet = $null
                $aCell = $null
                $eCell = $null
                $fCell = $null
                $hCell = $null
                $blockRange = $null
                try {
                    $targetSheet = $workbook.Worksheets.Item($item.SheetName)
                    if ($item.ClearBlock -eq $true) {
                        $aCell = $targetSheet.Cells.Item($item.Row, 1)
                        $aCell.ClearContents()
                        $blockRange = $targetSheet.Range(
                            $targetSheet.Cells.Item($item.Row, 5),
                            $targetSheet.Cells.Item($item.Row, 15)
                        )
                        $blockRange.ClearContents()
                        Write-Log "UPDATE" (
                            "Updated Sheet={0} Row={1} Code={2} [{3}]" -f
                            $item.SheetName, $item.Row, $item.Code, $item.Reason
                        )
                        continue
                    }
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
                    # 【2026-09-02追加】G/I/J/K/O列の復元をここで実行する
                    # （Eの書き込み後・全再計算前。ClearBlock行は上でcontinue
                    # 済みのためここには来ない＝クリア行を誤って復元することはない）
                    if ($item.RestorePlan.Count -gt 0) {
                        [void](Invoke-RestorePlan -Worksheet $targetSheet -Row $item.Row -Plan $item.RestorePlan -SheetName $item.SheetName -Code $item.Code)
                    }
                    Write-Log "UPDATE" (
                        "Updated Sheet={0} Row={1} Code={2} [{3}]" -f
                        $item.SheetName, $item.Row, $item.Code, $item.Reason
                    )
                }
                finally {
                    Release-Com $aCell
                    Release-Com $blockRange
                    Release-Com $hCell
                    Release-Com $fCell
                    Release-Com $eCell
                    Release-Com $targetSheet
                }
            }
            # 適用フェーズでE/F/H・復元数式を書き換えた結果を確実に反映してから保存する。
            Invoke-ExcelFullCalculation -Excel $excel
            $workbook.Save()
            Write-Log "INFO" ("Saved. Updated={0}" -f $candidates.Count)
        }
        else {
            Write-Log "INFO" "No update required."
        }
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
    $exitCode = 1
    if ($null -ne $_.Exception.Data -and $_.Exception.Data.Contains("ExitCode")) {
        $exitCode = [int]$_.Exception.Data["ExitCode"]
    }
    try {
        if (-not [string]::IsNullOrWhiteSpace($script:LogPath)) {
            Write-Log "ERROR" ("ExitCode={0} {1}" -f $exitCode, $errorText)
        }
        else {
            Write-Host ("ERROR ExitCode={0}: {1}" -f $exitCode, $errorText)
        }
    }
    catch {
        Write-Host ("ERROR ExitCode={0}: {1}" -f $exitCode, $errorText)
    }
    Write-Output ("ERROR ExitCode={0}: {1}" -f $exitCode, $errorText)
    [Console]::Error.WriteLine($errorText)
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
    exit $exitCode
}
