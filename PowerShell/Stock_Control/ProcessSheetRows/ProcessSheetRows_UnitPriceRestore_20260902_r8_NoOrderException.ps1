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

  復元方式：
    ×案A（上行(row-1)のFormula文字列をそのままコピー）
      → 上行がたまたま復活行・数式欠落行だった場合に事故る。
    ×Formula文字列の直接代入
      → 相対参照が自動調整されず、テンプレート行を参照したままの誤数式になる。
    ○採用：案B（直近の正常な数式セルを上方向へ探索）
      ＋ 【2026-09-02 r6更新】数式の複製は $dstCell.FormulaR1C1 =
        $srcCell.FormulaR1C1 というプロパティ直接代入で行う（詳細は
        Invoke-RestorePlan関数のコメントを参照）。R1C1形式の相対参照は
        セルの絶対位置に依存しない表記のため、この代入だけで相対参照が
        代入先セルを基準に自動的に再解決される。クリップボード
        （Copy/PasteSpecial/CutCopyMode）は一切使用しない。
      ・テンプレート行の判定は .HasFormula で行う。
      ・上方向探索は既定200行（-MaxFormulaSearchRows で変更可）で打ち切り、
        見つからない場合はSkipNoTemplateとして復元をスキップしログに残す。

  人手入力保護：
    復元対象は「数式が無く、かつ値も空（Test-CellEmptyForRestore）」の
    セルのみに限定する。

  切り戻し・安全装置：
    -SkipFormulaRestore スイッチを指定すると本機能全体を無効化できる
    （旧バージョンと同じ動作に戻す）。
    -MaxFormulaSearchRows で上方探索の最大行数を調整できる（既定200）。

  【2026-09-02追加】バックフィルモード（-BackfillMode）：
    通常のルール⑤は「追跡CSVに“追跡中”として登録済みの商品コードが今回NA
    でなくなった」という遷移のみを検知する。過去にルール⑥でE:O列がクリア
    されたが追跡履歴が無いまま放置された既存行（実例：仙台シート1・
    BN0365SH）は、通常運用では永久に検出されない。
    -BackfillMode を指定すると、追跡CSVの登録有無に関わらず全シートを
    再スキャンし、「C列が正常（非NA）」な行のうち単価シート参照設定で
    フラグが有効な列が実際に空欄の行を検出して数式を復元する。E/F/H列は
    変更しない。

  【2026-09-02 r4 修正】Invoke-RestorePlan内のCOMオブジェクト強制解放バグ修正：
    r3までのInvoke-RestorePlanは、呼び出しのたびに
      $appRef = $Worksheet.Application
      ...
      Release-Com $appRef   （内部で FinalReleaseComObject を実行）
    としていた。$Worksheet.Application はメインループで既に開いている
    Excel.Applicationインスタンス（$excel変数）と同一のCOMオブジェクトを
    指す「別の参照」ではなく、実質的に同一RCW（Runtime Callable Wrapper）
    を指す。FinalReleaseComObjectはそのRCWの参照カウントを問答無用で0に
    するため、BackfillModeで数百件のRestore処理を行った結果、最終的に
    メインの$excel変数が「切断された（分割された）RCW」を参照する状態に
    陥り、全処理完了後の最終全体再計算（Invoke-ExcelFullCalculation）で
    以下の例外が発生してUpdateモードの処理全体が異常終了する不具合が
    実運用データ（仙台在庫帳.xlsx、BackfillMode）で発生した。
      "Calculation" の設定中に例外が発生しました:
      "基になる RCW から分割された COM オブジェクトを使うことはできません。"
    この不具合はUpdateモードでの書き込み後の保存（Workbook.Save()）を
    妨げ、実行済みの更新が失われるおそれがあるため、修正は最優先とした。
    対策（r4時点）：Invoke-RestorePlan は $Worksheet.Application を独自に
    取得・解放することをやめた。ただしr4はCutCopyModeのクリア処理自体は
    残しており、その代入値（$false）がXlCutCopyMode列挙型へキャストできず
    新たな例外が発生した（r5でこれを数値0に修正）。
    【2026-09-02 r6】さらに、Copy/PasteSpecial/CutCopyModeというクリップ
    ボード操作自体を全廃し、$dstCell.FormulaR1C1 = $srcCell.FormulaR1C1
    というプロパティ直接代入に置き換えた。これによりApplicationオブジェクト
    への参照が完全に不要になり、r4/r5で問題になったCutCopyModeの型キャスト
    懸念そのものが構造的に解消された（詳細はInvoke-RestorePlan関数内の
    コメントを参照）。

  【2026-09-02 r7追加】J列テンプレートの単価シート参照先チェック：
    r6までは「単価シート参照設定」から UnitPriceSheet を読み込んでいたが、
    J列の復元テンプレート選定ではその値を照合していなかった。
    r7ではJ列のみ、テンプレート探索時に数式が設定済みのUnitPriceSheet
    （例：仙台→仙台単価）を実際に参照していることを必須条件とする。
    最寄りのJ列数式が別シート参照なら採用せず、探索上限内でさらに上へ探す。
    UnitPriceSheet未設定、または一致するテンプレートが無ければJ列は復元しない。
    Update直前にも同じ参照先を再確認する二重ガードを行う。
    G/I/K/O列、FormulaR1C1方式、BackfillMode、追跡CSV、E列復活、その他の
    既存ルールは変更しない。

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
          ただし D列（在庫数）が 0 以下の場合は対象外
    処理：
      ・E列を「今のE － 今のD」に書き換える（上記の実消費量）
      ・F列は「クリア」ではなく「今日の日付」に更新する
      ・回数制限は設けず、条件を満たすたびに毎回適用する

    除外条件（D列=0）：
      在庫が0まで落ちている場合、消費量が「E以上」であることしか
      分からず超過分は観測できないため、D列<=0の行はE列・F列とも
      一切変更せずスキップする。スキップ時はログに LongTermSkip(D=0) を
      出力する。
    ※ただしルール③の条件も同時に満たす場合はルール③を優先する

  ルール③（発注点割れ・欠品の両方が長期化＝管理除外）
    条件：ルール①の条件に加え、I列=1 かつ H列（欠品継続日時）も
          同じ拠点別リセット日数以上前
    処理：E列を 0 にする（ルール①より優先）

  ルール⑤（廃止コードの復活）
    条件：C列（商品名）が前回まで #N/A エラーだった品番が、今回は正常に
          商品名を表示するようになった
    処理：E列を、バーコード貼付シートD列(発注点マスタ)から取得した本来の
          発注点値に書き換える
    【2026-09-02追加】あわせて、単価シート参照設定でフラグが「有」の
          G/I/J/K/O列についても、直近の正常な数式セルから復元する。


  【2026-09-02追加】業務例外ルール：SHOT場「手配不可」品の管理対象外固定
    背景：
      外注鉄／成果連絡表(色物)では、数値シートC列（商品名のVLOOKUP結果）に
      「手配不可」を含むSHOT場素材は、在庫が残っている場合や在庫量が変動した
      場合でも補充しない、という業務上の取り決めがある。
      通常ロジックのままではE列（発注点）が正数のとき、D<EとなるとG列の
      発注点割れ判定が1になり、補充対象として扱われる可能性がある。

    処理：
      対象拠点を「外注鉄」「成果連絡表(色物)」に限定し、数値シートC列の
      表示値に「手配不可」が含まれる行は、E列（発注点）を0に固定する。
      G列そのものは書き換えず、既存数式の再計算によりG=0（管理対象外）とする。

    優先順位：
      C列#N/Aのルール⑥判定後、廃止コード復活（ルール⑤）・Backfill・
      長期滞留等の通常ロジックより先に判定する。
      したがって「手配不可」品には発注点マスタ値を復元せず、後段処理も
      適用しない。これは在庫ロジックではなく、業務上の明示的な例外処理である。

    安全範囲：
      他拠点には適用しない。対象拠点名とキーワードはスクリプト内の
      $NoOrderExceptionSites / $NoOrderExceptionKeyword に明示する。

  ルール⑥（N/A行のE:O列クリア）
    条件：C列（商品名）が #N/A エラーである行
    処理：A列およびE列～O列の内容をすべてクリアする

  ルール①③の基準日数は、拠点別変数表の「発注点リセット_部署別設定」シート
  から、拠点名・シート名をキーに都度読み取る。
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
    [string]$VariableTablePath = "",
    [string]$DiscontinuedTrackingPath = "",
    [int]$DefaultResetDays = 90,
    [int]$MaxFormulaSearchRows = 200,
    [switch]$SkipFormulaRestore,
    [switch]$BackfillMode
)
Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"
# バージョン管理：改修のたびにこの値を更新する
$ScriptVersion = "2026-09-02-UnitPriceFormulaRestore-r8-StrictBackfill-CF-NoOrderException"
$excel = $null
$workbook = $null
$varExcel = $null
$varWorkbook = $null
$startTime = Get-Date

# 【業務例外】SHOT場の「手配不可」品は補充しない。
# 適用範囲を明示的に限定し、他拠点へ波及させない。
# 判定対象は数値シートC列の「表示された商品名」であり、元CSVや
# バーコード貼付シートを直接判定するものではない。
$NoOrderExceptionSites = @("外注鉄", "成果連絡表(色物)")
$NoOrderExceptionKeyword = "手配不可"
function Write-Log {
    param(
        [string]$Level,
        [string]$Message
    )
    $line = "{0} [{1}] {2}" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss"), $Level, $Message
    Write-Host $line
    Add-Content -LiteralPath $script:LogPath -Value $line -Encoding UTF8
}
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
function Test-CellEmptyForRestore {
    param([Parameter(Mandatory = $true)]$Cell)
    if ([bool]$Cell.HasFormula) {
        return $false
    }
    return (Test-CellBlank $Cell.Value2)
}
function ConvertTo-DateSafe {
    param([object]$Value)
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
function Test-FormulaReferencesSheet {
    param(
        [Parameter(Mandatory = $true)][string]$Formula,
        [Parameter(Mandatory = $true)][string]$ExpectedSheetName
    )
    if ([string]::IsNullOrWhiteSpace($Formula) -or [string]::IsNullOrWhiteSpace($ExpectedSheetName)) {
        return $false
    }

    # Excel数式ではシート名がそのまま記載される場合と、
    # 'Sheet Name'!A1 のようにシングルクォートで囲まれる場合がある。
    # シート名中の ' は '' と二重化されるため、その表記も考慮する。
    $escapedSheetName = $ExpectedSheetName.Replace("'", "''")
    $quotedToken = "'" + $escapedSheetName + "'!"

    # クォート付き参照は文字列完全一致で判定。
    if ($Formula.IndexOf($quotedToken, [System.StringComparison]::OrdinalIgnoreCase) -ge 0) {
        return $true
    }

    # クォート無し参照は、期待シート名の直前が文字・数字・_・. ではない
    # ことも確認し、例：Expected=仙台単価 に対して 旧仙台単価! を
    # 誤一致させない。
    $plainPattern = "(?<![\p{L}\p{N}_.])" + [System.Text.RegularExpressions.Regex]::Escape($ExpectedSheetName) + "!"
    return [System.Text.RegularExpressions.Regex]::IsMatch(
        $Formula,
        $plainPattern,
        [System.Text.RegularExpressions.RegexOptions]::IgnoreCase
    )
}
function Find-NearestFormulaRow {
    param(
        [Parameter(Mandatory = $true)] $Worksheet,
        [Parameter(Mandatory = $true)][int]$Column,
        [Parameter(Mandatory = $true)][int]$StartRow,
        [int]$MinRow = 2,
        [int]$MaxSearchRows = 200,
        [string]$ExpectedReferencedSheet = ""
    )
    $searchRow = $StartRow - 1
    $checked = 0
    while ($searchRow -ge $MinRow -and $checked -lt $MaxSearchRows) {
        $cell = $null
        try {
            $cell = $Worksheet.Cells.Item($searchRow, $Column)
            if ([bool]$cell.HasFormula) {
                if ([string]::IsNullOrWhiteSpace($ExpectedReferencedSheet)) {
                    return $searchRow
                }
                $formula = [string]$cell.Formula
                if (Test-FormulaReferencesSheet -Formula $formula -ExpectedSheetName $ExpectedReferencedSheet) {
                    return $searchRow
                }
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
                ColName                = $def.ColName
                Column                 = $def.Column
                Status                 = "SkipHasContent"
                TemplateRow            = $null
                ExpectedUnitPriceSheet = $null
            })
            continue
        }

        $expectedReferencedSheet = ""
        if ($def.ColName -eq "J") {
            $expectedReferencedSheet = Normalize-Code $Config.UnitPriceSheet
            if ([string]::IsNullOrWhiteSpace($expectedReferencedSheet)) {
                [void]$plan.Add([PSCustomObject]@{
                    ColName                = $def.ColName
                    Column                 = $def.Column
                    Status                 = "SkipUnitPriceSheetNotConfigured"
                    TemplateRow            = $null
                    ExpectedUnitPriceSheet = $null
                })
                continue
            }
        }

        $templateRow = Find-NearestFormulaRow -Worksheet $Worksheet -Column $def.Column -StartRow $Row -MaxSearchRows $MaxSearchRows -ExpectedReferencedSheet $expectedReferencedSheet
        if ($null -eq $templateRow) {
            $status = "SkipNoTemplate"
            if ($def.ColName -eq "J") {
                $status = "SkipNoMatchingUnitPriceTemplate"
            }
            [void]$plan.Add([PSCustomObject]@{
                ColName                = $def.ColName
                Column                 = $def.Column
                Status                 = $status
                TemplateRow            = $null
                ExpectedUnitPriceSheet = $(if ($def.ColName -eq "J") { $expectedReferencedSheet } else { $null })
            })
        }
        else {
            [void]$plan.Add([PSCustomObject]@{
                ColName                = $def.ColName
                Column                 = $def.Column
                Status                 = "Restore"
                TemplateRow            = $templateRow
                ExpectedUnitPriceSheet = $(if ($def.ColName -eq "J") { $expectedReferencedSheet } else { $null })
            })
        }
    }
    return $plan
}
# ---------------------------------------------------------------------------
# 【2026-09-02 r4修正】Invoke-RestorePlan：
# r3までは内部で $Worksheet.Application を都度取得し、処理末尾で
# Release-Com（内部で FinalReleaseComObject）を呼んでいた。
# $Worksheet.Application はメインループで開いている唯一のExcel.Application
# インスタンス（$excel）と同一のRCWを指すため、この解放操作は実質的に
# メインの$excel変数が保持するCOM参照そのものを破壊する。BackfillMode等で
# 本関数が数百回呼ばれるケースでは、最終的に$excelが「分割されたRCW」に
# なり、全処理完了後の最終再計算（Invoke-ExcelFullCalculation）で
# 「基になる RCW から分割された COM オブジェクトを使うことはできません。」
# という例外が発生し、Updateモードの処理全体が異常終了し、保存前の変更が
# 失われるおそれがあった（実運用データで発生確認済み）。
# 修正：$Worksheet.Application を独自取得しない。呼び出し元（メインループ）
# が管理する唯一のExcel.Applicationインスタンスを -ExcelApp として明示的に
# 受け取り、CutCopyModeのクリアにのみ使用する。Release-Comは呼ばない
# （寿命管理はメインループのfinally節に一元化する）。
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# 【2026-09-02 r6修正】Invoke-RestorePlan：Copy/PasteSpecial/CutCopyModeを
# 全廃し、FormulaR1C1プロパティの直接代入に変更した。
#
# 背景（r4/r5で発生した問題）：
#   r3まで：$srcCell.Copy() → $dstCell.PasteSpecial(xlPasteFormulas) →
#           $Application.CutCopyMode = $false という手順で、クリップボード
#           経由の数式コピーを行っていた。
#   r4：Application.CutCopyModeの呼び出し元を$excelに一本化する過程で
#       $ExcelApp.CutCopyMode = $false としたところ、PowerShellのBoolean
#       型がXlCutCopyMode列挙型へキャストできず例外が発生した
#       （実運用データ・BackfillModeで実行時に発生確認）。
#   r5：$false を数値の 0 に変更して急場をしのいだが、根本的にはCopy/
#       PasteSpecial/CutCopyModeというクリップボード操作自体が不要であり、
#       クリップボードの状態管理（CutCopyModeの設定）という不安定要素を
#       抱え続けることになる。
#
# 解決（r6・本修正）：
#   FormulaR1C1プロパティはR1C1形式（相対参照は R[-1]C のような「現在
#   セルからのオフセット」表記）の数式文字列を扱う。相対参照部分は
#   セルの絶対位置に依存しない表記のため、
#       $dstCell.FormulaR1C1 = $srcCell.FormulaR1C1
#   と代入するだけで、Excelは代入先セルの実際の位置を基準にオフセットを
#   再解決する。これはCopy()→PasteSpecial(xlPasteFormulas)が内部で行う
#   相対参照調整と意味的に完全に等価であり（絶対参照部分もCopy/Pasteと
#   同一の解決結果になる）、クリップボードを一切使わない。
#   この変更により、
#     ・CutCopyModeの設定自体が不要になり、型キャストの懸念が構造的に
#       解消される。
#     ・Application オブジェクトへの参照（-ExcelAppパラメータ）が不要に
#       なり、呼び出し元のCOMオブジェクト管理がさらに単純になる。
#   既存の判定ロジック（Get-RestorePlan・Find-NearestFormulaRowによる
#   テンプレート探索、SkipHasContent/SkipNoTemplateの判定）は一切変更
#   していない。
# ---------------------------------------------------------------------------
# 【2026-09-02 r7追加】J列だけは、Get-RestorePlanで選定したテンプレートが
# 変数表のUnitPriceSheetを参照していることをUpdate直前にも再確認する。
# 誤った単価シート参照のJ列数式を書き込まないための二重ガードである。
# ---------------------------------------------------------------------------
function Set-GRestoreConditionalFormatting {
    param(
        [Parameter(Mandatory = $true)] $Worksheet,
        [Parameter(Mandatory = $true)][int]$Row
    )

    # 【r8】復元したG列セルに条件付き書式を設定する。
    #   1 = 黄色、0 = 緑、空白 = 塗りつぶしなし。
    # 復元対象Gセルの条件付き書式をこの3状態ルールに正規化する。
    # 重複ルールを避けるため、同一セルの既存条件付き書式を削除してから再設定する。
    $cell = $null
    $conditions = $null
    try {
        $cell = $Worksheet.Cells.Item($Row, 7)
        $conditions = $cell.FormatConditions
        $conditions.Delete()

        $address = $cell.Address($false, $false)
        $yellowRule = $conditions.Add(2, $null, ('=AND({0}<>"",{0}=1)' -f $address))
        try {
            $yellowRule.Interior.ColorIndex = 6
            $yellowRule.StopIfTrue = $true
        }
        finally {
            Release-Com $yellowRule
        }

        $greenRule = $conditions.Add(2, $null, ('=AND({0}<>"",{0}=0)' -f $address))
        try {
            $greenRule.Interior.ColorIndex = 4
            $greenRule.StopIfTrue = $true
        }
        finally {
            Release-Com $greenRule
        }
    }
    finally {
        Release-Com $conditions
        Release-Com $cell
    }
}

function Invoke-RestorePlan {
    param(
        [Parameter(Mandatory = $true)] $Worksheet,
        [Parameter(Mandatory = $true)][int]$Row,
        [Parameter(Mandatory = $true)] $Plan,
        [Parameter(Mandatory = $true)][string]$SheetName,
        [Parameter(Mandatory = $true)][string]$Code
    )
    $restoredCount = 0
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

            if ($entry.ColName -eq "J") {
                $expectedSheet = Normalize-Code $entry.ExpectedUnitPriceSheet
                $sourceFormula = [string]$srcCell.Formula
                if ([string]::IsNullOrWhiteSpace($expectedSheet) -or -not (Test-FormulaReferencesSheet -Formula $sourceFormula -ExpectedSheetName $expectedSheet)) {
                    Write-Log "WARN" ("RestoreSkip Sheet={0} Row={1} Code={2} Col=J Reason=UnitPriceSheetMismatch TemplateRow={3} ExpectedSheet={4} Formula={5}" -f $SheetName, $Row, $Code, $entry.TemplateRow, $expectedSheet, $sourceFormula)
                    continue
                }
            }

            # クリップボードを使わず、R1C1形式の数式文字列をそのまま
            # 代入する。相対参照はExcelが$dstCellの実位置を基準に
            # 再解決するため、Copy/PasteSpecialと同じ結果になる。
            $dstCell.FormulaR1C1 = $srcCell.FormulaR1C1

            # 【r8】G列を復元した場合だけ、値に応じた条件付き書式を設定する。
            if ($entry.ColName -eq "G") {
                Set-GRestoreConditionalFormatting -Worksheet $Worksheet -Row $Row
                Write-Log "UPDATE" ("RestoreConditionalFormat Sheet={0} Row={1} Code={2} Col=G Rule=1:Yellow,0:Green,Blank:NoFill" -f $SheetName, $Row, $Code)
            }

            $restoredCount++
            Write-Log "UPDATE" ("RestoreFormula Sheet={0} Row={1} Code={2} Col={3} TemplateRow={4}" -f $SheetName, $Row, $Code, $entry.ColName, $entry.TemplateRow)
        }
        finally {
            Release-Com $dstCell
            Release-Com $srcCell
        }
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

        $resetSheet = $localWorkbook.Worksheets.Item("発注点リセット_部署別設定")
        $lastRow = Get-LastRow $resetSheet
        for ($r = 2; $r -le $lastRow; $r++) {
            $siteCell = $null
            $sheetCell = $null
            $daysCell = $null
            try {
                $siteCell = $resetSheet.Cells.Item($r, 2)
                $sheetCell = $resetSheet.Cells.Item($r, 3)
                $daysCell = $resetSheet.Cells.Item($r, 4)
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
                    $pSiteNameCell = $priceSheet.Cells.Item($r, 2)
                    $pSheetCell    = $priceSheet.Cells.Item($r, 3)
                    $gCell = $priceSheet.Cells.Item($r, 4)
                    $iCell = $priceSheet.Cells.Item($r, 5)
                    $jCell = $priceSheet.Cells.Item($r, 6)
                    $kCell = $priceSheet.Cells.Item($r, 7)
                    $oCell = $priceSheet.Cells.Item($r, 8)
                    $pSiteName = Normalize-Code $pSiteNameCell.Value2
                    if ([string]::IsNullOrWhiteSpace($pSiteName)) {
                        continue
                    }
                    if ($result.UnitPriceConfigTable.ContainsKey($pSiteName)) {
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
        $trackingFolder = Split-Path -Parent $Path
        if (-not [string]::IsNullOrWhiteSpace($trackingFolder)) {
            if (-not (Test-Path -LiteralPath $trackingFolder)) {
                New-Item -ItemType Directory -Path $trackingFolder -Force | Out-Null
            }
        }
        $rows | Export-Csv -LiteralPath $Path -NoTypeInformation -Encoding UTF8
    }
}
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
    Write-Log "INFO" ("BackfillMode={0}" -f [bool]$BackfillMode)

    $vtData = Load-VariableTableData -Path $VariableTablePath -DefaultDays $DefaultResetDays
    $resetDaysTable = $vtData.ResetDaysTable
    $unitPriceConfigTable = $vtData.UnitPriceConfigTable

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
    Invoke-ExcelFullCalculation -Excel $excel
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

                    # =================================================================
                    # 【業務例外】SHOT場「手配不可」品
                    # -----------------------------------------------------------------
                    # 外注鉄／成果連絡表(色物)に限り、数値シートC列の表示値に
                    # 「手配不可」が含まれる商品は、在庫量や在庫変動に関係なく
                    # 補充対象外とする。
                    #
                    # E列（発注点）を0に固定し、G列は直接書き換えない。
                    # G列の既存数式（例：D<Eなら1）が再計算されることで
                    # G=0（管理対象外）になる設計を維持する。
                    #
                    # この判定は、廃止コード復活・Backfill・長期滞留等より優先。
                    # 「手配不可」なのに発注点マスタからEを復元してしまうことを
                    # 防ぐため、判定後は必ず continue する。
                    # =================================================================
                    $nameText = ""
                    try {
                        $nameText = [string]$nameCell.Text
                    }
                    catch {
                        $nameText = [string]$nameCell.Value2
                    }
                    $isNoOrderExceptionSite = ($NoOrderExceptionSites -contains $SiteName)
                    $isNoOrderItem = (
                        $isNoOrderExceptionSite -and
                        (-not [string]::IsNullOrWhiteSpace($nameText)) -and
                        ($nameText.IndexOf($NoOrderExceptionKeyword, [System.StringComparison]::OrdinalIgnoreCase) -ge 0)
                    )
                    if ($isNoOrderItem) {
                        $oldEValue = $eCell.Value2
                        $eAlreadyZero = $false
                        $oldEDecimal = [decimal]0
                        if (Try-Decimal $oldEValue ([ref]$oldEDecimal)) {
                            $eAlreadyZero = ($oldEDecimal -eq 0)
                        }

                        # 過去に#N/Aで追跡中だったコードが「手配不可」として復帰した場合も、
                        # ルール⑤の通常復活（発注点マスタ値の復元）は行わない。
                        # 追跡状態だけは復活済みに閉じ、復活時発注点を0として記録する。
                        if ($trackingDict.ContainsKey($trackKey) -and $trackingDict[$trackKey].状態 -eq "追跡中") {
                            $entry = $trackingDict[$trackKey]
                            $entry.状態 = "復活済み"
                            $entry.復活日 = $today.ToString("yyyy-MM-dd")
                            $entry.復活時発注点 = "0"
                            Write-Log "INFO" ("DiscontinuedTracking: no-order item closed as excluded Sheet=$sheetName Code=$code E=0")
                        }

                        if (-not $eAlreadyZero) {
                            $noOrderCandidate = [PSCustomObject]@{
                                SheetName   = $sheetName
                                Row         = $row
                                Code        = $code
                                EOld        = $oldEValue
                                ENew        = [decimal]0
                                FOld        = $null
                                FNew        = $null
                                HOld        = $null
                                HNew        = $null
                                ClearBlock  = $false
                                RestorePlan = @()
                                Reason      = "業務例外_手配不可品(補充対象外,E=0固定)"
                            }
                            [void]$candidates.Add($noOrderCandidate)
                        }
                        else {
                            Write-Log "INFO" ("NoOrderException already E=0 Sheet=$sheetName Row=$row Code=$code")
                        }
                        continue
                    }

                    if ($trackingDict.ContainsKey($trackKey) -and $trackingDict[$trackKey].状態 -eq "追跡中") {
                        if ($reorderPointMaster.ContainsKey($code)) {
                            $masterReorderPoint = $reorderPointMaster[$code]
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
                    if ($BackfillMode -and ($null -ne $unitPriceConfig)) {
                        $backfillPlan = Get-RestorePlan -Worksheet $sheet -Row $row -Config $unitPriceConfig -MaxSearchRows $MaxFormulaSearchRows

                        # 【r8】追跡履歴なしのBackfillは、G/I/J/K/Oが全て復元可能な
                        # 「全欠損」行だけを対象とする。Kだけ等の単独欠損は触らない。
                        # 5列全てが変数表で有効、かつ5列全てStatus=Restoreの場合のみ候補化する。
                        $requiredCols = @("G", "I", "J", "K", "O")
                        $restoreCols = @($backfillPlan | Where-Object { $_.Status -eq "Restore" } | ForEach-Object { $_.ColName })
                        $allConfigured = ([bool]$unitPriceConfig.RestoreG -and [bool]$unitPriceConfig.RestoreI -and [bool]$unitPriceConfig.RestoreJ -and [bool]$unitPriceConfig.RestoreK -and [bool]$unitPriceConfig.RestoreO)
                        $allFiveRestore = $allConfigured
                        foreach ($requiredCol in $requiredCols) {
                            if ($restoreCols -notcontains $requiredCol) {
                                $allFiveRestore = $false
                                break
                            }
                        }

                        if ($allFiveRestore) {
                            foreach ($planItem in $backfillPlan) {
                                Write-Log "INFO" ("BackfillPlan Sheet={0} Row={1} Code={2} Col={3} Status={4} TemplateRow={5}" -f $sheetName, $row, $code, $planItem.ColName, $planItem.Status, $planItem.TemplateRow)
                            }
                            $backfillCandidate = [PSCustomObject]@{
                                SheetName   = $sheetName
                                Row         = $row
                                Code        = $code
                                EOld        = $null
                                ENew        = $null
                                FOld        = $null
                                FNew        = $null
                                HOld        = $null
                                HNew        = $null
                                ClearBlock  = $false
                                RestorePlan = $backfillPlan
                                Reason      = "バックフィル(追跡履歴なし,G/I/J/K/O全欠損のみ)"
                            }
                            [void]$candidates.Add($backfillCandidate)
                        }
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
                    if ($gVal -eq 1 -and $fBlank) {
                        $newF = $today
                        [void]$reasons.Add("発注点割れ新規検知(F更新)")
                    }
                    elseif ($gVal -eq 0 -and -not $fBlank) {
                        $newF = $Script:ClearMarker
                        [void]$reasons.Add("発注点割れ解消(Fクリア)")
                    }
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
                    # 【2026-09-02 r6修正】FormulaR1C1直接代入方式に変更した
                    # ことでクリップボード・Applicationオブジェクトへの依存が
                    # 無くなったため、-ExcelAppパラメータは不要になった。
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
