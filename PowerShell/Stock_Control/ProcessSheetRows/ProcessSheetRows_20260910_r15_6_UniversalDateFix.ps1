<#
===============================================================================
ProcessSheetRows.ps1  (r15.5：Phase3新規登録J列 IFNA必須化)

【2026-09-10 r15.5 緊急修正】Phase3で新規登録するJ列（単価式）をIFNA必須化
  背景：r15.4までは、既存行のJ列 #N/A はr13ロジックで IFNA(...,0) 化するが、
  Phase3で同一実行中に新規挿入した行は既存行スキャン後に作成されるため、
  単価マスタ未登録の商品ではJ列が #N/A、K列（=D*J）も #N/A になる抜けがあった。
  福岡在庫帳の数字シート13/19で発生条件を確認。

  修正方針：
    ・Phase3呼出時だけ Invoke-RestorePlan に -EnsureUnitPriceIFNA を指定する。
    ・J列テンプレートのFormulaR1C1を復元するとき、既にIFNAでなければ
      =IFNA(<元式>,0) で包んでから新規行へ設定する。
    ・既に =IFNA(...) または =_xlfn.IFNA(...) の式は二重化しない。
    ・通常の復活/Backfill等からの Invoke-RestorePlan 呼出しは従来動作のまま。
    ・K列式は変更しない。J=0となるため既存の =D*J が自然に0を返す。
    ・単価シート自体は変更しない。
    ・Phase3の事後検証では、Jテンプレートを使用した場合にJが数式かつ
      IFNA形式であることを必須確認する。

===============================================================================

ProcessSheetRows.ps1  (r13：r10安定版 + 数値シートJ列 #N/A 単価参照保護)


【2026-09-03 r13 追加】数値シートJ列の単価参照 #N/A を0へ吸収
  方針：単価シートは各営業所側がメンテするため、ProcessSheetRowsからは追加・変更しない。
  数値シートJ列（単価）の既存数式が #N/A を返している行だけを検出し、
  その「既存数式そのもの」を IFNA(既存式,0) で包む。

  例：
    =VLOOKUP(B29,仙台単価!$A$1:$E$19573,4,FALSE)
      ↓
    =IFNA(VLOOKUP(B29,仙台単価!$A$1:$E$19573,4,FALSE),0)

  ・単価シートには書き込まない。
  ・正常に単価取得できているJ列数式は変更しない。
  ・J列が #N/A でも数式でない場合は変更せずWARNログのみ。
  ・既にIFNAで包まれている式が #N/A の場合も二重化せずWARNログのみ。
  ・元のFormulaR1C1をそのまま包むため、拠点別の単価シート名や参照範囲を
    ハードコードしない。

【2026-09-03 r10 統合】
  ベース：2026-09-02 r8 StrictBackfill-CF-NoOrderException。
  分派を廃止し、以後は本r10系列を正本とする。

  1) N/A管理対象外処理はr8の実装を継承する。
     C列が#N/Aの行は、A列およびE:O列をClearContentsする。
     （A列クリア問題の修正を含む。）

  2) SHOT場「手配不可」業務例外をハードコードから変数表駆動へ変更。
     拠点別変数表「単価シート参照設定」のJ列
       手配不可品管理対象外 = 有
     かつK列
       補充除外キーワード = 例：手配不可
     のときのみ有効とする。対象拠点名・キーワードをスクリプト内に固定しない。
     C列表示値にK列キーワードを含む行はE=0とし、G列は既存数式の再計算に任せる。

  3) ProcessSheetRows.log は実行開始時に直近3日分（当日＋過去2日）を保持し、
     それより古い実行ブロックを削除する。ログ整理失敗は本体処理を停止させず、
     WARN相当としてコンソールに通知する。

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
      拠点別変数表「単価シート参照設定」のJ列「手配不可品管理対象外」が「有」、
      かつK列「補充除外キーワード」が設定されている拠点のみ対象とする。
      数値シートC列の表示値にK列のキーワードが含まれる行は、E列（発注点）を
      0に固定する。G列そのものは書き換えず、既存数式の再計算により
      G=0（管理対象外）とする。

    優先順位：
      C列#N/Aのルール⑥判定後、廃止コード復活（ルール⑤）・Backfill・
      長期滞留等の通常ロジックより先に判定する。
      したがって「手配不可」品には発注点マスタ値を復元せず、後段処理も
      適用しない。これは在庫ロジックではなく、業務上の明示的な例外処理である。

    安全範囲：
      J列が「有」でない拠点、またはK列が空欄の拠点には適用しない。
      対象範囲は変数表で明示し、スクリプト内には拠点名をハードコードしない。

  ルール⑥（N/A行のE:O列クリア）
    条件：C列（商品名）が #N/A エラーである行
    処理：A列およびE列～O列の内容をすべてクリアする

  ルール①③の基準日数は、拠点別変数表の「発注点リセット_部署別設定」シート
  から、拠点名・シート名をキーに都度読み取る。
===============================================================================


【2026-09-07 r14 統合試験版】新規商品コード待避機能を統合
  原典: UpdateNewProductCodes_20260907_r4.ps1
  方針: r4を直接貼り付けず、判定仕様を独立関数として移植する。
  ・数字シートの既存行走査（2行目開始）は変更しない。
  ・新規商品コード用 managedCodes は同じ行走査中の row>=3 だけで収集する。
  ・候補判定は全数字シート走査完了後に1回だけ行う。
  ・バーコード貼付は1行目見出し、2行目空行のため、必ず3行目から走査する。
  ・新規コード機能は既存Excel.Application/Workbookを共有し、独自Open/Saveしない。
  ・保存は既存変更と新規待避変更をまとめ、Backup→適用→再計算→Save 1回。
  ・新規コード管理_実行 != 有 は当該機能だけSKIPし、r13本体を止めない。
  ・新規コード管理=有かつバーコード貼付欠落は当該機能だけERROR/SKIPし、
    理由付きCSVを出力してr13本体は継続する。
  ・MaxNewProductCandidates超過はUpdate全体を保存前にfail-closed停止する。
  【追加試験】
  Test 11: MaxNewProductCandidates超過時、Backup/セル更新前にExitCode=11で停止し、Workbook.Saveを呼ばない。
  Test 12: バーコード貼付の走査開始をrow=3固定とし、1行目見出し・2行目空行を候補化しない。
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
    [int]$MaxNewProductCandidates = 5000,
    [int]$MaxNewProductMoves = 200,
    [string]$VariableTablePath = "",
    [string]$BarcodeSheetName = "バーコード貼付",
    [string]$NewCodeSheetName = "追加新規商品コード",
    [string]$NewProductOutputDirectory = "",
    [string]$DiscontinuedTrackingPath = "",
    [int]$DefaultResetDays = 90,
    [int]$MaxFormulaSearchRows = 200,
    [switch]$SkipFormulaRestore,
    [switch]$BackfillMode
)
Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"
# バージョン管理：改修のたびにこの値を更新する
$ScriptVersion = "2026-09-10-r15.6-UniversalDateFix"
$excel = $null
$workbook = $null
$varExcel = $null
$varWorkbook = $null
$startTime = Get-Date
$LogRetentionDays = 3

function Trim-LogRetention {
    param(
        [string]$Path,
        [int]$RetentionDays = 3
    )
    if ($RetentionDays -lt 1) {
        return [PSCustomObject]@{ Trimmed = $false; RemovedLines = 0; Cutoff = $null }
    }
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        return [PSCustomObject]@{ Trimmed = $false; RemovedLines = 0; Cutoff = $null }
    }

    $cutoff = (Get-Date).Date.AddDays(-($RetentionDays - 1))
    $tempPath = $Path + ".trimtmp_" + [Guid]::NewGuid().ToString("N")
    $reader = $null
    $writer = $null
    $removed = 0
    $kept = 0
    $keepCurrentBlock = $false
    $pendingSeparator = $null
    try {
        $reader = New-Object System.IO.StreamReader($Path, $true)
        $utf8Bom = New-Object System.Text.UTF8Encoding($true)
        $writer = New-Object System.IO.StreamWriter($tempPath, $false, $utf8Bom)
        while (($line = $reader.ReadLine()) -ne $null) {
            if ($line -eq ("=" * 80)) {
                $pendingSeparator = $line
                $keepCurrentBlock = $false
                continue
            }

            $lineDate = [datetime]::MinValue
            $hasDate = $false
            if ($line -match '^(\d{4}-\d{2}-\d{2})\s') {
                $hasDate = [datetime]::TryParseExact(
                    $Matches[1],
                    "yyyy-MM-dd",
                    [System.Globalization.CultureInfo]::InvariantCulture,
                    [System.Globalization.DateTimeStyles]::None,
                    [ref]$lineDate
                )
            }

            if ($hasDate) {
                $keepCurrentBlock = ($lineDate.Date -ge $cutoff)
                if ($keepCurrentBlock) {
                    if ($null -ne $pendingSeparator) {
                        $writer.WriteLine($pendingSeparator)
                        $kept++
                    }
                    $writer.WriteLine($line)
                    $kept++
                }
                else {
                    if ($null -ne $pendingSeparator) { $removed++ }
                    $removed++
                }
                $pendingSeparator = $null
                continue
            }

            if ($keepCurrentBlock) {
                if ($null -ne $pendingSeparator) {
                    $writer.WriteLine($pendingSeparator)
                    $kept++
                    $pendingSeparator = $null
                }
                $writer.WriteLine($line)
                $kept++
            }
            else {
                if ($null -ne $pendingSeparator) {
                    $removed++
                    $pendingSeparator = $null
                }
                $removed++
            }
        }
    }
    finally {
        if ($null -ne $reader) { $reader.Close() }
        if ($null -ne $writer) { $writer.Close() }
    }

    Move-Item -LiteralPath $tempPath -Destination $Path -Force
    return [PSCustomObject]@{ Trimmed = ($removed -gt 0); RemovedLines = $removed; KeptLines = $kept; Cutoff = $cutoff }
}

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
# r14: 新規商品コード側はr4互換のTrim()のみ。既存Normalize-Codeは変更しない。
function Normalize-NewProductCode {
    param([object]$Value)
    if ($null -eq $Value) {
        return ""
    }
    return ([string]$Value).Trim()
}

function Get-NewProductConfig {
    param(
        [hashtable]$SiteTable,
        [hashtable]$WorkbookTable,
        [string]$RequestedSiteName,
        [string]$WorkbookFileName
    )
    $siteKey = Normalize-NewProductCode $RequestedSiteName
    if (-not [string]::IsNullOrWhiteSpace($siteKey) -and $SiteTable.ContainsKey($siteKey)) {
        return $SiteTable[$siteKey]
    }
    $bookKey = Normalize-NewProductCode $WorkbookFileName
    if (-not [string]::IsNullOrWhiteSpace($bookKey) -and $WorkbookTable.ContainsKey($bookKey)) {
        return $WorkbookTable[$bookKey]
    }
    return $null
}

function Get-ExistingNewProductCodes {
    param(
        [Parameter(Mandatory = $true)]$Workbook,
        [Parameter(Mandatory = $true)][string]$SheetName
    )
    $codes = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)
    $sheet = $null
    $used = $null
    $exists = $false
    try {
        try {
            $sheet = $Workbook.Worksheets.Item($SheetName)
            $exists = $true
        }
        catch {
            return [PSCustomObject]@{ Exists = $false; Codes = $codes }
        }

        $used = $sheet.UsedRange
        $lastRow = [int]$used.Row + [int]$used.Rows.Count - 1
        for ($row = 3; $row -le $lastRow; $row++) {
            $cell = $null
            try {
                $cell = $sheet.Cells.Item($row, 1)
                $code = Normalize-NewProductCode $cell.Value2
                if (-not [string]::IsNullOrWhiteSpace($code)) {
                    [void]$codes.Add($code)
                }
            }
            finally {
                Release-Com $cell
            }
        }
        return [PSCustomObject]@{ Exists = $exists; Codes = $codes }
    }
    finally {
        Release-Com $used
        Release-Com $sheet
    }
}

function Get-NewProductCandidates {
    param(
        [Parameter(Mandatory = $true)]$Workbook,
        [Parameter(Mandatory = $true)][string]$BarcodeSheetName,
        [Parameter(Mandatory = $true)]$ManagedCodes,
        [Parameter(Mandatory = $true)]$ExistingNewCodes
    )
    $barcodeSheet = $null
    $used = $null
    $barcodeByCode = @{}
    try {
        try {
            $barcodeSheet = $Workbook.Worksheets.Item($BarcodeSheetName)
        }
        catch {
            throw "新規商品コード機能: 「$BarcodeSheetName」シートが見つかりません。"
        }

        $used = $barcodeSheet.UsedRange
        $lastRow = [int]$used.Row + [int]$used.Rows.Count - 1

        # 仕様固定: 1行目=見出し、2行目=空行、3行目から実データ。
        # Test 12の見出し行混入防止をコード上でも担保する。
        for ($row = 3; $row -le $lastRow; $row++) {
            $codeCell = $null
            $nameCell = $null
            $stockCell = $null
            try {
                $codeCell = $barcodeSheet.Cells.Item($row, 1)
                $code = Normalize-NewProductCode $codeCell.Value2
                if ([string]::IsNullOrWhiteSpace($code)) {
                    continue
                }
                $nameCell = $barcodeSheet.Cells.Item($row, 2)
                $stockCell = $barcodeSheet.Cells.Item($row, 3)
                $name = ([string]$nameCell.Text).Trim()
                $stock = $stockCell.Value2

                if (-not $barcodeByCode.ContainsKey($code)) {
                    $barcodeByCode[$code] = [PSCustomObject]@{
                        Code           = $code
                        Name           = $name
                        Stock          = $stock
                        SourceRow      = $row
                        DuplicateCount = 1
                    }
                }
                else {
                    $barcodeByCode[$code].DuplicateCount++
                }
            }
            finally {
                Release-Com $stockCell
                Release-Com $nameCell
                Release-Com $codeCell
            }
        }

        $list = New-Object System.Collections.Generic.List[object]
        foreach ($entry in $barcodeByCode.Values) {
            if ($ManagedCodes.Contains($entry.Code)) { continue }
            if ($ExistingNewCodes.Contains($entry.Code)) { continue }
            $state = "未振分"
            if ($entry.DuplicateCount -gt 1) {
                $state = "未振分（バーコード重複確認要）"
            }
            $list.Add([PSCustomObject]@{
                Code           = $entry.Code
                Name           = $entry.Name
                Stock          = $entry.Stock
                SourceRow      = $entry.SourceRow
                DuplicateCount = $entry.DuplicateCount
                State          = $state
            })
        }
        return @($list | Sort-Object Code)
    }
    finally {
        Release-Com $used
        Release-Com $barcodeSheet
    }
}

function Export-NewProductCandidateCsv {
    param(
        [System.Collections.IEnumerable]$Candidates,
        [string]$Directory,
        [string]$ResolvedSiteName,
        [string]$CurrentMode,
        [string]$ErrorReason = ""
    )
    if ([string]::IsNullOrWhiteSpace($Directory)) { return $null }
    if (-not (Test-Path -LiteralPath $Directory)) {
        New-Item -ItemType Directory -Path $Directory -Force | Out-Null
    }
    $safeSite = $ResolvedSiteName -replace '[\\/:*?"<>|]', '_'
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $path = Join-Path $Directory ("NewProductCandidates_{0}_{1}_{2}.csv" -f $safeSite, $CurrentMode, $stamp)

    if (-not [string]::IsNullOrWhiteSpace($ErrorReason)) {
        [PSCustomObject]@{
            拠点       = $ResolvedSiteName
            商品コード = ""
            商品名     = ""
            在庫数     = ""
            バーコード行 = ""
            重複行数   = ""
            状態       = "ERROR"
            理由       = $ErrorReason
        } | Export-Csv -LiteralPath $path -NoTypeInformation -Encoding UTF8
        return $path
    }

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

function Write-NewProductStaging {
    param(
        [Parameter(Mandatory = $true)]$Workbook,
        [Parameter(Mandatory = $true)][string]$SheetName,
        [Parameter(Mandatory = $true)]$Candidates,
        [Parameter(Mandatory = $true)][datetime]$Today
    )
    if (@($Candidates).Count -eq 0) { return 0 }

    $sheet = $null
    $used = $null
    $created = $false
    try {
        try {
            $sheet = $Workbook.Worksheets.Item($SheetName)
        }
        catch {
            $sheet = $Workbook.Worksheets.Add()
            $sheet.Name = $SheetName
            $created = $true
        }

        if ($created) {
            $titleRange = $null
            $headerRange = $null
            $colA = $null; $colB = $null; $colC = $null; $colD = $null; $colE = $null; $colF = $null
            try {
                $titleRange = $sheet.Range("A1:F1")
                [void]$titleRange.Merge()
                $sheet.Cells.Item(1,1).Value2 = "追加新規商品コード（自動検出／人確認用）"
                $headers = @("商品コード", "商品名", "在庫数", "移動先シート", "初回検知日", "状態")
                for ($i = 0; $i -lt $headers.Count; $i++) {
                    $hc = $null
                    try {
                        $hc = $sheet.Cells.Item(2, $i + 1)
                        $hc.Value2 = $headers[$i]
                    }
                    finally { Release-Com $hc }
                }
                $titleRange.Font.Bold = $true
                $headerRange = $sheet.Range("A2:F2")
                $headerRange.Font.Bold = $true
                $headerRange.Interior.ColorIndex = 15

                $colA = $sheet.Columns.Item("A"); $colA.ColumnWidth = 18; $colA.NumberFormat = "@"
                $colB = $sheet.Columns.Item("B"); $colB.ColumnWidth = 34
                $colC = $sheet.Columns.Item("C"); $colC.ColumnWidth = 12
                $colD = $sheet.Columns.Item("D"); $colD.ColumnWidth = 16; $colD.NumberFormat = "@"
                $colE = $sheet.Columns.Item("E"); $colE.ColumnWidth = 14; $colE.NumberFormat = "yyyy/mm/dd"
                $colF = $sheet.Columns.Item("F"); $colF.ColumnWidth = 30
            }
            finally {
                Release-Com $colF; Release-Com $colE; Release-Com $colD; Release-Com $colC; Release-Com $colB; Release-Com $colA
                Release-Com $headerRange; Release-Com $titleRange
            }
            $nextRow = 3
        }
        else {
            $used = $sheet.UsedRange
            $lastUsedRow = [int]$used.Row + [int]$used.Rows.Count - 1
            $nextRow = 3
            for ($row = $lastUsedRow; $row -ge 3; $row--) {
                $cell = $null
                try {
                    $cell = $sheet.Cells.Item($row, 1)
                    $codeAtRow = Normalize-NewProductCode $cell.Value2
                    if (-not [string]::IsNullOrWhiteSpace($codeAtRow)) {
                        $nextRow = $row + 1
                        break
                    }
                }
                finally { Release-Com $cell }
            }
        }

        $added = 0
        foreach ($item in $Candidates) {
            $cells = @()
            try {
                for ($col = 1; $col -le 6; $col++) {
                    $cells += $sheet.Cells.Item($nextRow, $col)
                }
                $cells[0].Value2 = [string]$item.Code
                $cells[1].Value2 = [string]$item.Name
                if ($null -eq $item.Stock -or [string]::IsNullOrWhiteSpace([string]$item.Stock)) {
                    $cells[2].Value2 = ""
                }
                else {
                    $cells[2].Value2 = [string]$item.Stock
                }
                $cells[3].Value2 = ""
                # Excel COM の Value2 へ日付文字列を直接代入すると、
                # 既存セルの書式・状態によって System.InvalidCastException になる場合がある。
                # 全拠点・既存データ共通で安定するよう、Excel標準のシリアル値(OLE Automation Date)
                # を数値として格納し、表示形式のみ yyyy/mm/dd に統一する。
                $cells[4].NumberFormat = "yyyy/mm/dd"
                $cells[4].Value2 = [double]$Today.Date.ToOADate()
                $cells[5].Value2 = [string]$item.State
                $added++
                $nextRow++
            }
            finally {
                foreach ($c in $cells) { Release-Com $c }
            }
        }
        return $added
    }
    finally {
        Release-Com $used
        Release-Com $sheet
    }
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
function Extend-GRestoreConditionalFormatting {
    param(
        [Parameter(Mandatory = $true)] $Worksheet,
        [Parameter(Mandatory = $true)][int]$TemplateRow,
        [Parameter(Mandatory = $true)][int]$DestinationRow,
        [Parameter(Mandatory = $true)][string]$SheetName,
        [Parameter(Mandatory = $true)][string]$Code
    )

    # 【r15.4】既存WorkbookのG列条件付き書式を再生成せず、適用先だけ拡張する。
    # 2026-09-09 本社Sheet16では同じG範囲に2本の2色カラースケール
    # （緑→黄 / 緑→赤）が重なっていることを確認した。
    # 既存ルールの種類・色・しきい値・優先順位は触らない。
    $templateCell = $null
    $destinationCell = $null
    $conditions = $null
    try {
        $templateCell = $Worksheet.Cells.Item($TemplateRow, 7)
        $destinationCell = $Worksheet.Cells.Item($DestinationRow, 7)
        $conditions = $templateCell.FormatConditions

        $conditionCount = [int]$conditions.Count
        if ($conditionCount -le 0) {
            Write-Log "INFO" ("RestoreConditionalFormatExtendSkip Sheet={0} Row={1} Code={2} Col=G Reason=TemplateHasNoConditionalFormatting TemplateRow={3}" -f $SheetName, $DestinationRow, $Code, $TemplateRow)
            return 0
        }

        $destAddress = [string]$destinationCell.Address($true, $true)
        $extendedCount = 0

        for ($i = 1; $i -le $conditionCount; $i++) {
            $condition = $null
            $appliesTo = $null
            $newAppliesTo = $null
            try {
                $condition = $conditions.Item($i)
                $appliesTo = $condition.AppliesTo

                $alreadyApplied = $false
                $areas = $null
                try {
                    $areas = $appliesTo.Areas
                    for ($a = 1; $a -le [int]$areas.Count; $a++) {
                        $area = $null
                        try {
                            $area = $areas.Item($a)
                            $firstRow = [int]$area.Row
                            $lastRow = $firstRow + [int]$area.Rows.Count - 1
                            $firstCol = [int]$area.Column
                            $lastCol = $firstCol + [int]$area.Columns.Count - 1
                            if ($DestinationRow -ge $firstRow -and $DestinationRow -le $lastRow -and
                                7 -ge $firstCol -and 7 -le $lastCol) {
                                $alreadyApplied = $true
                                break
                            }
                        }
                        finally {
                            Release-Com $area
                        }
                    }
                }
                finally {
                    Release-Com $areas
                }

                if ($alreadyApplied) {
                    continue
                }

                $oldAddress = [string]$appliesTo.Address($true, $true)
                $newAppliesTo = $Worksheet.Range(("{0},{1}" -f $oldAddress, $destAddress))
                [void]$condition.ModifyAppliesToRange($newAppliesTo)
                $extendedCount++

                $newAddress = ""
                $verifyAppliesTo = $null
                try {
                    $verifyAppliesTo = $condition.AppliesTo
                    $newAddress = [string]$verifyAppliesTo.Address($true, $true)
                }
                finally {
                    Release-Com $verifyAppliesTo
                }

                Write-Log "UPDATE" ("RestoreConditionalFormatExtend Sheet={0} Row={1} Code={2} Col=G RuleIndex={3} TemplateRow={4} AppliesTo={5}->{6}" -f $SheetName, $DestinationRow, $Code, $i, $TemplateRow, $oldAddress, $newAddress)
            }
            finally {
                Release-Com $newAppliesTo
                Release-Com $appliesTo
                Release-Com $condition
            }
        }

        return $extendedCount
    }
    finally {
        Release-Com $conditions
        Release-Com $destinationCell
        Release-Com $templateCell
    }
}

function Invoke-RestorePlan {
    param(
        [Parameter(Mandatory = $true)] $Worksheet,
        [Parameter(Mandatory = $true)][int]$Row,
        [Parameter(Mandatory = $true)] $Plan,
        [Parameter(Mandatory = $true)][string]$SheetName,
        [Parameter(Mandatory = $true)][string]$Code,
        [switch]$EnsureUnitPriceIFNA
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

            # クリップボードを使わず、R1C1形式の数式文字列を直接代入する。
            # 相対参照はExcelが$dstCellの実位置を基準に再解決する。
            # 【r15.5】Phase3呼出時のJ列だけは、単価マスタ未登録でも #N/A を
            # 出さないよう IFNA(...,0) を必須化する。既にIFNAなら二重化しない。
            if ($entry.ColName -eq "J" -and $EnsureUnitPriceIFNA.IsPresent) {
                $sourceFormulaR1C1 = [string]$srcCell.FormulaR1C1
                if (-not $sourceFormulaR1C1.StartsWith("=")) {
                    throw "Phase3 J列IFNA化: テンプレート数式が不正です。Sheet=$SheetName Row=$Row Code=$Code TemplateRow=$($entry.TemplateRow) FormulaR1C1=$sourceFormulaR1C1"
                }

                if ($sourceFormulaR1C1 -match '^=(?:_xlfn\.)?IFNA\s*\(') {
                    $dstCell.FormulaR1C1 = $sourceFormulaR1C1
                    Write-Log "INFO" ("Phase3UnitPriceIFNA AlreadyProtected Sheet={0} Row={1} Code={2} TemplateRow={3}" -f $SheetName, $Row, $Code, $entry.TemplateRow)
                }
                else {
                    $wrappedFormulaR1C1 = "=IFNA(" + $sourceFormulaR1C1.Substring(1) + ",0)"
                    $dstCell.FormulaR1C1 = $wrappedFormulaR1C1
                    Write-Log "UPDATE" ("Phase3UnitPriceIFNA Wrapped Sheet={0} Row={1} Code={2} TemplateRow={3} Old={4} New={5}" -f $SheetName, $Row, $Code, $entry.TemplateRow, $sourceFormulaR1C1, $wrappedFormulaR1C1)
                }
            }
            else {
                $dstCell.FormulaR1C1 = $srcCell.FormulaR1C1
            }

            # 【r15.4】G列は既存Workbookの条件付き書式を再生成せず、
            # TemplateRowに適用中のルールのAppliesToをDestinationRowまで拡張する。
            if ($entry.ColName -eq "G") {
                [void](Extend-GRestoreConditionalFormatting `
                    -Worksheet $Worksheet `
                    -TemplateRow ([int]$entry.TemplateRow) `
                    -DestinationRow $Row `
                    -SheetName $SheetName `
                    -Code $Code)
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
        ResetDaysTable                = @{}
        UnitPriceConfigTable          = @{}
        NewProductConfigTable         = @{}
        NewProductWorkbookConfigTable = @{}
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
    $newProductSettingSheet = $null
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
                $noOrderEnabledCell = $null
                $noOrderKeywordCell = $null
                try {
                    $pSiteNameCell = $priceSheet.Cells.Item($r, 2)
                    $pSheetCell    = $priceSheet.Cells.Item($r, 3)
                    $gCell = $priceSheet.Cells.Item($r, 4)
                    $iCell = $priceSheet.Cells.Item($r, 5)
                    $jCell = $priceSheet.Cells.Item($r, 6)
                    $kCell = $priceSheet.Cells.Item($r, 7)
                    $oCell = $priceSheet.Cells.Item($r, 8)
                    # r10: J/K列（シート上の物理列10/11）で「手配不可」例外を制御
                    $noOrderEnabledCell = $priceSheet.Cells.Item($r, 10)
                    $noOrderKeywordCell = $priceSheet.Cells.Item($r, 11)
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
                        NoOrderEnabled = (Normalize-Code $noOrderEnabledCell.Text) -eq "有"
                        NoOrderKeyword = Normalize-Code $noOrderKeywordCell.Text
                    }
                    $result.UnitPriceConfigTable[$pSiteName] = $config
                }
                finally {
                    Release-Com $noOrderKeywordCell
                    Release-Com $noOrderEnabledCell
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

        # r14: 同じ変数表Workbookを開いている間に「拠点別設定値」も読む。
        # 新規商品コード機能のためだけにExcel.Applicationを追加生成しない。
        try {
            $newProductSettingSheet = $localWorkbook.Worksheets.Item("拠点別設定値")
        }
        catch {
            Write-Log "WARN" "拠点別設定値シートが見つかりません。新規商品コード機能の設定は利用できません（r13本体は継続します）。"
            $newProductSettingSheet = $null
        }
        if ($null -ne $newProductSettingSheet) {
            $npUsed = $null
            try {
                $npUsed = $newProductSettingSheet.UsedRange
                $npLastCol = [int]$npUsed.Column + [int]$npUsed.Columns.Count - 1
                $npLastRow = [int]$npUsed.Row + [int]$npUsed.Rows.Count - 1
                $npHeaders = @{}
                for ($c = 1; $c -le $npLastCol; $c++) {
                    $hc = $null
                    try {
                        $hc = $newProductSettingSheet.Cells.Item(3, $c)
                        $hn = (Normalize-NewProductCode $hc.Text)
                        if (-not [string]::IsNullOrWhiteSpace($hn) -and -not $npHeaders.ContainsKey($hn)) {
                            $npHeaders[$hn] = $c
                        }
                    }
                    finally { Release-Com $hc }
                }
                foreach ($required in @("拠点名", "在庫帳ブック名", "新規コード管理_実行")) {
                    if (-not $npHeaders.ContainsKey($required)) {
                        Write-Log "WARN" ("拠点別設定値に必要列「{0}」がありません。新規商品コード機能設定を読み込めません。" -f $required)
                        $npHeaders = $null
                        break
                    }
                }
                if ($null -ne $npHeaders) {
                    for ($r = 4; $r -le $npLastRow; $r++) {
                        $siteCell = $null; $bookCell = $null; $enabledCell = $null; $noteCell = $null
                        try {
                            $siteCell = $newProductSettingSheet.Cells.Item($r, $npHeaders["拠点名"])
                            $bookCell = $newProductSettingSheet.Cells.Item($r, $npHeaders["在庫帳ブック名"])
                            $enabledCell = $newProductSettingSheet.Cells.Item($r, $npHeaders["新規コード管理_実行"])
                            $npSite = Normalize-NewProductCode $siteCell.Text
                            $npBook = Normalize-NewProductCode $bookCell.Text
                            $npEnabled = Normalize-NewProductCode $enabledCell.Text
                            $npNote = ""
                            if ($npHeaders.ContainsKey("新規コード管理_備考")) {
                                $noteCell = $newProductSettingSheet.Cells.Item($r, $npHeaders["新規コード管理_備考"])
                                $npNote = Normalize-NewProductCode $noteCell.Text
                            }
                            if ([string]::IsNullOrWhiteSpace($npSite) -and [string]::IsNullOrWhiteSpace($npBook)) { continue }
                            $cfg = [PSCustomObject]@{ SiteName=$npSite; WorkbookFileName=$npBook; Enabled=$npEnabled; Note=$npNote; Row=$r }
                            if (-not [string]::IsNullOrWhiteSpace($npSite) -and -not $result.NewProductConfigTable.ContainsKey($npSite)) {
                                $result.NewProductConfigTable[$npSite] = $cfg
                            }
                            if (-not [string]::IsNullOrWhiteSpace($npBook) -and -not $result.NewProductWorkbookConfigTable.ContainsKey($npBook)) {
                                $result.NewProductWorkbookConfigTable[$npBook] = $cfg
                            }
                        }
                        finally {
                            Release-Com $noteCell; Release-Com $enabledCell; Release-Com $bookCell; Release-Com $siteCell
                        }
                    }
                }
            }
            finally { Release-Com $npUsed }
        }
    }
    finally {
        Release-Com $newProductSettingSheet
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

# ===========================================================================
# r15 / Phase 3: 追加新規商品コード → 指定数字シートへ登録・管理開始
# ---------------------------------------------------------------------------
# 設計原則:
# - Phase 3は既存Workbook/Excel.Applicationを共有し、新規COM Applicationを作らない。
# - Auditでは一切Workbookを変更しない。
# - UpdateではWorkbook.Save()を本体の最後の1回に限定する。
# - Phase 3候補はPhase 1の新規待避追加より前に収集し、同一実行で追加された
#   新規待避行をPhase 3対象にしない。
# - 新規行は「末尾#N/Aクラスタを除外した最終アクティブ商品行の直後」へInsert。
# - A/C/DおよびG/Iは必須。J/K/Oはシート構造に応じて復元。
# - K/Oでシート全体に数式テンプレートが存在しない場合はNotApplicable。
# - L/M/Nの実データ値は近傍行からコピーしない。
#   完全未使用列はNotApplicable、値使用列は空欄+要人手確認ログ。
# - 実行中に必須条件が崩れた場合は例外を投げ、Save前に全変更を破棄する。
# ===========================================================================

function Test-Phase3ExcelErrorValue {
    param([object]$Value)

    if ($null -eq $Value) { return $false }

    if ($Value -is [System.Runtime.InteropServices.ErrorWrapper]) {
        return $true
    }

    # Excel CVErrが数値として返る環境への保険。
    # C列は商品名列なので、この範囲の数値を正常な商品名として扱う必要はない。
    if ($Value -is [int] -or $Value -is [long]) {
        $n = [long]$Value
        if (($n -ge 2000 -and $n -le 2042) -or ($n -le -2146826000 -and $n -ge -2146830000)) {
            return $true
        }
    }

    return $false
}

function Test-Phase3FormulaValue {
    param([object]$Value)
    if ($null -eq $Value) { return $false }
    $s = [string]$Value
    return ($s.StartsWith("="))
}

function Get-Phase3SheetProfile {
    param(
        [Parameter(Mandatory = $true)]$Worksheet
    )

    $used = $null
    $range = $null
    try {
        $used = $Worksheet.UsedRange
        $physicalLastRow = [int]$used.Row + [int]$used.Rows.Count - 1

        if ($physicalLastRow -lt 3) {
            return [PSCustomObject]@{
                PhysicalLastRow = $physicalLastRow
                LastActiveRow = $null
                ActiveRowCount = 0
                TrailingRowsAfterLastActive = 0
                Columns = @{}
            }
        }

        # B:Oを一括取得。巨大な#N/Aクラスタで1セルずつCOM往復しない。
        $rangeAddress = "B3:O{0}" -f $physicalLastRow
        $range = $Worksheet.Range($rangeAddress)
        $values = $range.Value2
        $formulas = $range.FormulaR1C1

        $profiles = @{}
        foreach ($colName in @("G","I","J","K","L","M","N","O")) {
            $profiles[$colName] = [PSCustomObject]@{
                FormulaCount = 0
                NonBlankCount = 0
                FirstFormulaRow = $null
                LastFormulaRow = $null
                TemplateExists = $false
            }
        }

        $columnIndex = @{
            G = 6   # Range B:O 内の1-based index
            I = 8
            J = 9
            K = 10
            L = 11
            M = 12
            N = 13
            O = 14
        }

        $activeCount = 0
        $lastActive = $null
        $rowCount = $physicalLastRow - 2

        for ($idx = 1; $idx -le $rowCount; $idx++) {
            $sheetRow = $idx + 2
            $rawCode = $values[$idx, 1]       # B
            $rawName = $values[$idx, 2]       # C
            $code = Normalize-NewProductCode $rawCode

            $isActive = (
                -not [string]::IsNullOrWhiteSpace($code) -and
                -not (Test-Phase3ExcelErrorValue $rawName)
            )

            if (-not $isActive) { continue }

            $activeCount++
            $lastActive = $sheetRow

            foreach ($colName in $columnIndex.Keys) {
                $ci = [int]$columnIndex[$colName]
                $formulaOrValue = $formulas[$idx, $ci]
                $value = $values[$idx, $ci]

                if (Test-Phase3FormulaValue $formulaOrValue) {
                    $profiles[$colName].FormulaCount++
                    if ($null -eq $profiles[$colName].FirstFormulaRow) {
                        $profiles[$colName].FirstFormulaRow = $sheetRow
                    }
                    $profiles[$colName].LastFormulaRow = $sheetRow
                }

                if ($null -ne $value -and -not [string]::IsNullOrWhiteSpace([string]$value)) {
                    $profiles[$colName].NonBlankCount++
                }
            }
        }

        foreach ($colName in $profiles.Keys) {
            $profiles[$colName].TemplateExists = ($profiles[$colName].FormulaCount -gt 0)
        }

        $tail = 0
        if ($null -ne $lastActive) {
            $tail = $physicalLastRow - [int]$lastActive
        }

        return [PSCustomObject]@{
            PhysicalLastRow = $physicalLastRow
            LastActiveRow = $lastActive
            ActiveRowCount = $activeCount
            TrailingRowsAfterLastActive = $tail
            Columns = $profiles
        }
    }
    finally {
        Release-Com $range
        Release-Com $used
    }
}

function Get-Phase3StagingCandidates {
    param(
        [Parameter(Mandatory = $true)]$Workbook,
        [Parameter(Mandatory = $true)][string]$SheetName
    )

    $sheet = $null
    $used = $null
    $items = New-Object System.Collections.ArrayList

    try {
        try {
            $sheet = $Workbook.Worksheets.Item($SheetName)
        }
        catch {
            Write-Log "INFO" ("Phase3: staging sheet not found. Sheet={0}" -f $SheetName)
            return @()
        }

        $used = $sheet.UsedRange
        $lastRow = [int]$used.Row + [int]$used.Rows.Count - 1

        for ($row = 3; $row -le $lastRow; $row++) {
            $a = $null; $b = $null; $c = $null; $d = $null
            try {
                $a = $sheet.Cells.Item($row, 1)
                $b = $sheet.Cells.Item($row, 2)
                $c = $sheet.Cells.Item($row, 3)
                $d = $sheet.Cells.Item($row, 4)

                $code = Normalize-NewProductCode $a.Value2
                $destination = Normalize-NewProductCode $d.Value2

                # Phase 3候補は「商品コードあり AND 移動先入力済み」の既存待避行だけ。
                if ([string]::IsNullOrWhiteSpace($code) -or [string]::IsNullOrWhiteSpace($destination)) {
                    continue
                }

                [void]$items.Add([PSCustomObject]@{
                    StagingRow = $row
                    Code = $code
                    Name = [string]$b.Value2
                    Stock = $c.Value2
                    DestinationSheet = $destination
                })
            }
            finally {
                Release-Com $d; Release-Com $c; Release-Com $b; Release-Com $a
            }
        }

        return @($items)
    }
    finally {
        Release-Com $used
        Release-Com $sheet
    }
}

function Test-Phase3CodeExistsInNumericSheets {
    param(
        [Parameter(Mandatory = $true)]$Workbook,
        [Parameter(Mandatory = $true)][string]$Code
    )

    $normalized = Normalize-NewProductCode $Code
    if ([string]::IsNullOrWhiteSpace($normalized)) { return $false }

    foreach ($ws in $Workbook.Worksheets) {
        try {
            $name = [string]$ws.Name
            if ($name -notmatch '^\d+$') { continue }

            $used = $null
            $range = $null
            try {
                $used = $ws.UsedRange
                $last = [int]$used.Row + [int]$used.Rows.Count - 1
                if ($last -lt 3) { continue }

                $rangeAddress = "B3:B{0}" -f $last
                $range = $ws.Range($rangeAddress)
                $values = $range.Value2
                $count = $last - 2

                if ($count -eq 1) {
                    if ((Normalize-NewProductCode $values) -eq $normalized) { return $true }
                }
                else {
                    for ($i = 1; $i -le $count; $i++) {
                        if ((Normalize-NewProductCode $values[$i,1]) -eq $normalized) {
                            return $true
                        }
                    }
                }
            }
            finally {
                Release-Com $range
                Release-Com $used
            }
        }
        finally {
            Release-Com $ws
        }
    }

    return $false
}

function Get-Phase3TemplateRow {
    param(
        [Parameter(Mandatory = $true)]$Worksheet,
        [Parameter(Mandatory = $true)][int]$Column,
        [Parameter(Mandatory = $true)][int]$InsertRow,
        [int]$MaxSearchRows = 200,
        [string]$ExpectedReferencedSheet = ""
    )

    return Find-NearestFormulaRow `
        -Worksheet $Worksheet `
        -Column $Column `
        -StartRow $InsertRow `
        -MaxSearchRows $MaxSearchRows `
        -ExpectedReferencedSheet $ExpectedReferencedSheet
}

function Get-Phase3LmnDisposition {
    param(
        [Parameter(Mandatory = $true)]$SheetProfile
    )

    $result = @{}
    foreach ($col in @("L","M","N")) {
        $p = $SheetProfile.Columns[$col]

        if ($p.FormulaCount -eq 0 -and $p.NonBlankCount -eq 0) {
            $result[$col] = "NotApplicable"
        }
        else {
            # 値/数式の意味は商品固有の可能性があるため自動複製しない。
            $result[$col] = "ManualReviewRequired"
        }
    }
    return $result
}


function Get-Phase3NextManagementNumber {
    param(
        [Parameter(Mandatory = $true)]$Worksheet,
        [Parameter(Mandatory = $true)][int]$ReferenceRow
    )

    $cell = $null
    try {
        $cell = $Worksheet.Cells.Item($ReferenceRow, 1)
        $raw = $cell.Value2
        $textValue = Normalize-Code $raw

        $currentNumber = 0
        if ([string]::IsNullOrWhiteSpace($textValue) -or
            -not [int]::TryParse($textValue, [ref]$currentNumber) -or
            $currentNumber -lt 1) {
            return [PSCustomObject]@{
                ReferenceRow = $ReferenceRow
                ReferenceValue = $textValue
                NextNumber = $null
                Status = "ERROR"
                Reason = "A列管理番号を数値として取得できません"
            }
        }

        return [PSCustomObject]@{
            ReferenceRow = $ReferenceRow
            ReferenceValue = $textValue
            NextNumber = ($currentNumber + 1)
            Status = "PASS"
            Reason = "OK"
        }
    }
    finally {
        Release-Com $cell
    }
}

function Get-Phase3Preflight {
    param(
        [Parameter(Mandatory = $true)]$Workbook,
        [Parameter(Mandatory = $true)]$Candidate,
        [Parameter(Mandatory = $true)]$ReorderPointMaster,
        [Parameter(Mandatory = $true)]$ManagedCodes,
        $UnitPriceConfig,
        [Parameter(Mandatory = $true)]$DuplicateCodes,
        [int]$MaxSearchRows = 200
    )

    $reasons = New-Object System.Collections.ArrayList
    $warnings = New-Object System.Collections.ArrayList
    $sheet = $null

    $result = [ordered]@{
        StagingRow = $Candidate.StagingRow
        Code = $Candidate.Code
        Name = $Candidate.Name
        Stock = $Candidate.Stock
        DestinationSheet = $Candidate.DestinationSheet
        DestinationRow = $null
        ReorderPoint = $null
        ValidationStatus = "ERROR"
        ValidationReason = ""
        AReferenceRow = $null
        AReferenceValue = ""
        AManagementNumber = $null
        CTemplateRow = $null
        DTemplateRow = $null
        GTemplateRow = $null
        ITemplateRow = $null
        JTemplateRow = $null
        KTemplateRow = $null
        OTemplateRow = $null
        KDisposition = ""
        ODisposition = ""
        LDisposition = ""
        MDisposition = ""
        NDisposition = ""
        InitialLastActiveRow = $null
        InitialPhysicalLastRow = $null
        InitialTrailingRows = $null
    }

    try {
        if ($Candidate.DestinationSheet -notmatch '^\d+$') {
            [void]$reasons.Add("移動先シートは数字シート名のみ指定可能")
        }

        if ($DuplicateCodes.Contains($Candidate.Code)) {
            [void]$reasons.Add("待避シート内で同一商品コードのPhase3候補が重複")
        }

        if ($ManagedCodes.Contains($Candidate.Code)) {
            [void]$reasons.Add("既に数字シートへ登録済み")
        }

        $masterKey = Normalize-Code $Candidate.Code
        if (-not $ReorderPointMaster.ContainsKey($masterKey)) {
            [void]$reasons.Add("発注点マスタ未登録")
        }
        else {
            $result.ReorderPoint = $ReorderPointMaster[$masterKey]
        }

        if ($reasons.Count -eq 0) {
            try {
                $sheet = $Workbook.Worksheets.Item($Candidate.DestinationSheet)
            }
            catch {
                [void]$reasons.Add("指定数字シート不存在")
            }
        }

        if ($reasons.Count -eq 0) {
            if ([string]$sheet.Name -notmatch '^\d+$') {
                [void]$reasons.Add("指定先が数字シートではない")
            }
        }

        if ($reasons.Count -eq 0) {
            $profile = Get-Phase3SheetProfile -Worksheet $sheet

            $result.InitialLastActiveRow = $profile.LastActiveRow
            $result.InitialPhysicalLastRow = $profile.PhysicalLastRow
            $result.InitialTrailingRows = $profile.TrailingRowsAfterLastActive

            if ($null -eq $profile.LastActiveRow) {
                [void]$reasons.Add("最終アクティブ商品行を決定できない")
            }
            else {
                $insertRow = [int]$profile.LastActiveRow + 1
                $result.DestinationRow = $insertRow

                # RK-10実運用ではPhase3前にSortInventorySheetByGがA列を管理番号へ振り直す。
                # A列は数式テンプレート復元ではなく、最終アクティブ行の管理番号+1を設定する。
                $aPlan = Get-Phase3NextManagementNumber -Worksheet $sheet -ReferenceRow ([int]$profile.LastActiveRow)
                $result.AReferenceRow = $aPlan.ReferenceRow
                $result.AReferenceValue = $aPlan.ReferenceValue
                $result.AManagementNumber = $aPlan.NextNumber
                if ($aPlan.Status -ne "PASS") {
                    [void]$reasons.Add(("A列管理番号決定FAIL: {0} RefRow={1} RefValue={2}" -f $aPlan.Reason, $aPlan.ReferenceRow, $aPlan.ReferenceValue))
                }

                $result.CTemplateRow = Get-Phase3TemplateRow $sheet 3 $insertRow $MaxSearchRows
                $result.DTemplateRow = Get-Phase3TemplateRow $sheet 4 $insertRow $MaxSearchRows

                if ($null -eq $result.CTemplateRow) { [void]$reasons.Add("C列必須数式テンプレートなし") }
                if ($null -eq $result.DTemplateRow) { [void]$reasons.Add("D列必須数式テンプレートなし") }

                if ($null -eq $UnitPriceConfig) {
                    [void]$reasons.Add("G/I復元設定が無効または未設定")
                }
                else {
                    $result.GTemplateRow = Get-Phase3TemplateRow $sheet 7 $insertRow $MaxSearchRows
                    $result.ITemplateRow = Get-Phase3TemplateRow $sheet 9 $insertRow $MaxSearchRows

                    if ($null -eq $result.GTemplateRow) { [void]$reasons.Add("G列必須数式テンプレートなし") }
                    if ($null -eq $result.ITemplateRow) { [void]$reasons.Add("I列必須数式テンプレートなし") }

                    if ([bool]$UnitPriceConfig.RestoreJ) {
                        $expectedPriceSheet = Normalize-Code $UnitPriceConfig.UnitPriceSheet
                        if ([string]::IsNullOrWhiteSpace($expectedPriceSheet)) {
                            [void]$warnings.Add("J列: 単価参照シート未設定")
                        }
                        else {
                            $result.JTemplateRow = Get-Phase3TemplateRow $sheet 10 $insertRow $MaxSearchRows $expectedPriceSheet
                            if ($null -eq $result.JTemplateRow) {
                                [void]$warnings.Add("J列: 指定単価シートを参照する近傍テンプレートなし")
                            }
                        }
                    }

                    if ($profile.Columns["K"].TemplateExists) {
                        $result.KDisposition = "RestoreExpected"
                        $result.KTemplateRow = Get-Phase3TemplateRow $sheet 11 $insertRow $MaxSearchRows
                        if ($null -eq $result.KTemplateRow) {
                            [void]$warnings.Add("K列: シート内テンプレートは存在するが探索範囲内にテンプレートなし")
                        }
                    }
                    else {
                        $result.KDisposition = "NotApplicable"
                    }

                    if ($profile.Columns["O"].TemplateExists) {
                        $result.ODisposition = "RestoreExpected"
                        $result.OTemplateRow = Get-Phase3TemplateRow $sheet 15 $insertRow $MaxSearchRows
                        if ($null -eq $result.OTemplateRow) {
                            [void]$warnings.Add("O列: シート内テンプレートは存在するが探索範囲内にテンプレートなし")
                        }
                    }
                    else {
                        $result.ODisposition = "NotApplicable"
                    }
                }

                $lmn = Get-Phase3LmnDisposition -SheetProfile $profile
                $result.LDisposition = $lmn["L"]
                $result.MDisposition = $lmn["M"]
                $result.NDisposition = $lmn["N"]
            }
        }

        if ($reasons.Count -eq 0) {
            $result.ValidationStatus = "PASS"
            if ($warnings.Count -gt 0) {
                $result.ValidationReason = "WARN: " + ($warnings -join "; ")
            }
            else {
                $result.ValidationReason = "OK"
            }
        }
        else {
            $result.ValidationStatus = "ERROR"
            $result.ValidationReason = ($reasons -join "; ")
            if ($warnings.Count -gt 0) {
                $result.ValidationReason += " / WARN: " + ($warnings -join "; ")
            }
        }

        return [PSCustomObject]$result
    }
    finally {
        Release-Com $sheet
    }
}

function Export-Phase3CandidateCsv {
    param(
        [System.Collections.IEnumerable]$Candidates,
        [string]$Directory,
        [string]$ResolvedSiteName,
        [string]$CurrentMode
    )

    if ([string]::IsNullOrWhiteSpace($Directory)) { return $null }
    if (-not (Test-Path -LiteralPath $Directory)) {
        New-Item -ItemType Directory -Path $Directory -Force | Out-Null
    }

    $safeSite = $ResolvedSiteName -replace '[\\/:*?"<>|]', '_'
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $path = Join-Path $Directory ("Phase3MoveCandidates_{0}_{1}_{2}.csv" -f $safeSite, $CurrentMode, $stamp)

    @($Candidates) | Select-Object `
        StagingRow, Code, Name, Stock, DestinationSheet, DestinationRow, ReorderPoint, `
        ValidationStatus, ValidationReason, `
        AReferenceRow, AReferenceValue, AManagementNumber, CTemplateRow, DTemplateRow, `
        GTemplateRow, ITemplateRow, JTemplateRow, KTemplateRow, OTemplateRow, `
        KDisposition, ODisposition, LDisposition, MDisposition, NDisposition, `
        InitialLastActiveRow, InitialPhysicalLastRow, InitialTrailingRows |
        Export-Csv -LiteralPath $path -NoTypeInformation -Encoding UTF8

    return $path
}

function New-Phase3RestorePlanItem {
    param(
        [string]$ColName,
        [int]$Column,
        [Nullable[int]]$TemplateRow,
        [string]$ExpectedUnitPriceSheet = ""
    )

    if ($null -eq $TemplateRow) { return $null }

    return [PSCustomObject]@{
        ColName = $ColName
        Column = $Column
        Status = "Restore"
        TemplateRow = [int]$TemplateRow
        ExpectedUnitPriceSheet = $(if ([string]::IsNullOrWhiteSpace($ExpectedUnitPriceSheet)) { $null } else { $ExpectedUnitPriceSheet })
    }
}

function Invoke-Phase3Move {
    param(
        [Parameter(Mandatory = $true)]$Workbook,
        [Parameter(Mandatory = $true)]$Excel,
        [Parameter(Mandatory = $true)]$Candidate,
        [Parameter(Mandatory = $true)]$ReorderPointMaster,
        [Parameter(Mandatory = $true)]$UnitPriceConfig,
        [int]$MaxSearchRows = 200
    )

    $sheet = $null
    $rowObj = $null
    $aDst = $null
    $cSrc = $null; $cDst = $null
    $dSrc = $null; $dDst = $null
    $bCell = $null; $eCell = $null; $fCell = $null; $hCell = $null
    $lCell = $null; $mCell = $null; $nCell = $null

    try {
        $sheet = $Workbook.Worksheets.Item($Candidate.DestinationSheet)

        # 同一シート複数候補対応: 毎回現在のWorkbook状態から再計算。
        $profile = Get-Phase3SheetProfile -Worksheet $sheet
        if ($null -eq $profile.LastActiveRow) {
            throw "Phase3 runtime: 最終アクティブ行を決定できません。Sheet=$($Candidate.DestinationSheet) Code=$($Candidate.Code)"
        }

        $insertRow = [int]$profile.LastActiveRow + 1

        # Insert前にA列管理番号と必須数式テンプレートを再検証。Audit時の値/行番号を信頼しない。
        $aPlan = Get-Phase3NextManagementNumber -Worksheet $sheet -ReferenceRow ([int]$profile.LastActiveRow)
        if ($aPlan.Status -ne "PASS" -or $null -eq $aPlan.NextNumber) {
            throw "Phase3 runtime: A列管理番号再検証FAIL。Sheet=$($Candidate.DestinationSheet) Code=$($Candidate.Code) RefRow=$($aPlan.ReferenceRow) RefValue=$($aPlan.ReferenceValue) Reason=$($aPlan.Reason)"
        }
        $managementNumber = [int]$aPlan.NextNumber

        $cTemplate = Get-Phase3TemplateRow $sheet 3 $insertRow $MaxSearchRows
        $dTemplate = Get-Phase3TemplateRow $sheet 4 $insertRow $MaxSearchRows
        $gTemplate = Get-Phase3TemplateRow $sheet 7 $insertRow $MaxSearchRows
        $iTemplate = Get-Phase3TemplateRow $sheet 9 $insertRow $MaxSearchRows

        if ($null -eq $cTemplate -or $null -eq $dTemplate -or
            $null -eq $gTemplate -or $null -eq $iTemplate) {
            throw "Phase3 runtime: 必須テンプレート再検証FAIL。Sheet=$($Candidate.DestinationSheet) Code=$($Candidate.Code) InsertRow=$insertRow C=$cTemplate D=$dTemplate G=$gTemplate I=$iTemplate"
        }

        $expectedPriceSheet = Normalize-Code $UnitPriceConfig.UnitPriceSheet
        $jTemplate = $null
        if ([bool]$UnitPriceConfig.RestoreJ -and -not [string]::IsNullOrWhiteSpace($expectedPriceSheet)) {
            $jTemplate = Get-Phase3TemplateRow $sheet 10 $insertRow $MaxSearchRows $expectedPriceSheet
        }

        $kTemplate = $null
        if ($profile.Columns["K"].TemplateExists) {
            $kTemplate = Get-Phase3TemplateRow $sheet 11 $insertRow $MaxSearchRows
        }

        $oTemplate = $null
        if ($profile.Columns["O"].TemplateExists) {
            $oTemplate = Get-Phase3TemplateRow $sheet 15 $insertRow $MaxSearchRows
        }

        $masterKey = Normalize-Code $Candidate.Code
        if (-not $ReorderPointMaster.ContainsKey($masterKey)) {
            throw "Phase3 runtime: 発注点マスタから商品コードが消失しました。Code=$($Candidate.Code)"
        }
        $reorderPoint = $ReorderPointMaster[$masterKey]

        Write-Log "INFO" (
            "Phase3RuntimePlan Sheet={0} Code={1} InsertRow={2} LastActive={3} PhysicalLast={4} Tail={5} A=ManagementNo:{6}/RefRow:{7}/RefValue:{8} C={9} D={10} G={11} I={12} J={13} K={14} O={15}" -f
            $Candidate.DestinationSheet, $Candidate.Code, $insertRow,
            $profile.LastActiveRow, $profile.PhysicalLastRow, $profile.TrailingRowsAfterLastActive,
            $managementNumber, $aPlan.ReferenceRow, $aPlan.ReferenceValue,
            $cTemplate, $dTemplate, $gTemplate, $iTemplate, $jTemplate, $kTemplate, $oTemplate
        )

        # xlShiftDown=-4121 / CopyOrigin=0 (xlFormatFromLeftOrAbove)
        $rowObj = $sheet.Rows.Item($insertRow)
        [void]$rowObj.Insert(-4121, 0)

        # A列はRK-10 Sort処理後の管理番号体系に合わせ、最終アクティブ行A+1を数値設定。
        # C/D列は従来どおりFormulaR1C1を近傍の有効行から直接複製。
        $aDst = $sheet.Cells.Item($insertRow, 1)
        $aDst.Value2 = [double]$managementNumber

        $cSrc = $sheet.Cells.Item($cTemplate, 3)
        $cDst = $sheet.Cells.Item($insertRow, 3)
        $cDst.FormulaR1C1 = $cSrc.FormulaR1C1

        $dSrc = $sheet.Cells.Item($dTemplate, 4)
        $dDst = $sheet.Cells.Item($insertRow, 4)
        $dDst.FormulaR1C1 = $dSrc.FormulaR1C1

        # 商品コード・発注点。F/Hは管理開始後の通常ロジックに委ねるため空欄。
        $bCell = $sheet.Cells.Item($insertRow, 2)
        $bCell.Value2 = [string]$Candidate.Code

        $eCell = $sheet.Cells.Item($insertRow, 5)
        $eCell.Value2 = [double]$reorderPoint

        $fCell = $sheet.Cells.Item($insertRow, 6)
        [void]$fCell.ClearContents()

        $hCell = $sheet.Cells.Item($insertRow, 8)
        [void]$hCell.ClearContents()

        # L/M/Nは値の意味を推測しない。Insert継承の有無に関係なく空欄を保証。
        $lCell = $sheet.Cells.Item($insertRow, 12); [void]$lCell.ClearContents()
        $mCell = $sheet.Cells.Item($insertRow, 13); [void]$mCell.ClearContents()
        $nCell = $sheet.Cells.Item($insertRow, 14); [void]$nCell.ClearContents()

        $lmn = Get-Phase3LmnDisposition -SheetProfile $profile
        foreach ($col in @("L","M","N")) {
            if ($lmn[$col] -eq "NotApplicable") {
                Write-Log "INFO" ("Phase3NotApplicable Sheet={0} Row={1} Code={2} Column={3}" -f $Candidate.DestinationSheet, $insertRow, $Candidate.Code, $col)
            }
            else {
                Write-Log "WARN" ("Phase3ManualReviewRequired Sheet={0} Row={1} Code={2} Column={3} Action=LeaveBlank" -f $Candidate.DestinationSheet, $insertRow, $Candidate.Code, $col)
            }
        }

        # G/I必須、J/K/Oは実シート構造に応じて復元。
        $plan = New-Object System.Collections.ArrayList
        [void]$plan.Add((New-Phase3RestorePlanItem "G" 7 $gTemplate))
        [void]$plan.Add((New-Phase3RestorePlanItem "I" 9 $iTemplate))

        if ($null -ne $jTemplate) {
            [void]$plan.Add((New-Phase3RestorePlanItem "J" 10 $jTemplate $expectedPriceSheet))
        }
        else {
            Write-Log "WARN" ("Phase3OptionalRestoreSkip Sheet={0} Row={1} Code={2} Column=J Reason=NoMatchingTemplate" -f $Candidate.DestinationSheet, $insertRow, $Candidate.Code)
        }

        if ($profile.Columns["K"].TemplateExists) {
            if ($null -ne $kTemplate) {
                [void]$plan.Add((New-Phase3RestorePlanItem "K" 11 $kTemplate))
            }
            else {
                Write-Log "WARN" ("Phase3OptionalRestoreSkip Sheet={0} Row={1} Code={2} Column=K Reason=TemplateExistsButNotWithinSearchRange" -f $Candidate.DestinationSheet, $insertRow, $Candidate.Code)
            }
        }
        else {
            Write-Log "INFO" ("Phase3NotApplicable Sheet={0} Row={1} Code={2} Column=K" -f $Candidate.DestinationSheet, $insertRow, $Candidate.Code)
        }

        if ($profile.Columns["O"].TemplateExists) {
            if ($null -ne $oTemplate) {
                [void]$plan.Add((New-Phase3RestorePlanItem "O" 15 $oTemplate))
            }
            else {
                Write-Log "WARN" ("Phase3OptionalRestoreSkip Sheet={0} Row={1} Code={2} Column=O Reason=TemplateExistsButNotWithinSearchRange" -f $Candidate.DestinationSheet, $insertRow, $Candidate.Code)
            }
        }
        else {
            Write-Log "INFO" ("Phase3NotApplicable Sheet={0} Row={1} Code={2} Column=O" -f $Candidate.DestinationSheet, $insertRow, $Candidate.Code)
        }

        [void](Invoke-RestorePlan -Worksheet $sheet -Row $insertRow -Plan $plan -SheetName $Candidate.DestinationSheet -Code $Candidate.Code -EnsureUnitPriceIFNA)

        # 同一シート次候補のLastActive判定と成立確認のため、各Move後に再計算。
        Invoke-ExcelFullCalculation -Excel $Excel

        $checkA = $null; $checkB = $null; $checkC = $null; $checkD = $null
        $checkG = $null; $checkI = $null; $checkJ = $null
        try {
            $checkA = $sheet.Cells.Item($insertRow,1)
            $checkB = $sheet.Cells.Item($insertRow,2)
            $checkC = $sheet.Cells.Item($insertRow,3)
            $checkD = $sheet.Cells.Item($insertRow,4)
            $checkG = $sheet.Cells.Item($insertRow,7)
            $checkI = $sheet.Cells.Item($insertRow,9)
            if ($null -ne $jTemplate) {
                $checkJ = $sheet.Cells.Item($insertRow,10)
            }

            $storedCode = Normalize-NewProductCode $checkB.Value2
            $cText = [string]$checkC.Text
            $dText = [string]$checkD.Text

            $aText = Normalize-Code $checkA.Value2
            $aNumber = 0
            $aIsValid = ([int]::TryParse($aText, [ref]$aNumber) -and $aNumber -eq $managementNumber)

            $jFormulaValid = $true
            $jFormulaR1C1 = ""
            if ($null -ne $jTemplate) {
                $jFormulaR1C1 = [string]$checkJ.FormulaR1C1
                $jFormulaValid = ([bool]$checkJ.HasFormula -and $jFormulaR1C1 -match '^=(?:_xlfn\.)?IFNA\s*\(')
            }

            if (-not $aIsValid -or
                $storedCode -ne (Normalize-NewProductCode $Candidate.Code) -or
                -not [bool]$checkC.HasFormula -or
                -not [bool]$checkD.HasFormula -or
                -not [bool]$checkG.HasFormula -or
                -not [bool]$checkI.HasFormula -or
                -not $jFormulaValid -or
                $cText -match '^\s*#N/A\s*$' -or
                $dText -match '^\s*#N/A\s*$') {
                throw "Phase3 runtime validation FAIL. Sheet=$($Candidate.DestinationSheet) Row=$insertRow Code=$($Candidate.Code) A=$aText ExpectedA=$managementNumber C.Text=$cText D.Text=$dText JFormulaR1C1=$jFormulaR1C1"
            }

            Write-Log "PASS" (
                "Phase3MoveValidated Sheet={0} Row={1} Code={2} A={3} C.Text={4} D.Text={5} GFormula={6} IFormula={7} JIFNA={8}" -f
                $Candidate.DestinationSheet, $insertRow, $Candidate.Code, $aNumber, $cText, $dText,
                [bool]$checkG.HasFormula, [bool]$checkI.HasFormula, $jFormulaValid
            )
        }
        finally {
            Release-Com $checkJ; Release-Com $checkI; Release-Com $checkG; Release-Com $checkD
            Release-Com $checkC; Release-Com $checkB; Release-Com $checkA
        }

        return [PSCustomObject]@{
            StagingRow = $Candidate.StagingRow
            Code = $Candidate.Code
            DestinationSheet = $Candidate.DestinationSheet
            DestinationRow = $insertRow
            ManagementNumber = $managementNumber
        }
    }
    finally {
        Release-Com $nCell; Release-Com $mCell; Release-Com $lCell
        Release-Com $hCell; Release-Com $fCell; Release-Com $eCell; Release-Com $bCell
        Release-Com $dDst; Release-Com $dSrc
        Release-Com $cDst; Release-Com $cSrc
        Release-Com $aDst
        Release-Com $rowObj
        Release-Com $sheet
    }
}

function Remove-Phase3StagingRows {
    param(
        [Parameter(Mandatory = $true)]$Workbook,
        [Parameter(Mandatory = $true)][string]$SheetName,
        [Parameter(Mandatory = $true)]$MovedItems
    )

    if (@($MovedItems).Count -eq 0) { return 0 }

    $sheet = $null
    $removed = 0

    try {
        $sheet = $Workbook.Worksheets.Item($SheetName)

        foreach ($item in @($MovedItems | Sort-Object StagingRow -Descending)) {
            $a = $null; $d = $null; $rowObj = $null
            try {
                $a = $sheet.Cells.Item([int]$item.StagingRow,1)
                $d = $sheet.Cells.Item([int]$item.StagingRow,4)

                $currentCode = Normalize-NewProductCode $a.Value2
                $currentDest = Normalize-NewProductCode $d.Value2

                if ($currentCode -ne (Normalize-NewProductCode $item.Code) -or
                    $currentDest -ne (Normalize-NewProductCode $item.DestinationSheet)) {
                    throw "Phase3 staging delete guard FAIL. Row=$($item.StagingRow) ExpectedCode=$($item.Code) CurrentCode=$currentCode ExpectedDest=$($item.DestinationSheet) CurrentDest=$currentDest"
                }

                Write-Log "UPDATE" (
                    "Phase3StagingDelete Row={0} Code={1} DestinationSheet={2} DestinationRow={3}" -f
                    $item.StagingRow, $item.Code, $item.DestinationSheet, $item.DestinationRow
                )

                $rowObj = $sheet.Rows.Item([int]$item.StagingRow)
                [void]$rowObj.Delete()
                $removed++
            }
            finally {
                Release-Com $rowObj; Release-Com $d; Release-Com $a
            }
        }

        return $removed
    }
    finally {
        Release-Com $sheet
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
    $logTrimResult = $null
    try {
        $logTrimResult = Trim-LogRetention -Path $LogPath -RetentionDays $LogRetentionDays
    }
    catch {
        Write-Host ("{0} [WARN] LogRetention cleanup failed. Processing continues. Path={1} Error={2}" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss"), $LogPath, $_.Exception.Message)
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
    Write-Log "INFO" ("MaxNewProductMoves={0}" -f $MaxNewProductMoves)
    Write-Log "INFO" ("BackfillMode={0}" -f [bool]$BackfillMode)
    Write-Log "INFO" ("LogRetentionDays={0}" -f $LogRetentionDays)
    if ($null -ne $logTrimResult -and $null -ne $logTrimResult.Cutoff) {
        Write-Log "INFO" ("LogRetention CutoffDate={0:yyyy-MM-dd} RemovedLines={1}" -f $logTrimResult.Cutoff, $logTrimResult.RemovedLines)
    }

    $vtData = Load-VariableTableData -Path $VariableTablePath -DefaultDays $DefaultResetDays
    $resetDaysTable = $vtData.ResetDaysTable
    $unitPriceConfigTable = $vtData.UnitPriceConfigTable
    $newProductConfigTable = $vtData.NewProductConfigTable
    $newProductWorkbookConfigTable = $vtData.NewProductWorkbookConfigTable
    if ([string]::IsNullOrWhiteSpace($NewProductOutputDirectory)) {
        $NewProductOutputDirectory = Join-Path $folder "作業用CSV"
    }
    $workbookFileName = [System.IO.Path]::GetFileName($WorkbookPath)
    $newProductConfig = Get-NewProductConfig -SiteTable $newProductConfigTable -WorkbookTable $newProductWorkbookConfigTable -RequestedSiteName $SiteName -WorkbookFileName $workbookFileName

    # r10: 単価数式復元設定と業務例外設定は同じ行から読むが、
    # -SkipFormulaRestore は数式復元だけを止める。手配不可ルールは独立して有効。
    $siteConfig = Get-UnitPriceConfig -Table $unitPriceConfigTable -SiteName $SiteName
    $noOrderEnabled = $false
    $noOrderKeyword = ""
    if ($null -eq $siteConfig) {
        Write-Log "WARN" ("単価シート参照設定にSite={0}の行が見つかりません。G/I/J/K/O列復元および手配不可業務例外は無効です。" -f $SiteName)
    }
    else {
        $noOrderKeyword = Normalize-Code $siteConfig.NoOrderKeyword
        $noOrderEnabled = ([bool]$siteConfig.NoOrderEnabled -and (-not [string]::IsNullOrWhiteSpace($noOrderKeyword)))
        if ([bool]$siteConfig.NoOrderEnabled -and [string]::IsNullOrWhiteSpace($noOrderKeyword)) {
            Write-Log "WARN" ("NoOrderConfig: Site={0} は手配不可品管理対象外=有ですが、補充除外キーワードが空欄のため無効化します。" -f $SiteName)
        }
        Write-Log "INFO" ("SiteConfig loaded. Site={0} UnitPriceSheet={1} RestoreG={2} RestoreI={3} RestoreJ={4} RestoreK={5} RestoreO={6} NoOrderEnabled={7} NoOrderKeyword={8}" -f `
            $SiteName, $siteConfig.UnitPriceSheet, $siteConfig.RestoreG, $siteConfig.RestoreI, $siteConfig.RestoreJ, $siteConfig.RestoreK, $siteConfig.RestoreO, $noOrderEnabled, $noOrderKeyword)
    }

    if ($SkipFormulaRestore) {
        Write-Log "INFO" "SkipFormulaRestoreが指定されているため、G/I/J/K/O列の復元処理はスキップします（手配不可業務例外は変数表設定に従い継続します）。"
        $unitPriceConfig = $null
    }
    else {
        $unitPriceConfig = $siteConfig
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
    $unitPriceFormulaFixes = New-Object System.Collections.ArrayList
    $sheetNames = New-Object System.Collections.ArrayList
    # r14: 新規商品コード判定用。既存数字シート走査の中で同時収集し、二重走査しない。
    $managedCodes = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)
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
                $jCell = $null
                try {
                    $codeCell = $sheet.Cells.Item($row, 2)
                    $nameCell = $sheet.Cells.Item($row, 3)
                    $code = Normalize-Code $codeCell.Value2
                    # r14: r13本体は2行目からの走査を維持。新規コード用集合だけ3行目以降。
                    if ($row -ge 3) {
                        $newProductCode = Normalize-NewProductCode $codeCell.Value2
                        if (-not [string]::IsNullOrWhiteSpace($newProductCode)) {
                            [void]$managedCodes.Add($newProductCode)
                        }
                    }
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
                    $jCell = $sheet.Cells.Item($row, 10)
                    $trackKey = "$sheetName|$code"

                    # =================================================================
                    # 【r13】J列（単価）VLOOKUP等の #N/A を IFNA(...,0) で吸収
                    # -----------------------------------------------------------------
                    # 単価シート自体は営業所管理のため一切変更しない。
                    # 現在 #N/A を返しているJ列の「既存数式」だけを対象に、
                    # FormulaR1C1をそのまま IFNA で包む。正常行は触らない。
                    # =================================================================
                    $jIsNA = $false
                    try {
                        $jIsNA = ([string]$jCell.Text -match "#N/A")
                    }
                    catch {
                        $jIsNA = $false
                    }
                    if ($jIsNA) {
                        $jHasFormula = $false
                        try { $jHasFormula = [bool]$jCell.HasFormula } catch { $jHasFormula = $false }
                        if ($jHasFormula) {
                            $jFormulaR1C1 = [string]$jCell.FormulaR1C1
                            if ($jFormulaR1C1 -match '^=IFNA\s*\(') {
                                Write-Log "WARN" ("UnitPriceIFNA SkipAlreadyIFNA Sheet={0} Row={1} Code={2} FormulaR1C1={3}" -f $sheetName, $row, $code, $jFormulaR1C1)
                            }
                            elseif ($jFormulaR1C1.StartsWith("=")) {
                                $wrappedFormulaR1C1 = "=IFNA(" + $jFormulaR1C1.Substring(1) + ",0)"
                                $fix = [PSCustomObject]@{
                                    SheetName       = $sheetName
                                    Row             = $row
                                    Code            = $code
                                    OldFormulaR1C1  = $jFormulaR1C1
                                    NewFormulaR1C1  = $wrappedFormulaR1C1
                                }
                                [void]$unitPriceFormulaFixes.Add($fix)
                                Write-Log "INFO" ("UnitPriceIFNA Candidate Sheet={0} Row={1} Code={2} Old={3} New={4}" -f $sheetName, $row, $code, $jFormulaR1C1, $wrappedFormulaR1C1)
                            }
                            else {
                                Write-Log "WARN" ("UnitPriceIFNA SkipUnexpectedFormula Sheet={0} Row={1} Code={2} FormulaR1C1={3}" -f $sheetName, $row, $code, $jFormulaR1C1)
                            }
                        }
                        else {
                            Write-Log "WARN" ("UnitPriceIFNA SkipNoFormula Sheet={0} Row={1} Code={2} JText=#N/A" -f $sheetName, $row, $code)
                        }
                    }

                    # =================================================================
                    # 【業務例外】SHOT場「手配不可」品
                    # -----------------------------------------------------------------
                    # 変数表「単価シート参照設定」で NoOrderEnabled=True の拠点に限り、
                    # 数値シートC列の表示値に NoOrderKeyword が含まれる商品は、
                    # 在庫量や在庫変動に関係なく補充対象外とする。
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
                    $isNoOrderItem = (
                        $noOrderEnabled -and
                        (-not [string]::IsNullOrWhiteSpace($nameText)) -and
                        ($nameText.IndexOf($noOrderKeyword, [System.StringComparison]::OrdinalIgnoreCase) -ge 0)
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
                    Release-Com $jCell
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
    # =====================================================================
    # r14 新規商品コード候補検出フェーズ
    # 必ず全数字シートの走査完了後に実施する（途中判定禁止）。
    # =====================================================================
    $newProductCandidates = @()
    $newProductFeatureEnabled = $false
    $newProductFeatureError = $false
    $newProductErrorReason = ""
    $newProductCsvPath = $null

    if ($null -eq $newProductConfig) {
        $newProductFeatureError = $true
        $newProductErrorReason = "変数表の拠点別設定値で新規商品コード管理設定を特定できません。Site=$SiteName Workbook=$workbookFileName"
        Write-Log "ERROR" ("NewProductStaging: {0} 新規コード機能のみSKIPし、r13本体を継続します。" -f $newProductErrorReason)
        $newProductCsvPath = Export-NewProductCandidateCsv -Candidates @() -Directory $NewProductOutputDirectory -ResolvedSiteName $SiteName -CurrentMode $Mode -ErrorReason $newProductErrorReason
    }
    elseif ($newProductConfig.Enabled -ne "有") {
        $skipMsg = "新規コード管理_実行が「有」ではないため新規コード機能のみスキップします。"
        if (-not [string]::IsNullOrWhiteSpace($newProductConfig.Note)) { $skipMsg += " 備考=$($newProductConfig.Note)" }
        Write-Log "SKIP" ("NewProductStaging: {0}" -f $skipMsg)
    }
    else {
        $newProductFeatureEnabled = $true
        try {
            $existingNewResult = Get-ExistingNewProductCodes -Workbook $workbook -SheetName $NewCodeSheetName
            if ($existingNewResult.Exists) {
                Write-Log "INFO" ("NewProductStaging: 既存「{0}」コード数={1}" -f $NewCodeSheetName, $existingNewResult.Codes.Count)
            }
            else {
                Write-Log "INFO" ("NewProductStaging: 「{0}」シートは未作成です。" -f $NewCodeSheetName)
            }

            $newProductCandidates = @(Get-NewProductCandidates -Workbook $workbook -BarcodeSheetName $BarcodeSheetName -ManagedCodes $managedCodes -ExistingNewCodes $existingNewResult.Codes)
            $dupNewProductCount = @($newProductCandidates | Where-Object { $_.DuplicateCount -gt 1 }).Count
            Write-Log "INFO" ("NewProductStaging: ManagedCodes={0} 未管理新規候補={1} バーコード重複候補={2}" -f $managedCodes.Count, $newProductCandidates.Count, $dupNewProductCount)
            $newProductCsvPath = Export-NewProductCandidateCsv -Candidates $newProductCandidates -Directory $NewProductOutputDirectory -ResolvedSiteName $SiteName -CurrentMode $Mode
        }
        catch {
            # レビュー合意修正1: バーコード貼付欠落等は新規コード機能だけERROR/SKIP。
            # r13本体の候補・更新処理は継続する。候補検出不能を理由付きCSVにも残す。
            $newProductFeatureError = $true
            $newProductErrorReason = $_.Exception.Message
            $newProductCandidates = @()
            Write-Log "ERROR" ("NewProductStaging: {0} 新規コード機能のみSKIPし、r13本体を継続します。" -f $newProductErrorReason)
            $newProductCsvPath = Export-NewProductCandidateCsv -Candidates @() -Directory $NewProductOutputDirectory -ResolvedSiteName $SiteName -CurrentMode $Mode -ErrorReason $newProductErrorReason
        }
    }
    if ($null -ne $newProductCsvPath) {
        Write-Log "INFO" ("NewProductStaging: 候補CSV={0}" -f $newProductCsvPath)
    }

    # =====================================================================
    # r15 Phase 3 候補収集 / Preflight
    # Phase 1のWrite-NewProductStagingより前に収集するため、同一実行で
    # 新たに待避された行はPhase 3対象にならない。
    # =====================================================================
    $phase3RawCandidates = @()
    $phase3Preflight = @()
    $phase3Eligible = @()
    $phase3CandidateCsvPath = $null
    $phase3FeatureEnabled = (
        $newProductFeatureEnabled -and
        (-not $newProductFeatureError)
    )

    if ($phase3FeatureEnabled) {
        try {
            $phase3RawCandidates = @(Get-Phase3StagingCandidates -Workbook $workbook -SheetName $NewCodeSheetName)

            $codeCounts = @{}
            foreach ($pc in $phase3RawCandidates) {
                $key = Normalize-NewProductCode $pc.Code
                if (-not $codeCounts.ContainsKey($key)) { $codeCounts[$key] = 0 }
                $codeCounts[$key]++
            }

            $duplicateCodes = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)
            foreach ($key in $codeCounts.Keys) {
                if ($codeCounts[$key] -gt 1) { [void]$duplicateCodes.Add($key) }
            }

            $preflightList = New-Object System.Collections.ArrayList
            foreach ($pc in $phase3RawCandidates) {
                $pf = Get-Phase3Preflight `
                    -Workbook $workbook `
                    -Candidate $pc `
                    -ReorderPointMaster $reorderPointMaster `
                    -ManagedCodes $managedCodes `
                    -UnitPriceConfig $unitPriceConfig `
                    -DuplicateCodes $duplicateCodes `
                    -MaxSearchRows $MaxFormulaSearchRows

                [void]$preflightList.Add($pf)

                Write-Log "INFO" (
                    "Phase3Preflight StagingRow={0} Code={1} Dest={2} DestRow={3} Status={4} Reason={5} A=ManagementNo:{6}/RefRow:{7}/RefValue:{8} C={9} D={10} G={11} I={12} J={13} K={14}/{15} O={16}/{17} L={18} M={19} N={20}" -f
                    $pf.StagingRow, $pf.Code, $pf.DestinationSheet, $pf.DestinationRow,
                    $pf.ValidationStatus, $pf.ValidationReason,
                    $pf.AManagementNumber, $pf.AReferenceRow, $pf.AReferenceValue,
                    $pf.CTemplateRow, $pf.DTemplateRow,
                    $pf.GTemplateRow, $pf.ITemplateRow, $pf.JTemplateRow,
                    $pf.KDisposition, $pf.KTemplateRow,
                    $pf.ODisposition, $pf.OTemplateRow,
                    $pf.LDisposition, $pf.MDisposition, $pf.NDisposition
                )
            }

            $phase3Preflight = @($preflightList)
            $phase3Eligible = @($phase3Preflight | Where-Object { $_.ValidationStatus -eq "PASS" })

            $phase3CandidateCsvPath = Export-Phase3CandidateCsv `
                -Candidates $phase3Preflight `
                -Directory $NewProductOutputDirectory `
                -ResolvedSiteName $SiteName `
                -CurrentMode $Mode

            Write-Log "INFO" (
                "Phase3Summary Raw={0} Eligible={1} Error={2} MaxNewProductMoves={3} CSV={4}" -f
                $phase3RawCandidates.Count,
                $phase3Eligible.Count,
                @($phase3Preflight | Where-Object { $_.ValidationStatus -ne "PASS" }).Count,
                $MaxNewProductMoves,
                $phase3CandidateCsvPath
            )
        }
        catch {
            # Preflightの想定外例外はPhase3だけを黙って無効化しない。
            # Updateで安全性判断不能なため本体catchへ送りSave前停止する。
            throw "Phase3 preflight failed: $($_.Exception.Message)"
        }
    }
    else {
        Write-Log "SKIP" ("Phase3: 新規商品コード機能が無効/ERRORのためPhase3もSKIPします。Enabled={0} FeatureError={1}" -f $newProductFeatureEnabled, $newProductFeatureError)
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
    Write-Log "INFO" ("UnitPriceIFNACandidates={0}" -f $unitPriceFormulaFixes.Count)
    Write-Log "INFO" ("NewProductCandidates={0} MaxNewProductCandidates={1} Enabled={2} FeatureError={3}" -f $newProductCandidates.Count, $MaxNewProductCandidates, $newProductFeatureEnabled, $newProductFeatureError)
    Write-Log "INFO" ("Phase3Candidates Raw={0} Eligible={1} MaxNewProductMoves={2}" -f $phase3RawCandidates.Count, $phase3Eligible.Count, $MaxNewProductMoves)
    if ($Mode -eq "Update") {
        $totalCandidateCount = $candidates.Count + $unitPriceFormulaFixes.Count
        # Test 11: 新規商品コード候補上限は既存MaxCandidatesと独立。
        # Backup/セル更新より前に判定し、超過時はWorkbookを一切Saveしない。
        if ($newProductFeatureEnabled -and (-not $newProductFeatureError) -and $newProductCandidates.Count -gt $MaxNewProductCandidates) {
            $ex = [System.Exception]::new(
                "新規商品コード候補件数が上限を超過しました。候補=$($newProductCandidates.Count), 上限=$MaxNewProductCandidates。保存前にfail-closed停止します。"
            )
            $ex.Data["ExitCode"] = 11
            throw $ex
        }
        if ($phase3Eligible.Count -gt $MaxNewProductMoves) {
            $ex = [System.Exception]::new(
                "Phase3移動候補件数が上限を超過しました。候補=$($phase3Eligible.Count), 上限=$MaxNewProductMoves。保存前にfail-closed停止します。"
            )
            $ex.Data["ExitCode"] = 12
            throw $ex
        }
        if ($totalCandidateCount -gt $MaxCandidates) {
            $ex = [System.Exception]::new(
                "候補件数が上限を超過しました。通常=$($candidates.Count), J列IFNA=$($unitPriceFormulaFixes.Count), 合計=$totalCandidateCount, 上限=$MaxCandidates"
            )
            $ex.Data["ExitCode"] = 10
            throw $ex
        }
        $hasNewProductChanges = ($newProductFeatureEnabled -and (-not $newProductFeatureError) -and ($newProductCandidates.Count -gt 0))
        $hasPhase3Changes = ($phase3FeatureEnabled -and ($phase3Eligible.Count -gt 0))
        if (($candidates.Count -gt 0) -or ($unitPriceFormulaFixes.Count -gt 0) -or $hasNewProductChanges -or $hasPhase3Changes) {
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

            # 【r13】単価シートには触れず、数値シートJ列の #N/A 数式だけをIFNAで包む。
            foreach ($fix in $unitPriceFormulaFixes) {
                $fixSheet = $null
                $fixJCell = $null
                try {
                    $fixSheet = $workbook.Worksheets.Item($fix.SheetName)
                    $fixJCell = $fixSheet.Cells.Item($fix.Row, 10)
                    $currentFormula = [string]$fixJCell.FormulaR1C1
                    # Audit時からUpdate時までの間に式が変わっていないかを確認してから書く。
                    if ($currentFormula -ne $fix.OldFormulaR1C1) {
                        Write-Log "WARN" ("UnitPriceIFNA SkipFormulaChanged Sheet={0} Row={1} Code={2} Expected={3} Current={4}" -f $fix.SheetName, $fix.Row, $fix.Code, $fix.OldFormulaR1C1, $currentFormula)
                        continue
                    }
                    $fixJCell.FormulaR1C1 = $fix.NewFormulaR1C1
                    Write-Log "UPDATE" ("UnitPriceIFNA Updated Sheet={0} Row={1} Code={2} FormulaR1C1={3}" -f $fix.SheetName, $fix.Row, $fix.Code, $fix.NewFormulaR1C1)
                }
                finally {
                    Release-Com $fixJCell
                    Release-Com $fixSheet
                }
            }

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
                        [void]$aCell.ClearContents()
                        $blockRange = $targetSheet.Range(
                            $targetSheet.Cells.Item($item.Row, 5),
                            $targetSheet.Cells.Item($item.Row, 15)
                        )
                        [void]$blockRange.ClearContents()
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
            $newProductAdded = 0
            if ($hasNewProductChanges) {
                # Phase 1: Saveしない。既存Workbookへ待避シート変更だけ適用する。
                $newProductAdded = Write-NewProductStaging -Workbook $workbook -SheetName $NewCodeSheetName -Candidates $newProductCandidates -Today $today
                Write-Log "UPDATE" ("NewProductStaging: Added={0}" -f $newProductAdded)
            }

            # -----------------------------------------------------------------
            # Phase 3 Move
            # Preflight PASS候補のみ処理。1件ごとに挿入位置・必須テンプレートを再計算。
            # 実行中に例外が出れば本体catchへ到達し、SaveせずClose(false)するため、
            # 通常更新・Phase1・Phase3を含むWorkbook変更全体が未保存で破棄される。
            # -----------------------------------------------------------------
            $phase3Moved = New-Object System.Collections.ArrayList
            if ($hasPhase3Changes) {
                $runtimeSeen = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)

                foreach ($pf in $phase3Eligible) {
                    $runtimeCode = Normalize-NewProductCode $pf.Code

                    # Preflight後～実行直前の二重登録ガード。
                    if ($runtimeSeen.Contains($runtimeCode) -or $managedCodes.Contains($runtimeCode)) {
                        throw "Phase3 runtime duplicate guard FAIL. Code=$runtimeCode"
                    }

                    if (Test-Phase3CodeExistsInNumericSheets -Workbook $workbook -Code $runtimeCode) {
                        throw "Phase3 runtime global duplicate guard FAIL. Code=$runtimeCode"
                    }

                    $moveOutput = @(Invoke-Phase3Move `
                        -Workbook $workbook `
                        -Excel $excel `
                        -Candidate $pf `
                        -ReorderPointMaster $reorderPointMaster `
                        -UnitPriceConfig $unitPriceConfig `
                        -MaxSearchRows $MaxFormulaSearchRows)

                    if ($moveOutput.Count -ne 1) {
                        throw "Phase3 runtime output contract FAIL. Code=$runtimeCode OutputCount=$($moveOutput.Count)"
                    }
                    $moved = $moveOutput[0]
                    if ($null -eq $moved.PSObject.Properties["StagingRow"]) {
                        throw "Phase3 runtime output contract FAIL. Code=$runtimeCode MissingProperty=StagingRow Type=$($moved.GetType().FullName)"
                    }

                    [void]$phase3Moved.Add($moved)
                    [void]$runtimeSeen.Add($runtimeCode)
                }
            }

            # 全Move成立後のみ、元の待避行を下から削除。
            $phase3StagingDeleted = 0
            if ($phase3Moved.Count -gt 0) {
                $phase3StagingDeleted = Remove-Phase3StagingRows `
                    -Workbook $workbook `
                    -SheetName $NewCodeSheetName `
                    -MovedItems $phase3Moved

                if ($phase3StagingDeleted -ne $phase3Moved.Count) {
                    throw "Phase3 staging deletion count mismatch. Moved=$($phase3Moved.Count) Deleted=$phase3StagingDeleted"
                }
            }

            # 保存責任を一本化:
            # 既存変更 + Phase1待避追加 + Phase3移動/待避削除をまとめて再計算し、Save()は1回のみ。
            Invoke-ExcelFullCalculation -Excel $excel
            $workbook.Save()
            Write-Log "INFO" (
                "Saved. Updated={0} UnitPriceIFNAUpdated={1} NewProductAdded={2} Phase3Moved={3} Phase3StagingDeleted={4} TotalWorkbookChanges={5}" -f
                $candidates.Count,
                $unitPriceFormulaFixes.Count,
                $newProductAdded,
                $phase3Moved.Count,
                $phase3StagingDeleted,
                ($candidates.Count + $unitPriceFormulaFixes.Count + $newProductAdded + $phase3Moved.Count + $phase3StagingDeleted)
            )
        }
        else {
            Write-Log "INFO" "No update required."
        }
        Save-DiscontinuedTracking -Path $DiscontinuedTrackingPath -Dict $trackingDict
        Write-Log "INFO" ("DiscontinuedTracking saved. Entries={0}" -f $trackingDict.Count)
    }
    else {
        Write-Log "INFO" ("Audit completed. Workbook was not changed. UnitPriceIFNACandidates={0} NewProductCandidates={1} NewProductFeatureError={2} Phase3Raw={3} Phase3Eligible={4}" -f $unitPriceFormulaFixes.Count, $newProductCandidates.Count, $newProductFeatureError, $phase3RawCandidates.Count, $phase3Eligible.Count)
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
