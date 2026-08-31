<#
===============================================================================
ProcessSheetRows.ps1  (長期滞留品自動処理・廃止コード復活処理を追加した版)

概要
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

    下方は「余りがD個」と正確に測定できるので一気に引き、上方は不足量が
    観測不能なので小刻みに増やす、という非対称性には必然性がある。

  ルール①（発注点割れ長期化）
    条件：G列=1 かつ F列（発注点割れ検知日）が拠点別リセット日数
          （＝補充リードタイム。既定90日、船舶(機械部)の一部シートのみ
          365日）以上前
          ただし D列（在庫数）が 0 以下の場合は対象外（下記の除外条件を参照）
    処理：
      ・E列を「今のE － 今のD」に書き換える（上記の実消費量）
      ・F列は「クリア」ではなく「今日の日付」に更新する
        （荘田室長confirmed・2026-07-29：もともと「発注点を差分値に
        変える」と同時に「変更した日付を記入する」運用ルールだった）
      ・回数制限は設けず、条件を満たすたびに毎回適用する
        （荘田室長confirmed・2026-07-29 口頭）。繰り返し補正することで
        真値へ収束する仕組みであり、EがDを下回った時点でG列（発注点
        割れ）が0になって自動的に停止するため、際限なく減り続けることは
        ない。F列を今日の日付に更新することで、次回の補正はさらに
        リードタイム経過後となり、1リードタイムにつき1回のペースで
        収束が進む。
    除外条件（D列=0、2026-07-29 福岡の実データで判明）：
      在庫が0まで落ちている場合、消費量が「E以上」であることしか
      分からず超過分は観測できないため、E-D は E のままとなり補正が
      成立しない（空振り）。さらに、この空振りでF列を今日の日付に
      更新してしまうと、ルール③がF列の日数条件の内側にあるため、
      本来管理除外へ進むべき在庫0の品目の判定が最大で基準日数ぶん
      先送りになる（福岡T1033で71日の遅延を確認）。
      よってD列<=0の行はE列・F列とも一切変更せずスキップし、F列の
      日数を積み上がるままにして、H列が基準日数に達した時点で
      ルール③が予定どおり発火するようにする。
      スキップ時はログに LongTermSkip(D=0) を出力する。
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

  ルール⑥（N/A行のE:O列クリア）※2026/8/7 荘田室長依頼により追加
    条件：C列（商品名）が #N/A エラー（＝TIMS側で商品コードが見つからない
          状態）である行。今回新規にNAになった行・以前からNAが続いている
          行の両方が対象（今回のNA判定であれば毎回無条件に対象とする）
    処理：A列およびE列～O列（発注点・発注点割れ日・判定フラグ・在庫欠品・単価・
          在庫額等、11列分）の内容をすべてクリアする
    理由：N/A＝TIMS側で商品コードが見つからない＝管理対象外品番、という
          意味。前回までの発注点・在庫額等の数値が残ったままだと、翌日の
          RK-10シナリオの並べ替え処理でこの行が下部に押しやられず、あたかも
          管理対象内の品番であるかのように扱われてしまう。E:O列を空にする
          ことで、翌日の並べ替えで自然に下部へ押しやられるようにする。
    実装：ルール⑤（廃止コード復活）とは対になる関係（NA化→本ルールで
          クリア／NAから復活→ルール⑤でE列を在庫数で復元）。このため
          isNameError判定の直後、ルール⑤判定より先に評価し、対象行は
          candidatesにClearBlock=$trueとして積んだうえでcontinueし、
          以降の4パターン・長期滞留ルールの判定には進ませない。

  ルール①③の基準日数は、拠点別変数表（在庫チェックRPA統合_拠点別変数表.xlsx）
  の「発注点リセット_部署別設定」シートから、拠点名・シート名をキーに
  都度読み取る。変数表が指定されない、またはそのシート・拠点の行が
  見つからない場合は、既定値90日にフォールバックし、WARNログを出す。

  廃止コード追跡CSV（DiscontinuedCodeTracking_{拠点名}.csv）は船舶(機械部)
  で実機検証済み（NA新規登録・復活検知・E列書き換え・CSV更新まで確認済み）。
  この仕組み自体（在庫帳とは別にCSVで追跡する設計）については荘田室長へ
  最終確認中（2026-07-28時点で回答待ち）。ロールバックが必要な場合は、
  旧ProcessSheetRows.ps1（4パターンのみ版）にファイルを差し戻せば良い。

  参考：
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

    [int]$MaxCandidates = 5000,

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
$ScriptVersion = "2026-08-31-NA-Aclear-r2"

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
# CalculationをAutomaticへ明示的に戻すことで、Manual設定が残っている場合でも
# 行走査前・保存前の数式結果を最新状態にする。
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
        # また、成果物のExcelと並んで見えると見づらいため、「作業用CSV」
        # サブフォルダに退避する（NAS環境ではWindowsの隠しファイル属性が
        # 保存されない場合があるため、隠し属性ではなくサブフォルダ方式で対応）。
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

    # FixReorderPoint等の直前処理でE列(発注点)が変更された場合に備え、
    # G列(発注点割れ)・I列(欠品)の数式結果を最新状態にしてから判定する。
    # Excelの計算モード／数式キャッシュに依存した判定ずれを防止するための保険処理。
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
                    # 以前はB列空欄を先にcontinueしていたため、C列=#N/Aの行が
                    # E:Oクリア対象にならないケースがあった（2026-08-21 修正）。
                    $codeCell = $sheet.Cells.Item($row, 2)
                    $nameCell = $sheet.Cells.Item($row, 3)
                    $code = Normalize-Code $codeCell.Value2

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

                    if ($isNameError) {
                        # 商品コードがあるN/A行だけ廃止コード追跡CSVへ登録する。
                        # B列空欄のN/A行は追跡キーを作れないため登録せず、
                        # ルール⑥(E:Oクリア)だけ適用する。
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
                        # 商品コード(B列)がバーコード貼付側で見つからず、C/D列が
                        # #N/A になっている行は管理対象外。E～O列(発注点・判定
                        # フラグ・単価・在庫額等)に前回までの値が残っていると、
                        # 翌日のRK-10シナリオの並べ替えでこの行が下部に押しやられず
                        # 管理対象外として扱われないため、N/Aである間は毎回無条件で
                        # E:O列をクリアする（既にクリア済みでも冪等なので害はない）。
                        # ※Set-StrictMode -Version 2.0下では、他の候補と異なる
                        # プロパティ構成のオブジェクトを$candidatesに混在させると
                        # 後段のログ出力・適用ループで存在しないプロパティへの
                        # アクセスがエラーになるため、EOld/ENew/FOld/FNew/HOld/HNew
                        # も$nullとして明示的に持たせ、他の候補と同一スキーマにする。
                        $clearCandidate = [PSCustomObject]@{
                            SheetName  = $sheetName
                            Row        = $row
                            Code       = $code
                            EOld       = $null
                            ENew       = $null
                            FOld       = $null
                            FNew       = $null
                            HOld       = $null
                            HNew       = $null
                            ClearBlock = $true
                            Reason     = "N/A行クリア(管理対象外,A列+E:O列)"
                        }
                        [void]$candidates.Add($clearCandidate)

                        # クリア候補として積んだので、以降の4パターン+長期滞留
                        # ルールの判定はスキップする。
                        continue
                    }

                    # N/Aでない通常行は、商品コードが空欄なら従来どおり対象外。
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
                    # （ヘッダー行、"#N/A"行）は従来の4パターンとしては対象外。
                    # ただし廃止コード判定（ルール⑤）は、E/D等が数値化できない
                    # 行（＝現在NA状態の行）でこそ意味を持つため、先に廃止コード
                    # チェックだけ行ってから、既存の数値変換ガードに入る。
                    $trackKey = "$sheetName|$code"

                    if ($trackingDict.ContainsKey($trackKey) -and $trackingDict[$trackKey].状態 -eq "追跡中") {
                        # --- ルール⑤：廃止コードの復活 ---
                        # 発注点(E)は、従来は「復活時点の在庫数(D)」を暫定的に
                        # 流用していたが、荘田室長のご指示（2026-08-10口頭）
                        # により、バーコード貼付シートD列(発注点マスタ)から
                        # 本来の発注点を取得して書き込む方式に変更した。
                        # 在庫数(D)が0であっても（＝復活直後で入荷未反映等）、
                        # マスタに値さえあれば通常通り発注点を書き込む
                        # （在庫数の多寡とマスタ参照は無関係な処理のため）。
                        if ($reorderPointMaster.ContainsKey($code)) {
                            $masterReorderPoint = $reorderPointMaster[$code]

                            $revivalCandidate = [PSCustomObject]@{
                                SheetName  = $sheetName
                                Row        = $row
                                Code       = $code
                                EOld       = $null
                                ENew       = $masterReorderPoint
                                FOld       = $null
                                FNew       = $null
                                HOld       = $null
                                HNew       = $null
                                ClearBlock = $false
                                Reason     = "廃止コード復活(E=発注点マスタ値)"
                            }
                            [void]$candidates.Add($revivalCandidate)

                            $entry = $trackingDict[$trackKey]
                            $entry.状態 = "復活済み"
                            $entry.復活日 = $today.ToString("yyyy-MM-dd")
                            $entry.復活時発注点 = [string]$masterReorderPoint
                            Write-Log "INFO" ("DiscontinuedTracking: revival detected Sheet=$sheetName Code=$code NewE(master)=$masterReorderPoint")
                        }
                        else {
                            # マスタに該当コードの発注点が見つからない場合は
                            # 書き込みをスキップし、状態は「追跡中」のまま
                            # 維持する（次回実行時に再判定される）。
                            Write-Log "WARN" ("ReorderPointMaster: code not found Sheet=$sheetName Code=$code. 発注点書き込みをスキップします。")
                        }
                        # このセルは復活処理（またはスキップ）を判定済みなので、
                        # 以降の通常4パターン+長期滞留ルールの判定はスキップする
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

                    # --- ルール①③：長期滞留品の自動処理 ---
                    # 判定は「今読んだ元のfRaw/hRaw」の日付age基準。ただし、
                    # パターン1-4が今回すでにF列を書き換え済み（$newFが
                    # 非nullになっている）場合は、そちらの判断（今日新規に
                    # 検知した／解消した）を優先し、ルール①③は評価しない。
                    # （$newFが未確定のまま古いfRawでage判定してしまうと、
                    # パターン2の「F=今日」をルール①の処理が上書きしてしまう
                    # 競合が起きるため）
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
                                if ($dVal -le 0) {
                                    # --- 在庫0（D=0）はルール①の対象外 ---
                                    # 発注点＝リードタイム中の消費量、という考え方では、
                                    # E-D はリードタイム終了時に「余った数」Dを引くことで
                                    # 実消費量を求める式。在庫が0まで落ちている場合は
                                    # 消費量が「E以上」であることしか分からず、どれだけ
                                    # 超えたかは観測できない（在庫はマイナスにならない）。
                                    # つまりD=0では測定が成立せず、E-D=Eの空振りになる。
                                    #
                                    # ここでF列を今日の日付に更新してしまうと、ルール③
                                    # （発注点割れ・欠品とも長期化→E=0で管理除外）が
                                    # F列の日数条件の内側にあるため、本来管理除外に進む
                                    # べき在庫0の品目の判定が最大で基準日数ぶん先送りに
                                    # なる。よってE列・F列とも一切触らずスキップし、
                                    # F列の日数を積み上がるままにしておく。
                                    Write-Log "INFO" (
                                        "LongTermSkip(D=0) Sheet={0} Row={1} Code={2} E={3} FAgeDays={4}" -f
                                        $sheetName, $row, $code, $eVal, $fAgeDays
                                    )
                                }
                                else {
                                    # --- ルール①：発注点割れのみ長期化 → 差分値に置換 ---
                                    # 発注点＝「補充リードタイム（基準日数）中に消費される
                                    # 数量」という考え方に基づく。発注点割れ日から基準日数が
                                    # 経過した時点で在庫がDだけ残っているということは、その
                                    # 期間の実消費量は E-D であり、これがあるべき発注点になる
                                    # （荘田室長の手書き図・2026-07-29）。
                                    #
                                    # 2回目以降も同じルールを自動適用する（荘田室長confirmed・
                                    # 2026-07-29、口頭）。繰り返し補正することで真値へ収束する
                                    # 仕組みであり、EがDを下回った時点でG列（発注点割れ）が
                                    # 0になり自動的に停止するため、際限なく減り続けることはない。
                                    $diff = $eVal - $dVal
                                    if ($diff -lt 0) { $diff = [decimal]0 }  # 念のための防御的クリップ
                                    $newE = $diff
                                    # F列は「クリア」ではなく「今日の日付」に更新する
                                    # （荘田室長confirmed・2026-07-29：もともとE列変更と
                                    # 同時にF列へ変更日を記入する運用ルールだった）。
                                    # これにより次回の補正は、さらに基準日数（＝リードタイム）
                                    # 経過後になり、1リードタイムにつき1回のペースで収束が進む。
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
                        SheetName  = $sheetName
                        Row        = $row
                        Code       = $code
                        EOld       = $eVal
                        ENew       = $newE
                        FOld       = $fRaw
                        FNew       = $newF
                        HOld       = $hRaw
                        HNew       = $newH
                        ClearBlock = $false
                        Reason     = ($reasons -join "; ")
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
            # 候補件数超過は他の異常(Excel操作エラー等)と区別できるよう、
            # ExitCode=10を持たせた例外としてthrowする。RK-10側で
            # Cmd_Result1=10を専用判定できるようにするための対応。
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
            # ファイル名は日付のみとし、同日中の複数回実行では上書きする
            # （時刻まで含めた退避は不要との判断。世代管理は日次で十分なため）
            $backupName = "{0}_{1}{2}" -f $baseName, (Get-Date -Format "yyyyMMdd"), $extension
            $backupPath = Join-Path $backupFolder $backupName

            Copy-Item -LiteralPath $WorkbookPath -Destination $backupPath -Force
            Write-Log "INFO" ("Backup={0}" -f $backupPath)

            # 直近2日分（当日・前日）のみ残し、3日以上前のバックアップを削除する
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
                        # N/A行（管理対象外）は、行番号のA列と管理用のE～O列をクリアする。
                        # B～D列は商品コード／商品名等の識別情報として残す。
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

            # 適用フェーズでE/F/Hを書き換えた結果をG/I等の数式へ確実に反映してから保存する。
            # 下流処理が古い数式キャッシュを読むことを防止する。
            Invoke-ExcelFullCalculation -Excel $excel

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

    # デフォルトは一般エラー。throw元でExitCodeが専用指定されていれば
    # それを使う（例：候補件数超過=10）。RK-10側でCmd_Result1の値により
    # エラー種別を判定できるようにするための対応。
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

    # RK-10側が拾うのは標準出力(stdout)または標準エラー出力(stderr)の
    # いずれかだが、どちらか確証がないため両方に明示的に書き出す。
    # Write-Hostのみだと、非対話起動・リダイレクト経由では捕捉されない。
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
