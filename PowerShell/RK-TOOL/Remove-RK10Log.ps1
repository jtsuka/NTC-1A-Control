#Requires -Version 5.1
<#
.SYNOPSIS
    RK-10 のシナリオログ / ライセンスログを削除する (シナリオ RK10LOG-REMOVE の PowerShell 版)

.DESCRIPTION
    LogType = '1'      : シナリオログ。LOG 直下のフォルダーのうち、作成日時が (今日 - 3日) の 0:00 より前のものをフォルダーごと削除
    LogType = '0' / '2': ライセンスログ。LOG\License 直下のファイルのうち、作成日時が (今日 - 7日) の 0:00 より前のものを削除

    終了コード
        0 : 正常終了 (削除対象なしを含む)
        1 : 一部の項目で削除に失敗 (他の項目は処理済み)
        2 : 引数不正 (何も削除していない)
        3 : 致命的エラー (対象フォルダー到達不可など)

.PARAMETER LogType
    '1' = シナリオログ / '0' または '2' = ライセンスログ。省略・その他の値は終了コード 2 (安全側)。

.PARAMETER DryRun
    指定すると削除せず、削除対象の一覧をログに出力するだけ。初回確認用。

.EXAMPLE
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Remove-RK10Log.ps1 1 -DryRun

.NOTES
    ファイルは UTF-8 (BOM 付き) で保存すること。Windows PowerShell 5.1 は BOM なしだと日本語パスが文字化けする。
#>
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$LogType,

    [string]$ScenarioLogDir = '\\192.168.0.35\disk\企画室\RPA\LOG',
    [string]$LicenseLogDir = '\\192.168.0.35\disk\企画室\RPA\LOG\License',

    [int]$ScenarioRetentionDays = 3,
    [int]$LicenseRetentionDays = 7,

    # シナリオログ側の走査で削除対象から外すフォルダー名 (License フォルダーの巻き込み防止)
    [string[]]$ExcludeFolderName = @('License'),

    # 本スクリプト自身のログ。省略時は <スクリプトのフォルダー>\Logs\RK10LOG-REMOVE_yyyyMMdd.log
    [string]$LogFile,
    [int]$ScriptLogRetentionDays = 30,

    [switch]$DryRun
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

$script:LogPath = $null
$script:Utf8Bom = New-Object System.Text.UTF8Encoding($true)

function Write-Log {
    param(
        [string]$Message,
        [string]$Level = 'INFO'
    )
    $line = '{0} [{1}] {2}' -f (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $Level, $Message
    Write-Host $line
    if ($script:LogPath) {
        try {
            [System.IO.File]::AppendAllText($script:LogPath, $line + [Environment]::NewLine, $script:Utf8Bom)
        }
        catch {
            # ログ書き込み失敗では本処理を止めない
        }
    }
}

function Initialize-ScriptLog {
    try {
        $path = $LogFile
        if (-not $path) {
            $base = $PSScriptRoot
            if (-not $base) { $base = $env:TEMP }
            $path = Join-Path $base ('Logs\RK10LOG-REMOVE_{0}.log' -f (Get-Date -Format 'yyyyMMdd'))
        }
        $dir = Split-Path -Parent $path
        if ($dir -and -not (Test-Path -LiteralPath $dir)) {
            New-Item -ItemType Directory -Path $dir -Force | Out-Null
        }
        $script:LogPath = $path

        # 自分自身のログの世代管理 (DryRun 時は触らない)
        if (-not $DryRun -and $dir) {
            $limit = (Get-Date).AddDays(-$ScriptLogRetentionDays)
            Get-ChildItem -LiteralPath $dir -Filter 'RK10LOG-REMOVE_*.log' -File |
                Where-Object { $_.LastWriteTime -lt $limit } |
                Remove-Item -Force -ErrorAction SilentlyContinue
        }
    }
    catch {
        $script:LogPath = $null
        Write-Host ('{0} [WARN] スクリプトログの初期化に失敗したためコンソール出力のみで続行します: {1}' -f (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $_.Exception.Message)
    }
}

function Format-Dt {
    param([datetime]$Value)
    return $Value.ToString('yyyy-MM-dd HH:mm:ss')
}

# ---------------------------------------------------------------------------
# シナリオログ: 直下フォルダーを作成日時で判定し、フォルダーごと削除
# ---------------------------------------------------------------------------
function Remove-OldFolders {
    param(
        [string]$Dir,
        [datetime]$Cutoff,
        [string[]]$Exclude
    )

    $stat = @{ Scanned = 0; Target = 0; Deleted = 0; Failed = 0; Excluded = 0; Skipped = 0 }

    $root = New-Object System.IO.DirectoryInfo($Dir)
    if (-not $root.Exists) { throw "対象フォルダーが存在またはアクセスできません: $Dir" }

    # 列挙中に削除しないよう、先に対象リストを確定する
    $targets = New-Object 'System.Collections.Generic.List[System.IO.DirectoryInfo]'
    foreach ($d in $root.EnumerateDirectories('*', [System.IO.SearchOption]::TopDirectoryOnly)) {
        $stat.Scanned++
        if ($d.CreationTime -ge $Cutoff) { continue }

        if ($Exclude -contains $d.Name) {
            $stat.Excluded++
            Write-Log ('除外(ExcludeFolderName): {0}' -f $d.FullName)
            continue
        }
        if (($d.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
            $stat.Skipped++
            Write-Log ('シンボリックリンク/ジャンクションのためスキップ: {0}' -f $d.FullName) 'WARN'
            continue
        }
        $targets.Add($d)
    }
    $stat.Target = $targets.Count

    foreach ($t in $targets) {
        try {
            if ($DryRun) {
                Write-Log ('[DryRun] 削除対象フォルダー: {0} (作成日時 {1})' -f $t.FullName, (Format-Dt $t.CreationTime))
            }
            else {
                $t.Delete($true)
                $stat.Deleted++
                Write-Log ('フォルダー削除: {0} (作成日時 {1})' -f $t.FullName, (Format-Dt $t.CreationTime))
            }
        }
        catch {
            $stat.Failed++
            Write-Log ('フォルダー削除失敗: {0} : {1}' -f $t.FullName, $_.Exception.Message) 'ERROR'
        }
    }
    return $stat
}

# ---------------------------------------------------------------------------
# ライセンスログ: 直下ファイルを作成日時で判定して削除 (サブフォルダーは対象外)
# ---------------------------------------------------------------------------
function Remove-OldFiles {
    param(
        [string]$Dir,
        [datetime]$Cutoff
    )

    $stat = @{ Scanned = 0; Target = 0; Deleted = 0; Failed = 0; Excluded = 0; Skipped = 0 }

    $root = New-Object System.IO.DirectoryInfo($Dir)
    if (-not $root.Exists) { throw "対象フォルダーが存在またはアクセスできません: $Dir" }

    $targets = New-Object 'System.Collections.Generic.List[System.IO.FileInfo]'
    foreach ($f in $root.EnumerateFiles('*', [System.IO.SearchOption]::TopDirectoryOnly)) {
        $stat.Scanned++
        if ($f.CreationTime -lt $Cutoff) { $targets.Add($f) }
    }
    $stat.Target = $targets.Count

    foreach ($t in $targets) {
        try {
            if ($DryRun) {
                Write-Log ('[DryRun] 削除対象ファイル: {0} (作成日時 {1})' -f $t.FullName, (Format-Dt $t.CreationTime))
            }
            else {
                if ($t.IsReadOnly) { $t.IsReadOnly = $false }
                $t.Delete()
                $stat.Deleted++
                Write-Log ('ファイル削除: {0} (作成日時 {1})' -f $t.FullName, (Format-Dt $t.CreationTime))
            }
        }
        catch {
            $stat.Failed++
            Write-Log ('ファイル削除失敗: {0} : {1}' -f $t.FullName, $_.Exception.Message) 'ERROR'
        }
    }
    return $stat
}

# ===========================================================================
# メイン
# ===========================================================================
$sw = [System.Diagnostics.Stopwatch]::StartNew()
$exitCode = 0

try {
    Initialize-ScriptLog
    Write-Log ('開始 LogType="{0}" DryRun={1}' -f $LogType, [bool]$DryRun)

    $mode = $null
    switch ($LogType) {
        '1' { $mode = 'Scenario' }
        { $_ -in '0', '2' } { $mode = 'License' }
        default { }
    }
    if (-not $mode) {
        Write-Log ('引数 LogType が不正です (指定値: "{0}")。1=シナリオログ, 0/2=ライセンスログ。何も削除せず終了します。' -f $LogType) 'ERROR'
        exit 2
    }

    # 原シナリオと同じく「(今日 - N日) の 0:00」を基準日時とする
    $now = Get-Date
    if ($mode -eq 'Scenario') {
        $dir = $ScenarioLogDir
        $days = $ScenarioRetentionDays
    }
    else {
        $dir = $LicenseLogDir
        $days = $LicenseRetentionDays
    }
    if ($days -lt 1) { throw "保持日数が 1 未満です: $days" }
    $cutoff = $now.AddDays(-$days).Date

    Write-Log ('モード={0} 対象={1} 保持日数={2} 基準日時={3} (作成日時がこれより前を削除)' -f $mode, $dir, $days, (Format-Dt $cutoff))

    if ($mode -eq 'Scenario') {
        $stat = Remove-OldFolders -Dir $dir -Cutoff $cutoff -Exclude $ExcludeFolderName
    }
    else {
        $stat = Remove-OldFiles -Dir $dir -Cutoff $cutoff
    }

    if ($stat.Failed -gt 0) { $exitCode = 1 }

    Write-Log ('終了 走査={0} 対象={1} 削除={2} 失敗={3} 除外={4} スキップ={5} 所要={6:N1}秒 終了コード={7}' -f `
            $stat.Scanned, $stat.Target, $stat.Deleted, $stat.Failed, $stat.Excluded, $stat.Skipped, $sw.Elapsed.TotalSeconds, $exitCode)
}
catch {
    Write-Log ('致命的エラー: {0}' -f $_.Exception.Message) 'ERROR'
    Write-Log ($_.ScriptStackTrace) 'ERROR'
    exit 3
}

exit $exitCode
