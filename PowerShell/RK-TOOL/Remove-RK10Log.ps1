#Requires -Version 5.1
<#
.SYNOPSIS
    RK-10 シナリオログ・認証履歴(License)・任意フォルダーの期限削除 v1.2.1。
.DESCRIPTION
    作成日時 < (実行PCの今日 - N日) の午前0時 を削除対象とする。

    LogType
      1       : シナリオログ。ScenarioLogDir 直下のフォルダーを中身ごと削除 (既定3日)
      0 / 2   : 認証履歴。LicenseLogDir 直下のファイルを削除 (既定7日)
      Both    : 上記を両方
      Folders : 汎用。-TargetDir 直下のフォルダーを中身ごと削除 (-Days 必須)
      Files   : 汎用。-TargetDir 直下のファイルを削除 (-Days 必須)

    DryRun は削除せず対象を列挙するだけ (監査ログは作成・追記する)。
    Parallel (1～16) はフォルダー削除の同時実行数。ファイル削除には効かない。
    DeepLinkCheck を付けると、削除対象フォルダー配下を全件走査してリンクの有無を確認する (遅い)。
    既定では、対象フォルダー自身と祖先のリンクのみ確認する。

    終了コード: 0=正常、1=削除失敗/安全スキップ、2=引数不正、3=致命的エラー。
    PowerShell自身の引数バインド失敗(未知の引数等)は上記コード2の対象外。
.EXAMPLE
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Remove-RK10Log.ps1 Both -Days 7 -DryRun
.EXAMPLE
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Remove-RK10Log.ps1 1 -Parallel 4
.EXAMPLE
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Remove-RK10Log.ps1 Folders -TargetDir "D:\Work\OldLogs" -Days 30 -DryRun
.NOTES
    Windows PowerShell 5.1対象。UTF-8 BOM付き。
    作成の古いフォルダーは新しい中身があっても削除する (従来仕様)。
    リンク検査と削除の間の外部変更を完全に防ぐものではない。
#>
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$LogType,

    # 共通日数。個別日数と同時指定はエラー。位置引数2としても指定可。
    [Parameter(Position = 1)]
    [string]$Days,

    # Folders / Files モード専用の対象フォルダー
    [string]$TargetDir,

    [string]$ScenarioLogDir = '\\192.168.0.35\disk\企画室\RPA\LOG',
    [string]$LicenseLogDir = '\\192.168.0.35\disk\企画室\RPA\LOG\License',

    [string]$ScenarioRetentionDays = 3,
    [string]$LicenseRetentionDays = 7,

    # 追加の除外名。License と設定した認証履歴パスは指定に関係なく保護。
    [string[]]$ExcludeFolderName = @('License'),

    # フォルダー削除の同時実行数 (1～16)。1=順次。
    [string]$Parallel = '1',

    # 削除対象フォルダー配下を全件走査してリンクを確認する (NASでは遅い)。
    [switch]$DeepLinkCheck,

    # 本スクリプト自身のログ。省略時は <スクリプトのフォルダー>\Logs\RK10LOG-REMOVE_yyyyMMdd.log
    [string]$LogFile,
    [string]$ScriptLogRetentionDays = 30,

    [switch]$DryRun
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

$script:Version = 'v1.2.1'
$script:LogPath = $null
$script:ProtectedLicense = $null
$script:Utf8Bom = New-Object System.Text.UTF8Encoding($true)

# 並列削除で各ランスペースが実行するスクリプト (結果は 'OK' または 'NG:理由')
$script:DeleteScript = @'
param($p)
try {
    (New-Object System.IO.DirectoryInfo($p)).Delete($true)
    'OK'
}
catch {
    'NG:' + $_.Exception.Message
}
'@

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
        # 自身のログは監査証跡として保持。自動削除は行わない。
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

# パスは区切り文字込みで比較し、似た名前の兄弟フォルダーを混同しない。
function Test-ContainsPath {
    param([string]$Parent, [string]$Child)
    $p = [IO.Path]::GetFullPath($Parent).TrimEnd('\', '/')
    $c = [IO.Path]::GetFullPath($Child).TrimEnd('\', '/')
    return ($c.Equals($p, [StringComparison]::OrdinalIgnoreCase) -or
        $c.StartsWith($p + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase))
}

function Test-TreeLink {
    param([string]$Path)
    $d = New-Object IO.DirectoryInfo($Path)
    if (($d.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) { return $true }
    foreach ($item in $d.EnumerateFileSystemInfos()) {
        if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) { return $true }
        if ($item -is [IO.DirectoryInfo]) {
            if (Test-TreeLink $item.FullName) { return $true }
        }
    }
    return $false
}

function Resolve-SafeRoot {
    param([string]$Path)
    if ([string]::IsNullOrWhiteSpace($Path) -or -not [IO.Path]::IsPathRooted($Path)) {
        throw "絶対パスで指定してください: $Path"
    }
    $full = [IO.Path]::GetFullPath($Path)
    $root = [IO.Path]::GetPathRoot($full)
    if ($full.TrimEnd('\', '/') -eq $root.TrimEnd('\', '/')) { throw 'ドライブ/共有ルートは指定できません' }
    $d = New-Object IO.DirectoryInfo($full)
    if (-not $d.Exists) { throw "対象フォルダーに到達できません: $full" }
    # 対象自身だけでなく祖先のジャンクションも拒否する。
    for ($a = $d; $null -ne $a; $a = $a.Parent) {
        if (($a.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) { throw "リンク経由の対象は不可: $full" }
    }
    return $d.FullName
}

# ---------------------------------------------------------------------------
# フォルダー単位の削除: 直下フォルダーを作成日時で判定し、中身ごと削除
# ---------------------------------------------------------------------------
function Remove-OldFolders {
    param(
        [string]$Dir,
        [datetime]$Cutoff,
        [string[]]$Exclude,
        [int]$Degree
    )

    $stat = @{ Scanned = 0; Target = 0; Deleted = 0; Failed = 0; Excluded = 0; Skipped = 0 }

    $root = New-Object System.IO.DirectoryInfo($Dir)
    if (-not $root.Exists) { throw "対象フォルダーが存在またはアクセスできません: $Dir" }

    $swScan = [System.Diagnostics.Stopwatch]::StartNew()

    # 列挙中に削除しないよう、先に対象リストを確定する。
    # 1件の確認エラーで全体を止めず、そのフォルダーだけスキップする。
    $targets = New-Object 'System.Collections.Generic.List[System.IO.DirectoryInfo]'
    foreach ($d in $root.EnumerateDirectories('*', [System.IO.SearchOption]::TopDirectoryOnly)) {
        $stat.Scanned++
        try {
            if ($d.CreationTime -ge $Cutoff) { continue }

            $reason = $null
            if ($Exclude -contains $d.Name) { $reason = 'ExcludeFolderName指定' }
            elseif ($d.Name -ieq 'License') { $reason = 'License名の保護' }
            elseif (Test-ContainsPath $d.FullName $script:ProtectedLicense) { $reason = '認証履歴フォルダーを含む' }
            elseif ($script:LogPath -and (Test-ContainsPath $d.FullName $script:LogPath)) { $reason = '監査ログを含む' }
            if ($reason) {
                $stat.Excluded++
                Write-Log ('除外({0}): {1}' -f $reason, $d.FullName)
                continue
            }
            if (($d.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
                $stat.Skipped++
                Write-Log ('シンボリックリンク/ジャンクションのためスキップ: {0}' -f $d.FullName) 'WARN'
                continue
            }
            if ($DeepLinkCheck -and (Test-TreeLink $d.FullName)) {
                $stat.Skipped++
                Write-Log ('配下にリンクがあるためスキップ: {0}' -f $d.FullName) 'WARN'
                continue
            }
            $targets.Add($d)
        }
        catch {
            $stat.Skipped++
            Write-Log ('確認できないためスキップ: {0} : {1}' -f $d.FullName, $_.Exception.Message) 'WARN'
        }
    }
    $stat.Target = $targets.Count
    Write-Log ('列挙完了 走査={0} 対象={1} 所要={2:N1}秒' -f $stat.Scanned, $stat.Target, $swScan.Elapsed.TotalSeconds)

    # 削除直前の再確認
    $confirmed = New-Object 'System.Collections.Generic.List[System.IO.DirectoryInfo]'
    foreach ($t in $targets) {
        try {
            $t.Refresh()
            if (-not $t.Exists) { throw '列挙後に対象が消失しました' }
            if ($t.CreationTime -ge $Cutoff) {
                $stat.Skipped++
                Write-Log ('再確認で作成日時が基準日時以降のためスキップ: {0} (作成日時 {1})' -f $t.FullName, (Format-Dt $t.CreationTime)) 'WARN'
                continue
            }
            if (($t.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
                $stat.Skipped++
                Write-Log ('再確認でリンク属性が検出されたためスキップ: {0} (Attributes={1})' -f $t.FullName, $t.Attributes) 'WARN'
                continue
            }
            if ($DeepLinkCheck -and (Test-TreeLink $t.FullName)) {
                $stat.Skipped++
                Write-Log ('再確認で配下にリンクが検出されたためスキップ: {0}' -f $t.FullName) 'WARN'
                continue
            }
            if ($DryRun) {
                Write-Log ('[DryRun] 削除対象フォルダー: {0} (作成日時 {1})' -f $t.FullName, (Format-Dt $t.CreationTime))
            }
            else {
                $confirmed.Add($t)
            }
        }
        catch {
            $stat.Failed++
            Write-Log ('フォルダー確認失敗: {0} : {1}' -f $t.FullName, $_.Exception.Message) 'ERROR'
        }
    }

    if ($confirmed.Count -eq 0) { return $stat }

    $swDel = [System.Diagnostics.Stopwatch]::StartNew()
    if ($Degree -le 1) {
        foreach ($t in $confirmed) {
            try {
                $t.Delete($true)
                $stat.Deleted++
                Write-Log ('フォルダー削除: {0} (作成日時 {1})' -f $t.FullName, (Format-Dt $t.CreationTime))
            }
            catch {
                $stat.Failed++
                Write-Log ('フォルダー削除失敗: {0} : {1}' -f $t.FullName, $_.Exception.Message) 'ERROR'
            }
        }
    }
    else {
        $pool = [System.Management.Automation.Runspaces.RunspaceFactory]::CreateRunspacePool(1, $Degree)
        $pool.Open()
        $handles = New-Object System.Collections.ArrayList
        try {
            foreach ($t in $confirmed) {
                $ps = [System.Management.Automation.PowerShell]::Create()
                $ps.RunspacePool = $pool
                [void]$ps.AddScript($script:DeleteScript).AddArgument($t.FullName)
                [void]$handles.Add(@{ PS = $ps; Async = $ps.BeginInvoke(); Target = $t })
            }
            foreach ($h in $handles) {
                $t = $h.Target
                try {
                    $res = @($h.PS.EndInvoke($h.Async))
                    $msg = ''
                    if ($res.Count -gt 0) { $msg = [string]$res[0] }
                    if ($msg -eq 'OK') {
                        $stat.Deleted++
                        Write-Log ('フォルダー削除: {0} (作成日時 {1})' -f $t.FullName, (Format-Dt $t.CreationTime))
                    }
                    else {
                        $stat.Failed++
                        Write-Log ('フォルダー削除失敗: {0} : {1}' -f $t.FullName, $msg) 'ERROR'
                    }
                }
                catch {
                    $stat.Failed++
                    Write-Log ('フォルダー削除失敗: {0} : {1}' -f $t.FullName, $_.Exception.Message) 'ERROR'
                }
                finally {
                    $h.PS.Dispose()
                }
            }
        }
        finally {
            $pool.Close()
            $pool.Dispose()
        }
    }
    Write-Log ('削除完了 同時実行={0} 所要={1:N1}秒' -f $Degree, $swDel.Elapsed.TotalSeconds)
    return $stat
}

# ---------------------------------------------------------------------------
# ファイル単位の削除: 直下ファイルを作成日時で判定して削除 (サブフォルダーは対象外)
# ---------------------------------------------------------------------------
function Remove-OldFiles {
    param(
        [string]$Dir,
        [datetime]$Cutoff
    )

    $stat = @{ Scanned = 0; Target = 0; Deleted = 0; Failed = 0; Excluded = 0; Skipped = 0 }

    $root = New-Object System.IO.DirectoryInfo($Dir)
    if (-not $root.Exists) { throw "対象フォルダーが存在またはアクセスできません: $Dir" }

    $swScan = [System.Diagnostics.Stopwatch]::StartNew()
    $targets = New-Object 'System.Collections.Generic.List[System.IO.FileInfo]'
    foreach ($f in $root.EnumerateFiles('*', [System.IO.SearchOption]::TopDirectoryOnly)) {
        $stat.Scanned++
        try {
            if (($f.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0 -or
                ($script:LogPath -and $f.FullName -ieq $script:LogPath)) {
                $stat.Skipped++
                continue
            }
            if ($f.CreationTime -lt $Cutoff) { $targets.Add($f) }
        }
        catch {
            $stat.Skipped++
            Write-Log ('確認できないためスキップ: {0} : {1}' -f $f.FullName, $_.Exception.Message) 'WARN'
        }
    }
    $stat.Target = $targets.Count
    Write-Log ('列挙完了 走査={0} 対象={1} 所要={2:N1}秒' -f $stat.Scanned, $stat.Target, $swScan.Elapsed.TotalSeconds)

    foreach ($t in $targets) {
        try {
            $t.Refresh()
            if (-not $t.Exists) { throw '列挙後に対象が消失しました' }
            if ($t.CreationTime -ge $Cutoff -or
                ($t.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
                $stat.Skipped++
                Write-Log ('再確認で対象外になったためスキップ: {0} (作成日時 {1}, Attributes={2})' -f $t.FullName, (Format-Dt $t.CreationTime), $t.Attributes) 'WARN'
                continue
            }
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
# 引数検証 (ログの初期化・削除より先に行う)
# ===========================================================================
$mode = switch ($LogType) {
    '1' { 'Scenario' }
    '0' { 'License' }
    '2' { 'License' }
    'Both' { 'Both' }
    'Folders' { 'Folders' }
    'Files' { 'Files' }
    default { '' }
}
if (-not $mode) {
    Write-Host '[ERROR] LogType: 1=シナリオログ、0/2=認証履歴、Both=両方、Folders/Files=-TargetDir 指定の汎用削除'
    exit 2
}

$isGeneric = ($mode -in @('Folders', 'Files'))
if ($isGeneric) {
    if ([string]::IsNullOrWhiteSpace($TargetDir)) { Write-Host '[ERROR] Folders/Files には -TargetDir が必要です'; exit 2 }
    if (-not $PSBoundParameters.ContainsKey('Days')) { Write-Host '[ERROR] Folders/Files には -Days (日数) が必須です'; exit 2 }
    if ($PSBoundParameters.ContainsKey('ScenarioRetentionDays') -or $PSBoundParameters.ContainsKey('LicenseRetentionDays')) {
        Write-Host '[ERROR] Folders/Files では -Days のみ使用できます'; exit 2
    }
}
elseif ($PSBoundParameters.ContainsKey('TargetDir')) {
    Write-Host '[ERROR] -TargetDir は LogType が Folders/Files のときだけ指定できます'; exit 2
}

if ($PSBoundParameters.ContainsKey('Days')) {
    if ($PSBoundParameters.ContainsKey('ScenarioRetentionDays') -or $PSBoundParameters.ContainsKey('LicenseRetentionDays')) {
        Write-Host '[ERROR] Days と個別日数は同時指定できません'; exit 2
    }
    $ScenarioRetentionDays = $Days
    $LicenseRetentionDays = $Days
}
foreach ($value in @($ScenarioRetentionDays, $LicenseRetentionDays, $ScriptLogRetentionDays)) {
    $n = 0
    if (-not [int]::TryParse($value, [ref]$n) -or $n -lt 1 -or $n -gt 36500) {
        Write-Host '[ERROR] 日数は1～36500の整数で指定してください'; exit 2
    }
}
$degree = 0
if (-not [int]::TryParse($Parallel, [ref]$degree) -or $degree -lt 1 -or $degree -gt 16) {
    Write-Host '[ERROR] Parallel は1～16の整数で指定してください'; exit 2
}

# ===========================================================================
# メイン
# ===========================================================================
$sw = [Diagnostics.Stopwatch]::StartNew()
$exitCode = 0
try {
    # 複数対象は事前に全ルートを確認してから処理する (削除自体は非トランザクション)。
    $script:ProtectedLicense = [IO.Path]::GetFullPath($LicenseLogDir)
    $jobs = @()
    if ($mode -in @('Scenario', 'Both')) {
        $path = Resolve-SafeRoot $ScenarioLogDir
        if (Test-ContainsPath $script:ProtectedLicense $path) { throw 'シナリオ対象を認証履歴フォルダー内には指定できません' }
        $jobs += @{ Label = 'シナリオログ'; Kind = 'Folders'; Dir = $path; Days = [int]$ScenarioRetentionDays }
    }
    if ($mode -in @('License', 'Both')) {
        $path = Resolve-SafeRoot $LicenseLogDir
        $jobs += @{ Label = '認証履歴'; Kind = 'Files'; Dir = $path; Days = [int]$LicenseRetentionDays }
    }
    if ($mode -eq 'Folders') {
        $path = Resolve-SafeRoot $TargetDir
        if (Test-ContainsPath $script:ProtectedLicense $path) { throw '対象を認証履歴フォルダー内には指定できません' }
        $jobs += @{ Label = '汎用(フォルダー)'; Kind = 'Folders'; Dir = $path; Days = [int]$Days }
    }
    if ($mode -eq 'Files') {
        $path = Resolve-SafeRoot $TargetDir
        $jobs += @{ Label = '汎用(ファイル)'; Kind = 'Files'; Dir = $path; Days = [int]$Days }
    }

    Initialize-ScriptLog
    if ($script:LogPath) { $script:LogPath = [IO.Path]::GetFullPath($script:LogPath) }
    Write-Log ('開始 {0} LogType={1} DryRun={2} Parallel={3} DeepLinkCheck={4}' -f $script:Version, $LogType, [bool]$DryRun, $degree, [bool]$DeepLinkCheck)

    $today = (Get-Date).Date
    foreach ($job in $jobs) {
        $cutoff = $today.AddDays(-$job.Days)
        $swJob = [Diagnostics.Stopwatch]::StartNew()
        Write-Log ('対象={0} パス={1} 日数={2} 基準日時={3} (作成日時がこれより前を削除)' -f $job.Label, $job.Dir, $job.Days, (Format-Dt $cutoff))
        if ($job.Kind -eq 'Folders') {
            $stat = Remove-OldFolders -Dir $job.Dir -Cutoff $cutoff -Exclude $ExcludeFolderName -Degree $degree
        }
        else {
            $stat = Remove-OldFiles -Dir $job.Dir -Cutoff $cutoff
        }
        if ($stat.Failed -gt 0 -or $stat.Skipped -gt 0) { $exitCode = 1 }
        Write-Log ('結果 対象={0} 走査={1} 対象数={2} 削除={3} 失敗={4} 除外={5} スキップ={6} 所要={7:N1}秒' -f $job.Label, $stat.Scanned, $stat.Target, $stat.Deleted, $stat.Failed, $stat.Excluded, $stat.Skipped, $swJob.Elapsed.TotalSeconds)
    }
    Write-Log ('終了 所要={0:N1}秒 終了コード={1}' -f $sw.Elapsed.TotalSeconds, $exitCode)
}
catch {
    Write-Log ('致命的エラー: {0}' -f $_.Exception.Message) 'ERROR'
    exit 3
}
exit $exitCode
