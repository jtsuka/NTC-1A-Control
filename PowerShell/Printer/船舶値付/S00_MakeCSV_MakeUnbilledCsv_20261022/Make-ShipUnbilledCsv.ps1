<#
.SYNOPSIS
  船舶値付用の入力CSV「未計上明細_218001_YYYYMMDD.csv」を作る。RK-10シナリオ
  「値付けデータ(船舶)for値付用紙書式VBマクロ版Ver4.0-自動」の手順1〜67（Excel操作）をPowerShellに置き換える。

.DESCRIPTION
  RK-10と同じ処理を、Excelを使わずに行う。
    1. 前営業日・休日判定（カレンダーExcelの「休日」シートから計算）
    2. 元データ「未計上明細（全件）_PRONES.csv」を読む
    3. 最終出荷日（I列）が前営業日の行だけを残す（見出し行は残す）。A〜JH列（268列）
    4. Excelが「CSVを開いて保存」したときの値の変換を再現する（先頭0の消去・日付化・末尾空白の除去 など）
    5. 「未計上明細_218001_YYYYMMDD.csv」を保存（CP932・CRLF・Excel流の引用符）

  終了コード: 0=作成した / 10=休日のため作成しない（RK-10の「シナリオを終了」と同じ） /
              20=元データが見つからない / 21=カレンダーを読めない / 22=元データの更新日が当日でない・書き込み中 / 30=該当行が0件（見出しだけ作成）/
              40=-CompareWith との比較で差があった / 1=その他のエラー
  入力・出力の場所は引数で自由に変えられる（既定は \\192.168.200.5\navicsv）。
    -SourceCsv  元データのCSV（ローカルでもUNCでも可）
    -OutputDir  出力先フォルダ（ファイル名は 未計上明細_<部門>_<当日>.csv）／ -OutputCsv 出力ファイルのフルパス（こちらが優先）
    -CompareWith 既存のCSV（RK-10の出力など）とバイト単位で比べる。-LogDir ログの保存先
  例（navicsv に触れない試験）:
    .\Make-ShipUnbilledCsv.ps1 -Date 20261022 -PrevBusinessDay 20261021 -SourceCsv D:\test\in\元.csv -OutputDir D:\test\out -AllowStaleSource
  Windows PowerShell 5.1。このファイルはUTF-8（BOM付き）。
#>
[CmdletBinding()]
param(
    [string]$SourceCsv   = '\\192.168.200.5\navicsv\未計上明細（全件）_PRONES.csv',
    [string]$OutputDir   = '\\192.168.200.5\navicsv',   # 出力先フォルダ。ファイル名は 未計上明細_<部門>_<当日>.csv
    [string]$OutputCsv   = '',                          # 出力ファイルのフルパス（指定時は -OutputDir・部門・当日によるファイル名より優先）
    [string]$Dept        = '218001',
    [string]$Date        = '',                          # 当日 yyyyMMdd。空なら今日
    [string]$PrevBusinessDay = '',                      # 前営業日 yyyyMMdd。空ならカレンダーから計算
    [string]$CalendarPath = 'C:\HPDB\カレンダー(2023).xlsx',
    [int]$FilterColumn   = 9,                           # 1始まり。9 = I列（最終出荷日）
    [int]$MaxColumns     = 268,                         # JH列まで
    [switch]$AllowStaleSource,                          # 元データの更新日が当日でなくても使う（試験用）
    [int]$WaitSeconds    = 600,                         # 元データが書き込み中のとき待つ最大秒数
    [switch]$SkipHolidayCheck,                        # 休日でも作る（試験用）
    [switch]$RawValues,                                 # Excel風の値の変換をしない（試験用）
    [string]$CompareWith = '',                          # 既存CSV（RK-10の出力）とバイト単位で比べる
    [string]$LogDir      = ''
)
$ErrorActionPreference = 'Stop'

# 入出力はどの場所でも指定できる（ローカル・UNC・相対パス）。相対パスは実行時のカレントフォルダ基準で絶対パスにする
function Abs([string]$p) { if ($p) { $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($p) } else { $p } }
$SourceCsv = Abs $SourceCsv; $OutputDir = Abs $OutputDir; $OutputCsv = Abs $OutputCsv
$CalendarPath = Abs $CalendarPath; $CompareWith = Abs $CompareWith; $LogDir = Abs $LogDir

$code = @'
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using System.Text.RegularExpressions;

public static class ShipCsv
{
    static readonly Regex Num = new Regex(@"^[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?$", RegexOptions.Compiled);
    static readonly Regex Dt = new Regex(@"^(\d{1,4})[/-](\d{1,2})[/-](\d{1,2})[ T](\d{1,2}):(\d{2})(:(\d{2})(\.\d+)?)?$", RegexOptions.Compiled);
    static readonly Regex Ymd = new Regex(@"^(\d{1,4})[/-](\d{1,2})[/-](\d{1,2})$", RegexOptions.Compiled);
    static readonly Regex Md = new Regex(@"^(\d{1,2})[/-](\d{1,2})$", RegexOptions.Compiled);
    static readonly Regex Tm = new Regex(@"^(\d{1,2}):(\d{2})(:(\d{2}))?$", RegexOptions.Compiled);
    static readonly Regex Yen = new Regex(@"^(-?)[\\\u00A5\uFFE5](\d+)(\.\d+)?$", RegexOptions.Compiled);

    public static int EqualSignCells = 0;   // 「=」で始まるセル（Excelでは数式になる）。変換せず数える

    public static List<string[]> Parse(string text)
    {
        var rows = new List<string[]>();
        var row = new List<string>();
        var sb = new StringBuilder();
        bool inQ = false, atStart = true, any = false;
        int n = text.Length;
        for (int i = 0; i < n; i++)
        {
            char c = text[i];
            if (inQ)
            {
                if (c == '"')
                {
                    if (i + 1 < n && text[i + 1] == '"') { sb.Append('"'); i++; }
                    else inQ = false;
                }
                else sb.Append(c);
                continue;
            }
            if (c == '"' && atStart) { inQ = true; atStart = false; any = true; continue; }
            if (c == ',') { row.Add(sb.ToString()); sb.Length = 0; atStart = true; any = true; continue; }
            if (c == '\r' || c == '\n')
            {
                if (c == '\r' && i + 1 < n && text[i + 1] == '\n') i++;
                if (any || sb.Length > 0 || row.Count > 0) { row.Add(sb.ToString()); rows.Add(row.ToArray()); }
                row = new List<string>(); sb.Length = 0; atStart = true; any = false;
                continue;
            }
            sb.Append(c); atStart = false; any = true;
        }
        if (any || sb.Length > 0 || row.Count > 0) { row.Add(sb.ToString()); rows.Add(row.ToArray()); }
        return rows;
    }

    static string Q(string s)
    {
        if (s.IndexOfAny(new char[] { ',', '"', '\r', '\n' }) >= 0) return "\"" + s.Replace("\"", "\"\"") + "\"";
        return s;
    }

    public static void Write(string path, List<string[]> rows, int cols)
    {
        var enc = Encoding.GetEncoding(932);
        var sb = new StringBuilder();
        foreach (var r in rows)
        {
            for (int i = 0; i < cols; i++)
            {
                if (i > 0) sb.Append(',');
                sb.Append(Q(i < r.Length ? r[i] : ""));
            }
            sb.Append("\r\n");
        }
        File.WriteAllBytes(path, enc.GetBytes(sb.ToString()));
    }

    static string General(double v)
    {
        if (v == 0) return "0";
        double a = Math.Abs(v);
        if (a >= 1e11 || a < 1e-4)
        {
            string s = v.ToString("E5", CultureInfo.InvariantCulture);
            int k = s.IndexOf('E');
            string m = s.Substring(0, k);
            if (m.IndexOf('.') >= 0) m = m.TrimEnd('0').TrimEnd('.');
            string e = s.Substring(k + 1);
            string sign = e.Substring(0, 1);
            string digits = e.Substring(1).TrimStart('0').PadLeft(2, '0');
            return m + "E" + sign + digits;
        }
        string r = v.ToString("0.##########", CultureInfo.InvariantCulture);
        return r;
    }

    static string FmtNum(string s)
    {
        string t = s.Trim(' ');
        bool neg = t.StartsWith("-");
        if (t.StartsWith("-") || t.StartsWith("+")) t = t.Substring(1);
        if (t.IndexOfAny(new char[] { 'e', 'E' }) >= 0)
        {
            double v;
            if (!double.TryParse((neg ? "-" : "") + t, NumberStyles.Float, CultureInfo.InvariantCulture, out v) || double.IsInfinity(v)) return s;
            return General(v);
        }
        string ip, fp;
        int dot = t.IndexOf('.');
        if (dot >= 0) { ip = t.Substring(0, dot); fp = t.Substring(dot + 1); } else { ip = t; fp = ""; }
        ip = ip.TrimStart('0'); if (ip.Length == 0) ip = "0";
        fp = fp.TrimEnd('0');
        if (ip != "0" && (ip.Length > 11 || ip.Length + fp.Length > 15))
            return General(double.Parse((neg ? "-" : "") + ip + (fp.Length > 0 ? "." + fp : ""), CultureInfo.InvariantCulture));
        string r = ip + (fp.Length > 0 ? "." + fp : "");
        if (r == "0") return "0";
        return (neg ? "-" : "") + r;
    }

    static bool ValidDate(int y, int m, int d)
    {
        if (y < 1900 || y > 9999 || m < 1 || m > 12 || d < 1) return false;
        return d <= DateTime.DaysInMonth(y, m);
    }

    public static string[] Convert(string[] r, bool raw, int cols)
    {
        var o = new string[cols];
        for (int i = 0; i < cols; i++) o[i] = (i < r.Length) ? (raw ? r[i] : Conv(r[i])) : "";
        return o;
    }

    // Excel（ja-JP）がCSVを開いて保存したときの、1セルの値を返す
    public static string Conv(string s)
    {
        if (s.Length == 0) return s;
        string t = s.Trim(' ');
        if (t.Length == 0) return s;
        if (t[0] == '=') { EqualSignCells++; return s; }
        if (Num.IsMatch(t)) return FmtNum(s);
        Match m = Dt.Match(t);
        if (m.Success)
        {
            int y = int.Parse(m.Groups[1].Value), mo = int.Parse(m.Groups[2].Value), d = int.Parse(m.Groups[3].Value);
            int h = int.Parse(m.Groups[4].Value), mi = int.Parse(m.Groups[5].Value);
            if (y < 100) y += (y < 30) ? 2000 : 1900;
            if (ValidDate(y, mo, d) && h < 24 && mi < 60) return y + "/" + mo + "/" + d + " " + h + ":" + mi.ToString("00");
        }
        m = Ymd.Match(t);
        if (m.Success)
        {
            int y = int.Parse(m.Groups[1].Value), mo = int.Parse(m.Groups[2].Value), d = int.Parse(m.Groups[3].Value);
            if (y < 100) y += (y < 30) ? 2000 : 1900;
            if (ValidDate(y, mo, d)) return y + "/" + mo + "/" + d;
        }
        m = Md.Match(t);
        if (m.Success)
        {
            int mo = int.Parse(m.Groups[1].Value), d = int.Parse(m.Groups[2].Value);
            if (ValidDate(2000, mo, d)) return mo + "月" + d + "日";
        }
        m = Tm.Match(t);
        if (m.Success && int.Parse(m.Groups[1].Value) < 24 && int.Parse(m.Groups[2].Value) < 60)
            return int.Parse(m.Groups[1].Value) + ":" + m.Groups[2].Value + (m.Groups[3].Success ? ":" + m.Groups[4].Value : "");
        m = Yen.Match(t);   // 通貨（例 \2.200 → \2.20）。1例のみで確認した推定規則
        if (m.Success)
        {
            decimal dv = decimal.Parse(m.Groups[2].Value + m.Groups[3].Value, CultureInfo.InvariantCulture);
            string body = m.Groups[3].Success ? dv.ToString("#,##0.00", CultureInfo.InvariantCulture) : dv.ToString("#,##0", CultureInfo.InvariantCulture);
            return m.Groups[1].Value + "\\" + body;
        }
        string u = t.ToUpperInvariant();
        if (u == "TRUE" || u == "FALSE") return u;
        return s;
    }
}
'@
if (-not ('ShipCsv' -as [type])) { Add-Type -TypeDefinition $code -Language CSharp }

if (-not $LogDir) { $LogDir = Join-Path $PSScriptRoot 'logs' }
New-Item -ItemType Directory -Force $LogDir | Out-Null
$today = if ($Date) { [datetime]::ParseExact($Date, 'yyyyMMdd', $null) } else { (Get-Date).Date }
$todayStr = $today.ToString('yyyyMMdd')
$logFile = Join-Path $LogDir "Make-ShipUnbilledCsv_$todayStr.log"
function Log($msg) {
    $line = '{0} {1}' -f (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $msg
    Write-Host $line
    Add-Content -Path $logFile -Value $line -Encoding UTF8
}

# --- カレンダー（休日シートA列）を、Excelを使わずに読む ---
function Get-HolidaySet([string]$path) {
    Add-Type -AssemblyName System.IO.Compression
    $fs = New-Object System.IO.FileStream($path, 'Open', 'Read', 'ReadWrite')   # Excelが開いていても読めるよう共有読み取り
    try {
        $zip = New-Object System.IO.Compression.ZipArchive($fs, 'Read')
        function ReadEntry($name) {
            $e = $zip.GetEntry($name); if (-not $e) { return $null }
            $sr = New-Object System.IO.StreamReader($e.Open(), [Text.Encoding]::UTF8); $x = $sr.ReadToEnd(); $sr.Close(); return $x
        }
        $wb = [xml](ReadEntry 'xl/workbook.xml')
        $rels = [xml](ReadEntry 'xl/_rels/workbook.xml.rels')
        $relNs = 'http://schemas.openxmlformats.org/officeDocument/2006/relationships'
        $sheet = $wb.SelectNodes("//*[local-name()='sheet']") | Where-Object { $_.GetAttribute('name') -eq '休日' } | Select-Object -First 1
        if (-not $sheet) { throw "カレンダーに「休日」シートがありません: $path" }
        $rid = $sheet.GetAttribute('id', $relNs)
        $target = ($rels.SelectNodes("//*[local-name()='Relationship']") | Where-Object { $_.GetAttribute('Id') -eq $rid }).GetAttribute('Target')
        $target = $target.TrimStart('/'); if ($target -notlike 'xl/*') { $target = 'xl/' + $target }
        $ss = @()
        $ssx = ReadEntry 'xl/sharedStrings.xml'
        if ($ssx) { $ssd = [xml]$ssx; foreach ($si in $ssd.SelectNodes("//*[local-name()='si']")) { $ss += (($si.SelectNodes(".//*[local-name()='t']") | ForEach-Object { $_.InnerText }) -join '') } }
        $sx = [xml](ReadEntry $target)
        $set = New-Object 'System.Collections.Generic.HashSet[string]'
        $max = [datetime]'1900-01-01'
        foreach ($c in $sx.SelectNodes("//*[local-name()='c']")) {
            $r = $c.GetAttribute('r'); if ($r -notmatch '^A\d+$') { continue }
            $v = $c.SelectSingleNode("*[local-name()='v']"); if (-not $v) { continue }
            $d = $null
            if ($c.GetAttribute('t') -eq 's') { $txt = $ss[[int]$v.InnerText]; $dd = [datetime]::MinValue; if ([datetime]::TryParse($txt, [ref]$dd)) { $d = $dd } }
            else { $d = ([datetime]'1899-12-30').AddDays([double]$v.InnerText) }
            if ($d) { [void]$set.Add($d.Date.ToString('yyyyMMdd')); if ($d -gt $max) { $max = $d } }
        }
        return @{ Set = $set; Max = $max }
    } finally { $fs.Close() }
}

try {
    Log "開始: 当日=$todayStr 部門=$Dept 元=$SourceCsv"
    $prevStr = $PrevBusinessDay
    if ($prevStr -and $prevStr -notmatch '^\d{8}$') { Log "-PrevBusinessDay は yyyyMMdd で指定してください: $prevStr"; exit 1 }
    # RK-10がカレンダーで休日判定・前営業日を決める運用では、-PrevBusinessDay を渡す。その場合カレンダーは読まない
    if ($prevStr) { Log "前営業日はRK-10から受け取り（カレンダーは読まない）" }
    elseif (-not $SkipHolidayCheck -or -not $prevStr) {
        try { $cal = Get-HolidaySet $CalendarPath } catch { Log "カレンダーを読めません: $($_.Exception.Message)"; exit 21 }
        if ($cal.Max -lt $today.AddDays(30)) { Log "警告: カレンダーの休日リストの最終日は $($cal.Max.ToString('yyyy-MM-dd')) です。更新してください" }
        if (-not $SkipHolidayCheck -and $cal.Set.Contains($todayStr)) { Log "休日（$todayStr はカレンダーの休日リストにある）のため、作成しません"; exit 10 }
        if (-not $prevStr) {
            $d = $today
            do { $d = $d.AddDays(-1) } while ($d.DayOfWeek -in 'Saturday', 'Sunday' -or $cal.Set.Contains($d.ToString('yyyyMMdd')))   # WORKDAY.INTL(今日,-1,1,休日)
            $prevStr = $d.ToString('yyyyMMdd')
        }
    }
    Log "前営業日=$prevStr"

    if (-not (Test-Path -LiteralPath $SourceCsv)) { Log "元データが見つかりません: $SourceCsv"; exit 20 }
    $srcInfo = Get-Item -LiteralPath $SourceCsv
    Log ("元データ: {0:N0} バイト 更新 {1}" -f $srcInfo.Length, $srcInfo.LastWriteTime)
    # 元データは基幹システムが早朝(1:00〜2:30)に作り、以後24時間は変わらない。前日のファイルを誤って使わないよう更新日を確かめる
    if (-not $AllowStaleSource -and $srcInfo.LastWriteTime.Date -ne $today) { Log "元データの更新日が当日ではありません（$($srcInfo.LastWriteTime)）。基幹システムの出力が終わっていない可能性があります"; exit 22 }
    # 作成中（ロック中・サイズが変化中）のファイルを読まないよう、安定するまで待つ
    $deadline = (Get-Date).AddSeconds($WaitSeconds); $lastLen = -1
    while ($true) {
        $len = (Get-Item -LiteralPath $SourceCsv).Length
        try { $fs0 = [IO.File]::Open($SourceCsv, 'Open', 'Read', 'None'); $fs0.Close(); $locked = $false } catch { $locked = $true }
        if (-not $locked -and $len -eq $lastLen) { break }
        if ((Get-Date) -gt $deadline) { Log '元データが書き込み中のままです（待機時間切れ）'; exit 22 }
        $lastLen = $len; Start-Sleep -Seconds 5
    }
    $text = [Text.Encoding]::GetEncoding(932).GetString([IO.File]::ReadAllBytes($SourceCsv))
    $rows = [ShipCsv]::Parse($text)
    if ($rows.Count -lt 1) { Log '元データが空です'; exit 20 }
    $fc = $FilterColumn - 1
    $out = New-Object 'System.Collections.Generic.List[string[]]'
    $hdr = [string[]]$rows[0]
    $out.Add([ShipCsv]::Convert($hdr, $RawValues.IsPresent, $MaxColumns))
    for ($i = 1; $i -lt $rows.Count; $i++) {
        $r = $rows[$i]
        if ($r.Length -le $fc) { continue }
        $key = if ($RawValues) { $r[$fc] } else { [ShipCsv]::Conv($r[$fc]) }
        if ($key -eq $prevStr) { $out.Add([ShipCsv]::Convert($r, $RawValues.IsPresent, $MaxColumns)) }
    }
    $n = $out.Count - 1
    Log "元 $($rows.Count - 1) 行 → 該当 $n 行（最終出荷日=$prevStr）"
    if ([ShipCsv]::EqualSignCells -gt 0) { Log "警告: 「=」で始まるセルが $([ShipCsv]::EqualSignCells) 件（Excelでは数式になる。変換していない）" }

    if ($OutputCsv) { $dest = $OutputCsv; $destDir = Split-Path -Parent $dest }
    else { $destDir = $OutputDir; $dest = Join-Path $OutputDir ("未計上明細_{0}_{1}.csv" -f $Dept, $todayStr) }
    New-Item -ItemType Directory -Force $destDir | Out-Null
    $tmp = $dest + '.tmp'
    [ShipCsv]::Write($tmp, $out, $MaxColumns)
    Move-Item -LiteralPath $tmp -Destination $dest -Force
    Log ("作成: {0}（{1:N0} バイト、見出しを除き {2} 行）" -f $dest, (Get-Item -LiteralPath $dest).Length, $n)

    if ($CompareWith) {
        $a = [IO.File]::ReadAllBytes($CompareWith); $b = [IO.File]::ReadAllBytes($dest)
        $same = ($a.Length -eq $b.Length)
        if ($same) { for ($k = 0; $k -lt $a.Length; $k++) { if ($a[$k] -ne $b[$k]) { $same = $false; break } } }
        if ($same) { Log "比較: $CompareWith と完全に一致（バイト単位）" }
        else {
            $enc = [Text.Encoding]::GetEncoding(932)
            $ea = [ShipCsv]::Parse($enc.GetString($a)); $eb = [ShipCsv]::Parse($enc.GetString($b))
            $cnt = 0; $shown = 0
            for ($i = 0; $i -lt [Math]::Max($ea.Count, $eb.Count); $i++) {
                if ($i -ge $ea.Count -or $i -ge $eb.Count) { $cnt++; if ($shown -lt 20) { Log "比較: 行 $($i + 1) が片方にしかない"; $shown++ }; continue }
                for ($j = 0; $j -lt [Math]::Max($ea[$i].Length, $eb[$i].Length); $j++) {
                    $x = if ($j -lt $ea[$i].Length) { $ea[$i][$j] } else { '' }; $y = if ($j -lt $eb[$i].Length) { $eb[$i][$j] } else { '' }
                    if ($x -ne $y) { $cnt++; if ($shown -lt 20) { Log ("比較: 行{0} 列{1}（{2}） 既存='{3}' 今回='{4}'" -f ($i + 1), ($j + 1), $ea[0][$j], $x, $y); $shown++ } }
                }
            }
            Log "比較: 既存と差あり（セル/行の差 $cnt 件）。バイトでは不一致"
            exit 40
        }
    }
    if ($n -eq 0) { Log '警告: 該当行が0件（見出しだけ作成）'; exit 30 }
    exit 0
} catch {
    Log "エラー: $($_.Exception.Message)"
    exit 1
}
