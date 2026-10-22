<#
  Make-ShipUnbilledCsv.ps1 のローカル試験（AI_TESTのサンドボックス内）。navicsv には書き込まない（読み取りのみ）。
  使い方: .\Run-LocalTest.ps1 -Date 20261022 -PrevBusinessDay 20261021
  結果: <このフォルダ>\Evidence\LocalTest_<日付>_<時刻>\ にログ・ハッシュ・結果表を保存する。
#>
param(
    [string]$Date = '20261022',
    [string]$PrevBusinessDay = '20261021',
    [string]$Dept = '218001',
    [string]$NaviDir = '\\192.168.200.5\navicsv'
)
$ErrorActionPreference = 'Stop'
$here = $PSScriptRoot
$script = Join-Path $here 'Make-ShipUnbilledCsv.ps1'
$ev = Join-Path $here ('Evidence\LocalTest_{0}_{1}' -f $Date, (Get-Date -Format 'HHmmss'))
New-Item -ItemType Directory -Force "$ev\logs", "$ev\in", "$ev\out" | Out-Null
$ref = Join-Path $NaviDir ("未計上明細_{0}_{1}.csv" -f $Dept, $Date)
$src = Join-Path $NaviDir '未計上明細（全件）_PRONES.csv'
$cal = 'C:\HPDB\AI_TEST\ShipValueTag\カレンダー(2023).xlsx'
function Sha($p) { (Get-FileHash -LiteralPath $p -Algorithm SHA256).Hash }
$results = @()
function Run($name, $arguments, $expectExit, $expectOutput) {
    $log = Join-Path "$ev\logs" "$name.console.log"
    $sw = [Diagnostics.Stopwatch]::StartNew()
    $cmd = $arguments -join ' '
    & powershell -NoProfile -ExecutionPolicy Bypass -File $script @arguments *> $log
    $code = $LASTEXITCODE
    $sec = [Math]::Round($sw.Elapsed.TotalSeconds, 1)
    $outSha = ''; $match = ''
    if ($expectOutput -and (Test-Path -LiteralPath $expectOutput)) { $outSha = Sha $expectOutput; $match = if ($outSha -eq $script:refSha) { '一致' } else { '不一致' } }
    elseif ($expectOutput) { $match = '出力なし' }
    $ok = ($code -eq $expectExit) -and (($expectOutput -eq $null) -or ($match -eq '一致'))
    $script:results += [pscustomobject]@{ Test = $name; ExitCode = $code; Expect = $expectExit; OutputSha256 = $outSha; VsRK10 = $match; Seconds = $sec; Result = $(if ($ok) { 'PASS' } else { 'FAIL' }) }
}

# 準備：基準（RK-10の出力）と元データのハッシュ
$refSha = Sha $ref; $srcSha = Sha $src
$srcItem = Get-Item -LiteralPath $src; $refItem = Get-Item -LiteralPath $ref
Copy-Item -LiteralPath $src -Destination "$ev\in\PRONES_copy.csv"      # 更新日時もコピーされる
(Get-Item "$ev\in\PRONES_copy.csv").LastWriteTime = $srcItem.LastWriteTime
Copy-Item -LiteralPath $src -Destination "$ev\in\PRONES_old.csv"
(Get-Item "$ev\in\PRONES_old.csv").LastWriteTime = $srcItem.LastWriteTime.AddDays(-1)   # 前日のファイルを想定

# T1 RK-10方式：既定の入力(navicsv)・前営業日を渡す・出力はサンドボックス
Run 'T1_RK10方式_既定入力' @('-Date', $Date, '-PrevBusinessDay', $PrevBusinessDay, '-Dept', $Dept, '-OutputDir', "$ev\out\T1", '-LogDir', "$ev\logs", '-CompareWith', $ref) 0 "$ev\out\T1\未計上明細_${Dept}_${Date}.csv"
# T2 単独方式：カレンダーから前営業日を計算
Run 'T2_単独方式_カレンダー' @('-Date', $Date, '-Dept', $Dept, '-CalendarPath', $cal, '-OutputDir', "$ev\out\T2", '-LogDir', "$ev\logs", '-CompareWith', $ref) 0 "$ev\out\T2\未計上明細_${Dept}_${Date}.csv"
# T3 入出力を別の場所・別の名前で（相対パス）
Push-Location $ev
Run 'T3_入出力を別の場所へ_相対パス' @('-Date', $Date, '-PrevBusinessDay', $PrevBusinessDay, '-SourceCsv', '.\in\PRONES_copy.csv', '-OutputCsv', '.\out\T3\別名_出力.csv', '-LogDir', "$ev\logs", '-CompareWith', $ref) 0 "$ev\out\T3\別名_出力.csv"
Pop-Location
# T4 異常系
Run 'T4a_元データなし' @('-Date', $Date, '-PrevBusinessDay', $PrevBusinessDay, '-SourceCsv', "$ev\in\ない.csv", '-OutputDir', "$ev\out\T4a", '-LogDir', "$ev\logs") 20 $null
Run 'T4b_前日の元データ' @('-Date', $Date, '-PrevBusinessDay', $PrevBusinessDay, '-SourceCsv', "$ev\in\PRONES_old.csv", '-OutputDir', "$ev\out\T4b", '-LogDir', "$ev\logs") 22 $null
Run 'T4c_休日_1023' @('-Date', '20261023', '-CalendarPath', $cal, '-OutputDir', "$ev\out\T4c", '-LogDir', "$ev\logs") 10 $null
Run 'T4d_前日の元データを許可' @('-Date', $Date, '-PrevBusinessDay', $PrevBusinessDay, '-SourceCsv', "$ev\in\PRONES_old.csv", '-AllowStaleSource', '-OutputDir', "$ev\out\T4d", '-LogDir', "$ev\logs", '-CompareWith', $ref) 0 "$ev\out\T4d\未計上明細_${Dept}_${Date}.csv"

$results | Export-Csv "$ev\結果表.csv" -NoTypeInformation -Encoding UTF8
$results | Format-Table -AutoSize | Out-String -Width 220 | Write-Host
$info = [ordered]@{
    実行日時 = (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'); 当日 = $Date; 前営業日 = $PrevBusinessDay; 部門 = $Dept
    スクリプト = $script; スクリプトSHA256 = (Sha $script)
    元データ = $src; 元データSHA256 = $srcSha; 元データ更新日時 = $srcItem.LastWriteTime.ToString('yyyy-MM-dd HH:mm:ss'); 元データ_バイト = $srcItem.Length
    基準RK10出力 = $ref; 基準SHA256 = $refSha; 基準更新日時 = $refItem.LastWriteTime.ToString('yyyy-MM-dd HH:mm:ss'); 基準バイト = $refItem.Length
    PS版 = $PSVersionTable.PSVersion.ToString()
}
$info | ConvertTo-Json | Set-Content "$ev\試験条件.json" -Encoding UTF8
Remove-Item "$ev\in\PRONES_copy.csv", "$ev\in\PRONES_old.csv" -Force   # 13MBの複製は証跡に残さない（ハッシュで代える）
Get-ChildItem $ev -Recurse -File | Where-Object { $_.Name -ne 'SHA256SUMS.txt' } | ForEach-Object { '{0}  {1}' -f (Sha $_.FullName), $_.FullName.Substring($ev.Length + 1) } | Set-Content "$ev\SHA256SUMS.txt" -Encoding UTF8
Write-Host "証跡: $ev"
if (@($results | Where-Object Result -ne 'PASS').Count -gt 0) { exit 1 } else { exit 0 }
