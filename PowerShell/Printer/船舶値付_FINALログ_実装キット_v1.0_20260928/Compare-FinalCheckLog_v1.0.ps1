param(
    [Parameter(Mandatory=$true)][string]$VbaCsv,
    [Parameter(Mandatory=$true)][string]$PsCsv,
    [string]$OutDir = ".\FINAL_COMPARE",
    [switch]$TrimEndForCompare
)

$ErrorActionPreference = "Stop"
if (-not (Test-Path $VbaCsv)) { throw "VBA CSV not found: $VbaCsv" }
if (-not (Test-Path $PsCsv))  { throw "PS CSV not found: $PsCsv" }
if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir -Force | Out-Null }

$complete = [IO.Path]::ChangeExtension($VbaCsv, ".complete")
if (-not (Test-Path $complete)) {
    throw "VBA complete marker not found. Incomplete Run is not comparable: $complete"
}

$vba = @(Import-Csv -Path $VbaCsv -Encoding Default)
$ps  = @(Import-Csv -Path $PsCsv -Encoding UTF8)

function Norm([string]$s) {
    if ($null -eq $s) { return "" }
    if ($TrimEndForCompare) { return $s.TrimEnd() }
    return $s
}
function Key($r) {
    return ("{0}|{1}|{2}|{3}" -f $r.OrderNo,$r.DetailNo,$r.Area,$r.Position)
}

# AH1 is a physical-page/header concern and is checked in T-PAGE.
$ps = @($ps | Where-Object { $_.Position -ne "AH1" -and $_.Cell -ne "AH1" })

# PS Page2+ repeats TOP/HEAD etc. Collapse identical duplicates.
$psNorm = New-Object System.Collections.Generic.List[object]
$psGroups = $ps | Group-Object { Key $_ }
foreach ($g in $psGroups) {
    $rows = @($g.Group)
    $values = @($rows | ForEach-Object { Norm ([string]$_.Value) } | Select-Object -Unique)
    $lines  = @($rows | ForEach-Object { [string]$_.LineNo } | Select-Object -Unique)
    if ($values.Count -gt 1) {
        [PSCustomObject]@{Key=$g.Name;Issue="PS_REPEAT_VALUE_CONFLICT";Values=($values -join " || ")} |
            Export-Csv -Path (Join-Path $OutDir "PS_REPEAT_CONFLICT.csv") -NoTypeInformation -Encoding UTF8 -Append
        continue
    }
    if ($lines.Count -gt 1 -and [int]$rows[0].DetailNo -gt 0) {
        [PSCustomObject]@{Key=$g.Name;Issue="PS_REPEAT_LINENO_CONFLICT";Values=($lines -join " || ")} |
            Export-Csv -Path (Join-Path $OutDir "PS_REPEAT_CONFLICT.csv") -NoTypeInformation -Encoding UTF8 -Append
        continue
    }
    $psNorm.Add($rows[0]) | Out-Null
}

$vbaMap=@{}
$vbaDup=New-Object System.Collections.Generic.List[object]
foreach($r in $vba){
    $k=Key $r
    if($vbaMap.ContainsKey($k)){
        $vbaDup.Add([PSCustomObject]@{Key=$k;Side="VBA"})|Out-Null
    } else {$vbaMap[$k]=$r}
}
$psMap=@{}
$psDup=New-Object System.Collections.Generic.List[object]
foreach($r in $psNorm){
    $k=Key $r
    if($psMap.ContainsKey($k)){
        $psDup.Add([PSCustomObject]@{Key=$k;Side="PS"})|Out-Null
    } else {$psMap[$k]=$r}
}

$results=New-Object System.Collections.Generic.List[object]
$allKeys=@($vbaMap.Keys + $psMap.Keys | Sort-Object -Unique)
foreach($k in $allKeys){
    $vr=if($vbaMap.ContainsKey($k)){$vbaMap[$k]}else{$null}
    $pr=if($psMap.ContainsKey($k)){$psMap[$k]}else{$null}
    $status=""
    if($null -eq $vr){$status="PS_ONLY"}
    elseif($null -eq $pr){$status="VBA_ONLY"}
    else{
        $vText=Norm ([string]$vr.Text)
        $pText=Norm ([string]$pr.Value)
        if($vText -ne $pText){$status="VALUE_DIFF"}
        elseif([string]$vr.LineNo -ne [string]$pr.LineNo -and [int]$vr.DetailNo -gt 0){$status="LINENO_DIFF"}
        else{$status="PASS"}
    }
    $results.Add([PSCustomObject]@{
        Key=$k;Status=$status
        VBA_Text=if($vr){[string]$vr.Text}else{""}
        PS_Value=if($pr){[string]$pr.Value}else{""}
        VBA_Value2=if($vr){[string]$vr.Value2}else{""}
        VBA_LineNo=if($vr){[string]$vr.LineNo}else{""}
        PS_LineNo=if($pr){[string]$pr.LineNo}else{""}
        VBA_ActualCell=if($vr){[string]$vr.ActualCell}else{""}
        PS_Page=if($pr){[string]$pr.Page}else{""}
        VBA_X=if($vr){[string]$vr.X}else{""}
        PS_X=if($pr){[string]$pr.X}else{""}
        VBA_Y=if($vr){[string]$vr.Y}else{""}
        PS_Y=if($pr){[string]$pr.Y}else{""}
        VBA_Width=if($vr){[string]$vr.Width}else{""}
        PS_Width=if($pr){[string]$pr.Width}else{""}
        VBA_Height=if($vr){[string]$vr.Height}else{""}
        PS_Height=if($pr){[string]$pr.Height}else{""}
        TextHashFlag=if($vr){[string]$vr.TextHashFlag}else{""}
    })|Out-Null
}

$results | Export-Csv -Path (Join-Path $OutDir "FINAL_COMPARE_ALL.csv") -NoTypeInformation -Encoding UTF8
$results | Where-Object {$_.Status -ne "PASS"} |
    Export-Csv -Path (Join-Path $OutDir "FINAL_COMPARE_DIFF.csv") -NoTypeInformation -Encoding UTF8
@($vbaDup)+@($psDup) |
    Export-Csv -Path (Join-Path $OutDir "FINAL_COMPARE_DUPLICATES.csv") -NoTypeInformation -Encoding UTF8

$summary = $results | Group-Object Status | Sort-Object Name
Write-Host "============================================================"
Write-Host " FINAL CheckLog Compare"
Write-Host "============================================================"
Write-Host ("VBA rows : {0}" -f $vba.Count)
Write-Host ("PS rows  : {0} (normalized {1})" -f $ps.Count,$psNorm.Count)
foreach($g in $summary){ Write-Host ("{0,-16} {1,6}" -f $g.Name,$g.Count) }
Write-Host ("VBA duplicate keys: {0}" -f $vbaDup.Count)
Write-Host ("PS duplicate keys : {0}" -f $psDup.Count)
Write-Host ""
Write-Host "NOTE: Coordinate PASS/FAIL is intentionally NOT judged yet."
Write-Host "      Fix origin transform/tolerance after T0 observation, before FINAL coordinate comparison."
if($TrimEndForCompare){
    Write-Host "NOTE: TrimEndForCompare is ON. Use only after T0 establishes this normalization rule."
}
