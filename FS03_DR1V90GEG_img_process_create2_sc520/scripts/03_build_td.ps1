# =====================================================================
#  03_build_td.ps1  (FS03_DR1V90GEG_img_process_create2_sc520)
#  Run Anlogic TD (TangDynasty) synthesis + P&R + bitstream on the new
#  project, then extract resource / timing results.
#  Usage:
#    .\03_build_td.ps1
#    .\03_build_td.ps1 -Gui
# =====================================================================
[CmdletBinding()]
param(
    [string]$PrjRoot = 'D:\Documents\bc10anlogictd\FS03_DR1V90GEG_img_process_create2_sc520',
    [string]$TdBin   = 'D:\Anlogic\TD_6.2.1_Engineer_6.2.168.116\bin',
    [switch]$Gui
)

$ErrorActionPreference = 'Stop'

$New  = 'FS03_DR1V90GEG_img_process_create2_sc520'
$NAME = 'fs03_img_process_create2_sc520'
$PRJ  = Join-Path $PrjRoot "prj"
$AL   = Join-Path $PRJ   "$NAME.al"
$RUNS = Join-Path $PRJ   "${NAME}_Runs"
$TD   = Join-Path $TdBin 'td_commands_prompt.exe'
$TDGUI= Join-Path $TdBin 'td.exe'

if (-not (Test-Path $AL)) { throw "project not found: $AL (run 02_create_project.ps1 first)" }
if (-not (Test-Path $TD)) { throw "TD console not found: $TD" }

function Invoke-Td {
    param(
        [string]$TclPath,
        [string]$WorkDir,
        [string]$Tag,
        [int]$MaxMin = 40
    )
    $arg    = $TclPath -replace '\\', '/'
    $outLog = Join-Path $WorkDir 'out.log'
    $errLog = Join-Path $WorkDir 'err.log'
    if (Test-Path $outLog) { Remove-Item $outLog -Force -ErrorAction SilentlyContinue }
    if (Test-Path $errLog) { Remove-Item $errLog -Force -ErrorAction SilentlyContinue }

    Write-Host ("[td ] {0} -> {1}" -f $Tag, $arg)
    $proc = Start-Process -FilePath $TD -ArgumentList $arg -WorkingDirectory $WorkDir -RedirectStandardOutput $outLog -RedirectStandardError $errLog -NoNewWindow -PassThru

    $deadline = (Get-Date).AddMinutes($MaxMin)
    while (-not $proc.HasExited) {
        Start-Sleep -Seconds 10
        if ((Get-Date) -gt $deadline) {
            Write-Warning ("[td ] {0} timeout ({1} min), force stop" -f $Tag, $MaxMin)
            Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
            break
        }
    }
    Write-Host ("[td ] {0} done" -f $Tag)

    if (Test-Path $outLog) {
        $bad = @(Select-String -Path $outLog -Pattern 'ERROR' -ErrorAction SilentlyContinue | Select-Object -First 10)
        if ($bad.Count -gt 0) {
            Write-Warning ("[td ] {0} has ERROR in log:" -f $Tag)
            foreach ($b in $bad) { Write-Host ("      " + $b.Line) }
        }
    }
    return
}

if ($Gui) {
    Write-Host '[gui] opening TD GUI ...'
    Start-Process -FilePath $TDGUI -ArgumentList $AL -WorkingDirectory $PRJ
    return
}

Get-ChildItem -Path $PRJ -File -ErrorAction SilentlyContinue |
    Where-Object { $_.Name -match '^(probe|open_only|td_run|td_batch)' } |
    Remove-Item -Force -ErrorAction SilentlyContinue

if (Test-Path $RUNS) {
    Write-Host '[td ] clearing old _Runs ...'
    Remove-Item -LiteralPath $RUNS -Recurse -Force -ErrorAction SilentlyContinue
}

$openTcl = Join-Path $PRJ 'open_only.tcl'

Write-Host '[td ] creating run dir (syn_1) ...'
Set-Content -Path $openTcl -Value @("open_project ./$NAME.al", 'launch_runs syn_1', 'after 25000', 'exit') -Encoding ASCII
Invoke-Td -TclPath $openTcl -WorkDir $PRJ -Tag 'open/syn_1' -MaxMin 8

Write-Host '[td ] creating run dir (phy_1) ...'
Set-Content -Path $openTcl -Value @("open_project ./$NAME.al", 'launch_runs phy_1', 'after 25000', 'exit') -Encoding ASCII
Invoke-Td -TclPath $openTcl -WorkDir $PRJ -Tag 'open/phy_1' -MaxMin 8

foreach ($run in @('syn_1', 'phy_1')) {
    $cfg = Join-Path $RUNS "$run"
    $cfg = Join-Path $cfg 'settings.cfg'
    if (-not (Test-Path $cfg)) {
        Write-Warning "[cfg] missing $cfg"
        continue
    }
    $c = Get-Content $cfg
    $c = $c -replace '(?m)^set device_name\s*$',  'set device_name dr1_90.db'
    $c = $c -replace '(?m)^set package_name\s*$', 'set package_name DR1V90GEG484'
    Set-Content -Path $cfg -Value $c -Encoding ASCII
    $d = @($c | Where-Object { $_ -match 'device_name|package_name' })
    $txt = $d -join ' | '
    Write-Host ("[cfg] {0} : {1}" -f $run, $txt)
}

$synDir = Join-Path $RUNS 'syn_1'
$synTcl = Join-Path $synDir "$NAME.tcl"
if (-not (Test-Path $synTcl)) { throw "[td ] flow script not found: $synTcl" }
Invoke-Td -TclPath $synTcl -WorkDir $synDir -Tag 'syn_1' -MaxMin 40

$phyDir = Join-Path $RUNS 'phy_1'
$phyTcl = Join-Path $phyDir "$NAME.tcl"
if (-not (Test-Path $phyTcl)) { throw "[td ] flow script not found: $phyTcl" }
Invoke-Td -TclPath $phyTcl -WorkDir $phyDir -Tag 'phy_1' -MaxMin 60

$area = @(Get-ChildItem -Path $RUNS -Recurse -Filter '*_phy.area' -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending)
if ($area.Count -gt 0) {
    Write-Host ''
    Write-Host ("[rpt] resource report: {0}" -f $area[0].FullName)
    $lines = @(Get-Content $area[0].FullName | Where-Object { $_ -match '^\s*#' } | Select-Object -First 40)
    foreach ($l in $lines) { Write-Host $l }
}
else {
    Write-Warning '[rpt] no *_phy.area found, check _Runs\phy_1\out.log'
}

$ts = @(Get-ChildItem -Path $RUNS -Recurse -Filter '*.ts' -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending)
if ($ts.Count -gt 0) {
    Write-Host ''
    Write-Host ("[rpt] timing summary: {0}" -f $ts[0].FullName)
    $lines = @(Get-Content $ts[0].FullName | Where-Object { $_ -match 'SWNS|HWNS|STNS|HTNS' })
    foreach ($l in $lines) { Write-Host $l }
}

$bit = @(Get-ChildItem -Path $PRJ -Recurse -Filter '*.bit' -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending)
if ($bit.Count -gt 0) { Write-Host ("[rpt] bitstream: {0}" -f $bit[0].FullName) }

Write-Host ''
Write-Host '== 03_build_td.ps1 done =='
