# dirt2 -- launch dirt2.3dsx in an isolated Azahar instance, capture, shut down.
#
# Copied from sfs/tools/run_azahar.ps1 and adapted, rather than referenced across
# projects. Same two rules it was written for, both learned the hard way:
#
#  1. Runs a RENAMED COPY (azahar_dirt2.exe). Parallel sessions on this machine
#     run `taskkill /F /IM azahar.exe`, which kills every instance by name; a
#     differently-named binary is immune and still shares the virtual SD card.
#  2. We stop ONLY our own PID. Never -Name, never /IM.
#
# Usage:
#   powershell -NoProfile -File tools/run_azahar.ps1 -Seconds 14 -Shots "3,7,12"

param(
    [string]$Emu     = "",
    [string]$Rom     = "",
    [int]   $Seconds = 14,
    [string]$Shots   = "3,7,12",
    [string]$OutDir  = ""
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)

if (-not $Emu)    { $Emu    = "C:\Users\steve\Documents\3ds-project-folder\azahar\azahar_dirt2.exe" }
if (-not $Rom)    { $Rom    = Join-Path $root "dirt2.3dsx" }
if (-not $OutDir) { $OutDir = Join-Path $env:TEMP "dirt2-shots" }

if (-not (Test-Path $Emu)) { Write-Error "emulator not found: $Emu"; exit 2 }
if (-not (Test-Path $Rom)) { Write-Error "rom not found: $Rom"; exit 3 }
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
Get-ChildItem -Path $OutDir -Filter *.png -ErrorAction SilentlyContinue | Remove-Item -Force

# A uniformly black capture usually means the workstation is LOCKED, not that the
# GL surface is unreadable -- a whole session was once lost to that misdiagnosis.
# Refuse up front instead of producing black frames and interpreting them.
if (Get-Process LogonUI -ErrorAction SilentlyContinue) {
    Write-Error "LogonUI is running: the workstation is locked, every capture would be black"
    exit 7
}

$shotTimes = $Shots.Split(",") | ForEach-Object { [int]$_.Trim() } | Sort-Object

$proc = Start-Process -FilePath $Emu -ArgumentList "`"$Rom`"" -PassThru
Write-Output "launched pid=$($proc.Id) rom=$Rom"

$elapsed = 0
foreach ($t in $shotTimes) {
    Start-Sleep -Seconds ($t - $elapsed)
    $elapsed = $t
    if ($proc.HasExited) { Write-Output "EXITED EARLY at ${t}s code=$($proc.ExitCode)"; break }
    $proc.Refresh()
    $out = Join-Path $OutDir ("t{0:d3}.png" -f $t)
    & powershell -NoProfile -File (Join-Path $root "tools\capture.ps1") -ProcId $proc.Id -Out $out
}

if (-not $proc.HasExited) {
    Start-Sleep -Seconds ([Math]::Max(0, $Seconds - $elapsed))
}

if ($proc.HasExited) {
    Write-Output "process exited on its own, code=$($proc.ExitCode)"
} else {
    Stop-Process -Id $proc.Id -Force        # our PID only, never -Name
    Write-Output "stopped pid=$($proc.Id)"
}
Write-Output "shots in $OutDir"
