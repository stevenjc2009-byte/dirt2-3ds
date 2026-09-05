# dirt2 -- launch dirt2.3dsx, HOLD emulator buttons, capture at timed marks, shut down.
#
# Why this exists: the host probe drives vehicle_step() directly. That proves the
# physics, but it does NOT prove the shipped input path
# (hidScanInput -> input_update -> vehicle_step) reaches the car on real firmware.
# A still screenshot of an idle frame cannot prove "it drives" either. This drives
# the input for real instead of adding a demo mode to the game.
#
# Adapted from sfs/tools/run_azahar_keys.ps1 -- COPIED into this project, not
# referenced across projects. The touch/stylus machinery is dropped (dirt2 uses no
# touch); the Focus() mechanism is kept verbatim because it is the proven part.
#
# Same two isolation rules as run_azahar.ps1:
#   1. Runs a RENAMED COPY (azahar_dirt2.exe) -- a parallel session's
#      `taskkill /F /IM azahar.exe` cannot kill us.
#   2. We stop ONLY our own PID. Never -Name, never /IM.
#
# KEY CODES are read out of %APPDATA%\Azahar\config\qt-config.ini [Controls]
# profile 1, NOT assumed -- steve's profile is remapped away from Citra defaults:
#   button_a=code:65 'A'   button_l=code:81 'Q'   button_r=code:69 'E'
#   circle_pad = analog_from_button on Qt::Key_Left/Up/Right/Down (16777234..7),
#   i.e. the ARROW KEYS.
# dirt2's own mapping (source/input/input.c:227) is: KEY_R throttle, KEY_L brake,
# KEY_A handbrake, circle pad steer. So E accelerates, Q brakes, arrows steer.
#
# Usage:
#   powershell -NoProfile -File tools/run_azahar_keys.ps1 -Steps "4:none,8:E,12:E,16:Q"

param(
    [string]$Emu    = "",
    [string]$Rom    = "",
    [string]$Steps  = "4:none",
    [string]$OutDir = "",
    [int]   $Tail   = 2,
    [int]   $WinW   = 880,      # logical px -- this process is deliberately DPI-unaware
    [int]   $WinH   = 1128
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
if (Get-Process LogonUI -ErrorAction SilentlyContinue) {
    Write-Error "LogonUI is running: the workstation is locked, every capture would be black"
    exit 7
}

Add-Type @"
using System;
using System.Runtime.InteropServices;
[StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
[StructLayout(LayoutKind.Sequential)] public struct POINT { public int X, Y; }
public class Dirt2Keys {
    [DllImport("user32.dll")] public static extern void keybd_event(byte k, byte s, uint f, UIntPtr e);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
    [DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
    [DllImport("user32.dll")] public static extern bool GetCursorPos(out POINT p);
    [DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
    [DllImport("user32.dll")] public static extern void mouse_event(uint f, uint dx, uint dy, uint d, UIntPtr e);
    [DllImport("user32.dll")] public static extern bool MoveWindow(IntPtr h, int x, int y, int w, int ht, bool repaint);

    public static void Resize(IntPtr h, int w, int ht) { MoveWindow(h, 40, 8, w, ht, true); }

    public const uint LEFTDOWN = 0x0002, LEFTUP = 0x0004;
    public const uint KEYUP = 0x0002;
    public static void Down(byte k) { keybd_event(k, 0, 0, UIntPtr.Zero); }
    public static void Up(byte k)   { keybd_event(k, 0, KEYUP, UIntPtr.Zero); }

    // Windows refuses SetForegroundWindow from a background process unless the
    // calling thread has recent input, and a bare call is flaky -- a held key can
    // silently fail to arrive and the missing effect gets blamed on the game.
    //
    // The fix is a real mouse click INTO THE TOP-SCREEN AREA, which both satisfies
    // the foreground rule and gives the render widget keyboard focus.
    //   - Do NOT tap ALT to unlock the foreground: in Qt that activates the menu bar
    //     and every following letter key is eaten as a mnemonic.
    //   - Click at 30% height: inside the top screen, clear of the menu bar, never
    //     on the touch-sensitive bottom screen.
    public static bool Focus(IntPtr h) {
        RECT r; if (!GetWindowRect(h, out r)) return false;
        POINT old; GetCursorPos(out old);
        int x = (r.Left + r.Right) / 2;
        int y = r.Top + (int)((r.Bottom - r.Top) * 0.30);
        for (int i = 0; i < 6; i++) {
            SetCursorPos(x, y);
            mouse_event(LEFTDOWN, 0, 0, 0, UIntPtr.Zero);
            System.Threading.Thread.Sleep(60);
            mouse_event(LEFTUP, 0, 0, 0, UIntPtr.Zero);
            SetForegroundWindow(h);
            System.Threading.Thread.Sleep(200);
            if (GetForegroundWindow() == h) { SetCursorPos(old.X, old.Y); return true; }
        }
        SetCursorPos(old.X, old.Y);
        return false;
    }
}
"@

# One CHARACTER per key, because a step's key list is split character by character
# below. Codes read out of qt-config.ini, never assumed.
$VK = @{
    "E" = [byte]69            # button_r  -> dirt2 THROTTLE  (input.c:230)
    "Q" = [byte]81            # button_l  -> dirt2 BRAKE     (input.c:231)
    "A" = [byte]65            # button_a  -> dirt2 HANDBRAKE (input.c:232)
    # Circle pad, via the arrow keys (analog_from_button). Named with symbols
    # because "L"/"R" would collide with nothing here but the arrows have no letter.
    "<" = [byte]0x25; "^" = [byte]0x26
    ">" = [byte]0x27; "v" = [byte]0x28
}

$plan = @()
foreach ($s in $Steps.Split(",")) {
    $bits = $s.Split(":")
    if ($bits.Count -ne 2) { Write-Error "malformed step '$s' - expected <seconds>:<keys>"; exit 7 }
    $keys = $bits[1].Trim()
    if ($keys -ne "none") {
        foreach ($c in $keys.ToCharArray()) {
            if (-not $VK.ContainsKey([string]$c)) {
                Write-Error "step '$s' names unknown key '$c' - valid: E Q A < > ^ v, or 'none'"
                exit 8
            }
        }
    }
    $plan += [pscustomobject]@{ T = [int]$bits[0].Trim(); Keys = $keys }
}
$plan = $plan | Sort-Object T

$proc = Start-Process -FilePath $Emu -ArgumentList "`"$Rom`"" -PassThru
Write-Output "launched pid=$($proc.Id) rom=$Rom"

# $mainHwnd CACHES the top-level window the moment it first exists. Do not read
# $proc.MainWindowHandle again once set: it can come back EMPTY later, and `-ne 0`
# is true for empty, which throws on the next Focus() call.
$mainHwnd = [IntPtr]::Zero
for ($i = 0; $i -lt 40; $i++) {
    Start-Sleep -Milliseconds 250
    $proc.Refresh()
    if ($proc.MainWindowHandle -ne 0) {
        $mainHwnd = $proc.MainWindowHandle
        [void][Dirt2Keys]::Resize($mainHwnd, $WinW, $WinH)
        Write-Output "sized window to ${WinW}x${WinH} logical px (hwnd=$mainHwnd)"
        break
    }
}

$held = @()
$elapsed = 0
foreach ($step in $plan) {
    if ($proc.HasExited) { Write-Output "EXITED EARLY code=$($proc.ExitCode)"; break }

    Start-Sleep -Seconds ([Math]::Max(0, $step.T - $elapsed - 1))

    $proc.Refresh()
    if ($mainHwnd -ne [IntPtr]::Zero) {
        if (-not [Dirt2Keys]::Focus($mainHwnd)) {
            Write-Output "WARNING: could not focus emulator for step $($step.T):$($step.Keys) - keys will not arrive"
        }
    }
    Start-Sleep -Milliseconds 300

    # Release only what this step does NOT want, and re-press what it does. Keeping a
    # continuing key DOWN across steps matters here: throttle is ramped in software
    # (input.c throttle_ramp_rate), so a key that is released and re-pressed every
    # step never reaches full throttle.
    $want = @()
    if ($step.Keys -ne "none") { foreach ($c in $step.Keys.ToCharArray()) { $want += [string]$c } }
    foreach ($k in $held) { if ($want -notcontains $k) { [Dirt2Keys]::Up($VK[$k]) } }
    foreach ($k in $want) { [Dirt2Keys]::Down($VK[$k]) }   # re-Down of a held key is harmless
    $held = $want

    Start-Sleep -Milliseconds 700
    $elapsed = $step.T

    # '<' and '>' are illegal in a Windows filename (Bitmap.Save throws and the shot
    # is silently lost); '[' and ']' would be PowerShell wildcards. Spell them out.
    $tag = $step.Keys.Replace("<", "left").Replace(">", "right").Replace("^", "up")
    $out = Join-Path $OutDir ("t{0:d3}_{1}.png" -f $step.T, $tag)
    # -WindowStyle Hidden: a visible console would steal focus from the emulator and
    # drop whatever key is being held for the next step.
    & powershell -NoProfile -WindowStyle Hidden -File (Join-Path $root "tools\capture.ps1") -ProcId $proc.Id -Out $out
}

foreach ($k in $held) { [Dirt2Keys]::Up($VK[$k]) }

if (-not $proc.HasExited) {
    Start-Sleep -Seconds $Tail
    Stop-Process -Id $proc.Id -Force        # our PID only, never -Name
    Write-Output "stopped pid=$($proc.Id)"
} else {
    Write-Output "process exited on its own, code=$($proc.ExitCode)"
}
Write-Output "shots in $OutDir"
