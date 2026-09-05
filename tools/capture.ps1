# dirt2 — capture the emulator window to a PNG.
#
# Three hard-won rules, all of which cost real time to learn:
#
#  1. Capture with PrintWindow(hwnd, hdc, 2). Flag 2 is PW_RENDERFULLCONTENT, which
#     is the only mode that pulls a GPU-composited window. Desktop CopyFromScreen
#     grabs whatever is on top instead and silently returns the wrong picture.
#
#  2. Target a PROCESS ID, never a window title. Multiple Azahar instances run on
#     this machine at once (a parallel Model Kit / Blocksmith session), and every one
#     of them titles itself "Azahar <version>". A title-matched capture came back
#     showing the wrong project's screen.
#
#  3. MAKE THIS PROCESS DPI-AWARE FIRST. This machine runs a 3440x1440 display at
#     125% scaling, so the logical desktop is 2752x1152. PowerShell starts DPI-
#     UNAWARE: GetWindowRect then returns LOGICAL pixels while PrintWindow renders
#     the window at its real DEVICE resolution. The bitmap comes out 1/1.25 of the
#     size it needs to be and the shot is silently cropped on the right and bottom —
#     which on 2026-08-18 was misread as "the emulator window is too small" and cost
#     three failed window-resizing attempts and one unnecessary redesign of the
#     app's own on-screen layout. The window was never wrong; the capture was.
#
# Usage:  powershell -NoProfile -File tools/capture.ps1 -ProcId 1234 -Out shot.png

param(
    [Parameter(Mandatory=$true)][int]$ProcId,
    [Parameter(Mandatory=$true)][string]$Out
)

Add-Type -AssemblyName System.Drawing

Add-Type @"
using System;
using System.Runtime.InteropServices;
public class Win32Cap {
    [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr hWnd, IntPtr hdcBlt, uint nFlags);
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr hWnd, out RECT lpRect);
    [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }

    // -4 == DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2. Present on Windows 10 1703+.
    [DllImport("user32.dll")] public static extern bool SetProcessDpiAwarenessContext(IntPtr ctx);
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();

    public static string MakeDpiAware() {
        try { if (SetProcessDpiAwarenessContext(new IntPtr(-4))) return "per-monitor-v2"; }
        catch (EntryPointNotFoundException) { }
        // Older Windows, or the context call was refused: system-DPI awareness is
        // enough here because the emulator only ever runs on the primary display.
        return SetProcessDPIAware() ? "system" : "FAILED";
    }
}
"@

# Must happen before any window is measured, and before System.Drawing is used.
$dpiMode = [Win32Cap]::MakeDpiAware()
if ($dpiMode -eq "FAILED") {
    Write-Error "could not make this process DPI-aware; the capture would be cropped"
    exit 6
}

$proc = Get-Process -Id $ProcId -ErrorAction SilentlyContinue
if (-not $proc) { Write-Error "no process with id $ProcId"; exit 2 }

$hwnd = $proc.MainWindowHandle
if ($hwnd -eq [IntPtr]::Zero) { Write-Error "process $ProcId has no main window yet"; exit 3 }

$rect = New-Object Win32Cap+RECT
[void][Win32Cap]::GetWindowRect($hwnd, [ref]$rect)
$w = $rect.Right - $rect.Left
$h = $rect.Bottom - $rect.Top
if ($w -le 0 -or $h -le 0) { Write-Error "window has no size ($w x $h)"; exit 4 }

$bmp = New-Object System.Drawing.Bitmap $w, $h
$gfx = [System.Drawing.Graphics]::FromImage($bmp)
$hdc = $gfx.GetHdc()
$ok  = [Win32Cap]::PrintWindow($hwnd, $hdc, 2)
$gfx.ReleaseHdc($hdc)
$gfx.Dispose()

if (-not $ok) { Write-Error "PrintWindow failed"; exit 5 }

$bmp.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
$bmp.Dispose()
Write-Output "captured pid=$ProcId hwnd=$hwnd ${w}x${h} dpi=$dpiMode -> $Out"
