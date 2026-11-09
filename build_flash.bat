@echo off
setlocal
where pwsh.exe >nul 2>&1
if errorlevel 1 (
    echo PowerShell 7 is required. Run build.ps1 with pwsh.exe.
    exit /b 1
)
echo Building the independent HCI/VHCI host. This launcher is build-only.
pwsh.exe -NoProfile -File "%~dp0build.ps1" %*
exit /b %errorlevel%
