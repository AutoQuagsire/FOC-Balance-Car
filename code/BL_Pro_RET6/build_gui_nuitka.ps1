param(
    [switch]$UseTuna,
    [switch]$Clean,
    [string]$PythonVersion = "3.9",
    [string]$VenvDir = ""
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
if ([string]::IsNullOrWhiteSpace($VenvDir)) {
    $VenvDir = Join-Path $RepoRoot ".venv-gui-pack$($PythonVersion.Replace('.',''))"
}
$VenvPython = Join-Path $VenvDir "Scripts\\python.exe"
$OutDir = Join-Path $RepoRoot "dist_nuitka"
$GuiEntry = Join-Path $RepoRoot "tools\\debuglink_gui.py"

function Assert-LastExitCode {
    param([string]$Step)
    if ($LASTEXITCODE -ne 0) {
        throw "[NUITKA] $Step failed (exit code=$LASTEXITCODE)"
    }
}

if (-not (Test-Path $VenvPython)) {
    throw "[NUITKA] missing venv python: $VenvPython"
}

& $VenvPython -c "import sys; print('[NUITKA] python=' + sys.version.split()[0])"
Assert-LastExitCode "query python"

if ($Clean -and (Test-Path $OutDir)) {
    Write-Host "[NUITKA] cleaning: $OutDir"
    Remove-Item -Recurse -Force -LiteralPath $OutDir
}

if ($UseTuna) {
    & $VenvPython -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -U nuitka ordered-set zstandard
} else {
    & $VenvPython -m pip install -U nuitka ordered-set zstandard
}
Assert-LastExitCode "install nuitka"

& $VenvPython -m nuitka `
    $GuiEntry `
    --standalone `
    --enable-plugin=pyside6 `
    --assume-yes-for-downloads `
    --output-dir=$OutDir `
    --windows-console-mode=force
Assert-LastExitCode "nuitka build"

$ExePath = Join-Path $OutDir "debuglink_gui.dist\\debuglink_gui.exe"
if (-not (Test-Path $ExePath)) {
    throw "[NUITKA] exe not found: $ExePath"
}

$LauncherPath = Join-Path $OutDir "run_DebugLinkGUI_Nuitka.cmd"
$Launcher = @"
@echo off
setlocal
set "APP_DIR=%~dp0debuglink_gui.dist\"
set "EXE_PATH=%APP_DIR%debuglink_gui.exe"
if not exist "%EXE_PATH%" (
  echo [GUI] executable not found: "%EXE_PATH%"
  exit /b 1
)
"%EXE_PATH%" %*
exit /b %errorlevel%
"@
Set-Content -Path $LauncherPath -Value $Launcher -Encoding ASCII

Write-Host "[NUITKA] success"
Write-Host "[NUITKA] exe: $ExePath"
Write-Host "[NUITKA] launcher: $LauncherPath"
