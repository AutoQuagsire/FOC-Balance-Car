param(
    [switch]$UseTuna,
    [switch]$Clean,
    [switch]$OneFile,
    [switch]$Console,
    [string]$PythonVersion = "3.9",
    [string]$VenvDir = ""
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$CodeRoot = Split-Path -Parent $RepoRoot
$WorkspaceRoot = Split-Path -Parent $CodeRoot
$WorkspaceVenvDir = Join-Path $RepoRoot ".venv-gui-pack$($PythonVersion.Replace('.',''))"

if ([string]::IsNullOrWhiteSpace($VenvDir)) {
    $VenvDir = $WorkspaceVenvDir
}

$VenvPython = Join-Path $VenvDir "Scripts\\python.exe"
$ReqFile = Join-Path $RepoRoot "requirements-gui.txt"
$GuiEntry = Join-Path $RepoRoot "tools\\debuglink_gui.py"
$DistDir = Join-Path $RepoRoot "dist"
$BuildDir = Join-Path $RepoRoot "build\\pyinstaller"
$SpecPath = Join-Path $RepoRoot "DebugLinkGUI.spec"
$AppName = "DebugLinkGUI"
$PyVersionShort = ""

function Assert-LastExitCode {
    param(
        [string]$Step
    )
    if ($LASTEXITCODE -ne 0) {
        throw "[PKG] $Step failed (exit code=$LASTEXITCODE)"
    }
}

function Ensure-Venv {
    if (Test-Path $VenvPython) {
        return
    }

    Write-Host "[PKG] creating venv: $VenvDir"
    $PyLauncher = Get-Command py -ErrorAction SilentlyContinue
    if ($null -ne $PyLauncher) {
        & py "-$PythonVersion" -m venv $VenvDir
        Assert-LastExitCode "create venv with py -$PythonVersion"
    } else {
        Write-Host "[PKG] WARN: py launcher not found, fallback to default python"
        & python -m venv $VenvDir
        Assert-LastExitCode "create venv with default python"
    }
}

function Install-GuiDeps {
    Write-Host "[PKG] installing GUI dependencies"
    & $VenvPython -m pip install -U pip
    Assert-LastExitCode "upgrade pip"

    if ($PyVersionShort -eq "3.9") {
        Write-Host "[PKG] python 3.9 detected, using PySide6==6.10.3 compatibility set"
        if ($UseTuna) {
            & $VenvPython -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple "PySide6==6.10.3" "pyserial>=3.5,<4"
        } else {
            & $VenvPython -m pip install "PySide6==6.10.3" "pyserial>=3.5,<4"
        }
        Assert-LastExitCode "install GUI dependencies for python 3.9"
        return
    }

    if ($UseTuna) {
        & $VenvPython -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -r $ReqFile
    } else {
        & $VenvPython -m pip install -r $ReqFile
    }
    Assert-LastExitCode "install GUI dependencies from requirements-gui.txt"
}

function Ensure-GuiDeps {
    try {
        & $VenvPython -c "import PySide6.QtCore, serial" 1>$null 2>$null
        if ($LASTEXITCODE -eq 0) {
            return
        }
    } catch {
    }
    Install-GuiDeps
}

function Ensure-PyInstaller {
    try {
        & $VenvPython -c "import PyInstaller" 1>$null 2>$null
        if ($LASTEXITCODE -eq 0) {
            return
        }
    } catch {
    }

    Write-Host "[PKG] installing PyInstaller"
    if ($UseTuna) {
        & $VenvPython -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple pyinstaller
    } else {
        & $VenvPython -m pip install pyinstaller
    }
    Assert-LastExitCode "install pyinstaller"
}

Ensure-Venv

if (-not (Test-Path $VenvPython)) {
    throw "[PKG] venv python not found: $VenvPython"
}

Write-Host "[PKG] venv python: $VenvPython"
& $VenvPython -c "import sys; print('[PKG] python=' + sys.version.split()[0]); print(sys.version_info[0], sys.version_info[1], sep='.')"
Assert-LastExitCode "query python version"
$PyVersionShort = (& $VenvPython -c "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')").Trim()

Ensure-GuiDeps
Ensure-PyInstaller

if ($Clean) {
    Write-Host "[PKG] cleaning previous outputs"
    if (Test-Path $BuildDir) {
        Remove-Item -Recurse -Force -LiteralPath $BuildDir
    }
    if (Test-Path $DistDir) {
        Remove-Item -Recurse -Force -LiteralPath $DistDir
    }
    if (Test-Path $SpecPath) {
        Remove-Item -Force -LiteralPath $SpecPath
    }
}

$PyArgs = @(
    "-m", "PyInstaller",
    "--noconfirm",
    "--clean",
    "--name", $AppName,
    "--distpath", $DistDir,
    "--workpath", $BuildDir,
    "--specpath", $RepoRoot,
    "--collect-all", "PySide6",
    "--collect-all", "shiboken6"
)

if ($Console) {
    $PyArgs += "--console"
} else {
    $PyArgs += "--windowed"
}

if ($OneFile) {
    $PyArgs += "--onefile"
} else {
    $PyArgs += "--onedir"
}

$PyArgs += $GuiEntry

Write-Host "[PKG] building $AppName ..."
& $VenvPython @PyArgs
Assert-LastExitCode "pyinstaller build"

$ExePath = if ($OneFile) {
    Join-Path $DistDir "$AppName.exe"
} else {
    Join-Path $DistDir "$AppName\\$AppName.exe"
}

if (-not (Test-Path $ExePath)) {
    throw "[PKG] build finished but exe not found: $ExePath"
}

$LauncherPath = if ($OneFile) {
    Join-Path $DistDir "run_DebugLinkGUI.cmd"
} else {
    Join-Path (Join-Path $DistDir $AppName) "run_DebugLinkGUI.cmd"
}

$LauncherContent = @"
@echo off
setlocal
set "APP_DIR=%~dp0"
set "EXE_PATH=%APP_DIR%$AppName.exe"
if not exist "%EXE_PATH%" (
  echo [GUI] executable not found: "%EXE_PATH%"
  exit /b 1
)
set "PATH=%APP_DIR%_internal;%APP_DIR%_internal\PySide6;%APP_DIR%_internal\shiboken6;%PATH%"
set "QT_PLUGIN_PATH=%APP_DIR%_internal\PySide6\plugins"
"%EXE_PATH%" %*
set "RET=%errorlevel%"
if not "%RET%"=="0" (
  echo.
  echo [GUI] executable exited with code %RET%
  pause
)
exit /b %RET%
"@

Set-Content -Path $LauncherPath -Value $LauncherContent -Encoding ASCII

Write-Host "[PKG] success"
Write-Host "[PKG] exe: $ExePath"
Write-Host "[PKG] launcher: $LauncherPath"
Write-Host "[PKG] run: `"$ExePath`" --port COM33 --baud 921600 --rate 100"
