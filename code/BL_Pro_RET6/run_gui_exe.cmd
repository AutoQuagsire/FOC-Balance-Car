@echo off
setlocal

set "REPO_ROOT=%~dp0"
set "EXE_PATH=%REPO_ROOT%dist\DebugLinkGUI\DebugLinkGUI.exe"
if not exist "%EXE_PATH%" (
  set "EXE_PATH=%REPO_ROOT%dist\DebugLinkGUI.exe"
)

if not exist "%EXE_PATH%" (
  echo [GUI] packaged exe not found: "%EXE_PATH%"
  echo [GUI] build it first:
  echo   powershell -ExecutionPolicy Bypass -File "%REPO_ROOT%build_gui.ps1" -UseTuna
  exit /b 1
)

for %%I in ("%EXE_PATH%") do set "APP_DIR=%%~dpI"
set "PATH=%APP_DIR%_internal;%APP_DIR%_internal\PySide6;%APP_DIR%_internal\shiboken6;%PATH%"
set "QT_PLUGIN_PATH=%APP_DIR%_internal\PySide6\plugins"

"%EXE_PATH%" %*
set "RET=%errorlevel%"
if not "%RET%"=="0" (
  echo.
  echo [GUI] executable exited with code %RET%
  echo [GUI] if this is a first run on a new PC, try:
  echo [GUI]   1^) use onedir package (dist\DebugLinkGUI\...)
  echo [GUI]   2^) run this launcher from cmd to capture errors
  pause
)
exit /b %RET%
