@echo off
setlocal

set "REPO_ROOT=%~dp0"
set "EXE_PATH=%REPO_ROOT%dist_nuitka\debuglink_gui.dist\debuglink_gui.exe"

if not exist "%EXE_PATH%" (
  echo [GUI] Nuitka executable not found: "%EXE_PATH%"
  echo [GUI] build it first:
  echo   powershell -ExecutionPolicy Bypass -File "%REPO_ROOT%build_gui_nuitka.ps1" -UseTuna -Clean
  exit /b 1
)

"%EXE_PATH%" %*
exit /b %errorlevel%
