@echo off
setlocal EnableExtensions
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    call install_windows_cn.bat
    if errorlevel 1 exit /b 1
)
call start_windows.bat
