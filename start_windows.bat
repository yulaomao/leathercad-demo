@echo off
setlocal EnableExtensions
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo LeatherCAD is not installed yet.
    echo Run install_windows_cn.bat first.
    pause
    exit /b 1
)

echo Starting LeatherCAD...
start "" "http://127.0.0.1:5000"
".venv\Scripts\python.exe" app.py
if errorlevel 1 (
    echo.
    echo LeatherCAD stopped with an error.
    pause
)
