@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple"
set "PIP_INDEX_URL=%MIRROR%"
set "PIP_DEFAULT_TIMEOUT=120"
set "PIP_DISABLE_PIP_VERSION_CHECK=1"

echo ============================================================
echo LeatherCAD V0.4.1 - China Mirror Installer
echo PyPI mirror: %MIRROR%
echo ============================================================
echo.

where py >nul 2>nul
if not errorlevel 1 (
    set "PYTHON=py"
    goto :python_found
)
where python >nul 2>nul
if not errorlevel 1 (
    set "PYTHON=python"
    goto :python_found
)

echo ERROR: Python was not found.
echo Please install Python 3.10, 3.11, or 3.12 first.
pause
exit /b 1

:python_found
echo [1/3] Creating virtual environment...
if not exist ".venv\Scripts\python.exe" (
    %PYTHON% -m venv .venv
    if errorlevel 1 goto :error
) else (
    echo Existing virtual environment found.
)

echo [2/3] Updating pip tools from China mirror...
".venv\Scripts\python.exe" -m pip install --index-url "%MIRROR%" --upgrade pip setuptools wheel
if errorlevel 1 goto :error

echo [3/3] Installing dependencies from China mirror...
".venv\Scripts\python.exe" -m pip install --index-url "%MIRROR%" -r requirements.txt
if errorlevel 1 goto :error

echo.
echo ============================================================
echo Installation completed successfully.
echo Run start_windows.bat to start LeatherCAD.
echo ============================================================
pause
exit /b 0

:error
echo.
echo ERROR: Installation failed. See the error above.
echo Mirror: %MIRROR%
pause
exit /b 1
