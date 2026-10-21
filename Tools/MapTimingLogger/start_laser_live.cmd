@echo off
setlocal
cd /d "%~dp0"
set "SERIAL_PORT=COM9"

set "PYTHON_EXE=E:\Python\python.exe"
if not exist "%PYTHON_EXE%" (
    for %%I in (python.exe) do set "PYTHON_EXE=%%~$PATH:I"
)
if not defined PYTHON_EXE (
    echo Python was not found. Install Python and pyserial first.
    pause
    exit /b 1
)

"%PYTHON_EXE%" -c "import serial" >nul 2>nul
if errorlevel 1 (
    echo pyserial is missing from %PYTHON_EXE%.
    echo Run: "%PYTHON_EXE%" -m pip install pyserial
    pause
    exit /b 1
)

echo Dashboard: http://127.0.0.1:8766/laser_live.html
echo Keep this window open while using the dashboard. Press Ctrl+C to stop.
"%PYTHON_EXE%" laser_live_server.py --port %SERIAL_PORT% --baud 115200 --http-port 8766 --open-browser
if errorlevel 1 (
    echo Dashboard could not start. Check whether port 8766 is already in use.
    pause
    exit /b 1
)
