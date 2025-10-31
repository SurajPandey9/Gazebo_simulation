@echo off
REM Start VcXsrv X Server for Docker GUI support
REM Run this before starting the Docker container

echo ========================================
echo Starting VcXsrv X Server
echo ========================================
echo.

REM Check if VcXsrv is already running
tasklist /FI "IMAGENAME eq vcxsrv.exe" 2>NUL | find /I /N "vcxsrv.exe">NUL
if "%ERRORLEVEL%"=="0" (
    echo VcXsrv is already running!
    echo.
    echo If you need to restart it:
    echo 1. Close VcXsrv from system tray
    echo 2. Run this script again
    echo.
    pause
    exit /b 0
)

REM Try common installation paths
set VCXSRV_PATH=""

if exist "C:\Program Files\VcXsrv\vcxsrv.exe" (
    set VCXSRV_PATH="C:\Program Files\VcXsrv\vcxsrv.exe"
)

if exist "C:\Program Files (x86)\VcXsrv\vcxsrv.exe" (
    set VCXSRV_PATH="C:\Program Files (x86)\VcXsrv\vcxsrv.exe"
)

if %VCXSRV_PATH%=="" (
    echo ERROR: VcXsrv not found!
    echo.
    echo Please install VcXsrv from:
    echo https://sourceforge.net/projects/vcxsrv/
    echo.
    pause
    exit /b 1
)

echo Starting VcXsrv...
echo Path: %VCXSRV_PATH%
echo.

REM Start VcXsrv with optimal settings for Docker
start "" %VCXSRV_PATH% :0 -ac -terminate -lesspointer -multiwindow -clipboard -wgl

echo.
echo VcXsrv started successfully!
echo.
echo Look for the X icon in your system tray.
echo.
echo You can now run: docker_run.bat
echo.
pause

