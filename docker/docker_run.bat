@echo off
REM Run UAV Simulation Docker Container
REM Make sure to run start_xserver.bat first!

echo ========================================
echo UAV Simulation Docker Container
echo ========================================
echo.

REM Check if Docker is running
docker info >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Docker is not running!
    echo.
    echo Please start Docker Desktop and try again.
    echo.
    pause
    exit /b 1
)

echo Docker is running...
echo.

REM Check if VcXsrv is running
tasklist /FI "IMAGENAME eq vcxsrv.exe" 2>NUL | find /I /N "vcxsrv.exe">NUL
if "%ERRORLEVEL%"=="1" (
    echo WARNING: VcXsrv X Server is not running!
    echo.
    echo GUI applications will not work without X Server.
    echo Please run start_xserver.bat first.
    echo.
    set /p CONTINUE="Continue anyway? (y/n): "
    if /i not "%CONTINUE%"=="y" exit /b 1
)

echo Starting Docker container...
echo.
echo Container name: uav_sim
echo Image: osrf/ros:noetic-desktop-full
echo Workspace: /workspace (mounted from current directory)
echo.

REM Get current directory in Unix format for Docker
set CURRENT_DIR=%CD%
set UNIX_PATH=%CURRENT_DIR:\=/%
set UNIX_PATH=%UNIX_PATH:C:=/c%
set UNIX_PATH=%UNIX_PATH:D:=/d%
set UNIX_PATH=%UNIX_PATH:E:=/e%
set UNIX_PATH=%UNIX_PATH:F:=/f%

echo Mounting: %UNIX_PATH% to /workspace
echo.

REM Run Docker container
docker run -it --rm ^
  -e DISPLAY=host.docker.internal:0 ^
  -e QT_X11_NO_MITSHM=1 ^
  -v "%UNIX_PATH%:/workspace" ^
  --name uav_sim ^
  osrf/ros:noetic-desktop-full ^
  bash

echo.
echo Container exited.
echo.
pause

