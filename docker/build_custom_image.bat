@echo off
REM Build custom Docker image with all dependencies pre-installed
REM This is optional but makes subsequent runs much faster

echo ========================================
echo Building Custom UAV Simulation Image
echo ========================================
echo.

REM Check Docker
docker info >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Docker is not running!
    echo Please start Docker Desktop and try again.
    pause
    exit /b 1
)

echo This will build a custom Docker image with all dependencies.
echo.
echo Benefits:
echo   - Faster startup (no need to install dependencies each time)
echo   - All Python packages pre-installed
echo   - All ROS packages pre-installed
echo.
echo This will take 10-15 minutes and use ~4GB disk space.
echo.
set /p CONTINUE="Continue? (y/n): "
if /i not "%CONTINUE%"=="y" exit /b 0

echo.
echo Building image: uav_simulation:latest
echo.
echo This may take a while...
echo.

REM Build the image
docker build -t uav_simulation:latest -f docker\Dockerfile .

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ========================================
    echo Build Successful!
    echo ========================================
    echo.
    echo Image: uav_simulation:latest
    echo.
    echo To use this image, edit docker_run.bat and change:
    echo   FROM: osrf/ros:noetic-desktop-full
    echo   TO:   uav_simulation:latest
    echo.
    echo Or run directly:
    echo   docker run -it --rm -e DISPLAY=host.docker.internal:0 -v /e/i5_psWaste/i2:/workspace uav_simulation:latest bash
    echo.
) else (
    echo.
    echo ========================================
    echo Build Failed!
    echo ========================================
    echo.
    echo Check the error messages above.
    echo.
)

pause

